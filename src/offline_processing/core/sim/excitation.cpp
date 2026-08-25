#include "sim/excitation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "dsp/conv.h"
#include "dsp/fft.h"
#include "dsp/fir_filter.h"
#include "dsp/resampler.h"
#include "dsp/window.h"
#include "util/log.h"

namespace sonar::sim {

// MATLAB chirp(t, f0, T1, f1) linear method: x = cos(2*pi*(f0*t + 0.5*k*t^2)),
// k = (f1 - f0) / T1.
static std::vector<double> chirp_linear(const std::vector<double>& t, double f0, double T1,
                                        double f1) {
    std::vector<double> x(t.size());
    const double k = (f1 - f0) / T1;
    for (size_t i = 0; i < t.size(); ++i) {
        const double phase = 2.0 * M_PI * (f0 * t[i] + 0.5 * k * t[i] * t[i]);
        x[i] = std::cos(phase);
    }
    return x;
}

namespace {

// ---------------------------------------------------------------------------
// Shared helpers (common tail of SonarInit.m shared by all tx_types)
// ---------------------------------------------------------------------------

// Resample each column of a complex matrix (MATLAB resample over dim=1).
MatC resample_cols(const MatC& in, int P, int Q) {
    const int C = in.cols();
    MatC out;
    for (int c = 0; c < C; ++c) {
        std::vector<cplx> col(in.rows());
        for (int r = 0; r < in.rows(); ++r) col[static_cast<size_t>(r)] = in(r, c);
        std::vector<cplx> rs = dsp::resample(col, P, Q);
        if (c == 0) out.resize(static_cast<int>(rs.size()), C);
        for (size_t r = 0; r < rs.size(); ++r) out(static_cast<int>(r), c) = rs[r];
    }
    return out;
}

// SignalFilter('SampleRate',fs,'Passband',max(BW),'StopbandAtten',60,
//              'lowpass','single','OutputSameLength',true):
//   Passband = max(array_params.BW) (scalar), NOT max(SubBW); effective
//   passband = BW/2 (SidebandMode single scales by 0.5).
std::vector<double> design_signal_filter(const SonarConfig& s) {
    return dsp::design_lowpass_single(s.BW * 0.5, 60.0, 0.5, s.fs);
}

// Common tail of SonarInit.m (all tx_types):
//   MF is at interp_factor*fs, one column per subband [L x Na].
//   MF    = resample(MF, 1, interp_factor)
//   MF_mix= MF .* exp(1j*2*pi*Subfc*[0:L-1]/fs)'   (negative-frequency LO)
//   MF_fir(:,i) = filt.process(MF_mix(:,i))   for i = 1..sector_num
//   MF_deci = resample(MF_fir, 1, decimation_factor)
// Fills s.MF_mix / s.MF_fir / s.MF(=MF_deci).
void process_mf_common(SonarConfig& s, MatC MF) {
    const int Na = MF.cols();
    MF = resample_cols(MF, 1, static_cast<int>(s.interp_factor));  // -> fs rate
    const int L = MF.rows();

    // Subfc per column (LFM: single entry broadcast; CDM/FDM: per subband)
    std::vector<double> subfc = s.Subfc;
    if (subfc.empty()) subfc = {s.fc};

    // MF_mix = MF .* exp(-j*2*pi*Subfc(a)*(0:L-1)/fs)  (conjugate transpose)
    MatC MF_mix(L, Na);
    for (int a = 0; a < Na; ++a) {
        const double sf = subfc[static_cast<size_t>(std::min(a, static_cast<int>(subfc.size()) - 1))];
        for (int i = 0; i < L; ++i) {
            const double ph = 2.0 * M_PI * sf * static_cast<double>(i) / s.fs;
            MF_mix(i, a) = MF(i, a) * cplx(std::cos(ph), -std::sin(ph));
        }
    }

    // MF_fir(:,i) = filt.process(MF_mix(:,i)), i = 1..sector_num
    const std::vector<double> lp = design_signal_filter(s);
    const int nproc = std::min(std::max(s.sector_num, 1), Na);
    MatC MF_fir(L, Na);
    for (int a = 0; a < nproc; ++a) {
        std::vector<cplx> col(static_cast<size_t>(L));
        for (int i = 0; i < L; ++i) col[static_cast<size_t>(i)] = MF_mix(i, a);
        std::vector<cplx> fy = dsp::fir_filter_same(col, lp);
        for (int i = 0; i < L; ++i) MF_fir(i, a) = fy[static_cast<size_t>(i)];
    }

    // MF_deci = resample(MF_fir, 1, decimation_factor)
    MatC MF_deci = resample_cols(MF_fir, 1, s.decimation_factor);

    s.MF_mix = std::move(MF_mix);
    s.MF_fir = std::move(MF_fir);
    s.MF = std::move(MF_deci);
}

// ---------------------------------------------------------------------------
// CDM: gen_multicode_cazac_excitation (TXSignalGen)
// ---------------------------------------------------------------------------
void gen_multicode_cazac(const SonarConfig& s, MatC& exc_nt, MatC& exc_ori) {
    const int Nt = s.Ntx;
    const double fs_out = s.interp_factor * s.fs;  // interp_factor*fs
    const double fc = s.Subfc.empty() ? s.fc : s.Subfc[0];
    const double fsym = s.SubBW.empty() ? s.BW : s.SubBW[0];
    // N = round(pulse_len / (1/SubBW(1))) = round(pulse_len*SubBW(1))
    const int N = static_cast<int>(std::lround(s.pulse_len * fsym));
    const std::vector<int> roots = {5, 7, 1, 3, 11, 9};
    const int Na = static_cast<int>(roots.size());
    const std::vector<double> amp = {1.2, 1.2, 1.0, 1.0, 1.2, 1.2};

    // angles_deg broadcast to Na (or must already be Na)
    std::vector<double> angles_deg = s.angles_deg;
    if (angles_deg.size() == 1 && Na > 1) {
        angles_deg.assign(static_cast<size_t>(Na), s.angles_deg[0]);
    }

    // ---- far-field unit direction + integer sample delays ----
    // tau(j,a) = dot(TX(j,:), S(a,:))/c ; S = [sin(th),0,cos(th)]
    // delays_samp = round(tau*fs_out); shift so min >= 0
    MatD delays(Nt, Na);
    double dmin = 0.0;
    for (int j = 0; j < Nt; ++j) {
        for (int a = 0; a < Na; ++a) {
            const double th = angles_deg[static_cast<size_t>(a)] * M_PI / 180.0;
            const double tau =
                (s.tx_xyz(j, 0) * std::sin(th) + s.tx_xyz(j, 2) * std::cos(th)) / s.c0;
            const double d = std::round(tau * fs_out);
            delays(j, a) = d;
            if (j == 0 && a == 0) dmin = d;
            dmin = std::min(dmin, d);
        }
    }
    if (dmin < 0.0)
        for (int j = 0; j < Nt; ++j)
            for (int a = 0; a < Na; ++a) delays(j, a) -= dmin;

    // ---- per-subband CAZAC waveform ----
    // X = exp(-1j*pi*r*n*(n+1)/N); x = ifft(X); x = x/sqrt(mean(|x|^2))
    // win 'none' -> no-op; x_fs = resample(x, fs_out, fsym)
    // sig = real(x_fs .* exp(1j*2*pi*fc*t)) * amp(a)
    std::vector<std::vector<double>> sigs(static_cast<size_t>(Na));
    int maxLen = 0;
    for (int a = 0; a < Na; ++a) {
        const int r = roots[static_cast<size_t>(a)];
        std::vector<cplx> X(static_cast<size_t>(N));
        for (int n = 0; n < N; ++n) {
            const double ph = -M_PI * static_cast<double>(r) * static_cast<double>(n) *
                              static_cast<double>(n + 1) / static_cast<double>(N);
            X[static_cast<size_t>(n)] = cplx(std::cos(ph), std::sin(ph));
        }
        std::vector<cplx> x = dsp::ifft(X);  // baseband CAZAC (1/N normalized)
        double p = 0.0;
        for (const auto& v : x) p += std::norm(v);
        p = std::sqrt(p / static_cast<double>(N));
        for (auto& v : x) v /= p;

        // resample from fsym to fs_out (MATLAB reduces p/q: 16e6/1e5 -> 160/1)
        std::vector<cplx> x_fs;
        if (std::fabs(fs_out - fsym) < 1e-12) {
            x_fs = x;
        } else {
            x_fs = dsp::resample(x, static_cast<int>(std::lround(fs_out)),
                                 static_cast<int>(std::lround(fsym)));
        }

        const int Lf = static_cast<int>(x_fs.size());
        std::vector<double> sig(static_cast<size_t>(Lf));
        for (int i = 0; i < Lf; ++i) {
            const double ph = 2.0 * M_PI * fc * static_cast<double>(i) / fs_out;
            // real( (a+jb) * (cos+j sin) ) = a*cos - b*sin
            sig[static_cast<size_t>(i)] =
                (x_fs[static_cast<size_t>(i)].real() * std::cos(ph) -
                 x_fs[static_cast<size_t>(i)].imag() * std::sin(ph)) *
                amp[static_cast<size_t>(a)];
        }
        sigs[static_cast<size_t>(a)] = std::move(sig);
        maxLen = std::max(maxLen, Lf);
    }

    // ---- excitation_ori = zeros(maxLen, Na) zero-padded ----
    exc_ori.resize(maxLen, Na);
    for (int a = 0; a < Na; ++a)
        for (size_t i = 0; i < sigs[static_cast<size_t>(a)].size(); ++i)
            exc_ori(static_cast<int>(i), a) = cplx(sigs[static_cast<size_t>(a)][i], 0.0);

    // ---- superposition onto each element channel ----
    double maxDelay = 0.0;
    for (int j = 0; j < Nt; ++j)
        for (int a = 0; a < Na; ++a) maxDelay = std::max(maxDelay, delays(j, a));
    int Mout = 0;
    for (int a = 0; a < Na; ++a)
        Mout = std::max(Mout, static_cast<int>(sigs[static_cast<size_t>(a)].size()) +
                                  static_cast<int>(maxDelay));

    exc_nt.resize(Nt, Mout);
    for (int a = 0; a < Na; ++a) {
        const int La = static_cast<int>(sigs[static_cast<size_t>(a)].size());
        for (int j = 0; j < Nt; ++j) {
            const int d = static_cast<int>(delays(j, a));
            for (int i = 0; i < La; ++i) {
                const int idx = d + i;
                if (idx < Mout)
                    exc_nt(j, idx) += cplx(sigs[static_cast<size_t>(a)][static_cast<size_t>(i)], 0.0);
            }
        }
    }

    // arrayWin = hamming(Nt); per-element gain
    const std::vector<double> aw = dsp::hamming(Nt);
    for (int j = 0; j < Nt; ++j)
        for (int m = 0; m < Mout; ++m) exc_nt(j, m) *= aw[static_cast<size_t>(j)];
}

// ---------------------------------------------------------------------------
// FDM: gen_multisubband_lfm_excitation (TXSignalGen)
// ---------------------------------------------------------------------------
void gen_multisubband_lfm(const SonarConfig& s, MatC& exc_nt, MatC& exc_ori) {
    const int Nt = s.Ntx;
    const double fs_out = s.interp_factor * s.fs;
    const double fs_internal = 10.0 * fs_out;  // MATLAB: fs = 10*fs at top
    const int Na = static_cast<int>(s.angles_deg.size());
    if (Na == 0) throw std::runtime_error("fdm: angles_deg is empty");
    if (static_cast<int>(s.Subfc.size()) != Na || static_cast<int>(s.SubBW.size()) != Na)
        throw std::runtime_error("fdm: Subfc/SubBW must match angles_deg length");

    // per-subband polarity (broadcast or per-entry)
    std::vector<std::string> pol(static_cast<size_t>(Na), s.pol);
    if (s.pol_vec.size() == static_cast<size_t>(Na))
        pol = s.pol_vec;
    else if (!s.pol_vec.empty())
        throw std::runtime_error("fdm: pol array length mismatch");

    // pw = repmat(pulse_len, [Na 1])
    std::vector<double> pw(static_cast<size_t>(Na), s.pulse_len);

    // ---- per-subband LFM chirp at fs_internal ----
    int Ma = 1;
    for (int a = 0; a < Na; ++a)
        Ma = std::max(Ma, static_cast<int>(std::lround(pw[static_cast<size_t>(a)] * fs_internal)));

    std::vector<std::vector<double>> sigs(static_cast<size_t>(Na));
    for (int a = 0; a < Na; ++a) {
        const int Mpwa =
            std::max(1, static_cast<int>(std::lround(pw[static_cast<size_t>(a)] * fs_internal)));
        std::vector<double> xa(static_cast<size_t>(Ma), 0.0);
        std::vector<double> t(static_cast<size_t>(Mpwa));
        for (int i = 0; i < Mpwa; ++i)
            t[static_cast<size_t>(i)] = static_cast<double>(i) / fs_internal;
        double f0 = s.Subfc[static_cast<size_t>(a)] - s.SubBW[static_cast<size_t>(a)] / 2.0;
        double f1 = s.Subfc[static_cast<size_t>(a)] + s.SubBW[static_cast<size_t>(a)] / 2.0;
        if (pol[static_cast<size_t>(a)] == "down") std::swap(f0, f1);
        std::vector<double> ch = chirp_linear(t, f0, pw[static_cast<size_t>(a)], f1);
        for (int i = 0; i < Mpwa; ++i) xa[static_cast<size_t>(i)] = ch[static_cast<size_t>(i)];
        // win = 'hamming' (default): xa .* hamming(Ma)
        const std::vector<double> win = dsp::hamming(Ma);
        for (int i = 0; i < Ma; ++i) xa[static_cast<size_t>(i)] *= win[static_cast<size_t>(i)];
        sigs[static_cast<size_t>(a)] = std::move(xa);
    }

    // ---- excitation_ori(:,a) = resample(sigs{a}, 1, 10)  (auto-grow) ----
    int maxOri = 0;
    for (int a = 0; a < Na; ++a)
        maxOri = std::max(maxOri, static_cast<int>(std::ceil(Ma / 10.0)));
    exc_ori.resize(maxOri, Na);
    for (int a = 0; a < Na; ++a) {
        std::vector<double> rs = dsp::resample(sigs[static_cast<size_t>(a)], 1, 10);
        for (size_t i = 0; i < rs.size(); ++i)
            exc_ori(static_cast<int>(i), a) = cplx(rs[static_cast<size_t>(i)], 0.0);
    }

    // ---- delays at fs_internal, shift so min >= 0 ----
    MatD delays(Nt, Na);
    double dmin = 0.0;
    for (int j = 0; j < Nt; ++j) {
        for (int a = 0; a < Na; ++a) {
            const double th = s.angles_deg[static_cast<size_t>(a)] * M_PI / 180.0;
            const double tau =
                (s.tx_xyz(j, 0) * std::sin(th) + s.tx_xyz(j, 2) * std::cos(th)) / s.c0;
            const double d = std::round(tau * fs_internal);
            delays(j, a) = d;
            if (j == 0 && a == 0) dmin = d;
            dmin = std::min(dmin, d);
        }
    }
    if (dmin < 0.0)
        for (int j = 0; j < Nt; ++j)
            for (int a = 0; a < Na; ++a) delays(j, a) -= dmin;

    // ---- superposition at fs_internal ----
    double maxDelay = 0.0;
    for (int j = 0; j < Nt; ++j)
        for (int a = 0; a < Na; ++a) maxDelay = std::max(maxDelay, delays(j, a));
    const int Mout = Ma + static_cast<int>(maxDelay);

    exc_nt.resize(Nt, Mout);
    for (int a = 0; a < Na; ++a) {
        const int La = static_cast<int>(sigs[static_cast<size_t>(a)].size());
        for (int j = 0; j < Nt; ++j) {
            const int d = static_cast<int>(delays(j, a));
            for (int i = 0; i < La; ++i) {
                const int idx = d + i;
                if (idx < Mout)
                    exc_nt(j, idx) +=
                        cplx(sigs[static_cast<size_t>(a)][static_cast<size_t>(i)], 0.0);
            }
        }
    }
    const std::vector<double> aw = dsp::hamming(Nt);
    for (int j = 0; j < Nt; ++j)
        for (int m = 0; m < Mout; ++m) exc_nt(j, m) *= aw[static_cast<size_t>(j)];

    // exc_nt = resample(exc_nt', 1, 10)'  (downsample each row back to fs_out)
    // Result stays [Nt x out_len] (row-major storage: rows = Nt).
    {
        MatC out;
        for (int j = 0; j < Nt; ++j) {
            std::vector<cplx> row(static_cast<size_t>(Mout));
            for (int m = 0; m < Mout; ++m) row[static_cast<size_t>(m)] = exc_nt(j, m);
            std::vector<cplx> rs = dsp::resample(row, 1, 10);
            if (j == 0) out.resize(Nt, static_cast<int>(rs.size()));
            for (size_t m = 0; m < rs.size(); ++m) out(j, static_cast<int>(m)) = rs[m];
        }
        exc_nt = std::move(out);
    }
}

// conj(flipud(hilbert(x))) per column (MATLAB matched-filter FIR form).
MatC mf_from_ori(const MatC& ori) {
    const int R = ori.rows();
    const int C = ori.cols();
    MatC mf(R, C);
    for (int c = 0; c < C; ++c) {
        std::vector<double> col(static_cast<size_t>(R));
        for (int r = 0; r < R; ++r) col[static_cast<size_t>(r)] = ori(r, c).real();
        std::vector<cplx> hb = dsp::hilbert(col);
        for (int r = 0; r < R; ++r) mf(r, c) = std::conj(hb[static_cast<size_t>(R - 1 - r)]);
    }
    return mf;
}

}  // namespace

void generate_excitation(SonarConfig& s) {
    const std::string tx_type = s.tx_type;

    if (tx_type == "lfm") {
        // ---- LFM branch (SonarInit.m) ----
        const double fs_hi = s.interp_factor * s.fs;  // excitation sample rate
        const int M = static_cast<int>(std::floor(s.pulse_len * fs_hi)) + 1;
        std::vector<double> t_p(static_cast<size_t>(M));
        for (int i = 0; i < M; ++i) t_p[static_cast<size_t>(i)] = static_cast<double>(i) / fs_hi;

        std::vector<double> chirp = chirp_linear(t_p, s.fc - s.BW / 2.0, s.pulse_len, s.fc + s.BW / 2.0);
        std::vector<double> win = dsp::hamming(M);
        // exc_nt = chirp .* hamming'  (row vector -> store as [1 x M])
        MatC exc(1, M);
        for (int i = 0; i < M; ++i) exc(0, i) = cplx(chirp[static_cast<size_t>(i)] * win[static_cast<size_t>(i)], 0.0);

        // MF = conj(flipud(hilbert(exc_nt(:))))  : analytic signal of the
        // WINDOWED excitation (chirp.*hamming), reversed, conjugated.
        // (at interp_factor*fs; process_mf_common does resample(1,interp_factor))
        std::vector<double> exc_real(static_cast<size_t>(M));
        for (int i = 0; i < M; ++i) exc_real[static_cast<size_t>(i)] = exc(0, i).real();
        std::vector<cplx> hilb = dsp::hilbert(exc_real);
        MatC MFcol(M, 1);
        for (int i = 0; i < M; ++i) {
            MFcol(i, 0) = std::conj(hilb[static_cast<size_t>(M - 1 - i)]);
        }
        process_mf_common(s, std::move(MFcol));

        s.exc_nt = std::move(exc);
        SONAR_LOG_INFO("LFM excitation: M=%d MF_deci=%d", M, static_cast<int>(s.MF.size()));
        return;
    }

    if (tx_type == "cdm") {
        // ---- CDM branch (SonarInit.m) ----
        MatC exc_nt, exc_ori;
        gen_multicode_cazac(s, exc_nt, exc_ori);
        MatC MF = mf_from_ori(exc_ori);  // conj(flipud(hilbert(exc_ori))) [maxLen x Na]
        process_mf_common(s, std::move(MF));
        s.exc_nt = std::move(exc_nt);
        SONAR_LOG_INFO("CDM excitation: Ntx=%d Mout=%d MF_deci=%dx%d", s.Ntx,
                       s.exc_nt.cols(), s.MF.rows(), s.MF.cols());
        return;
    }

    if (tx_type == "fdm") {
        // ---- FDM branch (SonarInit.m) ----
        MatC exc_nt, exc_ori;
        gen_multisubband_lfm(s, exc_nt, exc_ori);
        MatC MF = mf_from_ori(exc_ori);
        process_mf_common(s, std::move(MF));
        s.exc_nt = std::move(exc_nt);
        SONAR_LOG_INFO("FDM excitation: Ntx=%d Mout=%d MF_deci=%dx%d", s.Ntx,
                       s.exc_nt.cols(), s.MF.rows(), s.MF.cols());
        return;
    }

    throw std::runtime_error("tx_type '" + tx_type +
                             "' not yet implemented in C++ port (v1 supports 'lfm','cdm','fdm'; "
                             "see docs/matlab_diff_notes.md)");
}

SonarAttributes build_sonar_attributes(const SonarConfig& s, const std::string& timestamp) {
    SonarAttributes a;
    const std::string t = s.tx_type;
    if (t == "cdm") {
        a.array_type = "CDM";
    } else if (t == "fdm") {
        a.array_type = "FDM";
    } else {
        a.array_type = "Baseline";
    }
    a.signal_type = "Baseband";
    a.signal_win = s.signal_window;

    if (s.SubBW.size() > 1) {
        a.bandwidth = s.SubBW;
    } else {
        a.bandwidth = {s.BW};
    }
    a.sampling_frequency = s.fs;
    if (s.Subfc.size() > 1) {
        a.center_frequency = s.Subfc;
    } else {
        a.center_frequency = {s.fc};
    }
    a.decimate_factor = s.decimation_factor;
    a.sector_num = s.sector_num;
    a.match_filter_data = s.MF;  // complex [M x 1]
    a.receive_array_num = s.Nrx;
    a.receive_array_position = s.rx_xyz;

    a.receive_array_win.resize(s.Nrx, 1);
    std::vector<double> win = dsp::make_window(s.array_window, s.Nrx);
    for (int i = 0; i < s.Nrx; ++i) a.receive_array_win(i, 0) = win[static_cast<size_t>(i)];

    a.pulse_duration = s.pulse_len;
    a.sound_velocity = s.c0;
    a.velocity = s.velocity.empty() ? 0.0 : s.velocity[0];
    a.snr_level = s.snr_level.empty() ? 0.0 : s.snr_level[0];
    a.timestamp = timestamp;

    // scan_angle = concatenation of all sector angle vectors (DataMakerInit: angles_div{:})
    std::vector<double> scan;
    for (const auto& sec : s.angles_div) scan.insert(scan.end(), sec.begin(), sec.end());
    a.scan_angle = scan;

    // sector_div = [first(sec0); last(sec0); last(sec1); ... last(secN-1)]
    if (!s.angles_div.empty()) {
        std::vector<double> edges;
        edges.push_back(s.angles_div[0].front());
        for (const auto& sec : s.angles_div) edges.push_back(sec.back());
        a.sector_div = edges;
    }

    a.sample_delay = s.compensate_range.empty() ? std::vector<double>{0.0} : s.compensate_range;
    return a;
}

}  // namespace sonar::sim

