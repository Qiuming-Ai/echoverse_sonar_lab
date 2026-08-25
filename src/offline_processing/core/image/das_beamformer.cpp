#include "image/das_beamformer.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>

#include "dsp/resampler.h"
#include "dsp/window.h"
#include "util/log.h"

namespace sonar::image {

namespace {

inline double matlab_round(double x) { return std::round(x); }

}  // namespace

void das_plane_wave_id(const DasCfg& cfg, const MatC& rx, MatC& beam, MatD& ranges) {
    const int Nsamp = rx.rows();
    const int M = rx.cols();
    const int Na = static_cast<int>(cfg.angles.size());
    if (Na == 0 || M == 0) {
        beam = MatC(0, Na);
        ranges = MatD(0, 1);
        return;
    }

    const double fs = cfg.fs;
    const double c = cfg.c;
    const double t0 = cfg.t0;
    const double fs_calc = (cfg.fs_calc <= 0.0) ? fs : cfg.fs_calc;

    // angles = -cfg.angles(:).'  (MATLAB negates the angles)
    std::vector<double> angles(static_cast<size_t>(Na));
    for (int i = 0; i < Na; ++i) angles[static_cast<size_t>(i)] = -cfg.angles[static_cast<size_t>(i)];

    const int sgn = (cfg.fd_sign < 0) ? -1 : 1;

    // channel weights
    std::vector<double> w = cfg.w;
    if (w.empty()) w = dsp::hamming(M);
    if (static_cast<int>(w.size()) != M) w = std::vector<double>(static_cast<size_t>(M), 1.0);

    // CF options
    const bool cf_enable = cfg.cf_enable;
    const std::string cf_mode = cfg.cf_mode;
    const double cf_gamma = cfg.cf_gamma;
    const double cf_eps = cfg.cf_eps;
    const double gcf_p = cfg.gcf_p;
    int slsc_L = cfg.slsc_L;
    if (slsc_L <= 0) slsc_L = std::min(16, std::max(1, M / 4));
    slsc_L = std::min(slsc_L, std::max(0, M - 1));

    const double f0 = cfg.f0;

    // ------------------------------------------------------------------
    // Optional resample to fs_calc (MATLAB: if fs_calc ~= fs)
    // ------------------------------------------------------------------
    int Nsamp_new = Nsamp;
    const MatC* rx_p = &rx;
    MatC rx_new;
    if (fs_calc != fs) {
        const int P = static_cast<int>(std::llround(fs_calc / fs * 1000000));
        // simplify to a rational with the same ratio using integer rounding of
        // a 1e-6 tolerance (approximation of MATLAB rat(fs_calc/fs, 1e-6)).
        // We use P/Q = round(fs_calc/fs * 1e6) / 1e6.
        const int Q = 1000000;
        std::vector<cplx> col(Nsamp);
        int out_rows = 0;
        for (int m = 0; m < M; ++m) {
            for (int i = 0; i < Nsamp; ++i) col[static_cast<size_t>(i)] = rx(i, m);
            std::vector<cplx> rs = dsp::resample(col, P, Q);
            if (m == 0) {
                out_rows = static_cast<int>(rs.size());
                rx_new.resize(out_rows, M);
            }
            for (int i = 0; i < out_rows; ++i) rx_new(i, m) = rs[static_cast<size_t>(i)];
        }
        Nsamp_new = out_rows;
        rx_p = &rx_new;
        SONAR_LOG_INFO("das: resampled %d -> %d @ fs_calc=%.0f", Nsamp, Nsamp_new, fs_calc);
    }
    const double fs_work = fs_calc;

    // ------------------------------------------------------------------
    // Delay ranges over all angles (decide padding)
    // ------------------------------------------------------------------
    std::vector<double> dx(static_cast<size_t>(M)), dz(static_cast<size_t>(M));
    for (int m = 0; m < M; ++m) {
        dx[static_cast<size_t>(m)] = cfg.rx_xyz(m, 0);
        dz[static_cast<size_t>(m)] = cfg.rx_xyz(m, 2);
    }
    double min_d = std::numeric_limits<double>::infinity();
    double max_d = -std::numeric_limits<double>::infinity();
    for (int j = 0; j < Na; ++j) {
        const double ux = std::sin(angles[static_cast<size_t>(j)]);
        const double uz = std::cos(angles[static_cast<size_t>(j)]);
        for (int m = 0; m < M; ++m) {
            const double d = sgn * (fs_work / c) * (dx[static_cast<size_t>(m)] * ux + dz[static_cast<size_t>(m)] * uz);
            min_d = std::min(min_d, d);
            max_d = std::max(max_d, d);
        }
    }
    const int pad_left = static_cast<int>(std::max(0.0, -std::floor(min_d)));
    const int pad_right = static_cast<int>(std::max(0.0, std::ceil(max_d)));
    const int Lsig = Nsamp_new + pad_left + pad_right;

    MatC beam_work(Lsig, Na);
#pragma omp parallel for schedule(dynamic)
    for (int j = 0; j < Na; ++j) {
        const double ux = std::sin(angles[static_cast<size_t>(j)]);
        const double uz = std::cos(angles[static_cast<size_t>(j)]);

        // per-channel integer delays
        std::vector<int> n_int(static_cast<size_t>(M));
        std::vector<double> d_samp(static_cast<size_t>(M));
        for (int m = 0; m < M; ++m) {
            d_samp[static_cast<size_t>(m)] =
                sgn * (fs_work / c) * (cfg.rx_xyz(m, 0) * ux + cfg.rx_xyz(m, 2) * uz);
            n_int[static_cast<size_t>(m)] = static_cast<int>(matlab_round(d_samp[static_cast<size_t>(m)]));
        }

        // shifted matrix x_shift [Lsig x M]
        MatC x_shift(Lsig, M);
        for (int m = 0; m < M; ++m) {
            const int start = pad_left + n_int[static_cast<size_t>(m)];
            for (int i = 0; i < Nsamp_new; ++i) {
                const int dst = start + i;
                if (dst >= 0 && dst < Lsig) x_shift(dst, m) = (*rx_p)(i, m);
            }
            // carrier phase compensation
            if (f0 != 0.0) {
                const cplx ph = std::exp(cplx(0.0, -2.0 * M_PI * f0 * d_samp[static_cast<size_t>(m)] / fs_work));
                for (int i = 0; i < Lsig; ++i) x_shift(i, m) *= ph;
            }
        }

        // CF factor
        std::vector<double> cf_fac(static_cast<size_t>(Lsig), 1.0);
        if (cf_enable) {
            std::vector<cplx> s_coh(static_cast<size_t>(Lsig), {0.0, 0.0});
            std::vector<double> s_abs2(static_cast<size_t>(Lsig), 0.0);
            std::vector<double> s_abs1(static_cast<size_t>(Lsig), 0.0);
            for (int i = 0; i < Lsig; ++i) {
                for (int m = 0; m < M; ++m) {
                    const cplx v = x_shift(i, m);
                    s_coh[static_cast<size_t>(i)] += v;
                    s_abs2[static_cast<size_t>(i)] += std::norm(v);
                    s_abs1[static_cast<size_t>(i)] += std::abs(v);
                }
            }
            const double Meff = static_cast<double>(M);
            for (int i = 0; i < Lsig; ++i) {
                double fac = 0.0;
                if (cf_mode == "cf") {
                    fac = std::norm(s_coh[static_cast<size_t>(i)]) /
                          (Meff * s_abs2[static_cast<size_t>(i)] + cf_eps);
                } else if (cf_mode == "mcf") {
                    fac = std::norm(s_coh[static_cast<size_t>(i)]) /
                          (s_abs1[static_cast<size_t>(i)] * s_abs1[static_cast<size_t>(i)] + cf_eps);
                } else if (cf_mode == "gcf") {
                    double num = std::pow(std::abs(s_coh[static_cast<size_t>(i)]), gcf_p);
                    double den = std::pow(Meff, std::max(gcf_p - 1.0, 0.0));
                    double s_abs_p = 0.0;
                    for (int m = 0; m < M; ++m)
                        s_abs_p += std::pow(std::abs(x_shift(i, m)), gcf_p);
                    fac = num / (den * s_abs_p + cf_eps);
                } else if (cf_mode == "pcf") {
                    // | (1/Meff) * sum_m e^{j*angle(x_m)} |
                    // ph = x_m / max(|x_m|, eps);  fac = |sum ph| / Meff
                    cplx ph_sum(0.0, 0.0);
                    for (int m = 0; m < M; ++m) {
                        const cplx v = x_shift(i, m);
                        const double d = std::max(std::abs(v), cf_eps);
                        ph_sum += v / d;
                    }
                    fac = std::abs(ph_sum) / Meff;
                } else if (cf_mode == "slsc") {
                    // mean_k < | x_m x_{m-k}* | / (|x_m||x_{m-k}|) >
                    // over lags k = 1..slsc_L (k < M)
                    if (slsc_L > 0) {
                        double Csum = 0.0;
                        for (int k = 1; k <= slsc_L && k < M; ++k) {
                            double num = 0.0, den = 0.0;
                            for (int m = 0; m < M - k; ++m) {
                                const cplx x1 = x_shift(i, m + k);
                                const cplx x2 = x_shift(i, m);
                                num += std::abs(x1 * std::conj(x2));
                                den += std::abs(x1) * std::abs(x2);
                            }
                            Csum += num / (den + cf_eps);
                        }
                        fac = Csum / static_cast<double>(slsc_L);
                    } else {
                        fac = 1.0;
                    }
                } else {
                    fac = 1.0;  // unknown mode: neutral
                }
                fac = std::pow(fac, cf_gamma);
                fac = std::min(std::max(fac, 0.0), 1.0);
                cf_fac[static_cast<size_t>(i)] = fac;
            }
        }

        // beam = (x_shift * w) .* cf_fac
        for (int i = 0; i < Lsig; ++i) {
            cplx acc(0.0, 0.0);
            for (int m = 0; m < M; ++m) acc += x_shift(i, m) * w[static_cast<size_t>(m)];
            beam_work(i, j) = acc * cf_fac[static_cast<size_t>(i)];
        }
    }

    // ------------------------------------------------------------------
    // Resample back to fs (identity when fs_work == fs)
    // ------------------------------------------------------------------
    const int Nsamp_out = Nsamp + 30;
    beam.resize(Nsamp_out, Na);
    std::vector<cplx> col(Lsig);
    for (int j = 0; j < Na; ++j) {
        std::vector<cplx> out_col(static_cast<size_t>(Nsamp_out), {0.0, 0.0});
        if (fs_work == fs) {
            for (int i = 0; i < Lsig && i < Nsamp_out; ++i)
                out_col[static_cast<size_t>(i)] = beam_work(i, j);
        } else {
            for (int i = 0; i < Lsig; ++i) col[static_cast<size_t>(i)] = beam_work(i, j);
            std::vector<cplx> rs = dsp::resample(col, 1, 1);  // placeholder (see diff notes)
            for (size_t i = 0; i < out_col.size() && i < rs.size(); ++i)
                out_col[i] = rs[i];
        }
        for (int i = 0; i < Nsamp_out; ++i) beam(i, j) = out_col[static_cast<size_t>(i)];
    }

    ranges.resize(Nsamp_out, 1);
    for (int i = 0; i < Nsamp_out; ++i)
        ranges(i, 0) = c * (t0 + static_cast<double>(i) / fs) / 2.0;
}

}  // namespace sonar::image

