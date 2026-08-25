#include "sim/echo_simulator.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>

#ifdef SONAR_HAVE_OPENMP
#include <omp.h>
#endif

#include "dsp/fir_filter.h"
#include "dsp/noise.h"
#include "dsp/resampler.h"
#include "dsp/window.h"
#include "sim/echo_fft.h"
#include "util/log.h"
#include "util/perf_timer.h"

namespace sonar::sim {

namespace {

using cplxf = std::complex<float>;

struct DirectTap {
    int delay = 0;
    int transmitter = 0;
    float weight = 0.0f;
};

struct PhaseWaveform {
    int first_index = 0;
    std::vector<float> real;
    std::vector<float> imag;
};

thread_local bool g_last_cpu_fft_used = false;

bool env_enabled(const char* name, bool fallback) {
    const char* value = std::getenv(name);
    if (!value || !*value) return fallback;
    return std::strcmp(value, "0") != 0 && std::strcmp(value, "false") != 0 &&
           std::strcmp(value, "FALSE") != 0 && std::strcmp(value, "off") != 0 &&
           std::strcmp(value, "OFF") != 0;
}

int env_positive_int(const char* name, int fallback) {
    const char* value = std::getenv(name);
    if (!value || !*value) return fallback;
    const int parsed = std::atoi(value);
    return parsed > 0 ? parsed : fallback;
}

// MATLAB round == C++ std::round (round half away from zero).
inline double matlab_round(double x) { return std::round(x); }

// Column helpers that keep the per-column processing in double and the big
// matrices in single precision (matching MATLAB's 'single' storage).

std::vector<cplx> col_to_double(const MatFC& m, int c) {
    std::vector<cplx> v(static_cast<size_t>(m.rows()));
    for (int i = 0; i < m.rows(); ++i) v[static_cast<size_t>(i)] = cplx(m(i, c));
    return v;
}
void col_from_double(MatFC& m, int c, const std::vector<cplx>& v) {
    for (int i = 0; i < m.rows(); ++i) m(i, c) = cplxf(static_cast<float>(v[static_cast<size_t>(i)].real()),
                                                       static_cast<float>(v[static_cast<size_t>(i)].imag()));
}

void resample_mat_columns(MatFC& m, int P, int Q) {
    const int Nr = m.cols();
    const int R0 = m.rows();
    if (Nr == 0) return;
    // Extract all columns first (m is resized in-place during the loop).
    std::vector<std::vector<cplx>> cols(static_cast<size_t>(Nr));
    for (int r = 0; r < Nr; ++r) cols[static_cast<size_t>(r)] = col_to_double(m, r);
    // Resample every column in parallel, then resize + store once.
    std::vector<std::vector<cplx>> rs_cols(static_cast<size_t>(Nr));
#pragma omp parallel for schedule(dynamic)
    for (int r = 0; r < Nr; ++r) {
        rs_cols[static_cast<size_t>(r)] = dsp::resample(cols[static_cast<size_t>(r)], P, Q);
    }
    const int out_rows = static_cast<int>(rs_cols[0].size());
    m.resize(out_rows, Nr);
    for (int r = 0; r < Nr; ++r) col_from_double(m, r, rs_cols[static_cast<size_t>(r)]);
    (void)R0;
}

}  // namespace

bool last_echo_used_cpu_fft() { return g_last_cpu_fft_used; }

void sim_rx_from_scatterers(const MatD& P, const MatD& A, const MatD& TX, const MatD& RX,
                            double c, double fs, const MatC& excitation,
                            const EchoSimOptions& opts, MatFC& y, double& t0) {
    const int K = P.rows();
    const int Nt = TX.rows();
    const int Nr = RX.rows();
    g_last_cpu_fft_used = false;
    if (K == 0 || Nr == 0) {
        y = MatFC(0, Nr);
        t0 = 0.0;
        return;
    }

    // Optional CUDA accelerator: same algorithm, same accumulation order.
    if (cuda_echo_enabled()) {
        if (echo_cuda_sim(P, A, TX, RX, c, fs, excitation, opts, y, t0)) {
            SONAR_LOG_INFO("echo: CUDA backend (K=%d Nt=%d Nr=%d)", K, Nt, Nr);
            return;
        }
        SONAR_LOG_WARN("echo: CUDA backend failed, falling back to CPU");
    }

    // Independent internal echo oversampling. factor=5 is the balanced default:
    // interp_factor*fs = 16 MHz, working rate = 80 MHz, return rate = 16 MHz.
    const int oversampleFactor =
        env_positive_int("ESL_ECHO_OVERSAMPLE_FACTOR", opts.oversample_factor);
    const double fs_work = fs * static_cast<double>(oversampleFactor);

    // ---- upsample each excitation row to the internal working rate ----
    std::vector<std::vector<cplx>> x_all(static_cast<size_t>(Nt));
    int Mx = 0;
#pragma omp parallel for schedule(dynamic)
    for (int j = 0; j < Nt; ++j) {
        std::vector<cplx> row(static_cast<size_t>(excitation.cols()));
        for (int m = 0; m < excitation.cols(); ++m) row[static_cast<size_t>(m)] = excitation(j, m);
        x_all[static_cast<size_t>(j)] = dsp::resample(row, oversampleFactor, 1);
    }
    if (Nt > 0) Mx = static_cast<int>(x_all[0].size());

    std::vector<double> delayK(static_cast<size_t>(K), 0.0);
    if (!opts.delay.empty()) {
        if (static_cast<int>(opts.delay.size()) == K) delayK = opts.delay;
        else if (opts.delay.size() == 1) std::fill(delayK.begin(), delayK.end(), opts.delay[0]);
        else throw std::runtime_error("opts.delay size mismatch");
    }

    const bool atten_twoway = (opts.atten == "twoway_R");
    const bool atten_sqrt = (opts.atten == "sqrt_twoway_R");

    // ---- Pass 1: earliest / latest arrival ----
    double t_min = std::numeric_limits<double>::infinity();
    double t_max = -std::numeric_limits<double>::infinity();
    // MSVC's stable OpenMP front-end lacks min/max reductions. Keep this
    // bounds scan serial there; other compilers retain the reduction.
#if !defined(_MSC_VER)
#pragma omp parallel for reduction(min : t_min) reduction(max : t_max) schedule(static)
#endif
    for (int r = 0; r < Nr; ++r) {
        const double rx0 = RX(r, 0), rx1 = RX(r, 1), rx2 = RX(r, 2);
        for (int k = 0; k < K; ++k) {
            const double dx = P(k, 0) - rx0, dy = P(k, 1) - rx1, dz = P(k, 2) - rx2;
            const double Rrx = std::sqrt(dx * dx + dy * dy + dz * dz);
            for (int j = 0; j < Nt; ++j) {
                const double tdx = P(k, 0) - TX(j, 0), tdy = P(k, 1) - TX(j, 1), tdz = P(k, 2) - TX(j, 2);
                const double Rtx = std::sqrt(tdx * tdx + tdy * tdy + tdz * tdz);
                const double tau = (Rtx + Rrx) / c + delayK[static_cast<size_t>(k)];
                t_min = std::min(t_min, tau);
                t_max = std::max(t_max, tau);
            }
        }
    }
    t0 = t_min;

    // Preferred CPU path: build the sparse IR once per receiver and perform
    // batched linear convolution with FFTW. The original shift-add remains
    // available as a numerical reference and as a no-FFTW fallback.
    if (env_enabled("ESL_CPU_FFT_ECHO", opts.use_fft) && cpu_fft_echo_available()) {
        if (echo_fft_convolve(P, A, TX, RX, c, fs_work, x_all, delayK, opts.round,
                              opts.atten, oversampleFactor, t0, t_max, y)) {
            g_last_cpu_fft_used = true;
            return;
        }
        SONAR_LOG_WARN("echo: FFTW CPU path failed, falling back to direct shift-add");
    }

    // Split each upsampled excitation into float real/imag planes so the hot
    // direct reference loop is a pure single-precision FMA.
    std::vector<std::vector<float>> xre(static_cast<size_t>(Nt)), xim(static_cast<size_t>(Nt));
    for (int j = 0; j < Nt; ++j) {
        xre[static_cast<size_t>(j)].resize(static_cast<size_t>(Mx));
        xim[static_cast<size_t>(j)].resize(static_cast<size_t>(Mx));
        for (int m = 0; m < Mx; ++m) {
            xre[static_cast<size_t>(j)][static_cast<size_t>(m)] =
                static_cast<float>(x_all[static_cast<size_t>(j)][static_cast<size_t>(m)].real());
            xim[static_cast<size_t>(j)][static_cast<size_t>(m)] =
                static_cast<float>(x_all[static_cast<size_t>(j)][static_cast<size_t>(m)].imag());
        }
    }

    // ---- Pass 2: accumulate per-channel output via sparse shift-add ----
    // y[:,r] = sum_j conv(xj, ir_tx[:,j]) = sum_j sum_k w(k,j) * shift(xj, n(k,j))
    //
    // Tiled for memory bandwidth: the output column is accumulated in a small
    // on-chip buffer (y_sh) one sample-block at a time, so the repeated
    // read-modify-write of the large y column across (r,k) is eliminated;
    // only the excitation planes are streamed. The accumulation order per
    // output sample is identical to the plain serial loop (k,j increasing),
    // hence the result is bit-identical. Parallelised over the receive
    // channel r (each thread owns a whole column; no races).
    const int Lir = static_cast<int>(matlab_round((t_max - t0) * fs_work)) + 1;
    const int Mout = Lir + Mx - 1;

    const bool legacyDirect =
        env_enabled("ESL_ECHO_LEGACY_DIRECT", opts.use_legacy_direct);
    if (!legacyDirect) {
        // Optimized direct kernel. Geometry/taps are invariant across output
        // tiles, so compute them once per receive channel. Each channel is
        // downsampled immediately, avoiding the full working-rate matrix and
        // the former matrix-wide double-precision copy in resample_mat_columns.
        const int outputRows = static_cast<int>(std::ceil(
            static_cast<double>(Mout) / static_cast<double>(oversampleFactor)));
        y.resize(outputRows, Nr);
        const int BS = std::max(
            256, env_positive_int("ESL_ECHO_TILE_SAMPLES", 65536));

        // Filtering and downsampling are linear. Split each working-rate
        // excitation into fractional-delay phases so a tap at
        // d=q*factor+r accumulates a short, already-filtered waveform at the
        // final rate. The delay itself is still quantized on the 80 MHz grid.
        const bool fuseDownsample =
            env_enabled("ESL_ECHO_FUSED_DOWNSAMPLE", opts.fuse_downsample);
        std::vector<std::vector<PhaseWaveform>> phaseWaveforms;
        if (fuseDownsample) {
            phaseWaveforms.resize(static_cast<size_t>(Nt));
            for (auto& phases : phaseWaveforms)
                phases.resize(static_cast<size_t>(oversampleFactor));
            const std::vector<double> h =
                dsp::resample_filter(1, oversampleFactor);
            const int filterLength = static_cast<int>(h.size());
            const int filterHalf = (filterLength - 1) / 2;

#pragma omp parallel for schedule(dynamic)
            for (int phaseIndex = 0;
                 phaseIndex < Nt * oversampleFactor; ++phaseIndex) {
                const int j = phaseIndex / oversampleFactor;
                const int phase = phaseIndex % oversampleFactor;
                PhaseWaveform& waveform =
                    phaseWaveforms[static_cast<size_t>(j)][static_cast<size_t>(phase)];
                const int first = (phase - filterHalf) / oversampleFactor;
                const int last =
                    (Mx - 1 + phase + filterHalf) / oversampleFactor;
                waveform.first_index = first;
                const int count = last - first + 1;
                waveform.real.resize(static_cast<size_t>(count));
                waveform.imag.resize(static_cast<size_t>(count));

                for (int outIndex = first; outIndex <= last; ++outIndex) {
                    const int center =
                        filterHalf + outIndex * oversampleFactor - phase;
                    const int sourceFirst =
                        std::max(0, center - (filterLength - 1));
                    const int sourceLast = std::min(Mx - 1, center);
                    double real = 0.0;
                    double imag = 0.0;
                    // This matches dsp::upfirdn's filter-index order for one
                    // shifted excitation.
                    for (int source = sourceLast; source >= sourceFirst; --source) {
                        const double coeff =
                            h[static_cast<size_t>(center - source)];
                        real += coeff *
                                xre[static_cast<size_t>(j)][static_cast<size_t>(source)];
                        imag += coeff *
                                xim[static_cast<size_t>(j)][static_cast<size_t>(source)];
                    }
                    const size_t dst = static_cast<size_t>(outIndex - first);
                    waveform.real[dst] = static_cast<float>(real);
                    waveform.imag[dst] = static_cast<float>(imag);
                }
            }
        }

#pragma omp parallel
        {
            std::vector<DirectTap> taps;
            taps.reserve(static_cast<size_t>(K) * Nt);
            std::vector<float> yre(static_cast<size_t>(BS), 0.0f);
            std::vector<float> yim(static_cast<size_t>(BS), 0.0f);
            std::vector<cplxf> raw(
                fuseDownsample ? 0 : static_cast<size_t>(Mout));
            std::vector<cplx> rawDouble(
                fuseDownsample ? 0 : static_cast<size_t>(Mout));

#pragma omp for schedule(static)
            for (int r = 0; r < Nr; ++r) {
                taps.clear();
                const double rx0 = RX(r, 0), rx1 = RX(r, 1), rx2 = RX(r, 2);
                for (int k = 0; k < K; ++k) {
                    const double dx = P(k, 0) - rx0;
                    const double dy = P(k, 1) - rx1;
                    const double dz = P(k, 2) - rx2;
                    const double Rrx = std::sqrt(dx * dx + dy * dy + dz * dz);
                    const double Ak = A(k, 0);
                    for (int j = 0; j < Nt; ++j) {
                        const double tdx = P(k, 0) - TX(j, 0);
                        const double tdy = P(k, 1) - TX(j, 1);
                        const double tdz = P(k, 2) - TX(j, 2);
                        const double Rtx = std::sqrt(tdx * tdx + tdy * tdy + tdz * tdz);
                        const double tau =
                            (Rtx + Rrx) / c + delayK[static_cast<size_t>(k)];
                        long long n =
                            static_cast<long long>(matlab_round((tau - t0) * fs_work));
                        if (n < 0) n = 0;
                        if (n >= Lir) continue;

                        double w = Ak;
                        if (atten_twoway) w = Ak / (Rtx * Rrx);
                        else if (atten_sqrt) w = Ak / std::sqrt(Rtx * Rrx);
                        const float wf = static_cast<float>(w);
                        if (wf != 0.0f)
                            taps.push_back(
                                DirectTap{static_cast<int>(n), j, wf});
                    }
                }

                if (fuseDownsample) {
                    for (int m0 = 0; m0 < outputRows; m0 += BS) {
                        const int bsz = std::min(BS, outputRows - m0);
                        std::fill(yre.begin(), yre.begin() + bsz, 0.0f);
                        std::fill(yim.begin(), yim.begin() + bsz, 0.0f);
                        for (const DirectTap& tap : taps) {
                            const int coarseDelay = tap.delay / oversampleFactor;
                            const int phase = tap.delay % oversampleFactor;
                            const PhaseWaveform& waveform =
                                phaseWaveforms[static_cast<size_t>(tap.transmitter)]
                                              [static_cast<size_t>(phase)];
                            const int waveformLength =
                                static_cast<int>(waveform.real.size());
                            const int signalStart =
                                coarseDelay + waveform.first_index;
                            const int sourceFirst =
                                std::max(0, m0 - signalStart);
                            const int sourceLast = std::min(
                                waveformLength, m0 + bsz - signalStart);
                            const int count = sourceLast - sourceFirst;
                            if (count <= 0) continue;
                            const int dst = signalStart + sourceFirst - m0;
#if !defined(_MSC_VER)
#pragma omp simd
#endif
                            for (int sample = 0; sample < count; ++sample) {
                                yre[static_cast<size_t>(dst + sample)] +=
                                    tap.weight * waveform.real[
                                                     static_cast<size_t>(sourceFirst + sample)];
                                yim[static_cast<size_t>(dst + sample)] +=
                                    tap.weight * waveform.imag[
                                                     static_cast<size_t>(sourceFirst + sample)];
                            }
                        }
                        for (int i = 0; i < bsz; ++i)
                            y(m0 + i, r) =
                                cplxf(yre[static_cast<size_t>(i)],
                                      yim[static_cast<size_t>(i)]);
                    }
                } else {
                    for (int m0 = 0; m0 < Mout; m0 += BS) {
                        const int bsz = std::min(BS, Mout - m0);
                        std::fill(yre.begin(), yre.begin() + bsz, 0.0f);
                        std::fill(yim.begin(), yim.begin() + bsz, 0.0f);
                        for (const DirectTap& tap : taps) {
                            const int lo = std::max(0, m0 - tap.delay);
                            const int hi = std::min(Mx, m0 + bsz - tap.delay);
                            const int count = hi - lo;
                            if (count <= 0) continue;
                            const int dst = tap.delay + lo - m0;
                            const auto& xre_j =
                                xre[static_cast<size_t>(tap.transmitter)];
                            const auto& xim_j =
                                xim[static_cast<size_t>(tap.transmitter)];
#if !defined(_MSC_VER)
#pragma omp simd
#endif
                            for (int q = 0; q < count; ++q) {
                                yre[static_cast<size_t>(dst + q)] +=
                                    tap.weight * xre_j[static_cast<size_t>(lo + q)];
                                yim[static_cast<size_t>(dst + q)] +=
                                    tap.weight * xim_j[static_cast<size_t>(lo + q)];
                            }
                        }
                        for (int i = 0; i < bsz; ++i)
                            raw[static_cast<size_t>(m0 + i)] =
                                cplxf(yre[static_cast<size_t>(i)],
                                      yim[static_cast<size_t>(i)]);
                    }

                    for (int i = 0; i < Mout; ++i)
                        rawDouble[static_cast<size_t>(i)] =
                            cplx(raw[static_cast<size_t>(i)]);
                    const std::vector<cplx> resampled =
                        dsp::resample(rawDouble, 1, oversampleFactor);
                    for (int i = 0; i < outputRows; ++i) {
                        const cplx value = resampled[static_cast<size_t>(i)];
                        y(i, r) = cplxf(static_cast<float>(value.real()),
                                        static_cast<float>(value.imag()));
                    }
                }
            }
        }
        SONAR_LOG_INFO(
            "echo: optimized direct CPU (factor=%d, tile=%d, precomputed taps, %s)",
            oversampleFactor, BS,
            fuseDownsample ? "polyphase-fused downsample" : "per-channel resample");
        return;
    }

    // Historical tiled kernel retained for validation.
    y.resize(Mout, Nr);

    const int BS = 4096;  // samples per tile (~32 KB complex<float>, stays in L2)
#pragma omp parallel
    {
        std::vector<cplxf> ysh(static_cast<size_t>(BS), cplxf(0.0f, 0.0f));
#pragma omp for schedule(dynamic)
        for (int r = 0; r < Nr; ++r) {
            const double rx0 = RX(r, 0), rx1 = RX(r, 1), rx2 = RX(r, 2);
            cplxf* ycol = &y(0, r);
            for (int m0 = 0; m0 < Mout; m0 += BS) {
                const int bsz = std::min(BS, Mout - m0);
                std::fill(ysh.begin(), ysh.end(), cplxf(0.0f, 0.0f));
                for (int k = 0; k < K; ++k) {
                    const double dx = P(k, 0) - rx0, dy = P(k, 1) - rx1, dz = P(k, 2) - rx2;
                    const double Rrx = std::sqrt(dx * dx + dy * dy + dz * dz);
                    const double Ak = A(k, 0);
                    for (int j = 0; j < Nt; ++j) {
                        const double tdx = P(k, 0) - TX(j, 0), tdy = P(k, 1) - TX(j, 1),
                                     tdz = P(k, 2) - TX(j, 2);
                        const double Rtx = std::sqrt(tdx * tdx + tdy * tdy + tdz * tdz);
                        const double tau = (Rtx + Rrx) / c + delayK[static_cast<size_t>(k)];
                        long long n = static_cast<long long>(matlab_round((tau - t0) * fs_work));
                        if (n < 0) n = 0;
                        if (n >= Lir) continue;

                        double w = Ak;
                        if (atten_twoway) w = Ak / (Rtx * Rrx);
                        else if (atten_sqrt) w = Ak / std::sqrt(Rtx * Rrx);
                        if (w == 0.0) continue;

                        // overlap of shifted excitation [n, n+Mx) with tile [m0, m0+bsz)
                        const long long lo = std::max(0LL, static_cast<long long>(m0) - n);
                        const long long hi =
                            std::min(static_cast<long long>(Mx),
                                     static_cast<long long>(m0) + bsz - n);
                        const float wf = static_cast<float>(w);
                        const auto& xre_j = xre[static_cast<size_t>(j)];
                        const auto& xim_j = xim[static_cast<size_t>(j)];
                        for (long long t = lo; t < hi; ++t) {
                            const int ml = static_cast<int>(n + t - m0);
                            ysh[static_cast<size_t>(ml)] +=
                                cplxf(wf * xre_j[static_cast<size_t>(t)],
                                      wf * xim_j[static_cast<size_t>(t)]);
                        }
                    }
                }
                for (int i = 0; i < bsz; ++i) ycol[m0 + i] = ysh[static_cast<size_t>(i)];
            }
        }
    }

    // ---- resample back to the public fs passed into this function ----
    resample_mat_columns(y, 1, oversampleFactor);
}

static void echo_pipeline_stages_impl(const SonarConfig& s, const MatD& P, const MatD& A,
                                      EchoStages& st, bool keepStages) {
    const int Nr = s.Nrx;
    const bool profile = env_enabled("ESL_ECHO_PROFILE", false);
    Timer stageTimer;

    // sim_rx_from_scatterers_perTX(..., sonar.interp_factor*sonar.fs, ...)
    EchoSimOptions opt;
    opt.use_fft = s.cpu_fft_echo;
    opt.oversample_factor = s.echo_oversample_factor;
    MatFC v;
    double t0 = 0.0;
    sim_rx_from_scatterers(P, A, s.tx_xyz, s.rx_xyz, s.c0, s.interp_factor * s.fs, s.exc_nt, opt, v, t0);
    const double simMs = stageTimer.ms();
    stageTimer.reset();

    // ---- pad front zeros by propagation delay t (seconds) ----
    const long long delay_samples =
        std::max<long long>(0, static_cast<long long>(matlab_round(t0 * s.interp_factor * s.fs)));
    int N = v.rows();
    if (delay_samples > 0) {
        MatFC vpad(N + static_cast<int>(delay_samples), Nr);
#pragma omp parallel for schedule(static)
        for (int r = 0; r < Nr; ++r)
            std::copy_n(&v(0, r), N,
                        &vpad(static_cast<int>(delay_samples), r));
        v = std::move(vpad);
        N = v.rows();
    }

    // ---- normalize to [-1, 1] (v is real-valued here for LFM) ----
    {
        float mn = std::numeric_limits<float>::infinity();
        float mx = -std::numeric_limits<float>::infinity();
#if !defined(_MSC_VER)
#pragma omp parallel for reduction(min : mn) reduction(max : mx) schedule(static)
#endif
        for (long long index = 0; index < static_cast<long long>(v.size()); ++index) {
            mn = std::min(mn, v.data()[index].real());
            mx = std::max(mx, v.data()[index].real());
        }
        const float range = mx - mn;
        if (range > 0.0f) {
#pragma omp parallel for schedule(static)
            for (long long index = 0; index < static_cast<long long>(v.size()); ++index) {
                cplxf val = (v.data()[index] - mn) / range;
                val = (val - cplxf(0.5f, 0.0f)) * cplxf(2.0f, 0.0f);
                v.data()[index] = val;
            }
        }
    }
    const double padNormalizeMs = stageTimer.ms();
    stageTimer.reset();

    // ---- v = resample(v, 1, interp_factor) : 16MHz -> 2MHz ----
    resample_mat_columns(v, 1, static_cast<int>(s.interp_factor));
    const double interpResampleMs = stageTimer.ms();
    stageTimer.reset();
    if (keepStages) st.v_sim = v;  // normalized signal at fs rate (before doppler/awgn)

    // ---- v_calc = resample(v, c0 + 2*velocity, c0)  (identity when velocity=0) ----
    MatFC v_calc;
    const double p = s.c0 + 2.0 * (s.velocity.empty() ? 0.0 : s.velocity[0]);
    const double q = s.c0;
    if (std::abs(p - q) < 1e-12) {
        if (keepStages) v_calc = v;
        else v_calc = std::move(v);
    } else {
        if (keepStages) v_calc = v;
        else v_calc = std::move(v);
        resample_mat_columns(v_calc, static_cast<int>(std::llround(p)),
                             static_cast<int>(std::llround(q)));
    }
    N = v_calc.rows();
    const double dopplerMs = stageTimer.ms();
    stageTimer.reset();

    // ---- awgn (measured) ----
    const double snr = s.snr_level.empty() ? 0.0 : s.snr_level[0];
#pragma omp parallel for schedule(static)
    for (int r = 0; r < Nr; ++r) {
        std::vector<cplx> col = col_to_double(v_calc, r);
        col = dsp::awgn(col, snr, 42 + static_cast<unsigned>(r));
        col_from_double(v_calc, r, col);
    }
    const double noiseMs = stageTimer.ms();
    stageTimer.reset();

    // ---- down-conversion mix : v_mix = v_calc .* exp(j*2*pi*Subfc(1)*(0:N-1)/fs)' ----
    // NOTE: MATLAB's trailing ' is the CONJUGATE TRANSPOSE -> negative-frequency
    // LO exp(-j*2*pi*f*t), so the imaginary part of the LO is -sin().
    const double subfc = s.Subfc.empty() ? s.fc : s.Subfc[0];
    MatFC v_mix(N, Nr);
    std::vector<cplxf> lo(static_cast<size_t>(N));
#pragma omp parallel for schedule(static)
    for (int i = 0; i < N; ++i) {
        const double ph = 2.0 * M_PI * subfc * static_cast<double>(i) / s.fs;
        lo[static_cast<size_t>(i)] =
            cplxf(static_cast<float>(std::cos(ph)), static_cast<float>(-std::sin(ph)));
    }
#pragma omp parallel for schedule(static)
    for (int r = 0; r < Nr; ++r)
        for (int i = 0; i < N; ++i)
            v_mix(i, r) = v_calc(i, r) * lo[static_cast<size_t>(i)];
    const double mixMs = stageTimer.ms();
    stageTimer.reset();

    // ---- FIR (lowpass, single sideband) ----
    std::vector<double> lp = dsp::design_lowpass_single(s.BW * 0.5, 60.0, 0.5, s.fs);
    MatFC y_fir(N, Nr);
#pragma omp parallel for schedule(static)
    for (int r = 0; r < Nr; ++r) {
        std::vector<cplx> col = col_to_double(v_mix, r);
        std::vector<cplx> fy = dsp::fir_filter_same(col, lp);
        col_from_double(y_fir, r, fy);
    }
    const double firMs = stageTimer.ms();
    stageTimer.reset();

    if (keepStages) {
        st.v_rs = std::move(v_calc);  // after resample/doppler/awgn (fs rate)
        st.v_mix = std::move(v_mix);
        st.y_fir = y_fir;  // before decimation
    }

    // ---- y_deci = resample(y_fir, 1, decimation_factor) ----
    resample_mat_columns(y_fir, 1, s.decimation_factor);
    const double decimateMs = stageTimer.ms();
    st.y_deci = std::move(y_fir);
    st.t0 = t0;
    if (profile) {
        SONAR_LOG_INFO(
            "echo profile ms: sim=%.1f pad+norm=%.1f rs_interp=%.1f doppler=%.1f noise=%.1f mix=%.1f fir=%.1f rs_decim=%.1f",
            simMs, padNormalizeMs, interpResampleMs, dopplerMs, noiseMs, mixMs,
            firMs, decimateMs);
    }
}

void echo_pipeline_stages(const SonarConfig& s, const MatD& P, const MatD& A,
                          EchoStages& st) {
    echo_pipeline_stages_impl(s, P, A, st, true);
}

void echo_pipeline(const SonarConfig& s, const MatD& P, const MatD& A, EchoFrame& out) {
    EchoStages st;
    echo_pipeline_stages_impl(s, P, A, st, false);
    out.y_deci = std::move(st.y_deci);
    out.fs_deci = s.fs / s.decimation_factor;
    out.t0 = st.t0;
    out.backend = last_echo_used_cpu_fft() ? "cpp_cpu_fftw" : "cpp_cpu_direct";
}

}  // namespace sonar::sim
