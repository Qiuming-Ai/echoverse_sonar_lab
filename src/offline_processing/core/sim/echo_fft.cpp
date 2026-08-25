#include "sim/echo_fft.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

#ifdef SONAR_HAVE_FFTW
#include <fftw3.h>
#endif

#ifdef SONAR_HAVE_OPENMP
#include <omp.h>
#endif

#include "dsp/resampler.h"
#include "util/log.h"

namespace sonar::sim {

namespace {

long long delay_index(double seconds, double fs, const std::string& rounding) {
    const double samples = seconds * fs;
    if (rounding == "floor") return static_cast<long long>(std::floor(samples));
    if (rounding == "ceil") return static_cast<long long>(std::ceil(samples));
    return static_cast<long long>(std::round(samples));
}

int next_fast_size(int minimum) {
    if (minimum <= 2) return std::max(1, minimum);
    for (int n = minimum; n < std::numeric_limits<int>::max(); ++n) {
        int rest = n;
        while ((rest % 2) == 0) rest /= 2;
        while ((rest % 3) == 0) rest /= 3;
        while ((rest % 5) == 0) rest /= 5;
        if (rest == 1) return n;
    }
    throw std::runtime_error("Echo FFT length exceeds INT_MAX");
}

int fft_thread_count() {
    int count = 1;
#ifdef SONAR_HAVE_OPENMP
    count = std::max(1, omp_get_max_threads());
#endif
    const char* value = std::getenv("ESL_FFTW_THREADS");
    if (value && *value) count = std::max(1, std::atoi(value));
    return count;
}

int echo_batch_size() {
    int batch = 32;
    const char* value = std::getenv("ESL_FFTW_ECHO_BATCH");
    if (value && *value) batch = std::max(1, std::atoi(value));
    return batch;
}

#ifdef SONAR_HAVE_FFTW

struct FftwDeleter {
    void operator()(fftwf_complex* ptr) const { fftwf_free(ptr); }
};
using FftwBuffer = std::unique_ptr<fftwf_complex, FftwDeleter>;

std::mutex& planner_mutex() {
    static std::mutex mutex;
    return mutex;
}

void init_threads_locked() {
    static bool initialized = false;
    if (!initialized) {
        if (fftwf_init_threads() == 0) throw std::runtime_error("fftwf_init_threads failed");
        initialized = true;
    }
    fftwf_plan_with_nthreads(fft_thread_count());
}

fftwf_plan make_plan(int nfft, int batch, int sign, fftwf_complex* data) {
    std::lock_guard<std::mutex> lock(planner_mutex());
    init_threads_locked();
    int dims[1] = {nfft};
    fftwf_plan plan = fftwf_plan_many_dft(1, dims, batch, data, nullptr, 1, nfft, data,
                                          nullptr, 1, nfft, sign, FFTW_ESTIMATE);
    if (!plan) throw std::runtime_error("Echo FFTW plan-many creation failed");
    return plan;
}

inline void add_single_precision(fftwf_complex& value, float increment) {
    // MATLAB's default IR is single. Quantize after every accumulation so the
    // sparse-IR construction has the same precision even though FFTW is double.
    value[0] += increment;
}

#endif

}  // namespace

bool cpu_fft_echo_available() {
#ifdef SONAR_HAVE_FFTW
    return true;
#else
    return false;
#endif
}

bool echo_fft_convolve(const MatD& points, const MatD& amplitudes, const MatD& tx,
                       const MatD& rx, double sound_speed, double fs_work,
                       const std::vector<std::vector<cplx>>& excitation,
                       const std::vector<double>& delays, const std::string& rounding,
                       const std::string& attenuation, int oversample_factor,
                       double t0, double tmax,
                       MatFC& output) {
#ifndef SONAR_HAVE_FFTW
    (void)points;
    (void)amplitudes;
    (void)tx;
    (void)rx;
    (void)sound_speed;
    (void)fs_work;
    (void)excitation;
    (void)delays;
    (void)rounding;
    (void)attenuation;
    (void)oversample_factor;
    (void)t0;
    (void)tmax;
    (void)output;
    return false;
#else
    const int scatterers = points.rows();
    const int transmitters = tx.rows();
    const int receivers = rx.rows();
    if (transmitters == 0 || excitation.empty()) return false;
    const int excitation_length = static_cast<int>(excitation[0].size());
    const int ir_length = static_cast<int>(delay_index(tmax - t0, fs_work, rounding)) + 1;
    const int convolution_length = ir_length + excitation_length - 1;
    const int nfft = next_fast_size(convolution_length);
    if (oversample_factor <= 0)
        throw std::runtime_error("Echo FFT oversample factor must be positive");
    const int output_rows = static_cast<int>(std::ceil(
        static_cast<double>(convolution_length) / static_cast<double>(oversample_factor)));
    output.resize(output_rows, receivers);

    const bool atten_twoway = attenuation == "twoway_R";
    const bool atten_sqrt = attenuation == "sqrt_twoway_R";

    // The excitation spectrum is invariant across all receive channels and is
    // therefore computed once per TX.
    std::vector<std::vector<std::complex<float>>> excitation_spectra(
        static_cast<size_t>(transmitters),
        std::vector<std::complex<float>>(static_cast<size_t>(nfft)));
    FftwBuffer excitation_buffer(static_cast<fftwf_complex*>(
        fftwf_malloc(sizeof(fftwf_complex) * static_cast<size_t>(nfft))));
    if (!excitation_buffer) throw std::bad_alloc();
    fftwf_plan excitation_plan = make_plan(nfft, 1, FFTW_FORWARD, excitation_buffer.get());
    for (int transmitter = 0; transmitter < transmitters; ++transmitter) {
        std::memset(excitation_buffer.get(), 0,
                    sizeof(fftwf_complex) * static_cast<size_t>(nfft));
        for (int i = 0; i < excitation_length; ++i) {
            // Match MATLAB x_all = cast(excitation, 'single').
            const std::complex<float> sample(
                static_cast<float>(excitation[static_cast<size_t>(transmitter)]
                                               [static_cast<size_t>(i)].real()),
                static_cast<float>(excitation[static_cast<size_t>(transmitter)]
                                               [static_cast<size_t>(i)].imag()));
            excitation_buffer.get()[i][0] = sample.real();
            excitation_buffer.get()[i][1] = sample.imag();
        }
        fftwf_execute(excitation_plan);
        for (int i = 0; i < nfft; ++i)
            excitation_spectra[static_cast<size_t>(transmitter)][static_cast<size_t>(i)] =
                std::complex<float>(excitation_buffer.get()[i][0],
                                    excitation_buffer.get()[i][1]);
    }
    fftwf_destroy_plan(excitation_plan);

    const int configured_batch = std::min(receivers, echo_batch_size());
    SONAR_LOG_INFO(
        "echo: FFTW sparse-IR convolution (factor=%d, Nfft=%d, channels/batch=%d, FFT threads=%d)",
        oversample_factor, nfft, configured_batch, fft_thread_count());

    // Reuse one aligned work area and one plan-many pair for every channel
    // block.  The final partial block is zero padded to the configured batch
    // size; avoiding repeated multi-million-point planning is much faster than
    // creating plans for each block.
    const size_t work_count = static_cast<size_t>(configured_batch) * nfft;
    FftwBuffer work(static_cast<fftwf_complex*>(
        fftwf_malloc(sizeof(fftwf_complex) * work_count)));
    if (!work) throw std::bad_alloc();
    FftwBuffer sum;
    if (transmitters > 1) {
        sum.reset(static_cast<fftwf_complex*>(
            fftwf_malloc(sizeof(fftwf_complex) * work_count)));
        if (!sum) throw std::bad_alloc();
    }
    fftwf_plan forward = make_plan(nfft, configured_batch, FFTW_FORWARD, work.get());
    fftwf_plan inverse = make_plan(nfft, configured_batch, FFTW_BACKWARD,
                                   transmitters == 1 ? work.get() : sum.get());

    for (int first = 0; first < receivers; first += configured_batch) {
        const int batch = std::min(configured_batch, receivers - first);
        if (transmitters > 1) {
            std::memset(sum.get(), 0, sizeof(fftwf_complex) * work_count);
        }

        for (int transmitter = 0; transmitter < transmitters; ++transmitter) {
            std::memset(work.get(), 0, sizeof(fftwf_complex) * work_count);

#pragma omp parallel for schedule(static)
            for (int b = 0; b < batch; ++b) {
                const int receiver = first + b;
                fftwf_complex* ir = work.get() + static_cast<size_t>(b) * nfft;
                const double rx0 = rx(receiver, 0), rx1 = rx(receiver, 1),
                             rx2 = rx(receiver, 2);
                const double tx0 = tx(transmitter, 0), tx1 = tx(transmitter, 1),
                             tx2 = tx(transmitter, 2);
                for (int k = 0; k < scatterers; ++k) {
                    const double rdx = points(k, 0) - rx0;
                    const double rdy = points(k, 1) - rx1;
                    const double rdz = points(k, 2) - rx2;
                    const double rrx = std::sqrt(rdx * rdx + rdy * rdy + rdz * rdz);
                    const double tdx = points(k, 0) - tx0;
                    const double tdy = points(k, 1) - tx1;
                    const double tdz = points(k, 2) - tx2;
                    const double rtx = std::sqrt(tdx * tdx + tdy * tdy + tdz * tdz);
                    const double tau = (rtx + rrx) / sound_speed +
                                       delays[static_cast<size_t>(k)];
                    long long index = delay_index(tau - t0, fs_work, rounding);
                    if (index < 0) index = 0;
                    if (index >= ir_length) continue;

                    double weight = amplitudes(k, 0);
                    if (atten_twoway) weight /= rtx * rrx;
                    else if (atten_sqrt) weight /= std::sqrt(rtx * rrx);
                    if (weight != 0.0)
                        add_single_precision(ir[static_cast<size_t>(index)],
                                             static_cast<float>(weight));
                }
            }

            fftwf_execute(forward);
            const auto& spectrum = excitation_spectra[static_cast<size_t>(transmitter)];
#pragma omp parallel for schedule(static)
            for (long long index = 0; index < static_cast<long long>(work_count); ++index) {
                const int frequency = static_cast<int>(index % nfft);
                const float ar = work.get()[index][0], ai = work.get()[index][1];
                const std::complex<float> x = spectrum[static_cast<size_t>(frequency)];
                const float real = ar * x.real() - ai * x.imag();
                const float imag = ar * x.imag() + ai * x.real();
                if (transmitters == 1) {
                    work.get()[index][0] = real;
                    work.get()[index][1] = imag;
                } else {
                    sum.get()[index][0] += real;
                    sum.get()[index][1] += imag;
                }
            }
        }

        fftwf_execute(inverse);
        const fftwf_complex* completed = transmitters == 1 ? work.get() : sum.get();
        const float inverse_scale = 1.0f / static_cast<float>(nfft);

        // Quantize the full convolution to single before MATLAB-compatible
        // resample(1,factor), then store directly at the final rate. This avoids
        // materializing the complete working-rate [samples x receivers] matrix.
        for (int b = 0; b < batch; ++b) {
            const fftwf_complex* column = completed + static_cast<size_t>(b) * nfft;
            std::vector<cplx> full(static_cast<size_t>(convolution_length));
            for (int i = 0; i < convolution_length; ++i) {
                const std::complex<float> value(
                    static_cast<float>(column[i][0] * inverse_scale),
                    static_cast<float>(column[i][1] * inverse_scale));
                full[static_cast<size_t>(i)] = cplx(value);
            }
            const std::vector<cplx> resampled =
                dsp::resample(full, 1, oversample_factor);
            for (int i = 0; i < output_rows; ++i) {
                const cplx value = resampled[static_cast<size_t>(i)];
                output(i, first + b) = std::complex<float>(
                    static_cast<float>(value.real()), static_cast<float>(value.imag()));
            }
        }
    }
    fftwf_destroy_plan(forward);
    fftwf_destroy_plan(inverse);
    return true;
#endif
}

}  // namespace sonar::sim

