#include "dsp/fft_convolution.h"

#include <algorithm>
#include <cstdint>
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

namespace sonar::dsp {

namespace {

bool env_enabled(const char* name, bool fallback) {
    const char* value = std::getenv(name);
    if (!value || !*value) return fallback;
    return std::strcmp(value, "0") != 0 && std::strcmp(value, "false") != 0 &&
           std::strcmp(value, "FALSE") != 0 && std::strcmp(value, "off") != 0 &&
           std::strcmp(value, "OFF") != 0;
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

int next_fast_size(int minimum) {
    if (minimum <= 2) return std::max(1, minimum);
    for (int n = minimum; n < std::numeric_limits<int>::max(); ++n) {
        int rest = n;
        while ((rest % 2) == 0) rest /= 2;
        while ((rest % 3) == 0) rest /= 3;
        while ((rest % 5) == 0) rest /= 5;
        if (rest == 1) return n;
    }
    throw std::runtime_error("FFT convolution length exceeds INT_MAX");
}

#ifdef SONAR_HAVE_FFTW

struct FftwDeleter {
    void operator()(fftw_complex* ptr) const { fftw_free(ptr); }
};
using FftwBuffer = std::unique_ptr<fftw_complex, FftwDeleter>;

std::mutex& planner_mutex() {
    static std::mutex mutex;
    return mutex;
}

void init_fftw_threads_locked() {
    static bool initialized = false;
    if (!initialized) {
        if (fftw_init_threads() == 0) throw std::runtime_error("fftw_init_threads failed");
        initialized = true;
    }
    fftw_plan_with_nthreads(fft_thread_count());
}

fftw_plan make_plan(int nfft, int batch, int sign, fftw_complex* buffer) {
    std::lock_guard<std::mutex> lock(planner_mutex());
    init_fftw_threads_locked();
    int dims[1] = {nfft};
    fftw_plan plan = fftw_plan_many_dft(1, dims, batch, buffer, nullptr, 1, nfft, buffer,
                                        nullptr, 1, nfft, sign, FFTW_ESTIMATE);
    if (!plan) throw std::runtime_error("FFTW plan-many creation failed");
    return plan;
}

struct FilterSpectrum {
    int nfft = 0;
    std::vector<cplx> filter;
    std::shared_ptr<const std::vector<cplx>> spectrum;
};

std::shared_ptr<const std::vector<cplx>> cached_filter_spectrum(
    const std::vector<cplx>& filter, int nfft) {
    static std::mutex cache_mutex;
    static std::vector<FilterSpectrum> cache;
    std::lock_guard<std::mutex> lock(cache_mutex);
    for (const auto& item : cache) {
        if (item.nfft == nfft && item.filter == filter) return item.spectrum;
    }

    FftwBuffer buffer(static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) *
                                                              static_cast<size_t>(nfft))));
    if (!buffer) throw std::bad_alloc();
    std::memset(buffer.get(), 0, sizeof(fftw_complex) * static_cast<size_t>(nfft));
    for (size_t i = 0; i < filter.size(); ++i) {
        buffer.get()[i][0] = filter[i].real();
        buffer.get()[i][1] = filter[i].imag();
    }
    fftw_plan plan = make_plan(nfft, 1, FFTW_FORWARD, buffer.get());
    fftw_execute(plan);
    fftw_destroy_plan(plan);

    FilterSpectrum item;
    item.nfft = nfft;
    item.filter = filter;
    auto spectrum = std::make_shared<std::vector<cplx>>(static_cast<size_t>(nfft));
    for (int i = 0; i < nfft; ++i)
        (*spectrum)[static_cast<size_t>(i)] =
            cplx(buffer.get()[i][0], buffer.get()[i][1]);
    item.spectrum = spectrum;
    cache.push_back(std::move(item));
    return spectrum;
}

#endif

}  // namespace

bool fft_convolution_available() {
#ifdef SONAR_HAVE_FFTW
    return true;
#else
    return false;
#endif
}

bool fft_convolve_columns_same(const MatC& input, const std::vector<cplx>& filter,
                               const std::vector<double>& channel_window, MatC& output) {
#ifndef SONAR_HAVE_FFTW
    (void)input;
    (void)filter;
    (void)channel_window;
    (void)output;
    return false;
#else
    if (!env_enabled("ESL_FFT_MATCH_FILTER", true)) return false;
    if (input.empty() || filter.empty()) {
        output.resize(input.rows(), input.cols());
        return true;
    }
    if (!channel_window.empty() && channel_window.size() != static_cast<size_t>(input.cols()))
        throw std::runtime_error("FFT convolution channel window size mismatch");

    const int rows = input.rows();
    const int cols = input.cols();
    const int full_length = rows + static_cast<int>(filter.size()) - 1;
    const int nfft = next_fast_size(full_length);
    const int offset = static_cast<int>(filter.size()) / 2;
    const auto filter_spectrum = cached_filter_spectrum(filter, nfft);
    output.resize(rows, cols);

    // Keep temporary FFT storage bounded for unusually long recordings.  For
    // ordinary 201-channel pings this evaluates to one plan-many batch.
    constexpr size_t max_work_bytes = size_t{512} * 1024 * 1024;
    const size_t bytes_per_column = sizeof(fftw_complex) * static_cast<size_t>(nfft);
    const int max_batch = std::max(1, static_cast<int>(max_work_bytes / bytes_per_column));

    for (int first = 0; first < cols; first += max_batch) {
        const int batch = std::min(max_batch, cols - first);
        const size_t count = static_cast<size_t>(batch) * nfft;
        FftwBuffer buffer(static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * count)));
        if (!buffer) throw std::bad_alloc();
        std::memset(buffer.get(), 0, sizeof(fftw_complex) * count);
        for (int b = 0; b < batch; ++b) {
            fftw_complex* column = buffer.get() + static_cast<size_t>(b) * nfft;
            for (int r = 0; r < rows; ++r) {
                const cplx value = input(r, first + b);
                column[r][0] = value.real();
                column[r][1] = value.imag();
            }
        }

        fftw_plan forward = make_plan(nfft, batch, FFTW_FORWARD, buffer.get());
        fftw_execute(forward);
        fftw_destroy_plan(forward);

#pragma omp parallel for schedule(static)
        for (long long k = 0; k < static_cast<long long>(count); ++k) {
            const int frequency = static_cast<int>(k % nfft);
            const double ar = buffer.get()[k][0], ai = buffer.get()[k][1];
            const cplx h = (*filter_spectrum)[static_cast<size_t>(frequency)];
            buffer.get()[k][0] = ar * h.real() - ai * h.imag();
            buffer.get()[k][1] = ar * h.imag() + ai * h.real();
        }

        fftw_plan inverse = make_plan(nfft, batch, FFTW_BACKWARD, buffer.get());
        fftw_execute(inverse);
        fftw_destroy_plan(inverse);

        const double scale = 1.0 / static_cast<double>(nfft);
        for (int b = 0; b < batch; ++b) {
            const fftw_complex* column = buffer.get() + static_cast<size_t>(b) * nfft;
            const double win = channel_window.empty()
                                   ? 1.0
                                   : channel_window[static_cast<size_t>(first + b)];
            for (int r = 0; r < rows; ++r) {
                const int source = offset + r;
                output(r, first + b) =
                    cplx(column[source][0] * scale * win,
                         column[source][1] * scale * win);
            }
        }
    }
    return true;
#endif
}

}  // namespace sonar::dsp

