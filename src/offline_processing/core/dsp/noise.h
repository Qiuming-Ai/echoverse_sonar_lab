#pragma once
// Noise / awgn (statistically equivalent to MATLAB awgn; not bit-exact).
#include <complex>
#include <random>
#include <vector>

namespace sonar::dsp {

// awgn(x, snr_db, 'measured') : add Gaussian noise so that the SNR of the
// measured signal power equals snr_db. MATLAB uses its own RNG; we use
// std::mt19937 and document statistical (not bit) equivalence.
// If snr_db is empty or <= 0 semantics: MATLAB awgn with snr 0 is
// mathematically meaningful, so we honour the exact formula.
inline std::vector<std::complex<double>> awgn(
    const std::vector<std::complex<double>>& x, double snr_db, unsigned int seed = 42) {
    std::vector<std::complex<double>> y = x;
    const size_t n = x.size();
    if (n == 0) return y;

    // measured signal power (complex) = mean(|x|^2)
    double power = 0.0;
    for (const auto& v : x) power += std::norm(v);
    power /= static_cast<double>(n);
    if (!(power > 0.0)) return y;

    // noise power such that snr_db = 10*log10(signal_power / noise_power)
    const double noise_power = power / std::pow(10.0, snr_db / 10.0);
    const double sigma = std::sqrt(noise_power / 2.0);  // per real/imag component

    std::mt19937 gen(seed);
    std::normal_distribution<double> dist(0.0, sigma);
    for (auto& v : y) {
        v += std::complex<double>(dist(gen), dist(gen));
    }
    return y;
}

inline std::vector<double> awgn(const std::vector<double>& x, double snr_db,
                                unsigned int seed = 42) {
    std::vector<double> y = x;
    const size_t n = x.size();
    if (n == 0) return y;
    double power = 0.0;
    for (double v : x) power += v * v;
    power /= static_cast<double>(n);
    if (!(power > 0.0)) return y;
    const double noise_power = power / std::pow(10.0, snr_db / 10.0);
    const double sigma = std::sqrt(noise_power);
    std::mt19937 gen(seed);
    std::normal_distribution<double> dist(0.0, sigma);
    for (auto& v : y) v += dist(gen);
    return y;
}

}  // namespace sonar::dsp

