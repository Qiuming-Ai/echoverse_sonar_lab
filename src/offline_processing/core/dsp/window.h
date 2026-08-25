#pragma once
// MATLAB-compatible window functions: hamming / hann / blackman.
#include <cmath>
#include <vector>

namespace sonar::dsp {

// MATLAB hamming(n): 0.54 - 0.46*cos(2*pi*n/(N-1)), n = 0..N-1
inline std::vector<double> hamming(int n) {
    std::vector<double> w(static_cast<size_t>(n));
    if (n == 1) {
        w[0] = 1.0;
        return w;
    }
    const double denom = static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        w[static_cast<size_t>(i)] = 0.54 - 0.46 * std::cos(2.0 * M_PI * i / denom);
    }
    return w;
}

// MATLAB hann(n): 0.5 - 0.5*cos(2*pi*n/(N-1)), n = 0..N-1
inline std::vector<double> hann(int n) {
    std::vector<double> w(static_cast<size_t>(n));
    if (n == 1) {
        w[0] = 1.0;
        return w;
    }
    const double denom = static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        w[static_cast<size_t>(i)] = 0.5 - 0.5 * std::cos(2.0 * M_PI * i / denom);
    }
    return w;
}

// MATLAB blackman(n): 0.42 - 0.5*cos(2*pi*n/(N-1)) + 0.08*cos(4*pi*n/(N-1))
inline std::vector<double> blackman(int n) {
    std::vector<double> w(static_cast<size_t>(n));
    if (n == 1) {
        w[0] = 1.0;
        return w;
    }
    const double denom = static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        w[static_cast<size_t>(i)] =
            0.42 - 0.5 * std::cos(2.0 * M_PI * i / denom) + 0.08 * std::cos(4.0 * M_PI * i / denom);
    }
    return w;
}

// Select window by MATLAB-style name; returns ones for unknown/custom names.
inline std::vector<double> make_window(const std::string& name, int n) {
    std::string lower = name;
    for (auto& ch : lower) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (lower == "hamming") return hamming(n);
    if (lower == "hann" || lower == "hanning") return hann(n);
    if (lower == "blackman") return blackman(n);
    return std::vector<double>(static_cast<size_t>(n), 1.0);
}

}  // namespace sonar::dsp

