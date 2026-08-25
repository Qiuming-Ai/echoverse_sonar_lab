#include "dsp/fir_filter.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace sonar::dsp {

double bessel_i0(double x) {
    // Series expansion, accurate for moderate |x|.
    const double ax = std::fabs(x);
    if (ax < 1e-12) return 1.0;
    double sum = 1.0;
    double term = 1.0;
    const double x2 = x * x / 4.0;
    for (int k = 1; k < 200; ++k) {
        term *= x2 / (static_cast<double>(k) * k);
        sum += term;
        if (term < 1e-18 * sum) break;
    }
    return sum;
}

std::vector<double> kaiser_window(int M, double beta) {
    std::vector<double> w(static_cast<size_t>(M));
    if (M <= 1) {
        if (M == 1) w[0] = 1.0;
        return w;
    }
    const double denom = static_cast<double>(M - 1);
    const double i0b = bessel_i0(beta);
    for (int n = 0; n < M; ++n) {
        const double arg = 2.0 * n / denom - 1.0;
        w[static_cast<size_t>(n)] = bessel_i0(beta * std::sqrt(std::max(0.0, 1.0 - arg * arg))) / i0b;
    }
    return w;
}

// Kaiser beta (MATLAB signal.internal.kaiserBeta).
static double kaiser_beta(double A) {
    if (A > 50.0) return 0.1102 * (A - 8.7);
    if (A >= 21.0) return 0.5842 * std::pow(A - 21.0, 0.4) + 0.07886 * (A - 21.0);
    return 0.0;
}

// MATLAB sinc(x) = sin(pi*x)/(pi*x), sinc(0)=1.
static double m_sinc(double x) {
    if (std::fabs(x) < 1e-15) return 1.0;
    return std::sin(M_PI * x) / (M_PI * x);
}

// Exact port of MATLAB fdesign kaiserwin lowpass (designfilt kaiserwin):
//   A   = [dpass dstop] (linear deviations)
//   [N, Wn, beta] = kaiserord([Fpass Fstop]/Fs, [1 0], A)   (Fs normalized)
//   h   = firls(N, [0 Wn Wn 1], [1 1 0 0]) .* kaiser(N+1, beta)
//   h   = h / sum(h)      (fir1 default scaling, unity DC gain)
// firls for this full-band / constant-weight case reduces to the closed form:
//   type II (N odd, M=N+1 even): a(j) = 2*Wn*sinc((j-0.5)*Wn), h = 0.5*[flip(a), a]
//   type I  (N even, M odd):     center = Wn, h[center±i] = Wn*sinc(i*Wn)
// Verified bit-exact against MATLAB designfilt/SignalFilter (golden).
std::vector<double> design_kaiser_lowpass(double Fpass, double Fstop, double Apass, double Astop,
                                          double Fs) {
    const double dpass = (std::pow(10.0, Apass / 20.0) - 1.0) / (std::pow(10.0, Apass / 20.0) + 1.0);
    const double dstop = std::pow(10.0, -Astop / 20.0);
    const double atten = -20.0 * std::log10(std::min(dpass, dstop));

    // kaiserord (normalized to Fs)
    const double df = (Fstop - Fpass) / Fs;
    const double L = (atten - 7.95) / (2.0 * M_PI * 2.285) / df + 1.0;
    int N = static_cast<int>(std::ceil(L)) - 1;
    if (N < 1) N = 1;
    // kaiserord: odd N with non-zero gain at Nyquist would be +1; lowpass has
    // zero gain at Nyquist, so N is kept as-is.
    const double Wn = (Fpass + Fstop) / Fs;
    const double beta = kaiser_beta(atten);

    const int M = N + 1;
    std::vector<double> h(static_cast<size_t>(M), 0.0);

    if (M % 2 == 0) {
        // type II (even length): h = 0.5*[flip(a), a], a(j) = 2*Wn*sinc((j-0.5)*Wn)
        const int half = M / 2;
        for (int j = 0; j < half; ++j) {
            const double a = 2.0 * Wn * m_sinc((static_cast<double>(j) + 0.5) * Wn);
            h[static_cast<size_t>(half - 1 - j)] = 0.5 * a;
            h[static_cast<size_t>(half + j)] = 0.5 * a;
        }
    } else {
        // type I (odd length): center = Wn, h[center±i] = Wn*sinc(i*Wn)
        const int center = M / 2;
        h[static_cast<size_t>(center)] = Wn;
        for (int i = 1; i <= center; ++i) {
            const double v = Wn * m_sinc(static_cast<double>(i) * Wn);
            h[static_cast<size_t>(center - i)] = v;
            h[static_cast<size_t>(center + i)] = v;
        }
    }

    // window + unity-DC scaling (fir1 default, NOT 'noscale')
    std::vector<double> win = kaiser_window(M, beta);
    double sum_h = 0.0;
    for (int n = 0; n < M; ++n) {
        h[static_cast<size_t>(n)] *= win[static_cast<size_t>(n)];
        sum_h += h[static_cast<size_t>(n)];
    }
    if (sum_h != 0.0)
        for (auto& v : h) v /= sum_h;
    return h;
}

std::vector<double> design_lowpass_single(double passband_hz, double stopband_atten_db,
                                          double passband_ripple_db, double fs) {
    // Mirror of SignalFilter.localLowpassBands for the 'single' sideband case.
    const double nyquist = fs / 2.0;
    // resolveTransitionWidth: default 0.1 * (nyquist - passband), min 0.05*limit + eps
    const double limit = nyquist - passband_hz;
    double width = 0.1 * limit;
    width = std::max(width, limit * 0.05 + 1e-16);
    width = std::min(width, limit - 1e-16);
    width = std::max(width, 1e-16);
    const double stopband = std::min(passband_hz + width, nyquist * 0.999);
    return design_kaiser_lowpass(passband_hz, stopband, passband_ripple_db, stopband_atten_db, fs);
}

std::vector<double> fir_filter_same(const std::vector<double>& x, const std::vector<double>& h) {
    const int n = static_cast<int>(x.size());
    const int k = static_cast<int>(h.size());
    if (n == 0) return {};
    std::vector<double> y(static_cast<size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        double acc = 0.0;
        const int jmax = std::min(k - 1, i);
        for (int j = 0; j <= jmax; ++j) {
            acc += h[static_cast<size_t>(j)] * x[static_cast<size_t>(i - j)];
        }
        y[static_cast<size_t>(i)] = acc;
    }
    return y;
}

std::vector<std::complex<double>> fir_filter_same(const std::vector<std::complex<double>>& x,
                                                  const std::vector<double>& h) {
    const int n = static_cast<int>(x.size());
    const int k = static_cast<int>(h.size());
    if (n == 0) return {};
    std::vector<std::complex<double>> y(static_cast<size_t>(n), {0.0, 0.0});
    for (int i = 0; i < n; ++i) {
        std::complex<double> acc(0.0, 0.0);
        const int jmax = std::min(k - 1, i);
        for (int j = 0; j <= jmax; ++j) {
            acc += h[static_cast<size_t>(j)] * x[static_cast<size_t>(i - j)];
        }
        y[static_cast<size_t>(i)] = acc;
    }
    return y;
}

}  // namespace sonar::dsp

