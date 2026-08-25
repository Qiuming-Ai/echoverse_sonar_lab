#include "dsp/fft.h"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace sonar::dsp {

static bool is_pow2(int n) { return n > 0 && (n & (n - 1)) == 0; }

static int next_pow2(int n) {
    int p = 1;
    while (p < n) p <<= 1;
    return p;
}

void fft_radix2(std::vector<cplx>& data, bool inverse) {
    const int n = static_cast<int>(data.size());
    if (n <= 1) return;
    assert(is_pow2(n));

    // bit-reversal permutation
    for (int i = 1, j = 0; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(data[static_cast<size_t>(i)], data[static_cast<size_t>(j)]);
    }

    for (int len = 2; len <= n; len <<= 1) {
        const double ang = 2.0 * M_PI / len * (inverse ? 1.0 : -1.0);
        const cplx wlen(std::cos(ang), std::sin(ang));
        for (int i = 0; i < n; i += len) {
            cplx w(1.0, 0.0);
            const int half = len >> 1;
            for (int j = 0; j < half; ++j) {
                const cplx u = data[static_cast<size_t>(i + j)];
                const cplx v = data[static_cast<size_t>(i + j + half)] * w;
                data[static_cast<size_t>(i + j)] = u + v;
                data[static_cast<size_t>(i + j + half)] = u - v;
                w *= wlen;
            }
        }
    }

    if (inverse) {
        const double inv_n = 1.0 / n;
        for (auto& v : data) v *= inv_n;
    }
}

std::vector<cplx> fft(const std::vector<cplx>& x, int n) {
    if (n < 0) n = static_cast<int>(x.size());
    std::vector<cplx> data(static_cast<size_t>(n), cplx(0.0, 0.0));
    const size_t m = std::min(x.size(), static_cast<size_t>(n));
    std::copy_n(x.begin(), m, data.begin());

    if (n <= 1) return data;
    if (is_pow2(n)) {
        fft_radix2(data, false);
        return data;
    }

    // Bluestein's algorithm
    const int M = next_pow2(2 * n - 1);
    const double theta = M_PI / n;
    std::vector<cplx> a(static_cast<size_t>(M), cplx(0.0, 0.0));
    std::vector<cplx> b(static_cast<size_t>(M), cplx(0.0, 0.0));
    for (int i = 0; i < n; ++i) {
        const double phase = theta * static_cast<double>(i) * i;
        const cplx w(std::cos(phase), -std::sin(phase));  // exp(-j*pi*i^2/n)
        a[static_cast<size_t>(i)] = data[static_cast<size_t>(i)] * w;
        b[static_cast<size_t>(i)] = std::conj(w);
    }
    for (int i = 1; i < n; ++i) {
        const double phase = theta * static_cast<double>(i) * i;
        const cplx w(std::cos(phase), -std::sin(phase));
        b[static_cast<size_t>(M - i)] = std::conj(w);
    }
    fft_radix2(a, false);  // A = FFT(a)
    fft_radix2(b, false);  // B = FFT(b)
    for (size_t i = 0; i < a.size(); ++i) a[i] *= b[i];  // A .* B
    fft_radix2(a, true);   // c = IFFT(A .* B)  (already scaled by 1/M inside)
    std::vector<cplx> out(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double phase = theta * static_cast<double>(i) * i;
        const cplx w(std::cos(phase), -std::sin(phase));
        out[static_cast<size_t>(i)] = a[static_cast<size_t>(i)] * w;
    }
    return out;
}

std::vector<cplx> ifft(const std::vector<cplx>& X, int n) {
    if (n < 0) n = static_cast<int>(X.size());
    // ifft(X) = conj(fft(conj(X))) / n  -- works for any length (radix-2 or
    // Bluestein) since fft() is exact.
    std::vector<cplx> conjX(static_cast<size_t>(n), cplx(0.0, 0.0));
    const size_t m = std::min(X.size(), static_cast<size_t>(n));
    for (size_t i = 0; i < m; ++i) conjX[i] = std::conj(X[i]);
    std::vector<cplx> Y = fft(conjX, n);
    const double inv = 1.0 / n;
    for (auto& v : Y) v = std::conj(v) * inv;
    return Y;
}

std::vector<cplx> fft_real(const std::vector<double>& x, int n) {
    if (n < 0) n = static_cast<int>(x.size());
    std::vector<cplx> xc(static_cast<size_t>(n), cplx(0.0, 0.0));
    const size_t m = std::min(x.size(), static_cast<size_t>(n));
    for (size_t i = 0; i < m; ++i) xc[i] = cplx(x[i], 0.0);
    return fft(xc, n);
}

std::vector<cplx> hilbert(const std::vector<double>& x) {
    const int n = static_cast<int>(x.size());
    if (n == 0) return {};
    std::vector<cplx> X = fft_real(x, n);
    // MATLAB hilbert: h = [1; 2*ones(ceil(N/2)-1,1); 1; zeros(floor(N/2)-1,1)]
    // (1-based) for even N; h = [1; 2*ones((N-1)/2,1); zeros((N-1)/2,1)] for odd N.
    // 0-based:
    //   even: X[0]*=1, X[1..N/2-1]*=2, X[N/2]*=1, X[N/2+1..]*=0
    //   odd : X[0]*=1, X[1..N/2]*=2,   X[N/2+1..]*=0
    const int half = n / 2;
    if (n % 2 == 0) {
        for (int k = 1; k < half; ++k) X[static_cast<size_t>(k)] *= 2.0;
        for (int k = half + 1; k < n; ++k) X[static_cast<size_t>(k)] = cplx(0.0, 0.0);
    } else {
        for (int k = 1; k <= half; ++k) X[static_cast<size_t>(k)] *= 2.0;
        for (int k = half + 1; k < n; ++k) X[static_cast<size_t>(k)] = cplx(0.0, 0.0);
    }
    return ifft(X, n);
}

}  // namespace sonar::dsp

