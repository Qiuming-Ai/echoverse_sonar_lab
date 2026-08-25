#include "dsp/resampler.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>

#include "dsp/fir_filter.h"

namespace sonar::dsp {

namespace {

// MATLAB's resample reduces p/q to lowest terms during parsing:
//   [obj.p, obj.q] = rat(p2/q2, 1e-12);  %--- reduce to lowest terms
// For integer P,Q this is exactly the gcd reduction (e.g. CDM's
// resample(x,16e6,1e5) -> 160/1). We replicate it so the anti-alias
// filter is designed with the REDUCED max(P,Q) like MATLAB, otherwise
// a 320M-tap filter would be generated.
void reduce_ratio(int& P, int& Q) {
    if (P <= 0 || Q <= 0) return;
    int a = P, b = Q;
    while (b != 0) {
        const int t = a % b;
        a = b;
        b = t;
    }
    if (a > 1) {
        P /= a;
        Q /= a;
    }
}

// Design the resample anti-aliasing filter, replicating MATLAB's
// uniformParserNBetaAndDesignFilter (N=10, BTA=5):
//   fc = 1/(2*max(P,Q)); L = 2*10*max(P,Q)+1;
//   h = firls(L-1,[0 2*fc 2*fc 1],[1 1 0 0]) .* kaiser(L,5)';
//   h = P * h / sum(h);
// firls for this piecewise-constant spec equals the ideal lowpass (sinc),
// which is what we implement here.
std::vector<double> design_resample_filter(int P, int Q) {
    const int m = std::max(P, Q);
    const double fc = 1.0 / (2.0 * m);
    const int L = 2 * 10 * m + 1;  // N = 10
    std::vector<double> win = kaiser_window(L, 5.0);  // BTA = 5
    const double mid = static_cast<double>(L - 1) / 2.0;
    std::vector<double> h(static_cast<size_t>(L));
    double sum_h = 0.0;
    for (int n = 0; n < L; ++n) {
        const double arg = 2.0 * fc * (static_cast<double>(n) - mid);
        const double ideal =
            (std::fabs(arg) < 1e-12) ? 2.0 * fc : std::sin(M_PI * arg) / (M_PI * arg) * (2.0 * fc);
        h[static_cast<size_t>(n)] = ideal * win[static_cast<size_t>(n)];
        sum_h += h[static_cast<size_t>(n)];
    }
    // normalize so the passband gain equals P (interpolation compensation)
    if (sum_h != 0.0) {
        const double scale = static_cast<double>(P) / sum_h;
        for (auto& v : h) v *= scale;
    }
    return h;
}

// MATLAB findDelayAndZeroPadFilter + computeZeroPadLength.
struct PaddedFilter {
    std::vector<double> h;  // zero-padded filter (filterWithPadding)
    int delay = 0;          // filterDelay (samples removed from the start)
};

PaddedFilter pad_filter(const std::vector<double>& filt, int P, int Q, int Lx) {
    const int L = static_cast<int>(filt.size());
    int Lhalf = (L - 1) / 2;
    const int nZeroBegin = static_cast<int>(std::floor(static_cast<double>(Q - (Lhalf % Q))));
    Lhalf += nZeroBegin;
    const int filterDelay = static_cast<int>(std::floor(std::ceil(static_cast<double>(Lhalf)) / Q));

    int lenH = L + nZeroBegin;
    int nZeroEnd = 0;
    const double target = std::ceil(static_cast<double>(Lx) * P / Q);
    while (std::ceil((static_cast<double>((Lx - 1) * P) + lenH + nZeroEnd) / Q) -
               filterDelay <
           target) {
        ++nZeroEnd;
    }

    PaddedFilter out;
    out.h.assign(static_cast<size_t>(nZeroBegin + L + nZeroEnd), 0.0);
    for (int i = 0; i < L; ++i)
        out.h[static_cast<size_t>(nZeroBegin + i)] = filt[static_cast<size_t>(i)];
    out.delay = filterDelay;
    return out;
}

// upfirdn(x, h, P, Q): upsample by P, filter (full), downsample by Q.
template <typename T>
std::vector<T> upfirdn(const std::vector<T>& x, const std::vector<double>& h, int P, int Q) {
    const int Lx = static_cast<int>(x.size());
    const int Lh = static_cast<int>(h.size());
    if (Lx == 0) return {};
    const int64_t Lu = static_cast<int64_t>(Lx) * P;
    const int Ly = static_cast<int>((Lu + Lh - 2) / Q + 1);
    std::vector<T> y(static_cast<size_t>(Ly), T{});

    // Polyphase evaluation.  The previous implementation scanned every filter
    // tap and retained only taps for which (m*Q-n) was divisible by P.  Large,
    // nearly-unity Doppler ratios such as 376/375 therefore tested ~7521 taps
    // per output even though only ~20 can contribute.  Solve
    //
    //     n = m*Q - k*P,  k = input sample index
    //
    // directly and visit only those valid taps.  Iterating k downwards keeps n
    // increasing, exactly matching the old accumulation order.
    const auto ceil_div_positive_den = [](int64_t a, int64_t b) {
        return a >= 0 ? (a + b - 1) / b : a / b;
    };
    for (int m = 0; m < Ly; ++m) {
        T acc{};
        const int64_t mq = static_cast<int64_t>(m) * Q;
        const int64_t kMin = std::max<int64_t>(
            0, ceil_div_positive_den(mq - (Lh - 1), P));
        const int64_t kMax = std::min<int64_t>(Lx - 1, mq / P);
        for (int64_t k = kMax; k >= kMin; --k) {
            const int64_t n = mq - k * P;
            acc += static_cast<T>(h[static_cast<size_t>(n)] * x[static_cast<size_t>(k)]);
        }
        y[static_cast<size_t>(m)] = acc;
    }
    return y;
}

template <typename T>
std::vector<T> resample_impl(const std::vector<T>& x, int P, int Q) {
    const int Lx = static_cast<int>(x.size());
    if (Lx == 0) return {};
    // MATLAB reduces p/q by gcd during parsing; p==q (1/1) short-circuits.
    reduce_ratio(P, Q);
    if (P == Q) return x;

    std::vector<double> filt = design_resample_filter(P, Q);
    PaddedFilter pf = pad_filter(filt, P, Q, Lx);
    std::vector<T> yfull = upfirdn(x, pf.h, P, Q);
    const int yLen = static_cast<int>(std::ceil(static_cast<double>(Lx) * P / Q));
    std::vector<T> y(static_cast<size_t>(yLen), T{});
    for (int i = 0; i < yLen; ++i) {
        const int src = pf.delay + i;
        y[static_cast<size_t>(i)] =
            (src >= 0 && src < static_cast<int>(yfull.size())) ? yfull[static_cast<size_t>(src)] : T{};
    }
    return y;
}

}  // namespace

std::vector<double> resample_filter(int P, int Q) {
    // MATLAB bypass: for p==q (reduced to 1/1) the filter is ones(1,1).
    reduce_ratio(P, Q);
    if (P == Q) return {1.0};
    return design_resample_filter(P, Q);
}

std::vector<double> resample(const std::vector<double>& x, int P, int Q) {
    return resample_impl(x, P, Q);
}
std::vector<std::complex<double>> resample(const std::vector<std::complex<double>>& x, int P,
                                           int Q) {
    return resample_impl(x, P, Q);
}

template <typename T>
static void resample_columns_impl(const std::vector<T>& in, int rows, int cols, int P, int Q,
                                  std::vector<T>& out, int& out_rows) {
    const int Lout = static_cast<int>(std::ceil(static_cast<double>(rows) * P / Q));
    out.assign(static_cast<size_t>(Lout) * cols, T{});
    for (int c = 0; c < cols; ++c) {
        std::vector<T> col(static_cast<size_t>(rows));
        for (int r = 0; r < rows; ++r) col[static_cast<size_t>(r)] = in[static_cast<size_t>(r + rows * c)];
        std::vector<T> rcol = resample_impl(col, P, Q);
        for (int r = 0; r < Lout; ++r) out[static_cast<size_t>(r + Lout * c)] = rcol[static_cast<size_t>(r)];
    }
    out_rows = Lout;
}

void resample_columns(const std::vector<double>& in, int rows, int cols, int P, int Q,
                      std::vector<double>& out, int& out_rows) {
    resample_columns_impl(in, rows, cols, P, Q, out, out_rows);
}
void resample_columns(const std::vector<std::complex<double>>& in, int rows, int cols, int P, int Q,
                      std::vector<std::complex<double>>& out, int& out_rows) {
    resample_columns_impl(in, rows, cols, P, Q, out, out_rows);
}

}  // namespace sonar::dsp

