#pragma once
// 1-D convolution compatible with MATLAB conv(u, v, 'full'|'same').
//
// NOTE (diff note): MATLAB 'same' returns the central part of the full
// convolution with the same length as u. The 0-based offset into the full
// result used here is (Lv-1)/2 (integer division), which is the standard FIR
// alignment. This is marked as a candidate for numeric validation against
// MATLAB (see docs/matlab_diff_notes.md).
#include <cassert>
#include <complex>
#include <string>
#include <vector>

namespace sonar::dsp {

namespace detail {
// full convolution, output length = Lu + Lv - 1
template <typename T>
inline std::vector<T> conv_full(const std::vector<T>& u, const std::vector<T>& v) {
    const int Lu = static_cast<int>(u.size());
    const int Lv = static_cast<int>(v.size());
    if (Lu == 0 || Lv == 0) return {};
    std::vector<T> y(static_cast<size_t>(Lu + Lv - 1), T{});
    for (int i = 0; i < Lu; ++i) {
        if (u[static_cast<size_t>(i)] == T{}) continue;
        for (int j = 0; j < Lv; ++j) {
            y[static_cast<size_t>(i + j)] += u[static_cast<size_t>(i)] * v[static_cast<size_t>(j)];
        }
    }
    return y;
}
}  // namespace detail

template <typename T>
inline std::vector<T> conv(const std::vector<T>& u, const std::vector<T>& v, const char* shape) {
    std::vector<T> y = detail::conv_full(u, v);
    std::string s = shape ? shape : "full";
    if (s == "full") return y;
    if (s == "same") {
        const int Lu = static_cast<int>(u.size());
        const int Lv = static_cast<int>(v.size());
        if (Lu == 0) return {};
        // MATLAB conv(u,v,'same') 0-based offset into the full convolution is
        // floor(Lv/2) (verified empirically, incl. even-length filters).
        const int offset = Lv / 2;
        assert(offset >= 0);
        std::vector<T> out(static_cast<size_t>(Lu), T{});
        for (int i = 0; i < Lu; ++i) {
            const int src = offset + i;
            out[static_cast<size_t>(i)] =
                (src >= 0 && src < static_cast<int>(y.size())) ? y[static_cast<size_t>(src)] : T{};
        }
        return out;
    }
    assert(false && "conv shape must be 'full' or 'same'");
    return y;
}

}  // namespace sonar::dsp

