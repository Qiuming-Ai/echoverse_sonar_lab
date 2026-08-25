#include "sim/pointcloud_ops.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>

namespace sonar::sim {

void point_cloud_decimate(const MatD& in_pos, const MatD& in_amp, double keepFraction,
                          int rngSeed, MatD& out_pos, MatD& out_amp) {
    const int n = in_pos.rows();
    if (in_amp.rows() != n)
        throw std::runtime_error("PointCloudDecimate: point_amplitudes row mismatch");
    if (keepFraction <= 0.0 || keepFraction > 1.0)
        throw std::runtime_error("PointCloudDecimate: keepFraction must be in (0,1]");

    if (n == 0 || keepFraction >= 1.0) {
        out_pos = in_pos;
        out_amp = in_amp;
        return;
    }

    int nKeep = static_cast<int>(std::round(n * keepFraction));
    nKeep = std::max(0, std::min(n, nKeep));

    std::vector<int> idx(static_cast<size_t>(n));
    std::iota(idx.begin(), idx.end(), 0);
    std::mt19937 gen(rngSeed >= 0 ? static_cast<unsigned>(rngSeed) : std::random_device{}());
    std::shuffle(idx.begin(), idx.end(), gen);

    out_pos.resize(nKeep, in_pos.cols());
    out_amp.resize(nKeep, in_amp.cols());
    for (int i = 0; i < nKeep; ++i) {
        const int src = idx[static_cast<size_t>(i)];
        for (int c = 0; c < in_pos.cols(); ++c) out_pos(i, c) = in_pos(src, c);
        for (int c = 0; c < in_amp.cols(); ++c) out_amp(i, c) = in_amp(src, c);
    }
}

}  // namespace sonar::sim

