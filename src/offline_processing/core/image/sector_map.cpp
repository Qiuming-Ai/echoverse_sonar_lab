#include "image/sector_map.h"

#include <algorithm>
#include <cmath>

namespace sonar::image {

SectorImage sector_map(const std::vector<double>& angles_in, const std::vector<double>& ranges_in,
                       const MatC& img, double dynRange) {
    const int Na = static_cast<int>(angles_in.size());
    const int Nr0 = static_cast<int>(ranges_in.size());
    if (Na == 0 || Nr0 == 0) return {};

    std::vector<double> angles = angles_in;
    std::vector<double> ranges = ranges_in;

    // remove negative ranges (MATLAB: validRangeMask = ranges >= 0)
    std::vector<int> keep;
    for (int i = 0; i < Nr0; ++i)
        if (ranges[static_cast<size_t>(i)] >= 0.0) keep.push_back(i);
    const int Nr = static_cast<int>(keep.size());
    if (Nr == 0) return {};

    std::vector<double> rr(static_cast<size_t>(Nr));
    for (int i = 0; i < Nr; ++i) rr[static_cast<size_t>(i)] = ranges[static_cast<size_t>(keep[static_cast<size_t>(i)])];
    ranges = rr;

    // I = abs(img); I = I/max(I); IdB = 20*log10(I+eps); IdB = max(IdB, -dynRange)
    MatD I(Nr, Na);
    double imax = 0.0;
    for (int i = 0; i < Nr; ++i)
        for (int j = 0; j < Na; ++j) {
            const double v = std::abs(img(keep[static_cast<size_t>(i)], j));
            I(i, j) = v;
            imax = std::max(imax, v);
        }
    constexpr double eps = 2.220446049250313e-16;
    const double denom = imax + eps;
    MatD IdB(Nr, Na);
    for (int i = 0; i < Nr; ++i)
        for (int j = 0; j < Na; ++j) {
            double v = 20.0 * std::log10(I(i, j) / denom + eps);
            IdB(i, j) = std::max(v, -dynRange);
        }

    // [TH, RR] = meshgrid(angles, ranges); X = RR.*sin(TH); Z = RR.*cos(TH)
    MatD X(Nr, Na), Z(Nr, Na);
    for (int i = 0; i < Nr; ++i)
        for (int j = 0; j < Na; ++j) {
            X(i, j) = rr[static_cast<size_t>(i)] * std::sin(angles[static_cast<size_t>(j)]);
            Z(i, j) = rr[static_cast<size_t>(i)] * std::cos(angles[static_cast<size_t>(j)]);
        }

    SectorImage out;
    out.X = X;
    out.Z = Z;
    out.IdB = IdB;
    out.angles = angles;
    out.ranges = ranges;
    out.dynRange = dynRange;
    return out;
}

}  // namespace sonar::image

