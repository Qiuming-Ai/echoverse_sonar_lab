#include "image/render.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "image/png_writer.h"
#include "util/log.h"

namespace sonar::image {

std::vector<unsigned char> render_sector_gray(const SectorImage& img,
                                              const RenderOptions& opt) {
    const int W = opt.width;
    const int H = opt.height;
    const int Nr = img.X.rows();
    const int Na = img.X.cols();
    if (Nr == 0 || Na == 0)
        return std::vector<unsigned char>(static_cast<size_t>(W) * H, 0);

    // The sector grid is structured: X = r*sin(theta), Z = r*cos(theta).
    // Precompute the (sorted) range and angle axes once, then locate each
    // output pixel by direct inverse mapping + binary search instead of a
    // brute-force nearest-neighbour scan.
    std::vector<double> ranges = img.ranges;  // Nr (ascending)
    std::vector<double> angles = img.angles;  // Na (ascending)

    // bounds of the sector scene
    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double zmin = std::numeric_limits<double>::infinity();
    double zmax = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < Nr; ++i)
        for (int j = 0; j < Na; ++j) {
            xmin = std::min(xmin, img.X(i, j));
            xmax = std::max(xmax, img.X(i, j));
            zmin = std::min(zmin, img.Z(i, j));
            zmax = std::max(zmax, img.Z(i, j));
        }
    if (!(xmax > xmin) || !(zmax > zmin))
        return std::vector<unsigned char>(static_cast<size_t>(W) * H, 0);

    // nearest index in a sorted array
    auto nearest_idx = [](const std::vector<double>& v, double x) -> int {
        int lo = 0, hi = static_cast<int>(v.size()) - 1;
        if (x <= v[0]) return 0;
        if (x >= v[hi]) return hi;
        while (hi - lo > 1) {
            const int mid = (lo + hi) / 2;
            if (v[static_cast<size_t>(mid)] <= x) lo = mid;
            else hi = mid;
        }
        return (x - v[static_cast<size_t>(lo)] < v[static_cast<size_t>(hi)] - x) ? lo : hi;
    };

    std::vector<unsigned char> out(static_cast<size_t>(W) * H, 0);
    const double dr = opt.dynRange;
#pragma omp parallel for schedule(dynamic)
    for (int py = 0; py < H; ++py) {
        const double z = zmax - static_cast<double>(py) / (H - 1) * (zmax - zmin);  // z down
        for (int px = 0; px < W; ++px) {
            const double x = xmin + static_cast<double>(px) / (W - 1) * (xmax - xmin);
            const double r = std::sqrt(x * x + z * z);
            double th = std::atan2(x, z);  // matches X = r sin(th), Z = r cos(th)
            if (th < angles.front()) th = angles.front();
            if (th > angles.back()) th = angles.back();
            const int ri = nearest_idx(ranges, r);
            const int aj = nearest_idx(angles, th);
            const double v = img.IdB(ri, aj);  // in [-dr, 0]
            const double t = (v + dr) / dr;    // 0..1
            const double clipped = std::max(0.0, std::min(1.0, t));
            out[static_cast<size_t>(py) * W + px] =
                static_cast<unsigned char>(std::lround(clipped * 255.0));
        }
    }
    return out;
}

void render_sector_to_png(const SectorImage& img, const RenderOptions& opt,
                          const std::string& pngPath) {
    std::vector<unsigned char> gray = render_sector_gray(img, opt);
    write_png_gray(pngPath, opt.width, opt.height, gray);
    SONAR_LOG_INFO("PNG written: %s (%dx%d)", pngPath.c_str(), opt.width, opt.height);
}

}  // namespace sonar::image

