#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace standalone_mvp {

struct PointCloudPolarFrame {
    std::size_t width = 0;
    std::size_t height = 0;
    std::size_t point_count = 0; // width * height
    float invalid_value = -1.0f;
    std::vector<float> range_image_m;      // row-major: [v * width + h]
    std::vector<float> intensity_image;    // row-major: [v * width + h]
    const char* layout = "row-major";
    const char* rounding_policy = "floor";
};

} // namespace standalone_mvp
