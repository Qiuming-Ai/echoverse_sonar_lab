#pragma once
// PNG output (OpenCV imgcodecs). 8-bit grayscale.
#include <cstdint>
#include <string>
#include <vector>

namespace sonar::image {

// Write a single-channel grayscale PNG. Throws std::runtime_error on failure.
void write_png_gray(const std::string& path, int width, int height,
                    const std::vector<unsigned char>& gray);

}  // namespace sonar::image
