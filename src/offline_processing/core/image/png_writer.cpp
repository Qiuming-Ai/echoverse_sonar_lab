#include "image/png_writer.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <stdexcept>

namespace sonar::image {

void write_png_gray(const std::string& path, const int width, const int height,
                    const std::vector<unsigned char>& gray) {
    if (width <= 0 || height <= 0 ||
        gray.size() != static_cast<size_t>(width) * static_cast<size_t>(height)) {
        throw std::runtime_error("write_png_gray: invalid dimensions or buffer size");
    }

    const cv::Mat image(height, width, CV_8UC1,
                        const_cast<unsigned char*>(gray.data()),
                        static_cast<size_t>(width));
    if (!cv::imwrite(path, image)) {
        throw std::runtime_error("write_png_gray: failed to write " + path);
    }
}

}  // namespace sonar::image
