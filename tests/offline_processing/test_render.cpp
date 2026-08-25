#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "image/render.h"

using namespace sonar;

int main(int argc, char** argv) {
    const std::string path = (argc >= 2) ? argv[1] : "test_gray.png";
    try {
        image::SectorImage img;
        img.angles = {-0.5, 0.5};
        img.ranges = {1.0, 2.0};
        img.X.resize(2, 2);
        img.Z.resize(2, 2);
        img.IdB.resize(2, 2);
        for (int r = 0; r < 2; ++r) {
            for (int c = 0; c < 2; ++c) {
                img.X(r, c) = img.ranges[static_cast<size_t>(r)] *
                              std::sin(img.angles[static_cast<size_t>(c)]);
                img.Z(r, c) = img.ranges[static_cast<size_t>(r)] *
                              std::cos(img.angles[static_cast<size_t>(c)]);
                img.IdB(r, c) = (r == c) ? 0.0 : -40.0;
            }
        }

        image::RenderOptions opt;
        opt.width = 32;
        opt.height = 24;
        opt.dynRange = 40.0;
        const auto gray = image::render_sector_gray(img, opt);
        if (gray.size() != static_cast<size_t>(opt.width * opt.height)) {
            std::printf("FAILED: grayscale buffer size\n");
            return 1;
        }
        image::render_sector_to_png(img, opt, path);

        std::ifstream f(path, std::ios::binary);
        std::array<unsigned char, 26> header{};
        f.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
        if (f.gcount() != static_cast<std::streamsize>(header.size()) || header[24] != 8 ||
            header[25] != 0) {
            std::printf("FAILED: PNG is not 8-bit grayscale (depth=%u color=%u)\n",
                        static_cast<unsigned>(header[24]), static_cast<unsigned>(header[25]));
            return 1;
        }
        f.close();
        std::filesystem::remove(path);
        std::printf("GRAYSCALE PNG TEST PASSED\n");
        return 0;
    } catch (const std::exception& e) {
        std::filesystem::remove(path);
        std::printf("FAILED: %s\n", e.what());
        return 1;
    }
}

