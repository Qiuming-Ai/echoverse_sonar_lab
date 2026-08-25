#pragma once
// Data-driven rasterizer for the sector image. The clipped dB range is mapped
// linearly to a true single-channel grayscale PNG.
#include <cstdint>
#include <string>
#include <vector>

#include "image/sector_map.h"

namespace sonar::image {

struct RenderOptions {
    int width = 800;        // output pixel width
    int height = 600;       // output pixel height
    double dynRange = 40.0; // dB dynamic range (same as sector_map)
};

// Rasterize SectorImage -> grayscale buffer (width*height bytes).
std::vector<unsigned char> render_sector_gray(const SectorImage& img,
                                              const RenderOptions& opt);

// Convenience: render + write PNG with the given output path.
void render_sector_to_png(const SectorImage& img, const RenderOptions& opt,
                          const std::string& pngPath);

}  // namespace sonar::image

