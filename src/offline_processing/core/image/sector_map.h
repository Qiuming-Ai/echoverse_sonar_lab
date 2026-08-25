#pragma once
// Sector map (port of Visualize/sector_plot.m, data part only).
#include "types.h"

namespace sonar::image {

struct SectorImage {
    MatD X;    // [Nr x Na] lateral
    MatD Z;    // [Nr x Na] range/depth
    MatD IdB;  // [Nr x Na] dB (clipped to [-dynRange, 0])
    std::vector<double> angles;  // rad (1 x Na)
    std::vector<double> ranges;  // (Nr x 1)
    double dynRange = 40.0;
};

// angles : Na angles (rad); ranges : Nr ranges (m); img : [Nr x Na] (complex or real).
SectorImage sector_map(const std::vector<double>& angles, const std::vector<double>& ranges,
                       const MatC& img, double dynRange);

}  // namespace sonar::image

