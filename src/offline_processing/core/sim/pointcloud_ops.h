#pragma once
// Point cloud processing (port of ESL3D/PointCloudDecimate.m / Shuffle.m).
#include "types.h"

namespace sonar::sim {

// Randomly keep ~keepFraction of points (port of PointCloudDecimate.m).
// rngSeed >= 0 fixes the RNG (MATLAB 'twister' equivalent); -1 = random seed.
void point_cloud_decimate(const MatD& in_pos, const MatD& in_amp, double keepFraction,
                          int rngSeed, MatD& out_pos, MatD& out_amp);

}  // namespace sonar::sim

