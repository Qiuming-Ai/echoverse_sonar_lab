#pragma once
// .esl3d binary reader (port of ESL3D/esl3d.m).
//
// File format:
//   header (56 bytes, little-endian):
//     magic u32 = 0x5033534E ("P3SN"), version u16 = 1, header_bytes u16 = 56,
//     seq u64, ts_us u64, width u32, height u32, point_count u32,
//     metadata_bytes u32, range_bytes u32, intensity_bytes u32,
//     payload_bytes u32, reserved u32
//   payload:
//     metadata (JSON, UTF-8), range (float32 width*height), intensity (float32)
//
// range/intensity are reshaped (width,height) then transposed in MATLAB, i.e.
// frame image element (i,j) = raw[i*width + j]  (0-based).
#include <cstdint>
#include <memory>
#include <string>

#include "types.h"

namespace sonar::io {

struct Esl3dFrame {
    uint64_t seq = 0;
    uint64_t ts_us = 0;
    int width = 0;
    int height = 0;
    MatD range;       // [height x width] (after MATLAB reshape+transpose)
    MatD intensity;   // [height x width]
    std::string metadata_json;  // raw JSON metadata string
};

class Esl3dReader {
public:
    // Throws std::runtime_error on failure.
    void load(const std::string& path);
    int frameCount() const { return static_cast<int>(frames_.size()); }

    const Esl3dFrame& frame(int idx) const;  // 0-based; throws if out of range

    // Convert a frame's range/intensity to a scatterer point cloud
    // (port of esl3d.rangeToPointCloud).
    void getPointCloud(int idx, FramePointCloud& out) const;

    // Total scatterer count across all frames (for logging).
    size_t totalPointCount() const;

private:
    std::vector<Esl3dFrame> frames_;
};

}  // namespace sonar::io

