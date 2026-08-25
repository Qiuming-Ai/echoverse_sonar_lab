#pragma once
// Plane-wave DAS beamformer (port of Beamformer/das_plane_wave_id.m).
// Integer delay + optional carrier phase compensation + optional CF
// (full family: cf | pcf | gcf | mcf | slsc; resample path supported).
#include <vector>

#include "types.h"

namespace sonar::image {

struct DasCfg {
    double t0 = 0.0;
    double fs = 0.0;      // output sample rate
    double c = 1500.0;
    std::vector<double> angles;  // rad (1 x Na)
    MatD rx_xyz;                 // [M x 3]
    int fd_sign = 1;
    std::vector<double> w;       // channel weights; empty => hamming(M)
    double f0 = 0.0;             // carrier phase compensation (0 = off)
    double fs_calc = 0.0;        // 0 => use fs (no resample)

    // CF family (disabled by default)
    bool cf_enable = false;
    std::string cf_mode = "cf";  // cf | pcf | gcf | mcf | slsc
    double cf_gamma = 1.0;
    double cf_eps = 1e-12;
    double gcf_p = 2.0;
    int slsc_L = 0;
};

// rx : [Nsamp x M] complex channel data (column-major).
// beam : [Nsamp_out x Na]; ranges : [Nsamp_out x 1] (m).
void das_plane_wave_id(const DasCfg& cfg, const MatC& rx, MatC& beam, MatD& ranges);

}  // namespace sonar::image

