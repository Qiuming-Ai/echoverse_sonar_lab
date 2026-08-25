#pragma once

#include <vector>

#include "types.h"

namespace sonar::image {

// Beamformed data for one HDF5 ping. For CDM/FDM, columns from all sectors
// are concatenated in sector order and angles contains the matching axis.
struct File2ImageResult {
    MatC beam;
    MatD ranges;
    std::vector<double> angles;  // rad
};

// Resolve one angle vector per matched-filter column. This also supports
// MATLAB files produced by DataMakerInit.m that contain only the first
// sector in scan_angle but contain complete sector_div boundaries.
std::vector<std::vector<double>> resolve_sector_angles_deg(const SonarAttributes& attrs,
                                                           int sectorCount);

// Match-filter, TVG-compensate and beamform a ping. A multi-column matched
// filter is processed one column/sector at a time, following ImagePlot.m.
File2ImageResult process_ping_for_image(const SonarAttributes& attrs,
                                        const MatC& ping);

}  // namespace sonar::image

