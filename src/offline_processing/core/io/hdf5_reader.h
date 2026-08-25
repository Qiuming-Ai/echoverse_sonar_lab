#pragma once
// HDF5 reader replicating ReadBaselineHDF5.m:
//  - reads /raw_data/.attributes (scalar attrs + datasets + complex real/imag groups)
//  - reads /raw_data/ping_N datasets/groups (complex aware)
#ifdef SONAR_HAVE_HDF5
#include <map>
#include <string>
#include <vector>

#include "types.h"

namespace sonar::io {

struct Hdf5Data {
    SonarAttributes attributes;
    std::vector<MatC> pings;                        // [samples x channels] each
    std::map<std::string, double> attr_scalars;     // numeric attributes (unknown fields)
    std::map<std::string, std::string> attr_strings;
    std::map<std::string, MatC> attr_datasets;      // non-ping datasets under .attributes
};

// Throws std::runtime_error on failure.
Hdf5Data read_baseline_hdf5(const std::string& path);

// Generic read of a single object (dataset, or group with real/imag) from an
// arbitrary HDF5 file, e.g. golden baselines exported by MATLAB.
// Returns false if the object does not exist or cannot be read.
bool read_hdf5_dataset(const std::string& file, const std::string& path, MatC& out);

// Map generic attribute datasets/scalars into SonarAttributes fields by name.
void map_attributes_into(SonarAttributes& a, const std::map<std::string, double>& scalars,
                         const std::map<std::string, std::string>& strings,
                         const std::map<std::string, MatC>& datasets);

}  // namespace sonar::io
#endif  // SONAR_HAVE_HDF5

