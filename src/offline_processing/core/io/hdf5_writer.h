#pragma once
// HDF5 writer replicating SonarDataMaker.m layout exactly (REQUIRED for
// interop with ReadBaselineHDF5.m):
//   /raw_data/ping_N            (complex -> /real + /imag subdatasets, 'complex' attr)
//   /raw_data/.attributes/...   (scalar attrs as attributes, vectors/matrices as
//                                single-precision datasets, complex split real/imag)
//   /raw_data/.attributes/ping_num
//
// Dimension convention (MATLAB interop): MATLAB h5create(file, ds, [R C])
// stores HDF5 dims (C, R). We therefore create rank-2 dataspaces with flipped
// dims and write the raw column-major buffer unchanged.
#include <string>
#include <vector>

#include "types.h"

namespace sonar::io {

class Hdf5Writer {
public:
    Hdf5Writer() = default;
    ~Hdf5Writer();
    Hdf5Writer(const Hdf5Writer&) = delete;
    Hdf5Writer& operator=(const Hdf5Writer&) = delete;

    // Create/truncate file and write sonar attributes (port of start()).
    void start(const std::string& filePath, const SonarAttributes& attrs);

    // Append one waveform frame to /raw_data/ping_N (port of write()).
    // data is [R x C] column-major (MATLAB size).
    void write(const MatC& data);

    // Finalize and write ping_num (port of close()).
    void close();

    int frameCount() const { return frameCount_; }

private:
    std::string filePath_;
    std::string rawGroup_ = "/raw_data";
    std::string attrsGroup_ = "/raw_data/.attributes";
    bool started_ = false;
    bool closed_ = false;
    int frameCount_ = 0;
    long long file_ = -1;  // hid_t (kept opaque to avoid hdf5.h in the header)
};

}  // namespace sonar::io

