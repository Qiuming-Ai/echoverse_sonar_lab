#include "io/hdf5_writer.h"

#ifdef SONAR_HAVE_HDF5

#include <hdf5.h>

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "util/log.h"

namespace sonar::io {

namespace {

[[noreturn]] void h5fail(const std::string& what) { throw std::runtime_error("HDF5: " + what); }

void write_attr_scalar_double(hid_t loc, const char* name, double v) {
    hid_t space = H5Screate(H5S_SCALAR);
    hid_t attr = H5Acreate2(loc, name, H5T_IEEE_F64LE, space, H5P_DEFAULT, H5P_DEFAULT);
    if (attr < 0) h5fail(std::string("H5Acreate2 ") + name);
    H5Awrite(attr, H5T_IEEE_F64LE, &v);
    H5Aclose(attr);
    H5Sclose(space);
}

void write_attr_string(hid_t loc, const char* name, const std::string& v) {
    hid_t type = H5Tcopy(H5T_C_S1);
    H5Tset_size(type, v.size());
    hid_t space = H5Screate(H5S_SCALAR);
    hid_t attr = H5Acreate2(loc, name, type, space, H5P_DEFAULT, H5P_DEFAULT);
    if (attr < 0) h5fail(std::string("H5Acreate2 ") + name);
    H5Awrite(attr, type, v.data());
    H5Aclose(attr);
    H5Tclose(type);
    H5Sclose(space);
}

// Real dataset of MATLAB size [R x C]: HDF5 dims (C, R) + 'complex'=0 attr.
void write_dataset_real(hid_t loc, const char* path, const std::vector<float>& data, int R, int C) {
    hsize_t dims[2] = {static_cast<hsize_t>(C), static_cast<hsize_t>(R)};
    hid_t space = H5Screate_simple(2, dims, nullptr);
    hid_t dset = H5Dcreate2(loc, path, H5T_IEEE_F32LE, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    if (dset < 0) h5fail(std::string("H5Dcreate2 ") + path);
    H5Dwrite(dset, H5T_IEEE_F32LE, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.data());
    // MATLAB SonarDataMaker writes every real scalar attribute as double.
    // Keep the marker type identical so MATLAB code can consume a C++ file
    // without integer/double propagation surprises.
    write_attr_scalar_double(dset, "complex", 0.0);
    H5Dclose(dset);
    H5Sclose(space);
}

// Complex data as group path with /real + /imag float32 datasets + 'complex'=1 attr.
void write_dataset_complex(hid_t loc, const char* path, const std::vector<cplx>& data, int R,
                           int C) {
    hsize_t dims[2] = {static_cast<hsize_t>(C), static_cast<hsize_t>(R)};
    std::vector<float> re(static_cast<size_t>(R) * C), im(static_cast<size_t>(R) * C);
    for (size_t i = 0; i < data.size(); ++i) {
        re[i] = static_cast<float>(data[i].real());
        im[i] = static_cast<float>(data[i].imag());
    }
    hid_t space = H5Screate_simple(2, dims, nullptr);
    hid_t grp = H5Gcreate2(loc, path, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    if (grp < 0) h5fail(std::string("H5Gcreate2 ") + path);
    hid_t dre = H5Dcreate2(grp, "real", H5T_IEEE_F32LE, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    hid_t dim = H5Dcreate2(grp, "imag", H5T_IEEE_F32LE, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    if (dre < 0 || dim < 0) h5fail(std::string("H5Dcreate2 real/imag in ") + path);
    H5Dwrite(dre, H5T_IEEE_F32LE, H5S_ALL, H5S_ALL, H5P_DEFAULT, re.data());
    H5Dwrite(dim, H5T_IEEE_F32LE, H5S_ALL, H5S_ALL, H5P_DEFAULT, im.data());
    H5Dclose(dre);
    H5Dclose(dim);
    write_attr_scalar_double(grp, "complex", 1.0);
    H5Gclose(grp);
    H5Sclose(space);
}

}  // namespace

Hdf5Writer::~Hdf5Writer() {
    if (!closed_ && started_ && file_ >= 0) {
        try {
            close();
        } catch (...) {
        }
    }
}

void Hdf5Writer::start(const std::string& filePath, const SonarAttributes& attrs) {
    if (started_) throw std::runtime_error("Hdf5Writer already started");
    filePath_ = filePath;

    file_ = H5Fcreate(filePath_.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    if (file_ < 0) h5fail("H5Fcreate " + filePath_);
    const hid_t file = static_cast<hid_t>(file_);

    // groups /raw_data and /raw_data/.attributes
    hid_t g = H5Gcreate2(file, "/raw_data", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    if (g < 0) h5fail("create /raw_data");
    H5Gclose(g);
    g = H5Gcreate2(file, "/raw_data/.attributes", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    if (g < 0) h5fail("create /raw_data/.attributes");
    H5Gclose(g);

    hid_t ag = H5Gopen2(file, attrsGroup_.c_str(), H5P_DEFAULT);
    if (ag < 0) h5fail("open /raw_data/.attributes");

    auto write_vec = [&](const char* key, const std::vector<double>& v) {
        std::vector<float> f(v.begin(), v.end());
        write_dataset_real(ag, key, f, 1, static_cast<int>(v.size()));
    };
    auto write_mat = [&](const char* key, const MatD& m) {
        std::vector<float> f(m.size());
        for (size_t i = 0; i < m.size(); ++i) f[i] = static_cast<float>(m.data()[i]);
        write_dataset_real(ag, key, f, m.rows(), m.cols());
    };

    write_attr_string(ag, "array_type", attrs.array_type);
    write_attr_string(ag, "signal_type", attrs.signal_type);
    write_attr_string(ag, "signal_win", attrs.signal_win);
    write_vec("bandwidth", attrs.bandwidth);
    write_attr_scalar_double(ag, "sampling_frequency", attrs.sampling_frequency);
    write_vec("center_frequency", attrs.center_frequency);
    write_attr_scalar_double(ag, "decimate_factor", static_cast<double>(attrs.decimate_factor));
    write_attr_scalar_double(ag, "sector_num", static_cast<double>(attrs.sector_num));
    {
        const int R = attrs.match_filter_data.rows();
        const int C = attrs.match_filter_data.cols();
        std::vector<cplx> mf(static_cast<size_t>(R) * C);
        for (size_t i = 0; i < mf.size(); ++i) mf[i] = attrs.match_filter_data.data()[i];
        write_dataset_complex(ag, "match_filter_data", mf, R, C);
    }
    write_attr_scalar_double(ag, "receive_array_num", static_cast<double>(attrs.receive_array_num));
    write_mat("receive_array_position", attrs.receive_array_position);
    write_mat("receive_array_win", attrs.receive_array_win);
    write_attr_scalar_double(ag, "pulse_duration", attrs.pulse_duration);
    write_attr_scalar_double(ag, "sound_velocity", attrs.sound_velocity);
    write_attr_scalar_double(ag, "velocity", attrs.velocity);
    write_attr_scalar_double(ag, "snr_level", attrs.snr_level);
    write_attr_string(ag, "timestamp", attrs.timestamp);
    write_vec("scan_angle", attrs.scan_angle);
    write_vec("sector_div", attrs.sector_div);
    write_vec("sample_delay", attrs.sample_delay);

    H5Gclose(ag);
    started_ = true;
    frameCount_ = 0;
    SONAR_LOG_INFO("HDF5 file created: %s", filePath_.c_str());
}

void Hdf5Writer::write(const MatC& data) {
    if (!started_ || closed_) throw std::runtime_error("Hdf5Writer not writable");
    if (data.empty()) throw std::runtime_error("Hdf5Writer: empty waveform");

    ++frameCount_;
    std::string name = rawGroup_ + "/ping_" + std::to_string(frameCount_);
    std::vector<cplx> buf(data.size());
    for (size_t i = 0; i < buf.size(); ++i) buf[i] = data.data()[i];

    hid_t loc = H5Gopen2(static_cast<hid_t>(file_), rawGroup_.c_str(), H5P_DEFAULT);
    // Echo data is complex baseband; write real/imag split (MATLAB convention).
    write_dataset_complex(loc, name.c_str(), buf, data.rows(), data.cols());
    H5Gclose(loc);
}

void Hdf5Writer::close() {
    if (!started_ || closed_) throw std::runtime_error("Hdf5Writer not open");
    hid_t ag = H5Gopen2(static_cast<hid_t>(file_), attrsGroup_.c_str(), H5P_DEFAULT);
    write_attr_scalar_double(ag, "ping_num", static_cast<double>(frameCount_));
    H5Gclose(ag);
    H5Fclose(static_cast<hid_t>(file_));
    file_ = -1;
    closed_ = true;
    SONAR_LOG_INFO("HDF5 closed, %d ping(s) written", frameCount_);
}

}  // namespace sonar::io

#else  // !SONAR_HAVE_HDF5

#include <stdexcept>
namespace sonar::io {
Hdf5Writer::~Hdf5Writer() = default;
void Hdf5Writer::start(const std::string&, const SonarAttributes&) {
    throw std::runtime_error("HDF5 support not compiled in (SONAR_HAVE_HDF5 undefined)");
}
void Hdf5Writer::write(const MatC&) {
    throw std::runtime_error("HDF5 support not compiled in (SONAR_HAVE_HDF5 undefined)");
}
void Hdf5Writer::close() { throw std::runtime_error("HDF5 support not compiled in"); }
}  // namespace sonar::io

#endif  // SONAR_HAVE_HDF5

