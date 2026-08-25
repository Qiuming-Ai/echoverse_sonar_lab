#pragma once
// ---------------------------------------------------------------------------
// sonar_core: cross-stage core data structures.
// All matrices are COLUMN-MAJOR (like MATLAB) to avoid transpose bugs when
// porting from the MATLAB reference implementation.
// ---------------------------------------------------------------------------

#include <array>
#include <cassert>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace sonar {

using cplx = std::complex<double>;

// ---------------------------------------------------------------------------
// Mat<T> : minimal column-major 2D matrix (0-based indexing), mirroring
// MATLAB array layout: element (i,j) at data_[i + rows_*j].
// ---------------------------------------------------------------------------
template <typename T>
class Mat {
public:
    Mat() = default;
    Mat(int rows, int cols) : rows_(rows), cols_(cols), data_(static_cast<size_t>(rows) * cols, T{}) {}
    Mat(int rows, int cols, const T& fill) : rows_(rows), cols_(cols), data_(static_cast<size_t>(rows) * cols, fill) {}
    Mat(int rows, int cols, std::vector<T>&& data) : rows_(rows), cols_(cols), data_(std::move(data)) {
        assert(static_cast<size_t>(rows) * cols == data_.size());
    }

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    size_t size() const { return data_.size(); }
    bool empty() const { return data_.empty(); }

    T& operator()(int i, int j) {
        assert(i >= 0 && i < rows_ && j >= 0 && j < cols_);
        return data_[static_cast<size_t>(i) + static_cast<size_t>(rows_) * j];
    }
    const T& operator()(int i, int j) const {
        assert(i >= 0 && i < rows_ && j >= 0 && j < cols_);
        return data_[static_cast<size_t>(i) + static_cast<size_t>(rows_) * j];
    }

    T* data() { return data_.data(); }
    const T* data() const { return data_.data(); }

    void resize(int rows, int cols) {
        rows_ = rows;
        cols_ = cols;
        data_.assign(static_cast<size_t>(rows) * cols, T{});
    }
    void resize(int rows, int cols, const T& fill) {
        rows_ = rows;
        cols_ = cols;
        data_.assign(static_cast<size_t>(rows) * cols, fill);
    }

    // Column vector access
    std::vector<T> column(int j) const {
        std::vector<T> out(static_cast<size_t>(rows_));
        for (int i = 0; i < rows_; ++i) out[static_cast<size_t>(i)] = (*this)(i, j);
        return out;
    }
    // Row vector access
    std::vector<T> row(int i) const {
        std::vector<T> out(static_cast<size_t>(cols_));
        for (int j = 0; j < cols_; ++j) out[static_cast<size_t>(j)] = (*this)(i, j);
        return out;
    }
    void setColumn(int j, const std::vector<T>& v) {
        assert(static_cast<int>(v.size()) == rows_);
        for (int i = 0; i < rows_; ++i) (*this)(i, j) = v[static_cast<size_t>(i)];
    }

private:
    int rows_ = 0;
    int cols_ = 0;
    std::vector<T> data_;
};

using MatD = Mat<double>;
using MatC = Mat<cplx>;
using MatFC = Mat<std::complex<float>>;  // single-precision complex (MATLAB 'single')

// ---------------------------------------------------------------------------
// SonarConfig : parsed + derived sonar parameters (port of SonarInit.m output)
// ---------------------------------------------------------------------------
struct SonarConfig {
    std::string configPath;       // path of the JSON config that produced this

    // --- raw JSON: array_params ---
    double c0 = 1500.0;
    double fs = 2e6;
    double fc = 450e3;
    double BW = 100e3;
    int Nrx = 128;
    int Ntx = 1;
    double pulse_len = 0.005;
    double interp_factor = 8.0;
    int echo_oversample_factor = 5;  // balanced default: 16 MHz * 5 = 80 MHz
    double tx_interval_lambda = 0.0;
    double rx_interval_lambda = 0.5;
    std::vector<double> lightPos{0.0, 0.0, 0.0};
    std::vector<double> velocity{0.0};
    std::vector<double> snr_level{0.0};
    std::vector<double> compensate_range{0.0};

    // --- raw JSON: tx_signal_params ---
    std::string tx_type = "lfm";      // lfm | cdm | fdm
    int sector_num = 1;
    std::vector<double> angles_deg;   // [Na] deg
    std::vector<double> Subfc;        // [Na] Hz
    std::vector<double> SubBW;        // [Na] Hz
    std::string pol = "up";
    std::vector<std::string> pol_vec;  // per-subband polarity (FDM)

    // --- raw JSON: rx_signal_params ---
    std::string array_window = "hamming";
    std::string signal_window = "hamming";
    int decimation_factor = 16;
    std::vector<std::vector<double>> angles_div;  // sectors, deg

    // --- raw JSON: file_opt_params ---
    std::string esl3d_path;
    std::string output_path;
    bool cuda_echo = false;  // route echo synthesis to the GPU accelerator
    bool cpu_fft_echo = false;  // opt-in sparse-IR/FFTW CPU echo backend

    // --- derived (SonarInit) ---
    double lambda = 0.0;
    double dt = 0.0;
    MatD rx_xyz;   // [Nrx x 3]
    MatD tx_xyz;   // [Ntx x 3]

    // excitation / matched filter (column-major; Nt x M)
    MatC exc_nt;    // transmit excitation per element [Nt x M]
    MatC MF;        // match filter, decimated [M x 1] (SonarInit: MF_deci)
    MatC MF_mix;    // mixed MF, pre-FIR [M x 1]
    MatC MF_fir;    // FIR-filtered MF [M x Na]
};

// ---------------------------------------------------------------------------
// FramePointCloud : one esl3d frame converted to scatterer list
// ---------------------------------------------------------------------------
struct FramePointCloud {
    uint64_t seq = 0;
    uint64_t ts_us = 0;
    MatD points;      // [K x 3] x,y,z (m)
    MatD amplitudes;  // [K x 1] intensity
};

// ---------------------------------------------------------------------------
// EchoFrame : output of echo simulation (port of EchoInit output subset)
// ---------------------------------------------------------------------------
struct EchoFrame {
    MatFC y_deci;     // [samples x Nrx] decimated complex baseband (single)
    double fs_deci = 0.0;
    double t0 = 0.0;  // earliest arrival delay (s)
    std::string backend;
};

// ---------------------------------------------------------------------------
// SonarAttributes : HDF5 header attributes/datasets (port of DataMakerInit
// sonarInfo struct) and the information needed by file2image.
// ---------------------------------------------------------------------------
struct SonarAttributes {
    std::string array_type = "Baseline";
    std::string signal_type = "Baseband";
    std::string signal_win = "hamming";
    std::vector<double> bandwidth{0};
    double sampling_frequency = 0.0;
    std::vector<double> center_frequency{0};
    int decimate_factor = 1;
    int sector_num = 1;
    MatC match_filter_data;          // complex
    int receive_array_num = 0;
    MatD receive_array_position;     // [N x 3]
    MatD receive_array_win;          // [N x 1]
    double pulse_duration = 0.0;
    double sound_velocity = 1500.0;
    double velocity = 0.0;
    double snr_level = 0.0;
    std::string timestamp;
    std::vector<double> scan_angle;  // deg
    std::vector<double> sector_div;
    std::vector<double> sample_delay{0.0};
    int ping_num = 0;
};

}  // namespace sonar

