// HDF5 writer/reader round-trip test (requires SONAR_HAVE_HDF5).
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "io/hdf5_reader.h"
#include "io/hdf5_writer.h"
#include "types.h"

using namespace sonar;

int main(int argc, char** argv) {
    std::string path = (argc >= 2) ? argv[1] : "test_roundtrip.h5";
    try {
        // ---- write ----
        io::Hdf5Writer writer;
        SonarAttributes attrs;
        attrs.array_type = "Baseline";
        attrs.signal_type = "Baseband";
        attrs.signal_win = "hamming";
        attrs.bandwidth = {100000.0};
        attrs.sampling_frequency = 2e6;
        attrs.center_frequency = {450000.0};
        attrs.decimate_factor = 16;
        attrs.sector_num = 1;
        attrs.receive_array_num = 4;
        attrs.pulse_duration = 0.005;
        attrs.sound_velocity = 1500.0;
        attrs.velocity = 0.0;
        attrs.snr_level = 0.0;
        attrs.timestamp = "20260811_120000";
        attrs.scan_angle = {-60.0, 0.0, 60.0};
        // Keep the synthetic range axis positive so MATLAB file2image can
        // also consume this file in the cross-runtime compatibility test.
        attrs.sample_delay = {0.003};
        attrs.receive_array_position.resize(4, 3);
        for (int i = 0; i < 4; ++i) {
            attrs.receive_array_position(i, 0) = static_cast<double>(i) * 0.01;
            attrs.receive_array_position(i, 1) = 0.0;
            attrs.receive_array_position(i, 2) = 0.0;
        }
        attrs.receive_array_win.resize(4, 1);
        for (int i = 0; i < 4; ++i) attrs.receive_array_win(i, 0) = 0.5 + 0.1 * i;
        attrs.match_filter_data.resize(8, 1);
        for (int i = 0; i < 8; ++i)
            attrs.match_filter_data(i, 0) = cplx(std::cos(0.1 * i), std::sin(0.1 * i));

        writer.start(path, attrs);
        MatC ping(6, 4);
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 4; ++c) ping(r, c) = cplx(r + 1.0 + 0.5 * c, -(r + 1.0));
        writer.write(ping);
        writer.close();

        // ---- read back ----
        io::Hdf5Data data = io::read_baseline_hdf5(path);
        if (data.pings.size() != 1) {
            printf("FAILED: expected 1 ping, got %zu\n", data.pings.size());
            return 1;
        }
        const MatC& back = data.pings[0];
        if (back.rows() != 6 || back.cols() != 4) {
            printf("FAILED: ping dims %dx%d != 6x4\n", back.rows(), back.cols());
            return 1;
        }
        double maxerr = 0.0;
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 4; ++c)
                maxerr = std::max(maxerr,
                                  std::abs(back(r, c) - cplx(r + 1.0 + 0.5 * c, -(r + 1.0))));
        printf("ping max err: %.3e\n", maxerr);
        if (maxerr > 1e-5) {
            printf("FAILED: ping mismatch\n");
            return 1;
        }

        const auto& a = data.attributes;
        printf("attrs: array_type=%s sf=%.0f decim=%d rx=%d pos(3,0)=%.3f mf(0)=%.3f+%.3fj\n",
               a.array_type.c_str(), a.sampling_frequency, a.decimate_factor, a.receive_array_num,
               a.receive_array_position(3, 0), a.match_filter_data(0, 0).real(),
               a.match_filter_data(0, 0).imag());
        if (a.receive_array_num != 4 || a.decimate_factor != 16 ||
            std::abs(a.sampling_frequency - 2e6) > 1e-9 ||
            std::abs(a.receive_array_position(3, 0) - 0.03) > 1e-6) {
            printf("FAILED: attribute mismatch\n");
            return 1;
        }
        if (std::abs(a.match_filter_data(0, 0) - cplx(1.0, 0.0)) > 1e-6) {
            printf("FAILED: match_filter_data mismatch\n");
            return 1;
        }

        const char* keepOutput = std::getenv("ESL_KEEP_TEST_OUTPUT");
        if (!(keepOutput && *keepOutput && std::string(keepOutput) != "0"))
            std::filesystem::remove(path);
        else
            std::printf("kept test file: %s\n", path.c_str());
        printf("HDF5 ROUND-TRIP TEST PASSED\n");
        return 0;
    } catch (const std::exception& e) {
        printf("FAILED: %s\n", e.what());
        return 1;
    }
}
