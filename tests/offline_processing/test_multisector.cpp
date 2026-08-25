#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "image/file2image_pipeline.h"
#include "io/hdf5_reader.h"
#include "types.h"

using namespace sonar;

int main(int argc, char** argv) {
    try {
        if (argc == 3) {
            const io::Hdf5Data input = io::read_baseline_hdf5(argv[1]);
            if (input.pings.empty()) throw std::runtime_error("reference input has no pings");
            const image::File2ImageResult actual =
                image::process_ping_for_image(input.attributes, input.pings[0]);
            MatC refBeam, refRanges, refAngles;
            if (!io::read_hdf5_dataset(argv[2], "/beam", refBeam) ||
                !io::read_hdf5_dataset(argv[2], "/ranges", refRanges) ||
                !io::read_hdf5_dataset(argv[2], "/angles", refAngles))
                throw std::runtime_error("failed to read MATLAB multisector reference");
            if (actual.beam.rows() != refBeam.rows() || actual.beam.cols() != refBeam.cols() ||
                actual.ranges.size() != refRanges.size() || actual.angles.size() != refAngles.size())
                throw std::runtime_error("MATLAB/C++ multisector dimensions differ");

            double maxBeamError = 0.0, maxBeamReference = 0.0;
            for (size_t i = 0; i < actual.beam.size(); ++i) {
                maxBeamError = std::max(maxBeamError,
                                        std::abs(actual.beam.data()[i] - refBeam.data()[i]));
                maxBeamReference = std::max(maxBeamReference, std::abs(refBeam.data()[i]));
            }
            double maxAxisError = 0.0;
            for (size_t i = 0; i < actual.ranges.size(); ++i)
                maxAxisError = std::max(maxAxisError,
                                        std::abs(actual.ranges.data()[i] - refRanges.data()[i].real()));
            for (size_t i = 0; i < actual.angles.size(); ++i)
                maxAxisError = std::max(maxAxisError,
                                        std::abs(actual.angles[i] - refAngles.data()[i].real()));
            const double relativeBeamError = maxBeamError / std::max(maxBeamReference, 1e-30);
            std::printf("MATLAB/C++ multisector: max_abs=%.3e rel_max=%.3e axis=%.3e\n",
                        maxBeamError, relativeBeamError, maxAxisError);
            if (relativeBeamError > 2e-6 || maxAxisError > 1e-6)
                throw std::runtime_error("MATLAB/C++ multisector tolerance exceeded");
            std::printf("MULTISECTOR MATLAB REFERENCE TEST PASSED\n");
            return 0;
        }

        SonarAttributes a;
        a.sampling_frequency = 100000.0;
        a.decimate_factor = 1;
        a.sector_num = 2;
        a.receive_array_num = 2;
        a.sound_velocity = 1500.0;
        a.pulse_duration = 0.001;
        a.center_frequency = {45000.0, 48000.0};
        a.bandwidth = {10000.0, 10000.0};
        a.sample_delay = {0.0, 0.0};

        // MATLAB DataMakerInit legacy shape: scan_angle contains only the
        // first cell. sector_div is sufficient to reconstruct sector 2.
        a.scan_angle = {-60.0, -59.0, -58.0};
        a.sector_div = {-60.0, -58.0, -55.0};

        a.receive_array_position.resize(2, 3);
        a.receive_array_position(0, 0) = -0.003;
        a.receive_array_position(1, 0) = 0.003;

        a.match_filter_data.resize(3, 2);
        a.match_filter_data(1, 0) = cplx(1.0, 0.0);
        a.match_filter_data(1, 1) = cplx(0.0, 1.0);

        MatC ping(32, 2);
        for (int r = 0; r < ping.rows(); ++r) {
            const double phase = 0.17 * r;
            ping(r, 0) = std::polar(1.0, phase);
            ping(r, 1) = std::polar(0.8, phase + 0.1);
        }

        const auto sectors = image::resolve_sector_angles_deg(a, 2);
        if (sectors.size() != 2 || sectors[0].size() != 3 || sectors[1].size() != 3 ||
            std::abs(sectors[1][0] - (-57.0)) > 1e-12 ||
            std::abs(sectors[1][2] - (-55.0)) > 1e-12) {
            std::printf("FAILED: legacy sector angle reconstruction\n");
            return 1;
        }

        const image::File2ImageResult out = image::process_ping_for_image(a, ping);
        if (out.beam.rows() != 62 || out.beam.cols() != 6 ||
            out.ranges.rows() != 62 || out.angles.size() != 6) {
            std::printf("FAILED: multisector output dimensions beam=%dx%d ranges=%d angles=%zu\n",
                        out.beam.rows(), out.beam.cols(), out.ranges.rows(), out.angles.size());
            return 1;
        }
        for (size_t i = 0; i < out.beam.size(); ++i) {
            if (!std::isfinite(out.beam.data()[i].real()) ||
                !std::isfinite(out.beam.data()[i].imag())) {
                std::printf("FAILED: non-finite multisector beam\n");
                return 1;
            }
        }

        std::printf("MULTISECTOR IMAGE TEST PASSED\n");
        return 0;
    } catch (const std::exception& e) {
        std::printf("FAILED: %s\n", e.what());
        return 1;
    }
}

