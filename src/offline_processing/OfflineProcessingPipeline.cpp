#include "OfflineProcessingPipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "image/file2image_pipeline.h"
#include "image/render.h"
#include "image/sector_map.h"
#include "io/esl3d_reader.h"
#include "io/hdf5_writer.h"
#include "io/json_config.h"
#include "sim/echo_simulator.h"
#include "sim/excitation.h"
#include "sim/pointcloud_ops.h"
#include "types.h"
#include "util/log.h"
#include "util/perf_timer.h"
#include "util/timestamp.h"

namespace fs = std::filesystem;

namespace sonar::offline {
namespace {

void report(const ProgressCallback& callback, const int completed, const int total,
            const std::string& message) {
    if (callback) callback(ProcessingProgress{completed, total, message});
}

void validate_options(const ProcessingOptions& options) {
    if (options.esl3d_path.empty() || !fs::is_regular_file(options.esl3d_path)) {
        throw std::runtime_error("ESL3D input does not exist: " + options.esl3d_path);
    }
    if (options.sonar_config_path.empty() ||
        !fs::is_regular_file(options.sonar_config_path)) {
        throw std::runtime_error("Sonar configuration does not exist: " +
                                 options.sonar_config_path);
    }
    if (options.output_directory.empty()) {
        throw std::runtime_error("Waveform output directory is empty");
    }
    if (options.image_width <= 1 || options.image_height <= 1) {
        throw std::runtime_error("Image dimensions must both be greater than one pixel");
    }
    if (!(options.image_dynamic_range_db > 0.0) ||
        !std::isfinite(options.image_dynamic_range_db)) {
        throw std::runtime_error("Image dynamic range must be a finite positive value");
    }
}

int environment_nonnegative_int(const char* name, const int fallback) {
    const char* value = std::getenv(name);
    if (!value || !*value) return fallback;
    const int parsed = std::atoi(value);
    return parsed >= 0 ? parsed : fallback;
}

void apply_validation_caps(MatD& positions, MatD& amplitudes) {
    const char* max_range_value = std::getenv("ESL_MAX_RANGE");
    if (max_range_value && *max_range_value) {
        const double max_range = std::atof(max_range_value);
        if (max_range > 0.0) {
            std::vector<int> keep;
            keep.reserve(static_cast<size_t>(positions.rows()));
            for (int row = 0; row < positions.rows(); ++row) {
                const double range = std::sqrt(
                    positions(row, 0) * positions(row, 0) +
                    positions(row, 1) * positions(row, 1) +
                    positions(row, 2) * positions(row, 2));
                if (range <= max_range) keep.push_back(row);
            }
            MatD capped_positions(static_cast<int>(keep.size()), positions.cols());
            MatD capped_amplitudes(static_cast<int>(keep.size()), amplitudes.cols());
            for (size_t destination = 0; destination < keep.size(); ++destination) {
                const int source = keep[destination];
                for (int column = 0; column < positions.cols(); ++column) {
                    capped_positions(static_cast<int>(destination), column) =
                        positions(source, column);
                }
                for (int column = 0; column < amplitudes.cols(); ++column) {
                    capped_amplitudes(static_cast<int>(destination), column) =
                        amplitudes(source, column);
                }
            }
            positions = std::move(capped_positions);
            amplitudes = std::move(capped_amplitudes);
        }
    }

    const int max_scatterers =
        environment_nonnegative_int("ESL_MAX_SCATTERERS", positions.rows());
    if (positions.rows() > max_scatterers) {
        MatD capped_positions(max_scatterers, positions.cols());
        MatD capped_amplitudes(max_scatterers, amplitudes.cols());
        for (int row = 0; row < max_scatterers; ++row) {
            for (int column = 0; column < positions.cols(); ++column) {
                capped_positions(row, column) = positions(row, column);
            }
            for (int column = 0; column < amplitudes.cols(); ++column) {
                capped_amplitudes(row, column) = amplitudes(row, column);
            }
        }
        positions = std::move(capped_positions);
        amplitudes = std::move(capped_amplitudes);
    }
}

}  // namespace

ProcessingResult process_esl3d_to_images(const ProcessingOptions& options,
                                         const ProgressCallback& progress) {
    validate_options(options);
    fs::create_directories(options.output_directory);

    report(progress, 0, 0, "Loading sonar configuration");
    SonarConfig config = io::load_sonar_config(options.sonar_config_path);
    config.esl3d_path = options.esl3d_path;
    config.output_path = options.output_directory;
    config.cuda_echo = false;
    sim::set_cuda_echo_backend(false, {});
    sim::generate_excitation(config);

    io::Esl3dReader point_cloud_data;
    point_cloud_data.load(options.esl3d_path);
    const int available_frame_count = point_cloud_data.frameCount();
    if (available_frame_count <= 0) {
        throw std::runtime_error("ESL3D input contains no frames");
    }
    const int frame_count = std::min(
        available_frame_count,
        environment_nonnegative_int("ESL_MAX_FRAMES", available_frame_count));
    const int total_steps = frame_count * 2 + 2;
    report(progress, 1, total_steps, "Preparing HDF5 waveform output");

    const std::string stamp = util::timestamp_now();
    const fs::path input_path(options.esl3d_path);
    const fs::path output_dir(options.output_directory);
    const fs::path hdf5_path = output_dir / (input_path.stem().string() + ".h5");
    SonarAttributes attributes = sim::build_sonar_attributes(config, stamp);

    io::Hdf5Writer writer;
    writer.start(hdf5_path.string(), attributes);

    ProcessingResult result;
    result.hdf5_path = hdf5_path.string();
    result.frame_count = frame_count;
    result.image_paths.reserve(static_cast<size_t>(frame_count));

    std::ofstream performance_csv;
    const char* performance_path = std::getenv("ESL_OFFLINE_PERF_CSV");
    if (!performance_path || !*performance_path) {
        // Backward-compatible alias used by the original standalone port.
        performance_path = std::getenv("ESL_MATLAB_PERF_CSV");
    }
    if (performance_path && *performance_path) {
        const fs::path csv_path(performance_path);
        if (csv_path.has_parent_path()) fs::create_directories(csv_path.parent_path());
        performance_csv.open(csv_path, std::ios::out | std::ios::trunc);
        if (!performance_csv) {
            throw std::runtime_error("Cannot open offline performance CSV: " +
                                     csv_path.string());
        }
        performance_csv
            << "frame_index,total_ms,esl3d_read_ms,decimation_ms,echo_simulation_ms,"
               "hdf5_write_ms,input_scatterers,retained_scatterers,output_samples,"
               "output_channels,backend\n";
    }

    image::RenderOptions render_options;
    render_options.width = options.image_width;
    render_options.height = options.image_height;
    render_options.dynRange = options.image_dynamic_range_db;

    for (int frame = 0; frame < frame_count; ++frame) {
        Timer frame_timer;
        Timer stage_timer;
        report(progress, 1 + frame * 2, total_steps,
               "Synthesizing channel echoes for ping " +
                   std::to_string(frame + 1) + "/" + std::to_string(frame_count));

        FramePointCloud point_cloud;
        point_cloud_data.getPointCloud(frame, point_cloud);
        const double read_ms = stage_timer.ms();
        const int input_scatterers = point_cloud.points.rows();

        stage_timer.reset();
        MatD positions;
        MatD amplitudes;
        sim::point_cloud_decimate(point_cloud.points, point_cloud.amplitudes,
                                  1.0, -1, positions, amplitudes);
        apply_validation_caps(positions, amplitudes);
        const double decimation_ms = stage_timer.ms();
        const int retained_scatterers = positions.rows();

        stage_timer.reset();
        EchoFrame echo;
        sim::echo_pipeline(config, positions, amplitudes, echo);
        const double echo_simulation_ms = stage_timer.ms();
        MatC waveform(echo.y_deci.rows(), echo.y_deci.cols());
        for (size_t index = 0; index < echo.y_deci.size(); ++index) {
            waveform.data()[index] = cplx(echo.y_deci.data()[index]);
        }
        stage_timer.reset();
        writer.write(waveform);
        const double hdf5_write_ms = stage_timer.ms();
        const double total_ms = frame_timer.ms();
        if (performance_csv) {
            performance_csv << (frame + 1) << ',' << total_ms << ',' << read_ms << ','
                            << decimation_ms << ',' << echo_simulation_ms << ','
                            << hdf5_write_ms << ',' << input_scatterers << ','
                            << retained_scatterers << ',' << echo.y_deci.rows() << ','
                            << echo.y_deci.cols() << ',' << echo.backend << '\n';
        }

        report(progress, 2 + frame * 2, total_steps,
               "Reconstructing sonar image for ping " +
                   std::to_string(frame + 1) + "/" + std::to_string(frame_count));
        image::File2ImageResult formed =
            image::process_ping_for_image(attributes, waveform);
        std::vector<double> ranges(static_cast<size_t>(formed.ranges.rows()));
        for (int row = 0; row < formed.ranges.rows(); ++row) {
            ranges[static_cast<size_t>(row)] = formed.ranges(row, 0);
        }
        MatC magnitude = formed.beam;
        for (size_t index = 0; index < magnitude.size(); ++index) {
            magnitude.data()[index] = cplx(std::abs(magnitude.data()[index]), 0.0);
        }
        const image::SectorImage sector =
            image::sector_map(formed.angles, ranges, magnitude,
                              options.image_dynamic_range_db);

        char suffix[64];
        std::snprintf(suffix, sizeof(suffix), "ping%03d.png", frame + 1);
        const fs::path image_path = output_dir /
            util::timestamp_filename(input_path.stem().string(), stamp, suffix);
        image::render_sector_to_png(sector, render_options, image_path.string());
        result.image_paths.push_back(image_path.string());
        SONAR_LOG_INFO("offline pipeline: ping %d/%d complete", frame + 1,
                       frame_count);
    }

    report(progress, total_steps - 1, total_steps, "Finalizing HDF5 output");
    writer.close();
    report(progress, total_steps, total_steps, "Offline sonar processing complete");
    return result;
}

}  // namespace sonar::offline
