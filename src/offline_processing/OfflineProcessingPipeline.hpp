#pragma once

#include <functional>
#include <string>
#include <vector>

namespace sonar::offline {

struct ProcessingOptions {
    std::string esl3d_path;
    std::string sonar_config_path;
    std::string output_directory;
    double image_dynamic_range_db = 40.0;
    int image_width = 800;
    int image_height = 600;
};

struct ProcessingProgress {
    int completed_steps = 0;
    int total_steps = 0;
    std::string message;
};

using ProgressCallback = std::function<void(const ProcessingProgress&)>;

struct ProcessingResult {
    std::string hdf5_path;
    std::vector<std::string> image_paths;
    int frame_count = 0;
};

// Convert an ESL3D recording to channel-waveform HDF5 and one reconstructed
// grayscale PNG per ping. All signal processing executes in this process;
// the configuration file is read-only and path overrides remain in memory.
ProcessingResult process_esl3d_to_images(const ProcessingOptions& options,
                                         const ProgressCallback& progress = {});

}  // namespace sonar::offline
