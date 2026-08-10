#pragma once

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

namespace osg {
class Node;
}

namespace standalone_mvp {

struct SceneInventory {
    std::uint64_t node_count = 0;
    std::uint64_t drawable_count = 0;
    std::uint64_t vertex_count = 0;
    std::uint64_t triangle_count = 0;
};

struct PerformanceSample {
    std::string component;
    std::string module;
    std::int64_t frame_index = -1;
    double duration_ms = 0.0;
    std::uint64_t input_points = 0;
    std::uint64_t output_points = 0;
    std::uint64_t output_bytes = 0;
    std::uint64_t beam_count = 0;
    std::uint64_t bin_count = 0;
    std::uint64_t width = 0;
    std::uint64_t height = 0;
    double range_m = 0.0;
    SceneInventory scene;
};

// Opt-in CSV profiler used for reproducible performance and scale experiments.
// Enable it by setting ESL_CPP_PERF_CSV to an output path before launching.
class PerformanceProfiler {
public:
    static PerformanceProfiler& instance();

    bool enabled() const { return enabled_; }
    const std::string& outputPath() const { return output_path_; }
    void record(const PerformanceSample& sample);
    void recordSceneInventory(const SceneInventory& inventory);
    void flush();

private:
    PerformanceProfiler();
    ~PerformanceProfiler();
    PerformanceProfiler(const PerformanceProfiler&) = delete;
    PerformanceProfiler& operator=(const PerformanceProfiler&) = delete;

    bool enabled_ = false;
    std::string output_path_;
    std::string run_label_;
    std::string platform_;
    unsigned int hardware_threads_ = 0;
    std::uint64_t sample_stride_ = 1;
    std::uint64_t received_samples_ = 0;
    std::uint64_t written_samples_ = 0;
    std::ofstream stream_;
    mutable std::mutex mutex_;
};

SceneInventory collectSceneInventory(osg::Node* root);

} // namespace standalone_mvp

