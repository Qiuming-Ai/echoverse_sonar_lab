#include "PerformanceProfiler.hpp"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/NodeVisitor>
#include <osg/PrimitiveSet>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <thread>

namespace standalone_mvp {
namespace {

std::string csvEscape(const std::string& value) {
    if (value.find_first_of(",\"\r\n") == std::string::npos) {
        return value;
    }
    std::string escaped = "\"";
    for (const char ch : value) {
        escaped += ch == '\"' ? "\"\"" : std::string(1, ch);
    }
    escaped += '\"';
    return escaped;
}

std::string currentPlatform() {
#if defined(_WIN32)
    return "windows";
#elif defined(__linux__)
    return "linux";
#elif defined(__APPLE__)
    return "macos";
#else
    return "unknown";
#endif
}

std::uint64_t parseStride(const char* raw) {
    if (!raw || !*raw) {
        return 1;
    }
    try {
        return std::max<std::uint64_t>(1, std::stoull(raw));
    } catch (...) {
        return 1;
    }
}

class SceneInventoryVisitor final : public osg::NodeVisitor {
public:
    SceneInventoryVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}

    void apply(osg::Node& node) override {
        ++inventory.node_count;
        traverse(node);
    }

    void apply(osg::Geode& geode) override {
        ++inventory.node_count;
        for (unsigned int drawable_index = 0; drawable_index < geode.getNumDrawables(); ++drawable_index) {
            osg::Drawable* drawable = geode.getDrawable(drawable_index);
            if (!drawable) {
                continue;
            }
            ++inventory.drawable_count;
            osg::Geometry* geometry = drawable->asGeometry();
            if (!geometry) {
                continue;
            }
            if (const osg::Array* vertices = geometry->getVertexArray()) {
                inventory.vertex_count += vertices->getNumElements();
            }
            for (unsigned int primitive_index = 0;
                 primitive_index < geometry->getNumPrimitiveSets();
                 ++primitive_index) {
                const osg::PrimitiveSet* primitive = geometry->getPrimitiveSet(primitive_index);
                if (!primitive) {
                    continue;
                }
                const std::uint64_t primitive_count = primitive->getNumPrimitives();
                switch (primitive->getMode()) {
                case GL_TRIANGLES:
                case GL_TRIANGLE_STRIP:
                case GL_TRIANGLE_FAN:
                    inventory.triangle_count += primitive_count;
                    break;
                case GL_QUADS:
                case GL_QUAD_STRIP:
                    inventory.triangle_count += primitive_count * 2;
                    break;
                case GL_POLYGON:
                    if (primitive->getNumIndices() >= 3) {
                        inventory.triangle_count += primitive->getNumIndices() - 2;
                    }
                    break;
                default:
                    break;
                }
            }
        }
        traverse(geode);
    }

    SceneInventory inventory;
};

} // namespace

PerformanceProfiler& PerformanceProfiler::instance() {
    static PerformanceProfiler profiler;
    return profiler;
}

PerformanceProfiler::PerformanceProfiler() {
    const char* output = std::getenv("ESL_CPP_PERF_CSV");
    if (!output || !*output) {
        return;
    }
    output_path_ = output;
    run_label_ = std::getenv("ESL_PERF_RUN_LABEL") ? std::getenv("ESL_PERF_RUN_LABEL") : "";
    platform_ = currentPlatform();
    hardware_threads_ = std::thread::hardware_concurrency();
    sample_stride_ = parseStride(std::getenv("ESL_PERF_SAMPLE_STRIDE"));

    std::error_code error;
    const std::filesystem::path output_path(output_path_);
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path(), error);
    }
    stream_.open(output_path_, std::ios::out | std::ios::trunc);
    if (!stream_.is_open()) {
        output_path_.clear();
        return;
    }
    enabled_ = true;
    stream_ << "run_label,platform,hardware_threads,timestamp_epoch_ms,component,module,frame_index,"
               "duration_ms,input_points,output_points,output_bytes,beam_count,bin_count,width,height,range_m,"
               "scene_nodes,scene_drawables,scene_vertices,scene_triangles\n";
}

PerformanceProfiler::~PerformanceProfiler() {
    flush();
}

void PerformanceProfiler::record(const PerformanceSample& sample) {
    if (!enabled_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_samples_;
    if (((received_samples_ - 1) % sample_stride_) != 0) {
        return;
    }
    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    stream_ << csvEscape(run_label_) << ','
            << platform_ << ','
            << hardware_threads_ << ','
            << now_ms << ','
            << csvEscape(sample.component) << ','
            << csvEscape(sample.module) << ','
            << sample.frame_index << ','
            << std::fixed << std::setprecision(6) << sample.duration_ms << ','
            << sample.input_points << ','
            << sample.output_points << ','
            << sample.output_bytes << ','
            << sample.beam_count << ','
            << sample.bin_count << ','
            << sample.width << ','
            << sample.height << ','
            << std::setprecision(3) << sample.range_m << ','
            << sample.scene.node_count << ','
            << sample.scene.drawable_count << ','
            << sample.scene.vertex_count << ','
            << sample.scene.triangle_count << '\n';
    ++written_samples_;
    if ((written_samples_ % 50) == 0) {
        stream_.flush();
    }
}

void PerformanceProfiler::recordSceneInventory(const SceneInventory& inventory) {
    PerformanceSample sample;
    sample.component = "scene_inventory";
    sample.scene = inventory;
    record(sample);
}

void PerformanceProfiler::flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_.is_open()) {
        stream_.flush();
    }
}

SceneInventory collectSceneInventory(osg::Node* root) {
    if (!root) {
        return {};
    }
    SceneInventoryVisitor visitor;
    root->accept(visitor);
    return visitor.inventory;
}

} // namespace standalone_mvp
