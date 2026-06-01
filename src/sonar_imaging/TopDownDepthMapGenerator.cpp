#include "TopDownDepthMapGenerator.hpp"

#include "OffscreenCaptureRig.hpp"

#include <QColor>

#include <osg/ComputeBoundsVisitor>
#include <osg/Group>
#include <osg/Program>
#include <osg/Shader>
#include <osg/StateSet>

#include <algorithm>
#include <cmath>
#include <limits>

namespace sonar_imaging {
namespace {

double clamp01(double v) {
    return std::max(0.0, std::min(1.0, v));
}

osg::ref_ptr<osg::Node> buildTopDownDepthRenderNode(osg::ref_ptr<osg::Node> scene_root, double z_far) {
    if (!scene_root.valid()) {
        return {};
    }
    static const char* kVs = R"GLSL(
#version 130
out vec3 viewPos;
void main() {
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    viewPos = (gl_ModelViewMatrix * gl_Vertex).xyz;
}
)GLSL";
    static const char* kFs = R"GLSL(
#version 130
in vec3 viewPos;
uniform float farPlane;
void main() {
    // For top-down ortho view (camera looking -Z), -viewPos.z is view-axis depth.
    float d = clamp((-viewPos.z) / max(farPlane, 1e-6), 0.0, 1.0);
    gl_FragData[0] = vec4(0.0, d, 0.0, 1.0);
}
)GLSL";

    osg::ref_ptr<osg::Program> program = new osg::Program();
    program->addShader(new osg::Shader(osg::Shader::VERTEX, kVs));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, kFs));

    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(scene_root);

    osg::StateSet* ss = root->getOrCreateStateSet();
    ss->setAttributeAndModes(program, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    ss->addUniform(new osg::Uniform("farPlane", static_cast<float>(z_far)));
    return root;
}

} // namespace

QPointF TopDownDepthMapResult::pixelToWorld(const QPointF& pixel) const {
    if (width <= 1 || height <= 1) {
        return {};
    }
    const double u = clamp01(pixel.x() / static_cast<double>(width - 1));
    const double v = clamp01(pixel.y() / static_cast<double>(height - 1));
    const double x = min_x + u * (max_x - min_x);
    const double y = max_y - v * (max_y - min_y);
    return QPointF(x, y);
}

QPointF TopDownDepthMapResult::worldToPixel(const QPointF& world_xy) const {
    if (width <= 1 || height <= 1 || std::abs(max_x - min_x) < 1e-9 || std::abs(max_y - min_y) < 1e-9) {
        return {};
    }
    const double u = clamp01((world_xy.x() - min_x) / (max_x - min_x));
    const double v = clamp01((max_y - world_xy.y()) / (max_y - min_y));
    return QPointF(u * static_cast<double>(width - 1), v * static_cast<double>(height - 1));
}

double TopDownDepthMapResult::sampleZAtPixel(const QPointF& pixel) const {
    if (!valid()) {
        return 0.0;
    }
    const int px = std::clamp(static_cast<int>(std::lround(pixel.x())), 0, width - 1);
    const int py = std::clamp(static_cast<int>(std::lround(pixel.y())), 0, height - 1);
    const std::size_t idx = static_cast<std::size_t>(py) * static_cast<std::size_t>(width) + static_cast<std::size_t>(px);
    if (idx >= z_values.size()) {
        return 0.0;
    }
    if (idx >= hit_mask.size() || hit_mask[idx] == 0) {
        return 0.0;
    }
    return static_cast<double>(z_values[idx]);
}

bool TopDownDepthMapResult::hasHitAtPixel(const QPointF& pixel) const {
    if (!valid()) {
        return false;
    }
    const int px = std::clamp(static_cast<int>(std::lround(pixel.x())), 0, width - 1);
    const int py = std::clamp(static_cast<int>(std::lround(pixel.y())), 0, height - 1);
    const std::size_t idx = static_cast<std::size_t>(py) * static_cast<std::size_t>(width) + static_cast<std::size_t>(px);
    return idx < hit_mask.size() && hit_mask[idx] != 0;
}

TopDownDepthMapResult TopDownDepthMapGenerator::generate(osg::ref_ptr<osg::Node> scene_root, int width, int height) const {
    TopDownDepthMapResult out;
    if (!scene_root.valid()) {
        return out;
    }
    width = std::max(64, width);
    height = std::max(64, height);

    osg::ComputeBoundsVisitor bounds_visitor;
    scene_root->accept(bounds_visitor);
    const osg::BoundingBox bb = bounds_visitor.getBoundingBox();

    double min_x = -100.0;
    double max_x = 100.0;
    double min_y = -100.0;
    double max_y = 100.0;
    double center_z = 0.0;
    double terrain_span_z = 10.0;

    if (bb.valid()) {
        min_x = static_cast<double>(bb.xMin());
        max_x = static_cast<double>(bb.xMax());
        min_y = static_cast<double>(bb.yMin());
        max_y = static_cast<double>(bb.yMax());
        const double min_z = static_cast<double>(bb.zMin());
        const double max_z = static_cast<double>(bb.zMax());
        center_z = 0.5 * (min_z + max_z);
        terrain_span_z = std::max(1.0, max_z - min_z);
    }

    // Tighten map to world_models AABB, with only a small padding.
    const double pad_x = std::max(2.0, (max_x - min_x) * 0.05);
    const double pad_y = std::max(2.0, (max_y - min_y) * 0.05);
    min_x -= pad_x;
    max_x += pad_x;
    min_y -= pad_y;
    max_y += pad_y;

    const double center_x = 0.5 * (min_x + max_x);
    const double center_y = 0.5 * (min_y + max_y);
    const double half_w = std::max(5.0, 0.5 * (max_x - min_x));
    const double half_h = std::max(5.0, 0.5 * (max_y - min_y));
    const double max_half_extent = std::max(half_w, half_h);

    const osg::Vec3d center(center_x, center_y, center_z);
    const double camera_height = std::max(40.0, terrain_span_z * 4.0 + max_half_extent * 0.35);
    const double z_near = 0.1;
    const double z_far = std::max(200.0, camera_height + terrain_span_z * 4.0 + 80.0);

    osg::ref_ptr<osg::Node> depth_scene = buildTopDownDepthRenderNode(scene_root, z_far);
    if (!depth_scene.valid()) {
        return out;
    }

    OffscreenCaptureRig rig(static_cast<uint>(width), static_cast<uint>(height));
    rig.setProjectionOrtho(-half_w, half_w, -half_h, half_h, z_near, z_far);
    rig.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 0.0));
    rig.setViewPose(center + osg::Vec3d(0.0, 0.0, camera_height), center, osg::Vec3d(0.0, 1.0, 0.0));
    osg::ref_ptr<osg::Image> image = rig.captureNodeImage(depth_scene);
    if (!image.valid() || !image->data()) {
        return out;
    }

    out.width = static_cast<int>(image->s());
    out.height = static_cast<int>(image->t());
    out.min_x = min_x;
    out.max_x = max_x;
    out.min_y = min_y;
    out.max_y = max_y;
    out.far_plane_m = z_far;
    out.eye_z = center.z() + camera_height;
    out.depth_norm.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height), 0.0f);
    out.z_values.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height), 0.0f);
    out.hit_mask.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height), 0);
    out.image = QImage(out.width, out.height, QImage::Format_RGB32);
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();

    constexpr double kHitDepthThreshold = 1e-6;
    for (int y = 0; y < out.height; ++y) {
        const int src_y = out.height - 1 - y;
        for (int x = 0; x < out.width; ++x) {
            const float* px = reinterpret_cast<const float*>(image->data(x, src_y));
            const double dn = clamp01(static_cast<double>(px[1]));
            const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(out.width) + static_cast<std::size_t>(x);
            out.depth_norm[idx] = static_cast<float>(dn);
            const bool hit = dn > kHitDepthThreshold;
            out.hit_mask[idx] = static_cast<unsigned char>(hit ? 1 : 0);
            if (hit) {
                const double z = out.eye_z - dn * out.far_plane_m;
                out.z_values[idx] = static_cast<float>(z);
                min_z = std::min(min_z, z);
                max_z = std::max(max_z, z);
            }
        }
    }
    if (!(max_z > min_z)) {
        min_z = -1.0;
        max_z = 1.0;
    }
    out.z_min = min_z;
    out.z_max = max_z;

    for (int y = 0; y < out.height; ++y) {
        for (int x = 0; x < out.width; ++x) {
            const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(out.width) + static_cast<std::size_t>(x);
            if (out.hit_mask[idx] == 0) {
                out.image.setPixelColor(x, y, QColor(22, 26, 34));
                continue;
            }
            const double z = static_cast<double>(out.z_values[idx]);
            const double local = clamp01((z - min_z) / std::max(1e-9, (max_z - min_z)));
            // Color mapped by Z-axis (low Z to high Z).
            const int hue = static_cast<int>(std::lround(local * 200.0));
            const int value = static_cast<int>(std::lround(120.0 + local * 100.0));
            out.image.setPixelColor(
                x, y, QColor::fromHsv(std::clamp(hue, 0, 359), 200, std::clamp(value, 40, 255)));
        }
    }

    return out;
}

} // namespace sonar_imaging

