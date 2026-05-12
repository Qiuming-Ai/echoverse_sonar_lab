#include "PointCloudSonarSimulation.hpp"

#include <sonar_core/SignalProcessingUtils.hpp>

#include <opencv2/core.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>

namespace standalone_mvp {
namespace {

constexpr double kPi = 3.14159265358979323846;

double degToRad(double deg) {
    return deg * (kPi / 180.0);
}

double clampRangeMeters(double range_m) {
    return std::clamp(range_m, 0.0, 100.0);
}

double clampFovDeg(double fov_deg) {
    return std::clamp(fov_deg, 1.0, 179.0);
}

double clampResolutionDeg(double resolution_deg) {
    return std::clamp(resolution_deg, 0.01, 30.0);
}

float applySpeckleIfEnabled(float intensity, bool enable_speckle) {
    if (!enable_speckle) {
        return intensity;
    }
    static thread_local std::mt19937 rng(std::random_device{}());
    static thread_local std::normal_distribution<float> dist(0.95f, 0.30f);
    const float noise = std::abs(dist(rng));
    return std::clamp(intensity * noise, 0.0f, 1.0f);
}

float rockSigmoid(float x) {
    // Keep amplitude mapping consistent with sonar_core::BeamImageSynthesizer::sigmoid.
    constexpr float beta = 18.0f;
    constexpr float x0 = 0.666666667f;
    const float t = (x - x0) * beta;
    return 0.5f * std::tanh(0.5f * t) + 0.5f;
}

std::size_t floorSamples(double fov_deg, double resolution_deg) {
    const double value = std::floor(std::max(0.0, fov_deg) / std::max(1e-9, resolution_deg));
    return static_cast<std::size_t>(std::max(1.0, value));
}

} // namespace

PointCloudSonarSimulation::PointCloudSonarSimulation(const PointCloudSonarConfig& config,
                                                     osg::ref_ptr<osg::Group> shared_scene_root)
    : shared_root_(std::move(shared_scene_root)),
      config_(config),
      depth_composer_(static_cast<float>(clampRangeMeters(config.range_m))),
      capture_tool_(640u, 480u) {
    depth_composer_.attachSceneNode(shared_root_);
    depth_composer_.setDrawDepth(true);
    // shader.frag writes return intensity into Z/B channel only when drawNormal is enabled.
    depth_composer_.setDrawNormal(true);
    depth_composer_.setDrawReverb(config_.enable_reverb);
    capture_tool_.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
}

void PointCloudSonarSimulation::setConfig(const PointCloudSonarConfig& config) {
    config_ = config;
    depth_composer_.setMaxRange(static_cast<float>(clampRangeMeters(config_.range_m)));
    depth_composer_.setDrawReverb(config_.enable_reverb);
}

PointCloudSonarConfig PointCloudSonarSimulation::config() const {
    return config_;
}

PointCloudSamplingInfo PointCloudSonarSimulation::computeSamplingInfo(PointCloudSonarConfig& cfg) const {
    PointCloudSamplingInfo info;
    cfg.range_m = clampRangeMeters(cfg.range_m);
    cfg.horizontal_fov_deg = clampFovDeg(cfg.horizontal_fov_deg);
    cfg.vertical_fov_deg = clampFovDeg(cfg.vertical_fov_deg);

    const double original_hres = cfg.horizontal_angle_resolution_deg;
    const double original_vres = cfg.vertical_angle_resolution_deg;
    cfg.horizontal_angle_resolution_deg = clampResolutionDeg(cfg.horizontal_angle_resolution_deg);
    cfg.vertical_angle_resolution_deg = clampResolutionDeg(cfg.vertical_angle_resolution_deg);
    info.invalid_resolution_was_fixed =
        (std::abs(original_hres - cfg.horizontal_angle_resolution_deg) > 1e-9) ||
        (std::abs(original_vres - cfg.vertical_angle_resolution_deg) > 1e-9);

    // Explicitly use floor for point count derivation:
    // point_count = floor(h_fov/h_res) * floor(v_fov/v_res).
    info.requested_horizontal_samples = floorSamples(cfg.horizontal_fov_deg, cfg.horizontal_angle_resolution_deg);
    info.requested_vertical_samples = floorSamples(cfg.vertical_fov_deg, cfg.vertical_angle_resolution_deg);
    info.requested_point_count = info.requested_horizontal_samples * info.requested_vertical_samples;
    info.max_point_count = std::max<std::size_t>(1, cfg.max_point_count);

    info.horizontal_samples = info.requested_horizontal_samples;
    info.vertical_samples = info.requested_vertical_samples;

    if (info.requested_point_count > info.max_point_count) {
        info.clamped_by_budget = true;
        const double scale = std::sqrt(static_cast<double>(info.max_point_count) /
                                       static_cast<double>(info.requested_point_count));
        info.horizontal_samples =
            std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(info.horizontal_samples * scale)));
        info.vertical_samples =
            std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(info.vertical_samples * scale)));
        while (info.horizontal_samples * info.vertical_samples > info.max_point_count) {
            if (info.horizontal_samples >= info.vertical_samples && info.horizontal_samples > 1) {
                --info.horizontal_samples;
            } else if (info.vertical_samples > 1) {
                --info.vertical_samples;
            } else {
                break;
            }
        }
    }

    info.budgeted_point_count = info.horizontal_samples * info.vertical_samples;
    return info;
}

void PointCloudSonarSimulation::rebuildCaptureToolIfNeeded(const PointCloudSamplingInfo& sampling, const PointCloudSonarConfig& cfg) {
    const std::size_t wanted_height = std::clamp<std::size_t>(sampling.vertical_samples, 8, 2048);
    const bool need_rebuild = (wanted_height != capture_height_) ||
                              (std::abs(cfg.horizontal_fov_deg - capture_hfov_deg_) > 1e-6) ||
                              (std::abs(cfg.vertical_fov_deg - capture_vfov_deg_) > 1e-6);
    if (!need_rebuild) {
        return;
    }

    capture_tool_ = sonar_imaging::OffscreenCaptureRig(
        degToRad(cfg.vertical_fov_deg), degToRad(cfg.horizontal_fov_deg), static_cast<uint>(wanted_height), true);
    capture_tool_.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
    capture_height_ = wanted_height;
    capture_hfov_deg_ = cfg.horizontal_fov_deg;
    capture_vfov_deg_ = cfg.vertical_fov_deg;
}

void PointCloudSonarSimulation::updateCapturePose(const Eigen::Affine3d& pose_world) {
    // Keep camera convention consistent with sonar_core::AcousticRaySimulator.
    static const osg::Matrixd rock_coordinate_matrix =
        osg::Matrixd::rotate(kPi * 0.5, osg::Vec3(0, 0, 1)) *
        osg::Matrixd::rotate(-kPi * 0.5, osg::Vec3(1, 0, 0));
    osg::Matrixd matrix(pose_world.data());
    matrix.invert(matrix);
    osg::Matrixd m = matrix * rock_coordinate_matrix;
    osg::Vec3 eye;
    osg::Vec3 center;
    osg::Vec3 up;
    m.getLookAt(eye, center, up);
    capture_tool_.setViewPose(eye, center, up);
}

PointCloudFrame PointCloudSonarSimulation::simulatePointCloud(const Eigen::Affine3d& sonar_pose_world) {
    PointCloudFrame frame;
    frame.timestamp_us =
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::steady_clock::now().time_since_epoch())
                                       .count());
    frame.pose_position_world = sonar_pose_world.translation();
    frame.pose_forward_world = sonar_pose_world.linear().col(0).normalized();
    frame.pose_rotation_world = sonar_pose_world.linear();

    PointCloudSonarConfig cfg = config_;
    PointCloudSamplingInfo sampling = computeSamplingInfo(cfg);
    frame.config = cfg;
    frame.sampling = sampling;
    frame.polar_frame.width = sampling.requested_horizontal_samples;
    frame.polar_frame.height = sampling.requested_vertical_samples;
    frame.polar_frame.point_count = frame.polar_frame.width * frame.polar_frame.height;
    frame.polar_frame.range_image_m.assign(frame.polar_frame.point_count, frame.polar_frame.invalid_value);
    frame.polar_frame.intensity_image.assign(frame.polar_frame.point_count, frame.polar_frame.invalid_value);
    if (!cfg.enabled) {
        return frame;
    }

    const double attenuation = cfg.enable_attenuation
                                   ? sonar_core::computeAbsorptionCoefficient(
                                         cfg.frequency_khz, cfg.temperature_c, cfg.depth_m, cfg.salinity_ppt, cfg.acidity_ph)
                                   : 0.0;
    depth_composer_.setAttenuationCoefficient(static_cast<float>(attenuation));
    depth_composer_.setMaxRange(static_cast<float>(cfg.range_m));
    depth_composer_.setDrawReverb(cfg.enable_reverb);
    rebuildCaptureToolIfNeeded(sampling, cfg);
    updateCapturePose(sonar_pose_world);

    osg::ref_ptr<osg::Image> osg_image = capture_tool_.captureNodeImage(depth_composer_.depthComposerNode());
    if (!osg_image.valid()) {
        return frame;
    }

    cv::Mat cv_image;
    sonar_core::convertOsgImageToCvMat(osg_image, cv_image);
    if (cv_image.empty() || cv_image.type() != CV_32FC3) {
        return frame;
    }

    const int rows = cv_image.rows;
    const int cols = cv_image.cols;
    if (rows <= 0 || cols <= 0) {
        return frame;
    }

    const double hfov_rad = degToRad(cfg.horizontal_fov_deg);
    const double vfov_rad = degToRad(cfg.vertical_fov_deg);
    const double tan_hfov_half = std::tan(hfov_rad * 0.5);
    const double tan_vfov_half = std::tan(vfov_rad * 0.5);
    const double h_step = hfov_rad / static_cast<double>(std::max<std::size_t>(1, frame.polar_frame.width));
    const double v_step = vfov_rad / static_cast<double>(std::max<std::size_t>(1, frame.polar_frame.height));

    frame.points_world.reserve(sampling.budgeted_point_count);
    frame.point_intensities.reserve(sampling.budgeted_point_count);
    for (std::size_t v = 0; v < frame.polar_frame.height; ++v) {
        const double pitch = -vfov_rad * 0.5 + (static_cast<double>(v) + 0.5) * v_step;
        const double y_ndc =
            std::clamp(std::tan(pitch) / std::max(1e-9, tan_vfov_half), -1.0, 1.0);
        const int row = std::clamp(static_cast<int>(std::lround((1.0 - (y_ndc + 1.0) * 0.5) * (rows - 1))), 0, rows - 1);

        for (std::size_t h = 0; h < frame.polar_frame.width; ++h) {
            const double yaw = -hfov_rad * 0.5 + (static_cast<double>(h) + 0.5) * h_step;
            const double x_ndc =
                std::clamp(std::tan(yaw) / std::max(1e-9, tan_hfov_half), -1.0, 1.0);
            const int col = std::clamp(static_cast<int>(std::lround((x_ndc + 1.0) * 0.5 * (cols - 1))), 0, cols - 1);

            const float* px = cv_image.ptr<float>(row) + (col * 3);
            // Keep channel interpretation aligned with sonar_core::Sonar::accumulateBinsFromShader:
            // ptr[i*3 + 0] -> amplitude source, ptr[i*3 + 1] -> normalized distance.
            float intensity = std::clamp(rockSigmoid(px[0]), 0.0f, 1.0f);
            const float normalized_distance = px[1];
            if (!std::isfinite(normalized_distance) || normalized_distance <= 0.0f || normalized_distance > 1.0f) {
                continue;
            }

            const double distance_m = static_cast<double>(normalized_distance) * cfg.range_m;
            const std::size_t idx = v * frame.polar_frame.width + h;
            frame.polar_frame.range_image_m[idx] = static_cast<float>(distance_m);
            frame.polar_frame.intensity_image[idx] = intensity;
            if (frame.points_world.size() >= sampling.budgeted_point_count) {
                continue;
            }
            const double cp = std::cos(pitch);
            Eigen::Vector3d ray_sensor(std::cos(yaw) * cp, std::sin(yaw) * cp, std::sin(pitch));
            const double ray_norm = ray_sensor.norm();
            if (ray_norm < 1e-9) {
                continue;
            }
            ray_sensor /= ray_norm;
            const Eigen::Vector3d p_sensor = ray_sensor * distance_m;
            const Eigen::Vector3d p_world = sonar_pose_world * p_sensor;
            intensity = applySpeckleIfEnabled(intensity, cfg.enable_speckle);
            frame.points_world.emplace_back(static_cast<float>(p_world.x()),
                                            static_cast<float>(p_world.y()),
                                            static_cast<float>(p_world.z()));
            frame.point_intensities.push_back(intensity);
        }
    }

    frame.sampling.recovered_point_count = frame.points_world.size();
    return frame;
}

} // namespace standalone_mvp
