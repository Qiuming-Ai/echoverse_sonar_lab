#pragma once

#include "PointCloudPolarFrame.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <osg/Group>
#include <osg/Vec3f>
#include <osg/ref_ptr>

#include <sonar_imaging/OffscreenCaptureRig.hpp>
#include <sonar_imaging/SceneDepthComposer.hpp>

namespace standalone_mvp {

struct PointCloudSonarConfig {
    bool enabled = true;
    double range_m = 30.0; // clamped to [0, 100]
    double frequency_khz = 300.0; // currently used by attenuation model in geometric recovery path.
    double bandwidth_khz = 60.0;  // reserved for future intensity/noise modeling extensions.
    double horizontal_angle_resolution_deg = 0.75;
    double vertical_angle_resolution_deg = 0.75;
    double horizontal_fov_deg = 120.0;
    double vertical_fov_deg = 20.0;
    std::size_t max_point_count = 120000;
    int palette_index = 1; // Matches Rock SonarWidget default: 0 Jet, 1 Hot, 2 Gray, 3 Bronze.
    bool show_coordinate_overlay = true;
    bool tcp_output_enabled = false;
    bool file_output_enabled = false;
    std::string tcp_host = "0.0.0.0";
    std::uint16_t tcp_port = 30001;
    std::string file_output_path;

    // Environment parameters kept for acoustic extension compatibility.
    bool enable_attenuation = true;
    bool enable_reverb = true;
    bool enable_speckle = true; // reserved for future point intensity/noise extension.
    double temperature_c = 25.0;
    double salinity_ppt = 0.0;
    double acidity_ph = 8.0;
    double depth_m = 0.0;
    double attenuation_frequency_khz = 300.0;
    double sound_speed_mps = 1500.0;
};

struct PointCloudSamplingInfo {
    std::size_t requested_horizontal_samples = 0;
    std::size_t requested_vertical_samples = 0;
    std::size_t horizontal_samples = 0;
    std::size_t vertical_samples = 0;
    std::size_t requested_point_count = 0;
    std::size_t max_point_count = 0;
    std::size_t budgeted_point_count = 0;
    std::size_t recovered_point_count = 0;
    bool clamped_by_budget = false;
    bool invalid_resolution_was_fixed = false;
    std::string rounding_policy = "floor";
};

struct PointCloudFrame {
    std::vector<osg::Vec3f> points_world;
    std::vector<float> point_intensities;
    std::uint64_t timestamp_us = 0;
    Eigen::Vector3d pose_position_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d pose_forward_world = Eigen::Vector3d::UnitX();
    Eigen::Matrix3d pose_rotation_world = Eigen::Matrix3d::Identity();
    PointCloudSamplingInfo sampling;
    PointCloudSonarConfig config;
    PointCloudPolarFrame polar_frame;
    std::string coordinate_frame = "world";
};

class PointCloudSonarSimulation {
public:
    PointCloudSonarSimulation(const PointCloudSonarConfig& config, osg::ref_ptr<osg::Group> shared_scene_root);

    void setConfig(const PointCloudSonarConfig& config);
    PointCloudSonarConfig config() const;

    PointCloudFrame simulatePointCloud(const Eigen::Affine3d& sonar_pose_world);

private:
    PointCloudSamplingInfo computeSamplingInfo(PointCloudSonarConfig& cfg) const;
    void rebuildCaptureToolIfNeeded(const PointCloudSamplingInfo& sampling, const PointCloudSonarConfig& cfg);
    void updateCapturePose(const Eigen::Affine3d& pose_world);

    osg::ref_ptr<osg::Group> shared_root_;
    PointCloudSonarConfig config_;
    sonar_imaging::SceneDepthComposer depth_composer_;
    sonar_imaging::OffscreenCaptureRig capture_tool_;

    std::size_t capture_height_ = 0;
    double capture_hfov_deg_ = 0.0;
    double capture_vfov_deg_ = 0.0;
};

} // namespace standalone_mvp
