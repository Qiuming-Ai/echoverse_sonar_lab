#pragma once

#include "AppConfig.hpp"

#include <Eigen/Core>

#include <vector>

namespace standalone_mvp {

struct PathFollowerPose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double pitch = 0.0;
};

class PathFollower {
public:
    void configure(const std::vector<PathWaypointConfig>& waypoints, bool loop, double default_pitch_rad);
    void start();
    void stop();
    bool running() const;
    bool update(double dt_seconds, PathFollowerPose* out_pose);

private:
    Eigen::Vector3d waypointToWorld(const PathWaypointConfig& wp) const;
    bool advanceToNextSegment();
    void writeCurrentPose(PathFollowerPose* out_pose) const;

    std::vector<PathWaypointConfig> waypoints_;
    bool loop_ = false;
    bool running_ = false;
    std::size_t segment_index_ = 0;
    double segment_progress_m_ = 0.0;
    double default_pitch_rad_ = 0.0;
    PathFollowerPose pose_;
};

} // namespace standalone_mvp

