#include "PathFollower.hpp"

#include <algorithm>
#include <cmath>

namespace standalone_mvp {

void PathFollower::configure(const std::vector<PathWaypointConfig>& waypoints, bool loop, double default_pitch_rad) {
    waypoints_ = waypoints;
    loop_ = loop;
    default_pitch_rad_ = default_pitch_rad;
    segment_index_ = 0;
    segment_progress_m_ = 0.0;
    running_ = false;
    if (!waypoints_.empty()) {
        pose_.position = waypointToWorld(waypoints_.front());
    }
}

void PathFollower::start() {
    if (waypoints_.size() < 2) {
        running_ = false;
        return;
    }
    segment_index_ = 0;
    segment_progress_m_ = 0.0;
    pose_.position = waypointToWorld(waypoints_.front());
    running_ = true;
}

void PathFollower::stop() {
    running_ = false;
}

bool PathFollower::running() const {
    return running_;
}

bool PathFollower::update(double dt_seconds, PathFollowerPose* out_pose) {
    if (!running_ || waypoints_.size() < 2) {
        return false;
    }
    double remaining = std::max(0.0, dt_seconds) *
                       std::max(0.001, waypoints_[segment_index_].speed_mps);
    while (remaining > 1e-9 && running_) {
        const Eigen::Vector3d a = waypointToWorld(waypoints_[segment_index_]);
        const Eigen::Vector3d b = waypointToWorld(waypoints_[segment_index_ + 1]);
        const Eigen::Vector3d d = b - a;
        const double len = d.norm();
        if (len < 1e-9) {
            if (!advanceToNextSegment()) {
                running_ = false;
            }
            continue;
        }
        const double segment_left = std::max(0.0, len - segment_progress_m_);
        const double step = std::min(segment_left, remaining);
        segment_progress_m_ += step;
        remaining -= step;

        const Eigen::Vector3d dir = d / len;
        pose_.position = a + dir * segment_progress_m_;
        pose_.yaw = std::atan2(dir.y(), dir.x());
        const double planar = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
        pose_.pitch = std::atan2(dir.z(), std::max(1e-9, planar));

        if (segment_progress_m_ >= len - 1e-9) {
            if (!advanceToNextSegment()) {
                running_ = false;
            }
        }
    }
    if (!running_ && !waypoints_.empty()) {
        pose_.position = waypointToWorld(waypoints_.back());
        pose_.pitch = default_pitch_rad_;
    }
    writeCurrentPose(out_pose);
    return true;
}

Eigen::Vector3d PathFollower::waypointToWorld(const PathWaypointConfig& wp) const {
    return Eigen::Vector3d(wp.x, wp.y, wp.depth_m);
}

bool PathFollower::advanceToNextSegment() {
    if (waypoints_.size() < 2) {
        return false;
    }
    ++segment_index_;
    segment_progress_m_ = 0.0;
    if (segment_index_ + 1 < waypoints_.size()) {
        return true;
    }
    if (loop_) {
        segment_index_ = 0;
        return waypoints_.size() >= 2;
    }
    segment_index_ = waypoints_.size() - 2;
    return false;
}

void PathFollower::writeCurrentPose(PathFollowerPose* out_pose) const {
    if (!out_pose) {
        return;
    }
    *out_pose = pose_;
}

} // namespace standalone_mvp

