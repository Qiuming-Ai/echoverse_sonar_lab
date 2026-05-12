#pragma once

#include <string>

#include <sonar_types_v2/echoverse_math_types.hpp>
#include <sonar_types_v2/echoverse_time_types.hpp>

namespace sonar_types_v2 {
using Position = Vector3d;
using Orientation = Quaterniond;
using Position2D = Vector2d;
using Orientation2D = double;

inline double getYaw(const Orientation& orientation) {
    return orientation.toRotationMatrix().canonicalEulerAngles(2, 1, 0)[0];
}

struct Pose {
    Position position;
    Orientation orientation;

    Pose() : position(Position::Zero()), orientation(Orientation::Identity()) {}
    Pose(Position const& p, Orientation const& o) : position(p), orientation(o) {}

    double getYaw() const { return sonar_types_v2::getYaw(orientation); }
};

struct Pose2D {
    Position2D position;
    Orientation2D orientation;

    Pose2D() : position(Position2D::Zero()), orientation(0) {}
    Pose2D(Position2D const& p, Orientation2D const& o) : position(p), orientation(o) {}
};

namespace samples {
struct RigidBodyState {
    RigidBodyState() = default;

    sonar_types_v2::Time time;
    std::string sourceFrame;
    std::string targetFrame;
    sonar_types_v2::Position position = sonar_types_v2::Position::Zero();
    sonar_types_v2::Orientation orientation = sonar_types_v2::Orientation::Identity();
    sonar_types_v2::Vector3d velocity = sonar_types_v2::Vector3d::Zero();
    sonar_types_v2::Vector3d angular_velocity = sonar_types_v2::Vector3d::Zero();

    sonar_types_v2::Vector3d& linearVelocity() { return velocity; }
    const sonar_types_v2::Vector3d& linearVelocity() const { return velocity; }
};
} // namespace samples
} // namespace sonar_types_v2
