#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sonar_types_v2 {
template <typename T> inline T NaN() { return std::numeric_limits<T>::quiet_NaN(); }
template <typename T> inline bool isNaN(T value) { return std::isnan(value); }
template <typename T> inline T unset() { return std::numeric_limits<T>::quiet_NaN(); }
template <typename T> inline bool isUnset(T value) { return std::isnan(value); }
template <typename T> inline T unknown() { return std::numeric_limits<T>::quiet_NaN(); }
template <typename T> inline bool isUnknown(T value) { return std::isnan(value); }
template <typename T> inline T infinity() { return std::numeric_limits<T>::infinity(); }
template <typename T> inline bool isInfinity(T value) { return std::isinf(value); }

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using VectorXd = Eigen::VectorXd;
using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using MatrixXd = Eigen::MatrixXd;
using Quaterniond = Eigen::Quaterniond;
using AngleAxisd = Eigen::AngleAxisd;
using Affine3d = Eigen::Affine3d;
using Isometry3d = Eigen::Isometry3d;
using Transform3d = Affine3d;

template <typename Derived> inline bool isnotnan(const Eigen::MatrixBase<Derived>& x) {
    return ((x.array() == x.array())).all();
}

template <typename Derived> inline bool isfinite(const Eigen::MatrixBase<Derived>& x) {
    return isnotnan(x - x);
}

class Angle {
public:
    double rad;

    Angle() : rad(sonar_types_v2::unknown<double>()) {}

protected:
    explicit Angle(double _rad) : rad(_rad) { canonize(); }

    void canonize() {
        if (rad > M_PI || rad <= -M_PI) {
            double intp;
            const double side = std::copysign(M_PI, rad);
            rad = -side + 2 * M_PI * std::modf((rad - side) / (2 * M_PI), &intp);
        }
    }

public:
    static inline Angle fromRad(double r) { return Angle(r); }
    static inline Angle fromDeg(double deg) { return Angle(deg / 180.0 * M_PI); }
    static inline Angle unknown() { return Angle(); }
    static inline Angle PI() { return Angle(M_PI); }

    double getRad() const { return rad; }
    double getDeg() const { return rad / M_PI * 180.0; }
    bool isApprox(Angle other, double prec = 1e-5) const {
        return std::fabs(Angle(other.rad - rad).getRad()) < prec;
    }

    bool operator==(const Angle& other) const { return rad == other.rad; }
    bool operator<(const Angle& other) const { return rad < other.rad; }
    bool operator>(const Angle& other) const { return rad > other.rad; }
    bool operator<=(const Angle& other) const { return rad <= other.rad; }
    bool operator>=(const Angle& other) const { return rad >= other.rad; }

    Angle& operator+=(const Angle& other) {
        rad += other.rad;
        canonize();
        return *this;
    }
    Angle& operator-=(const Angle& other) {
        rad -= other.rad;
        canonize();
        return *this;
    }
    Angle operator+(const Angle& other) const { return Angle::fromRad(getRad() + other.getRad()); }
    Angle operator-(const Angle& other) const { return Angle::fromRad(getRad() - other.getRad()); }
    Angle operator*(const Angle& other) const { return Angle::fromRad(getRad() * other.getRad()); }
    Angle operator*(const double& val) const { return Angle::fromRad(getRad() * val); }
};

static inline Angle operator*(double a, Angle b) { return Angle::fromRad(a * b.getRad()); }
inline bool isUnknown(Angle value) { return std::isnan(value.getRad()); }
} // namespace sonar_types_v2
