//
// Created by yenkn on 2021/2/24.
//
#pragma once
#include "vec2d.h"

namespace common {
namespace math {

class Pose {
public:
  Pose() = default;
  Pose(double x, double y, double theta): x_(x), y_(y), theta_(theta) {}

  inline double x() const { return x_; }
  inline double y() const { return y_; }
  inline double theta() const { return theta_; }

  inline void set_x(double x) { x_ = x; }
  inline void set_y(double y) { y_ = y; }
  inline void set_xy(double x, double y) { x_ = x; y_ = y; }
  inline void set_theta(double theta) { theta_ = theta; }

  inline operator Vec2d() const { return { x_, y_}; }

  inline bool operator==(const Pose &rhs) const {
    return std::abs(x_ - rhs.x_) < kMathEpsilon && std::abs(y_ - rhs.y_) < kMathEpsilon
      && std::abs(theta_ - rhs.theta_) < kMathEpsilon;
  }

  inline bool operator!=(const Pose &rhs) const {
    return !(*this == rhs);
  }

  inline Pose relativeTo(const Pose &coord) const {
    double dx = x_ - coord.x();
    double dy = y_ - coord.y();
    return {
      dx * cos(coord.theta()) + dy * sin(coord.theta()),
      -dx * sin(coord.theta()) + dy * cos(coord.theta()),
      theta_ - coord.theta()
    };
  }

  inline Pose extend(double length) const {
    return transform({ length, 0, 0 });
  }

  inline Pose transform(const Pose &relative) const {
    return {
      x_ + relative.x() * cos(theta_) - relative.y() * sin(theta_),
      y_ + relative.x() * sin(theta_) + relative.y() * cos(theta_),
      theta_ + relative.theta_
    };
  }

  inline double distanceSquareTo(const Pose &rhs) const {
    double dx = x_ - rhs.x(), dy = y_ - rhs.y();
    return dx * dx + dy * dy;
  }

private:
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
};

struct PoseStamped {
  double time = 0.0;
  math::Pose pose;

  PoseStamped() = default;
  PoseStamped(double time, math::Pose pose): time(time), pose(pose) {}
};

}
}

