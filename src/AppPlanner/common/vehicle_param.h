#pragma once
#include <tuple>

#include "common/math/box2d.h"
#include "common/math/circle_2d.h"
#include "common/math/pose.h"

class VehicleParam {
 public:
  double rear_hang = 0.032;  // 0.929;
  double front_hang = 0.036;
  // the distance between the front and back wheels
  double wheel_base = 0.143;
  double length = 0.211;
  // double width = 0.191; + 0.05
  double width = 0.211;
  double max_acceleration = 0.1;
  double max_deceleration = -0.1;

  double max_velocity = 0.25;
  // set to zero is reverse is disabled
  double max_reverse_velocity = 0.0;

  // vehicle max steer angle
  double max_steer_angle = 0.6;
  double max_backward_steer_angle = 0.6;
  // vehicle max steer rate; how fast can the steering wheel turn.
  double max_steer_angle_rate = 0.5;
  // ratio between the turn of steering wheel and the turn of wheels
  double steer_ratio = 1;  // 16

  double radius;
  double buffer = 0.02;
  double c2x;

  void GenerateDiscs() {
    radius = hypot(length * 0.5, width * 0.5) + buffer;
    c2x = 0.5 * length - rear_hang;
  }

  // template<class T>
  // std::tuple<T, T, T, T> GetDiscPositions(const T &x, const T &y, const T
  // &theta) const {
  //   auto xf = x + f2x * cos(theta);
  //   auto xr = x + r2x * cos(theta);
  //   auto yf = y + f2x * sin(theta);
  //   auto yr = y + r2x * sin(theta);
  //   return std::make_0tuple(xf, yf, xr, yr);
  // }

  // std::tuple<common::math::Circle2d, common::math::Circle2d> GetDiscs(double
  // x, double y, double theta) const {
  //   double xf, yf, xr, yr;
  //   std::tie(xf, yf, xr, yr) = GetDiscPositions(x, y, theta);
  //   return std::make_tuple(common::math::Circle2d(xf, yf, radius),
  //   common::math::Circle2d(xr, yr, radius));
  // }

  common::math::Box2d GenerateBox(const common::math::Pose &pose) const {
    double distance = length / 2 - rear_hang;
    return {pose.extend(distance), pose.theta(), length, width+0.05};
  }
  common::math::Box2d GenerateHalfBox(const common::math::Pose &pose) const {
    double distance = length / 2 - rear_hang;
    return {pose.extend(distance), pose.theta(), length, width/2};
  }
};