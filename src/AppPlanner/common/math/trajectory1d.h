//
// Created by yenkn on 2021/2/26.
//
#pragma once
#include <vector>
#include "common/math/vec2d.h"

namespace common {
namespace math {

class Trajectory1d: public std::vector<Vec2d> {
public:
  using BaseType = std::vector<Vec2d>;
  Trajectory1d(): BaseType() {}
  explicit Trajectory1d(size_t len): BaseType(len) {}
  Trajectory1d(const std::vector<double> &xs, const std::vector<double> &ys);

  /**
   * Generate constant y
   * @param xs
   * @param y
   */
  Trajectory1d(const std::vector<double> &xs, double y);

  std::vector<double> GetY() {
    std::vector<double> ys;
    for(auto &pt: *this) ys.push_back(pt.y());
    return ys;
  }

  double Evaluate(double x) const;

  Trajectory1d Interpolate1d(const std::vector<double> &xi) const;

private:
};

using SVProfile = Trajectory1d;

}
}
