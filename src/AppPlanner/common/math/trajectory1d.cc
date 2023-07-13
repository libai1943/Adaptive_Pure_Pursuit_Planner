//
// Created by yenkn on 2021/2/26.
//
#include "common/math/trajectory1d.h"
#include "common/math/linear_interpolation.h"
#include <algorithm>

namespace common {
namespace math {

Trajectory1d::Trajectory1d(const std::vector<double> &xs, const std::vector<double> &ys) {
  resize(xs.size());
  for(int i = 0; i < size(); i++) {
    this->at(i) = Vec2d(xs[i], ys[i]);
  }
}

Trajectory1d::Trajectory1d(const std::vector<double> &xs, double y) {
  resize(xs.size());
  for(int i = 0; i < size(); i++) {
    this->at(i) = Vec2d(xs[i], y);
  }
}

double Trajectory1d::Evaluate(double x) const {
  if(x > back().x()) {
    return back().y();
  }

  auto iter = std::lower_bound(begin(), end(), x, [](const Vec2d &a, double xx) {
    return a.x() < xx;
  });

  if(iter == begin()) {
    return iter->y();
  }

  auto prev = std::prev(iter, 1);
  return lerp(prev->y(), prev->x(), iter->y(), iter->x(), x);
}

Trajectory1d Trajectory1d::Interpolate1d(const std::vector<double> &xi) const {
  std::vector<double> ys(xi.size());
  for (size_t i = 0; i < xi.size(); i++) {
    ys[i] = Evaluate(xi[i]);
  }
  return { xi, ys };
}

}
}

