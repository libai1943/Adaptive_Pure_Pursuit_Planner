//
// Created by 欧阳亚坤 on 2021/9/20.
//
#include "point_cloud.h"
#include "common/math/line_segment2d.h"

#include <algorithm>
#include <iostream>

namespace common {
namespace util {

void PointCloud::SetPoints(const std::vector<Vec2d> &points) {
  points_ = points;

  std::sort(points_.begin(), points_.end(), [](const Vec2d &a, const Vec2d &b) {
    return a.x() < b.x();
  });

  min_x_ = points_.front().x();
  max_x_ = points_.back().x();

  min_y_  = max_y_ = points_.front().y();
  for(auto &pt: points_) {
    min_y_ = std::min(pt.y(), min_y_);
    max_y_ = std::max(pt.y(), max_y_);
  }
}

bool PointCloud::CheckInBox(const Box2d &box) const {
  if(points_.empty()) {
    return false;
  }
  if(box.max_x() < points_.front().x() || box.min_x() > points_.back().x()) {
    return false;
  }

  auto comp = [](double val, const Vec2d &a) {
    return val < a.x();
  };

  auto check_start = std::upper_bound(points_.begin(), points_.end(), box.min_x(), comp);
  auto check_end = std::upper_bound(points_.begin(), points_.end(), box.max_x(), comp);

  std::advance(check_start, -1);

  for(auto iter = check_start; iter != check_end; iter++) {
    if(box.IsPointIn(*iter)) {
      return true;
    }
  }

  return false;
}

bool PointCloud::CheckInCircle(const Circle2d &circle) const {
  if(points_.empty()) {
    return false;
  }
  if(circle.max_x() < points_.front().x() || circle.min_x() > points_.back().x()) {
    return false;
  }

  auto comp = [](double val, const Vec2d &a) {
    return val < a.x();
  };

  auto check_start = std::upper_bound(points_.begin(), points_.end(), circle.min_x(), comp);
  auto check_end = std::upper_bound(points_.begin(), points_.end(), circle.max_x(), comp);

  std::advance(check_start, -1);

  for(auto iter = check_start; iter != check_end; iter++) {
    if(circle.IsPointIn(*iter)) {
      return true;
    }
  }

  return false;
}

}
}
