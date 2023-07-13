//
// Created by yenkn on 2021/07/28.
//
#pragma once
#include "aabox2d.h"
#include "polygon2d.h"
#include "line_segment2d.h"

namespace common {
namespace math {

class Circle2d {
public:
  Circle2d() = default;
  Circle2d(double x, double y, double r): center_(x, y), r_(r), r_square_(r * r) {}

  inline double min_x() const {
    return center_.x() - r_;
  }

  inline double max_x() const {
    return center_.x() + r_;
  }

  inline double min_y() const {
    return center_.y() - r_;
  }

  inline double max_y() const {
    return center_.y() + r_;
  }

  inline double radius() const {
    return r_;
  }

  inline Vec2d center() const {
    return center_;
  }

  inline bool IsPointIn(const Vec2d &point) const {
    return center_.DistanceSquareTo(point) <= r_square_;
  }

  inline bool HasOverlap(const Polygon2d &box) const {
    if(!AABox2d(center_, r_ * 2, r_ * 2).HasOverlap(box.AABoundingBox())) {
      return false;
    }

    std::vector<Vec2d> corners = box.points();

    for(auto &vec: corners) {
      if(IsPointIn(vec)) {
        return true;
      }
    }

    for(int i = 0; i < corners.size(); i++) {
      if(LineSegment2d(corners[i], corners[(i+1) % corners.size()]).DistanceTo(center_) <= r_) {
        return true;
      }
    }

    return false;
  }

private:
  Vec2d center_;
  double r_, r_square_;
};

}
}
