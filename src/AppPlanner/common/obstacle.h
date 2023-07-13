//
// Created by yenkn on 4/20/22.
//
#pragma once
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
// #include "common/frame.h"
// #include "common/environment.h"
#include "common/util/time.h"
#include "nlohmann/json_fwd.hpp"

namespace common {

class Obstacle {
public:
  Obstacle() = default;
  Obstacle(const math::Polygon2d &polygon, std::vector<math::PoseStamped> &trajectory): polygon_(polygon), trajectory_(trajectory) {}
  explicit Obstacle(const math::Polygon2d &polygon): polygon_(polygon) {}
  explicit Obstacle(const math::Polygon2d &polygon, const bool &is_move): polygon_(polygon), is_move_(is_move) {
  }
  bool is_moving() const {
    return !trajectory_.empty() || is_move_;
  }

  /**
   * calculate polygon states based on current time uses linear interpolation between poses.
   * @return
   */
  math::Polygon2d GetPolygon(double relative_time = 0.0) const;
  math::Pose GetVecPose(double relative_time = 0.0) const;

private:
  math::Polygon2d polygon_;
  std::vector<math::PoseStamped> trajectory_;
  bool is_move_ = false;

  friend void to_json(nlohmann::json& j, const Obstacle& p);
  friend void from_json(const nlohmann::json& j, Obstacle& p);
};

}