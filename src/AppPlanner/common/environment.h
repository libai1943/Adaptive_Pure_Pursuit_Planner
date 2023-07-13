//
// Created by yenkn on 2021/09/12.
//
#pragma once
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/util/point_cloud.h"
#include "common/vehicle_param.h"
#include "common/obstacle.h"
#include "common/data/discretized_trajectory.h"

#include <algorithm>

struct Environment {
  std::vector<common::Obstacle> obstacles;
  common::util::PointCloud points;
  VehicleParam vehicle;

  common::data::DiscretizedTrajectory reference;
  std::vector<common::data::DiscretizedTrajectory> guidances;

  int nfe = 100;

  bool CheckPoseCollision(const common::math::Pose &pose, double radius_buffer = 0.0);

  bool CheckCollision(const common::math::Box2d &box);

  void Visualize();

  void Clear() { 
    obstacles.clear();
    points.Clear();
    guidances.clear();
    reference.Clear();
  }
};

using Env = std::shared_ptr<Environment>;
