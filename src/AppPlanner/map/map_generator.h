#pragma once

#include <vector>

#include "common/data/discretized_trajectory.h"
#include "common/math/polygon2d.h"
#include "common/math/quintic_spiral_path.h"
#include "common/math/vec2d.h"
#include "common/util/color.h"
#include "common/util/vector.h"
#include "common/visualization_plot.h"
#include "common/obstacle.h"

namespace map_generator {

struct Road {
  common::data::Trajectory points;
  std::vector<int> next_seg;
};

class MapGenerator {
 public:
  MapGenerator();
  void GenerateMap();
  void GenerateBlocks();
  void GenerateRoadNodes();
  void GenerateRoads();
  void GenerateCurves();
  void Visualize();

  void road(int ind, Road &result) const {
      result = roads_[ind];
  }

  const std::vector<Road> roads() const { return roads_; }

  std::vector<common::Obstacle> block() const { return blocks_; }
  std::vector<common::data::TrajectoryPoint> road_node() const { return road_node_; }
  std::vector<common::data::Trajectory> road_segment() const { return road_segment_; }
  std::tuple<double, double, double, double> cross() const { return cross_; }

 private:
  double map_width_;
  double map_height_;

  double road_width_;
  double block_width_;
  double block_height_;
  double min_turn_radiu_;
  double x_straight_;
  double y_straight_;
  std::tuple<double, double, double, double> cross_;

  std::vector<common::Obstacle> blocks_;
  std::vector<common::math::Vec2d> blocks_points_;

  std::vector<common::data::TrajectoryPoint> road_node_;
  std::vector<std::vector<common::data::TrajectoryPoint>> spirit_curve_;
  std::vector<common::data::Trajectory> road_segment_;

  std::vector<Road> roads_;
};

}  // namespace map_generator