//
// Created by tsq on 10/28/22.
//

#pragma once
#include <fstream>
#include <memory>

#include "common/math/vec2d.h"
#include "common/math/pose.h"
#include "common/math/polygon2d.h"
#include "common/data/discretized_trajectory.h"
#include "common/environment.h"
#include "visualization_plot.h"
// #include "planning/optimizer/trajectory_nlp.h"

/**
 * Planning Frame
 * @brief contains all parameters for single planning cycle
 */

struct Frame {
public:
  Frame() {
      env_ = std::make_shared<Environment>();
  }

  const Env &env() const {
    return env_;
  }

  const common::data::TrajectoryPoint &current() const {
    return current_;
  }

  void set_current(const common::data::TrajectoryPoint &tp) {
    current_ = tp;
  }

  const common::data::TrajectoryPoint &goal() const {
    return goal_;
  }

  void set_goal(const common::data::TrajectoryPoint &tp) {
    goal_ = tp;
  }

  static Frame Read(const std::string &env_json);

  void Save(const std::string &env_file) const;

  void CollectResult(const std::string &env_file, const std::string &tag, double run_time) const;

  void Visualize() const {
    env_->Visualize();
    VisualizationPlot::PlotPose({current_.x, current_.y, current_.theta}, 2.0, common::util::Color::Red, 1, "Start Pose");
    VisualizationPlot::PlotPolygon(common::math::Polygon2d(env_->vehicle.GenerateBox({current_.x, current_.y, current_.theta})), 0.1, Color::Cyan, 1, "current_vehicle");
    // VisualizationPlot::PlotPolygon(common::math::Polygon2d(env_->vehicle.GetDiscs))

    VisualizationPlot::PlotPose({goal_.x, goal_.y, goal_.theta}, 2.0, common::util::Color::Green, 1, "Goal Pose");
    VisualizationPlot::Trigger();
  }
  
  std::tuple<double, double, double, double> cross;

  std::vector<common::data::Trajectory> ori_trajs;
  double nominal_v;
private:
  Env env_;
  
  common::data::TrajectoryPoint current_, goal_;
};