//
// Created by W YZ on 2023/3/5.
//

#include "dp_path_decider.h"

#include <bitset>
#include <iostream>
#include <utility>

#include "common/math/polygon2d.h"
#include "common/util/vector.h"
namespace dp_path_decider {
using namespace common::util;
using common::math::Polygon2d;

constexpr double kMathEpsilon = 1e-3;

DpPathDecider::DpPathDecider(const DpDeciderConfig &config, Env env)
    : env_(std::move(env)), config_(config) {
}
double DpPathDecider::GetCollisionCost(double parent_s, double parent_l,
                                       double cur_s, double cur_l) {
  common::data::TrajectoryPoint start_sl_plot;
  start_sl_plot.x = parent_s;
  start_sl_plot.y = parent_l;
  start_sl_plot.theta = 0.0;
  if (fabs(parent_s - state_.start_s) < kMathEpsilon) {
    start_sl_plot.theta =
        state_.start_theta - reference_line_.EvaluateStation(parent_s).theta;
  }

  common::data::TrajectoryPoint goal_sl_plot;
  goal_sl_plot.x = cur_s;
  goal_sl_plot.y = cur_l;
  goal_sl_plot.theta = 0.0;

  auto path = GenerateQuinticCurve(start_sl_plot, goal_sl_plot,
                                   config_.collision_check_resolution);

  double cost = 0.0;
  for (auto &pt : path) {
    auto cart = reference_line_.ToCartesian(pt.x, pt.y);
    auto ref = reference_line_.EvaluateStation(pt.x);
    double theta = ref.theta + pt.theta;
    auto vec_box = env_->vehicle.GenerateBox(
        common::math::Pose({cart.x(), cart.y(), theta}));
    for (auto &obstacle : env_->obstacles) {
      cost += exp(-cart.DistanceTo(obstacle.GetPolygon().center()) /
                  config_.w_obstacle_evasion_exp_denom);

      if (obstacle.GetPolygon().HasOverlap(vec_box)) {
        return config_.w_obstacle;
      }
    }
  }
  return cost;
}
void DpPathDecider::Clear() {
  station_.clear();
  lateral_.clear();
  common::data::DiscretizedTrajectory new_line;
  reference_line_ = new_line;
  state_ = StartState();
  state_space_.clear();
}
std::pair<double, double> DpPathDecider::GetCost(StateIndex parent_ind,
                                                 StateIndex cur_ind) {
  double parent_s = state_.start_s, grandparent_s = state_.start_s;
  double parent_l = state_.start_l, grandparent_l = state_.start_l;

  if (parent_ind.s >= 0) {
    auto &cell = state_space_[parent_ind.s][parent_ind.l];
    int grandparent_s_ind = cell.parent_s_ind;
    int grandparent_l_ind = cell.parent_l_ind;
    parent_s = cell.current_s;
    parent_l = GetLateralOffset(parent_s, parent_ind.l);

    if (parent_ind.s >= 1) {
      grandparent_s =
          state_space_[grandparent_s_ind][grandparent_l_ind].current_s;
      grandparent_l = GetLateralOffset(grandparent_s, grandparent_l_ind);
    }
  }
  double cur_s = state_.start_s + station_[cur_ind.s];
  double cur_l = GetLateralOffset(cur_s, cur_ind.l);
  double ds1 = cur_s - parent_s;
  double dl1 = cur_l - parent_l;
  double ds0 = parent_s - grandparent_s;
  double dl0 = parent_l - grandparent_l;
  double cost_obstacle = GetCollisionCost(parent_s, parent_l, cur_s, cur_l);
  // quintic curve
  common::data::TrajectoryPoint start_sl;
  start_sl.x = parent_s;
  start_sl.y = parent_l;
  start_sl.theta = 0.0;
  if (fabs(parent_s - state_.start_s) < kMathEpsilon) {
    start_sl.theta =
        state_.start_theta - reference_line_.EvaluateStation(parent_s).theta;
  }

  common::data::TrajectoryPoint goal_sl;
  goal_sl.x = cur_s;
  goal_sl.y = cur_l;
  goal_sl.theta = 0.0;
  double cost_near_guidance = 0.0;
  cost_near_guidance = fabs(cur_l + parent_l);
  double delta_cost =
      (cost_obstacle + config_.w_lateral * cost_near_guidance);

  return std::make_pair(cur_s, delta_cost);
}

bool DpPathDecider::Plan(
    const common::data::DiscretizedTrajectory &reference_line,
    common::math::Pose start_pose, common::math::Pose goal_pose, double start_v,
    common::data::Trajectory &path) {
  Clear();
  reference_line_ = reference_line;
  auto start_proj =
      reference_line_.GetProjection({start_pose.x(), start_pose.y()});
  auto goal_proj =
      reference_line_.GetProjection({goal_pose.x(), goal_pose.y()});
  double horizon_to_goal = goal_proj.x() - start_proj.x();
  double horizon =
      horizon_to_goal <= config_.horizon ? horizon_to_goal : config_.horizon;

  NS_ = ceil(horizon / config_.s_resolution);
  if (NS_ < 2) {
    std::cout << "Num of sample points is less than 2" << std::endl;
    return false;
  }
  station_ = common::util::LinSpaced(horizon / NS_, horizon, NS_);
  lateral_ = common::util::LinSpaced(0.0, 1.0, NL_);
  state_.start_s = start_proj.x();
  state_.start_l = start_proj.y();
  state_.start_theta = start_pose.theta();

  // reset state space
  for (int i = 0; i < NS_; i++) {
    auto col_state = std::vector<StateCell>(NL_);
    state_space_.push_back(col_state);
  }

  // evaluate first layer
  state_space_[0][int(NL_ / 2)].current_s = state_.start_s;
  state_space_[0][int(NL_ / 2)].cost = 0;

  // dynamic programming
  double a = ((double)duration_cast<milliseconds>(
                  system_clock::now().time_since_epoch())
                  .count());
  for (int i = 0; i < NS_ - 1; i++) {
    for (int j = 0; j < NL_; j++) {
      StateIndex parent_ind(i, j);
      for (int k = 0; k < NL_; k++) {
        StateIndex current_ind(i + 1, k);
        auto tup = GetCost(parent_ind, current_ind);
        double delta_cost = tup.second;
        double cur_s = tup.first;
        double cur_cost = state_space_[i][j].cost + delta_cost;
        if (cur_cost < state_space_[i + 1][k].cost) {
          state_space_[i + 1][k] = StateCell(cur_cost, cur_s, i, j);
        }
      }
    }
  }
  double b = ((double)duration_cast<milliseconds>(
                  system_clock::now().time_since_epoch())
                  .count());
  // std::cout << "dynamic programming time:" << b - a << std::endl;
  // find the least cost in final layer
  double min_cost = Inf;
  int min_s_ind = 0, min_l_ind = 0;
  for (int i = 0; i < NL_; i++) {
    double cost = state_space_[NS_ - 1][i].cost;
    if (state_space_[NS_ - 1][i].cost < min_cost) {
      min_l_ind = i;
      min_cost = cost;
    }
  }

  std::vector<std::pair<StateIndex, StateCell>> waypoints(NS_);

  // trace back layers to find optimum trajectory
  for (int i = NS_ - 1; i >= 0; i--) {
    auto &cell = state_space_[i][min_l_ind];
    waypoints[i] = std::make_pair(StateIndex(i, min_l_ind), cell);
    min_l_ind = cell.parent_l_ind;
  }

  // interpolation CHANGE
  path.clear();
  double angle = 0.0, last = state_.start_theta;

  common::data::Trajectory tmp_traj;
  std::vector<double> xs, ys, thetas;
  for (int i = 0; i < NS_; i++) {
    double parent_s = state_.start_s;
    double parent_l = state_.start_l;
    if (i > 0) {
      parent_s = waypoints[i - 1].second.current_s;
      parent_l = GetLateralOffset(parent_s, waypoints[i].second.parent_l_ind);
    }
    double cur_s = state_.start_s + station_[i];
    double cur_l = GetLateralOffset(cur_s, waypoints[i].first.l);

    common::data::TrajectoryPoint start_sl_plot;
    start_sl_plot.x = parent_s;
    start_sl_plot.y = parent_l;
    start_sl_plot.theta = 0.0;
    if (i == 0) {
      start_sl_plot.theta =
          state_.start_theta - reference_line_.EvaluateStation(parent_s).theta;
    }
    common::data::TrajectoryPoint goal_sl_plot;
    goal_sl_plot.x = cur_s;
    goal_sl_plot.y = cur_l;
    goal_sl_plot.theta = 0.0;
    auto xy = reference_line_.ToCartesian(cur_s, cur_l);
    xs.push_back(xy.x());
    ys.push_back(xy.y());
 
  }
  NewResample(xs, ys, 0.2, path);
  VisualizationPlot::Plot(xs, ys, 0.01, Color::Blue, 6, "dap_result");
  last_waypoints_.clear();
  for (auto &pt : path) {
    auto sl = reference_line_.GetProjection(Vec2d(pt.x, pt.y));
    last_waypoints_.emplace_back(Vec2d(sl.x(), sl.y()));
  }

  return true;
}
void DpPathDecider::NewResample(std::vector<double> xx, std::vector<double> yy,
                                const double resolution,
                                common::data::Trajectory &result) {
  int N = 20;
  result.clear();
  for (int i = 0; i < xx.size() - 1; i++) {
    auto resample_xx = common::util::LinSpaced(xx[i], xx[i + 1], N);
    auto resample_yy = common::util::LinSpaced(yy[i], yy[i + 1], N);
    for (int j = 0; j < resample_xx.size(); j++) {
      common::data::TrajectoryPoint tp;
      tp.x = resample_xx[j];
      tp.y = resample_yy[j];
      result.push_back(tp);
    }
  }
}
common::data::Trajectory DpPathDecider::GenerateQuinticCurve(
    const common::data::TrajectoryPoint &start,
    const common::data::TrajectoryPoint &end, double step) {
  double x0 = start.x;
  double y0 = start.y;
  double theta0 = start.theta;
  double x1 = end.x;
  double y1 = end.y;
  double theta1 = end.theta;
  double k1 = 0;

  while (theta0 < -M_PI / 2.0) {
    theta0 += M_PI;
  }
  while (theta0 > M_PI / 2.0) {
    theta0 -= M_PI;
  }

  while (theta1 < -M_PI / 2.0) {
    theta1 += M_PI;
  }
  while (theta1 > M_PI / 2.0) {
    theta1 -= M_PI;
  }

  theta0 = tan(theta0);
  theta1 = tan(theta1);
  double distance = hypot(x1 - x0, y1 - y0);
  int n_tp = int(distance / step);
  auto xs = common::util::LinSpaced(x0, x1, n_tp);
  auto ys = common::util::LinSpaced(y0, y1, n_tp);
  common::data::Trajectory path;
  common::data::TrajectoryPoint tp;
  for (size_t i = 0; i < xs.size(); i++) {
    tp.x = xs[i];
    tp.y = ys[i];
    tp.theta = atan2(y1 - y0, x1 - x0);
    path.push_back(tp);
  }
  return path;
}

}  // namespace dp_path_decider
