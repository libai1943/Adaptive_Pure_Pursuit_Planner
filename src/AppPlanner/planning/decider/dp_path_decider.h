#pragma once
#include <Eigen/Dense>
#include <bitset>
#include <cmath>
#include <limits>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "common/data/discretized_trajectory.h"
#include "common/math/trajectory1d.h"
// #include "common/datatypes.h"
#include "common/environment.h"
#include "common/math/pose.h"
#include "common/math/vec2d.h"
#include "common/visualization_plot.h"
#include "dp_decider_config.h"

namespace dp_path_decider {
using common::math::Polygon2d;
using common::math::Pose;
using common::math::Vec2d;

const double Inf = std::numeric_limits<double>::max();
const double NInf = std::numeric_limits<double>::min();

class DpPathDecider {
 public:
  DpPathDecider() = default;
  DpPathDecider(const DpDeciderConfig &config, Env env);

  bool Plan(const common::data::DiscretizedTrajectory &reference_line,
            common::math::Pose start_pose, common::math::Pose goal_pose,
            double start_v, common::data::Trajectory &trajectory);
  void Clear();
  void Resample(const common::data::DiscretizedTrajectory &org_traj,
                double start_s, double goal_s, const double resolution,
                common::data::Trajectory &result);
  void NewResample(std::vector<double> xx, std::vector<double> yy,
                   const double resolution, common::data::Trajectory &result);

 private:
  struct StateCell {
    double cost = Inf;
    double current_s = NInf;
    int parent_s_ind = -1;
    int parent_l_ind = -1;

    StateCell() = default;
    StateCell(double cost, double cur_s, int parent_s_ind, int parent_l_ind)
        : cost(cost),
          current_s(cur_s),
          parent_s_ind(parent_s_ind),
          parent_l_ind(parent_l_ind) {}
  };

  struct StateIndex {
    int s = -1, l = -1;

    StateIndex() = default;
    StateIndex(int ss, int ll) : s(ss), l(ll) {}
  };

  struct StartState {
    double start_s = 0;
    double start_l = 0;
    double start_theta = 0;
  };
  Env env_;
  DpDeciderConfig config_;
  int NS_;
  int NL_ = 7;
  std::vector<double> station_;
  std::vector<double> lateral_;
  common::data::DiscretizedTrajectory reference_line_;

  StartState state_;
  std::vector<std::vector<StateCell>> state_space_;
  ros::NodeHandle node_handle_;

  // StateCell state_space_[NS][NL];
  common::math::Trajectory1d last_waypoints_;
  double GetCollisionCost(double parent_s, double parent_l, double cur_s,
                          double cur_l);
  std::pair<double, double> GetCost(StateIndex parent_ind, StateIndex cur_ind);

  double GetLateralOffset(double s, int l_ind) {
    if (l_ind == int((NL_)/ 2)) return 0.0;

    double lb = -0.4 ;
    double ub = 0.4;
    return lb + (ub - lb) * lateral_[l_ind];

  }
  common::data::Trajectory GenerateQuinticCurve(
      const common::data::TrajectoryPoint &start,
      const common::data::TrajectoryPoint &end, double step);
  double GetCostFromObsSL(double cur_s, double cur_l);
  std::vector<Vec2d> LinearInterpolate(double parent_s, int parent_l_ind,
                                       int cur_s_ind, int cur_l_ind);
};

}  // namespace dp_path_decider
