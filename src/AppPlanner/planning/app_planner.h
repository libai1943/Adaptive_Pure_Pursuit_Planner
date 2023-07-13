// APP C++ version
#include <chrono>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "common/data/discretized_trajectory.h"
#include "common/frame.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/pose.h"
#include "common/math/trajectory1d.h"
#include "common/math/vec2d.h"
#include "common/obstacle.h"
#include "common/util/time.h"
#include "common/util/vector.h"
#include "nlohmann/json.hpp"

using common::math::kMathEpsilon;
using json = nlohmann::json;

namespace common {
namespace math {

inline void to_json(json& j, const Vec2d& p) { j = json{p.x(), p.y()}; }
inline void from_json(const json& j, Vec2d& p) {
  p.set_x(j[0]);
  p.set_y(j[1]);
}

inline void to_json(json& j, const Pose& p) {
  j = json{p.x(), p.y(), p.theta()};
}
inline void from_json(const json& j, Pose& p) {
  p.set_x(j[0]);
  p.set_y(j[1]);
  p.set_theta(j[2]);
}

inline void to_json(json& j, const Polygon2d& p) { j = p.points(); }
inline void from_json(const json& j, Polygon2d& p) {
  p = Polygon2d(j.get<std::vector<Vec2d>>());
}

}  // namespace math
}  // namespace common
class PurePursuitControl {
 public:
  PurePursuitControl(const common::data::Trajectory& input_path,
                     const common::data::TrajectoryPoint& start_state,
                     const common::data::TrajectoryPoint& goal_state,
                     common::data::Trajectory& output_result)
      : followed_path_(input_path), cur_state_(start_state) {
    last_index_ = followed_path_.size() - 1;
    output_result.clear();
    output_result.emplace_back(cur_state_);
    ranges_.emplace_back(0, followed_path_.size());
    while (cur_time_ < MAX_TIME && lookahead_index_ < last_index_ - 1) {
      GetLookaheadIndex();
      ImprovedController(output_result);
      cur_time_ += DT;
    }
  }

 private:
  const double wheel_base_ = 0.1445;
  const double nominal_v_ = 0.1;
  const double max_steer_angle_rate_ = 0.5;
  const double max_steer_angle_ = 0.6;
  const int intergrate_n_ = 5;
  double MAX_TIME = 100;
  double cur_time_ = 0;
  double DT = 0.1;
  int last_index_ = 0;
  int car_index_ = 0;
  int lookahead_index_ = 0;
  double lookahead_distance_ = 0.18;
  double test_pre_delta_ = 0;
  std::vector<std::pair<int, int>> ranges_;
  common::data::TrajectoryPoint cur_state_;
  common::data::Trajectory followed_path_;
  inline double distance_to_traj(int i, double x, double y) {
    return hypot(followed_path_[i].y - y, followed_path_[i].x - x);
  };
  void GetLookaheadIndex();
  void ImprovedController(common::data::Trajectory& output_result);
  void UpdateVehicleState(const double delta, const double dt,
                              common::data::TrajectoryPoint& state);
  void UpdateVehicleState(double delta);
  double Controller();
};
class APP_Planner {
 public:
  APP_Planner(const common::data::Trajectory& input_path,
              const common::data::TrajectoryPoint& start_state,
              const common::data::TrajectoryPoint& goal_state,
              const Frame& frame, const int max_Iter)
      : frame_(frame),
        max_Iter_(max_Iter),
        carrot_path_gloabl_(input_path),
        start_state_(start_state),
        goal_state_(goal_state) {}
  bool Plan(common::data::Trajectory& smooth_path);

 private:
  Frame frame_;
  common::data::Trajectory carrot_path_gloabl_;
  const common::data::TrajectoryPoint start_state_, goal_state_;
  const int inner_iter_max_ = 20;
  const double resample_resolution_ = 0.02;
  const double deform_cost_ = 0.02;
  int max_Iter_ = 0;
  int cur_iter_ = 0;
  double left_buffer_ = 0.5;
  double right_buffer_ = 0.5;
  double single_left_buffer_ = 0.2;
  double single_right_buffer_ = 0.2;
  double traceback_length_ = 0.1;
  void Resample(common::data::Trajectory& sample_path, const double step);
  void Resample(common::data::Trajectory& sample_path, const int N);
  void GetConflictingPointsId(const common::data::Trajectory& path,
                              std::vector<int>& conflict_id);
  void CreateConflictedSegements(
      const std::vector<int>& conflict_id,
      std::vector<std::vector<int>>& conflict_segements);
  void StitchGolbalPath(
      std::vector<std::vector<int>> conflict_segements_id,
      std::vector<common::data::Trajectory> all_deformated_paths);
  bool DeformatedLocalPath(const std::vector<int>& conflict_id,
                           const common::data::Trajectory& global_smooth_path,
                           common::data::Trajectory& deformated_path);
  int GetBoxColNums(const common::math::Pose& pose, const int check_nums,
                    const double check_size);
  int FindMatchCarrotPointId(const common::data::Trajectory& local_carrot_path,
                             const common::data::TrajectoryPoint& cur_waypoint,
                             const double traceback_length);
  double MeasureCollisionRate(const common::data::TrajectoryPoint& point);
  common::math::Vec2d GetDeformatedNor(
      const common::data::TrajectoryPoint& point);
};
