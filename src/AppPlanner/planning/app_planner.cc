#include "app_planner.h"

bool APP_Planner::Plan(common::data::Trajectory& smooth_path) {
  smooth_path.clear();
  Resample(carrot_path_gloabl_, resample_resolution_);
  while (cur_iter_ < max_Iter_) {
    PurePursuitControl pp_path(carrot_path_gloabl_, start_state_, goal_state_,
                               smooth_path);
    int n = carrot_path_gloabl_.size();
    Resample(smooth_path, n);
    std::vector<int> conflict_id = {};
    GetConflictingPointsId(smooth_path, conflict_id);
    if (conflict_id.empty()) {
      return true;
    }
    std::vector<std::vector<int>> conflict_segements_id;
    CreateConflictedSegements(conflict_id, conflict_segements_id);
    common::data::Trajectory deformated_carrot_path;
    std::vector<common::data::Trajectory> all_deformated_paths;
    for (auto seg : conflict_segements_id) {
      if (!DeformatedLocalPath(seg, smooth_path, deformated_carrot_path)) {
        break;
      }
      all_deformated_paths.emplace_back(deformated_carrot_path);
    }
    StitchGolbalPath(conflict_segements_id, all_deformated_paths);
    left_buffer_ += single_left_buffer_;
    right_buffer_ += single_right_buffer_;
    cur_iter_ += 1;
  }
  return false;
}
void APP_Planner::Resample(common::data::Trajectory& sample_path,
                           const double step) {
  double s_integral = 0;
  std::vector<double> xx, yy, thetas, ss;
  for (int i = 0; i < sample_path.size(); i++) {
    xx.emplace_back(sample_path[i].x);
    yy.emplace_back(sample_path[i].y);
    thetas.emplace_back(sample_path[i].theta);
    if (i < sample_path.size() - 1) {
      ss.emplace_back(s_integral);
      s_integral += std::hypot(sample_path[i + 1].y - sample_path[i].y,
                               sample_path[i + 1].x - sample_path[i].x);

    } else {
      ss.emplace_back(s_integral);
    }
  }
  int N = s_integral / step;
  auto time_sampled = common::util::LinSpaced(0, s_integral, N);
  auto new_x =
      common::math::Trajectory1d(ss, xx).Interpolate1d(time_sampled).GetY();
  auto new_y =
      common::math::Trajectory1d(ss, yy).Interpolate1d(time_sampled).GetY();
  auto new_theta =
      common::math::ToContinuousAngle(common::math::Trajectory1d(ss, thetas)
                                          .Interpolate1d(time_sampled)
                                          .GetY());
  sample_path.clear();
  for (int i = 0; i < new_x.size(); i++) {
    common::data::TrajectoryPoint pt;
    pt.x = new_x[i];
    pt.y = new_y[i];
    pt.theta = new_theta[i];
    sample_path.emplace_back(pt);
  }
}
void APP_Planner::Resample(common::data::Trajectory& sample_path, const int N) {
  double s_integral = 0;
  std::vector<double> xx, yy, thetas, ss;
  for (int i = 0; i < sample_path.size(); i++) {
    xx.emplace_back(sample_path[i].x);
    yy.emplace_back(sample_path[i].y);
    thetas.emplace_back(sample_path[i].theta);
    if (i < sample_path.size() - 1) {
      ss.emplace_back(s_integral);
      s_integral += std::hypot(sample_path[i + 1].y - sample_path[i].y,
                               sample_path[i + 1].x - sample_path[i].x);

    } else {
      ss.emplace_back(s_integral);
    }
  }
  auto time_sampled = common::util::LinSpaced(0, s_integral, N);
  auto new_x =
      common::math::Trajectory1d(ss, xx).Interpolate1d(time_sampled).GetY();
  auto new_y =
      common::math::Trajectory1d(ss, yy).Interpolate1d(time_sampled).GetY();
  auto new_theta =
      common::math::ToContinuousAngle(common::math::Trajectory1d(ss, thetas)
                                          .Interpolate1d(time_sampled)
                                          .GetY());
  sample_path.clear();
  for (int i = 0; i < new_x.size(); i++) {
    common::data::TrajectoryPoint pt;
    pt.x = new_x[i];
    pt.y = new_y[i];
    pt.theta = new_theta[i];
    sample_path.emplace_back(pt);
  }
}
void APP_Planner::StitchGolbalPath(
    std::vector<std::vector<int>> conflict_segements_id,
    std::vector<common::data::Trajectory> all_deformated_paths) {
  for (int i = 0; i < conflict_segements_id.size(); ++i) {
    for (int j = 0; j < conflict_segements_id[i].size(); ++j) {
      carrot_path_gloabl_[conflict_segements_id[i][j]] =
          all_deformated_paths[i][j];
    }
  }
}
bool APP_Planner::DeformatedLocalPath(
    const std::vector<int>& conflict_id,
    const common::data::Trajectory& global_smooth_path,
    common::data::Trajectory& deformated_path) {
  deformated_path.clear();
  common::data::Trajectory single_smooth_path;
  for (auto i : conflict_id) {
    deformated_path.emplace_back(carrot_path_gloabl_[i]);
    single_smooth_path.emplace_back(global_smooth_path[i]);
  }
  for (int iter = 0; iter < inner_iter_max_; ++iter) {
    for (int i = 0; i < single_smooth_path.size(); ++i) {
      common::math::Vec2d nor = GetDeformatedNor(single_smooth_path[i]);
      if (fabs(nor.x()) < 0.0001 && fabs(nor.y()) < 0.0001) {
      } else {
        common::data::TrajectoryPoint right, left;
        double right_x =
            single_smooth_path[i].x - sin(single_smooth_path[i].theta) *
                                          frame_.env()->vehicle.width * 0.25;
        double right_y =
            single_smooth_path[i].y + cos(single_smooth_path[i].theta) *
                                          frame_.env()->vehicle.width * 0.25;
        right.x = right_x;
        right.y = right_y;
        left.theta = right.theta = single_smooth_path[i].theta;
        double right_rate = MeasureCollisionRate(right);
        double left_x =
            single_smooth_path[i].x + sin(single_smooth_path[i].theta) *
                                          frame_.env()->vehicle.width * 0.25;
        double left_y =
            single_smooth_path[i].y - cos(single_smooth_path[i].theta) *
                                          frame_.env()->vehicle.width * 0.25;
        left.x = left_x;
        left.y = left_y;
        double left_rate = MeasureCollisionRate(left);
        double traceback_length =
            fabs(left_rate - right_rate) * traceback_length_;
        int index = FindMatchCarrotPointId(
            deformated_path, single_smooth_path[i], traceback_length);
        deformated_path[index].x += deform_cost_ * nor.x();
        deformated_path[index].y += deform_cost_ * nor.y();
      }
    }
    PurePursuitControl pp_path(deformated_path, deformated_path.front(),
                               deformated_path.back(), single_smooth_path);
    Resample(single_smooth_path, int(deformated_path.size()));
    std::vector<int> conflict_id = {};
    GetConflictingPointsId(single_smooth_path, conflict_id);
    if (conflict_id.empty()) {
      return true;
    }
  }
  return false;
}
double APP_Planner::MeasureCollisionRate(
    const common::data::TrajectoryPoint& point) {
  int counter = 0;
  int check_nums = 10;
  double check_size = 0.05;
  // left
  double left_x =
      point.x + sin(point.theta) * frame_.env()->vehicle.width * 0.25;
  double left_y =
      point.y - cos(point.theta) * frame_.env()->vehicle.width * 0.25;
  Pose left_down = Pose(left_x, left_y, point.theta)
                       .extend(-1 * frame_.env()->vehicle.rear_hang);
  int left_counter = GetBoxColNums(left_down, check_nums, check_size);
  counter = counter + left_counter;

  // middle
  Pose middle_down = Pose(point.x, point.y, point.theta)
                         .extend(-1 * frame_.env()->vehicle.rear_hang);
  int middle_counter = GetBoxColNums(middle_down, check_nums, check_size);
  counter = counter + middle_counter;

  // right
  double right_x =
      point.x - sin(point.theta) * frame_.env()->vehicle.width * 0.25;
  double right_y =
      point.y + cos(point.theta) * frame_.env()->vehicle.width * 0.25;
  Pose right_down = Pose(right_x, right_y, point.theta)
                        .extend(-1 * frame_.env()->vehicle.rear_hang);
  int right_counter = GetBoxColNums(right_down, check_nums, check_size);

  counter = counter + right_counter;

  return counter / (3 * check_nums);
}
int APP_Planner::GetBoxColNums(const common::math::Pose& pose,
                               const int check_nums, const double check_size) {
  int counter = 0;
  for (int i = 0; i < check_nums; ++i) {
    common::math::Box2d box = {
        pose.extend(frame_.env()->vehicle.length * i / 10), pose.theta(),
        check_size, check_size};
    if (frame_.env()->CheckCollision(box)) {
      counter++;
    }
  }
  return counter;
}
int APP_Planner::FindMatchCarrotPointId(
    const common::data::Trajectory& local_carrot_path,
    const common::data::TrajectoryPoint& cur_waypoint,
    const double traceback_length) {
  double min_distance = hypot(cur_waypoint.x - local_carrot_path[0].x,
                              cur_waypoint.y - local_carrot_path[0].y);
  int cur_index = 0;
  for (int i = 0; i < local_carrot_path.size(); i++) {
    double distance = hypot(cur_waypoint.x - local_carrot_path[i].x,
                            cur_waypoint.y - local_carrot_path[i].y);
    if (distance < min_distance) {
      cur_index = i;
      min_distance = distance;
    }
  }
  int res_index = 0;
  for (int i = cur_index; i >= 0; --i) {
    double distance = hypot(cur_waypoint.x - local_carrot_path[i].x,
                            cur_waypoint.y - local_carrot_path[i].y);
    if (distance > traceback_length) {
      res_index = i;
      break;
    }
  }
  return res_index;
}
common::math::Vec2d APP_Planner::GetDeformatedNor(
    const common::data::TrajectoryPoint& point) {
  double right_x =
      point.x - sin(point.theta) * frame_.env()->vehicle.width * 0.25;
  double right_y =
      point.y + cos(point.theta) * frame_.env()->vehicle.width * 0.25;
  double left_x =
      point.x + sin(point.theta) * frame_.env()->vehicle.width * 0.25;
  double left_y =
      point.y - cos(point.theta) * frame_.env()->vehicle.width * 0.25;
  auto right_box = frame_.env()->vehicle.GenerateHalfBox(
      Pose(right_x, right_y, point.theta));
  auto left_box =
      frame_.env()->vehicle.GenerateHalfBox(Pose(left_x, left_y, point.theta));
  if (frame_.env()->CheckCollision(right_box)) {
    return {sin(point.theta), -cos(point.theta)};
  } else if (frame_.env()->CheckCollision(left_box)) {
    return {-sin(point.theta), cos(point.theta)};
  } else {
    return {0, 0};
  }
}
void APP_Planner::CreateConflictedSegements(
    const std::vector<int>& conflict_id,
    std::vector<std::vector<int>>& conflict_segements) {
  conflict_segements.clear();
  std::unordered_map<int, int> umap;
  double ds = std::hypot(carrot_path_gloabl_[1].y - carrot_path_gloabl_[0].y,
                         carrot_path_gloabl_[1].x - carrot_path_gloabl_[0].x);
  int left, right = 5;
  left = int(left_buffer_ / ds);
  right = int(right_buffer_ / ds);
  for (auto i : conflict_id) {
    for (int j = 0; j < left; ++j) {
      int a = i - j < 0 ? 0 : i - j;
      umap[a]++;
    }
    for (int j = 0; j < right; ++j) {
      int a = i + j > carrot_path_gloabl_.size() - 1
                  ? carrot_path_gloabl_.size() - 1
                  : i + j;
      umap[a]++;
    }
  }
  std::vector<int> ids;
  for (auto it : umap) {
    if (it.second > 0) {
      ids.emplace_back(it.first);
    }
  }
  std::sort(ids.begin(), ids.end());
  std::vector<int> tmp;
  for (int i = 0; i < ids.size(); ++i) {
    tmp.emplace_back(ids[i]);
    if (i < ids.size() - 1) {
      if (ids[i + 1] != ids[i] + 1) {
        conflict_segements.emplace_back(tmp);
        tmp.clear();
      }
    } else {
      conflict_segements.emplace_back(tmp);
    }
  }
}
void APP_Planner::GetConflictingPointsId(const common::data::Trajectory& path,
                                         std::vector<int>& conflict_id) {
  for (int i = 0; i < path.size(); ++i) {
    auto vec_box = frame_.env()->vehicle.GenerateBox(
        Pose(path[i].x, path[i].y, path[i].theta));
    if (frame_.env()->CheckCollision(vec_box)) {
      conflict_id.emplace_back(i);
    }
  }
}

void PurePursuitControl::GetLookaheadIndex() {
  if (followed_path_.empty()) return;
  bool found_lookahead = false;
  double min_distance = FLT_MAX;
  int range = 50;
  int end = (car_index_ + range) < followed_path_.size() - 1
                ? (car_index_ + range)
                : followed_path_.size() - 1;
  for (int i = car_index_; i < end; i++) {
    double distance = distance_to_traj(i, cur_state_.x, cur_state_.y);
    if (distance < min_distance) {
      car_index_ = i;
      min_distance = distance;
    }
  }
  for (int i = car_index_; i < end; i++) {
    double distance = distance_to_traj(i, cur_state_.x, cur_state_.y);
    if (distance >= lookahead_distance_) {
      lookahead_index_ = i;
      found_lookahead = true;
      break;
    }
  }
  if (!found_lookahead) {
    lookahead_index_ = end - 1;
  }
}
double PurePursuitControl::Controller() {
  common::data::TrajectoryPoint& lookahead = followed_path_[lookahead_index_];
  auto lookahead_distance =
      distance_to_traj(lookahead_index_, cur_state_.x, cur_state_.y);
  double eta = atan2(lookahead.y - cur_state_.y, lookahead.x - cur_state_.x) -
               cur_state_.theta;
  // steering angle
  double delta = atan2(2.0 * wheel_base_ * sin(eta) / lookahead_distance, 1.0);
  return delta;
}
void PurePursuitControl::UpdateVehicleState(double delta) {
  cur_state_.v = nominal_v_;
  cur_state_.x = cur_state_.x + nominal_v_ * cos(cur_state_.theta) * DT;
  cur_state_.y = cur_state_.y + nominal_v_ * sin(cur_state_.theta) * DT;
  cur_state_.theta =
      cur_state_.theta + cur_state_.v / wheel_base_ * tan(delta) * DT;
}
void PurePursuitControl::ImprovedController(
    common::data::Trajectory& output_result) {
  common::data::TrajectoryPoint& lookahead = followed_path_[lookahead_index_];
  auto lookahead_distance =
      distance_to_traj(lookahead_index_, cur_state_.x, cur_state_.y);
  double eta = atan2(lookahead.y - cur_state_.y, lookahead.x - cur_state_.x) -
               cur_state_.theta;
  // steering angle
  double delta = atan2(2.0 * wheel_base_ * sin(eta) / lookahead_distance, 1.0);
  double dt = DT / intergrate_n_;
  double t_used = fabs((delta - test_pre_delta_) / max_steer_angle_rate_);
  int n_used = int(t_used / dt);
  n_used = n_used > intergrate_n_ ? intergrate_n_ : n_used;
  double steer_angle_rate = (delta - test_pre_delta_) / DT;
  double used_steer_angle_rate =
      steer_angle_rate > 0 ? max_steer_angle_rate_ : -max_steer_angle_rate_;
  common::data::TrajectoryPoint pt = cur_state_;
  for (int i = 1; i <= n_used; ++i) {
    double input_delta = test_pre_delta_ + used_steer_angle_rate * dt * i;
    if (fabs(input_delta) > max_steer_angle_) {
      input_delta = input_delta > 0 ? max_steer_angle_ : -max_steer_angle_;
    }
    UpdateVehicleState(input_delta, dt, pt);
    output_result.emplace_back(pt);
  }
  double input_delta = test_pre_delta_ + used_steer_angle_rate * dt * n_used;
  for (int i = n_used + 1; i <= intergrate_n_; ++i) {
    if (fabs(input_delta) > max_steer_angle_) {
      input_delta = input_delta > 0 ? max_steer_angle_ : -max_steer_angle_;
    }
    UpdateVehicleState(input_delta, dt, pt);
    output_result.emplace_back(pt);
  }
  test_pre_delta_ = input_delta;
  cur_state_ = pt;
}
void PurePursuitControl::UpdateVehicleState(
    const double delta, const double dt, common::data::TrajectoryPoint& state) {
  state.v = nominal_v_;
  state.x = state.x + nominal_v_ * cos(state.theta) * dt;
  state.y = state.y + nominal_v_ * sin(state.theta) * dt;
  state.theta = state.theta + state.v / wheel_base_ * tan(delta) * dt;
  state.phi = delta;
}
