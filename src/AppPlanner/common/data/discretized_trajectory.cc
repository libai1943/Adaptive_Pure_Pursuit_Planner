//
// Created by yenkn on 1/8/21.
//

#include "discretized_trajectory.h"

#include <algorithm>

#include "common/math/linear_interpolation.h"

namespace common {
namespace data {

void DiscretizedTrajectory::ResampleTrajectory(double resample_step) {
  resample_step_ = 0.0;  // activate binary search [WARN: no thread-safe guarantee]

  int len = floor((data_.back().s - data_.front().s) / resample_step) + 1;

  DataType data;
  for (int i = 0; i < len; i++) {
    double s = data_.front().s + i * resample_step;
    data.push_back(EvaluateStation(s));
  }

  //  double residual = std::fmod(data_.back().s - data_.front().s, resample_step);
  //  if(residual > 1e-3) {
  //    data.push_back(data_.back());
  //  }

  data_ = std::move(data);

  // for (size_t i = 0; i < data_.size() - 1; i++) {
  //   data_[i + 1].theta = atan2(data_[i + 1].y - data_[i].y, data_[i + 1].x - data_[i].x);
  // }

  resample_step_ = resample_step;
}

DiscretizedTrajectory::DiscretizedTrajectory(const DiscretizedTrajectory &rhs, size_t begin, size_t end) {
  if (end < 0) {
    end = rhs.data_.size();
  }
  data_.resize(end - begin);
  std::copy_n(std::next(rhs.data_.begin(), begin), data_.size(), data_.begin());
  resample_step_ = rhs.resample_step_;
}

void DiscretizedTrajectory::set_trajectory(const std::vector<common::data::TrajectoryPoint> &tps) {
  double s = 0;

  data_.clear();
  for (size_t i = 0; i < tps.size(); i++) {
    TrajectoryPoint tp;
    tp.x = tps[i].x;
    tp.y = tps[i].y;
    tp.theta = tps[i].theta;
    tp.s = s;
    data_.push_back(tp);
    s += hypot(tps[i + 1].y - tps[i].y, tps[i + 1].x - tps[i].x);
  }

  ResampleTrajectory(kResolution);
}

DiscretizedTrajectory DiscretizedTrajectory::SubTrajectory(size_t begin, size_t end) const {
  return DiscretizedTrajectory(*this, begin, end);
}

void DiscretizedTrajectory::Clip(size_t begin, size_t end) {
  if (end < 0) {
    end = data_.size();
  }

  data_.erase(std::next(data_.begin(), begin), std::next(data_.begin(), end));
}

SVProfile DiscretizedTrajectory::GetSVProfile() {
  SVProfile profile;

  for (auto &v : data_) {
    profile.push_back({v.s, v.v});
  }

  return profile;
}

void DiscretizedTrajectory::SetVelocityProfile(const std::vector<double> &profile) {
  assert(profile.size() == data_.size());

  for (size_t i = 0; i < profile.size(); i++) {
    data_[i].v = profile[i];
  }
}

void DiscretizedTrajectory::SampleTime() {
  for (int i = 1; i < data_.size(); i++) {
    double velocity = std::abs(data_[i].v);
    double relative_time = velocity < 1e-3 ? 0 : (data_[i].s - data_[i - 1].s) / velocity;
    data_[i].t = data_[i - 1].t + relative_time;
  }
}

DiscretizedTrajectory::DataType::const_iterator
DiscretizedTrajectory::QueryLowerBoundStationPoint(double station) const {
  if (station >= data_.back().s) {
    return data_.end() - 1;
  } else if (station < data_.front().s) {
    return data_.begin();
  }

  if (resample_step_ > 0.0) {
    return std::next(data_.begin(), (size_t)ceil((station - data_.front().s) / resample_step_));
  }

  return std::lower_bound(
      data_.begin(), data_.end(), station,
      [](const TrajectoryPoint &t, double station) {
        return t.s < station;
      });
}

TrajectoryPoint LinearInterpolateTrajectory(const TrajectoryPoint &p0, const TrajectoryPoint &p1, double s) {
  double s0 = p0.s;
  double s1 = p1.s;
  if (std::abs(s1 - s0) < common::math::kMathEpsilon) {
    return p0;
  }

  TrajectoryPoint pt;
  double weight = (s - s0) / (s1 - s0);
  pt.s = s;
  pt.x = (1 - weight) * p0.x + weight * p1.x;
  pt.y = (1 - weight) * p0.y + weight * p1.y;
  pt.theta = common::math::slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  pt.v = (1 - weight) * p0.v + weight * p1.v;
  pt.left_bound = (1 - weight) * p0.left_bound + weight * p1.left_bound;
  pt.right_bound = (1 - weight) * p0.right_bound + weight * p1.right_bound;

  return pt;
}

TrajectoryPoint DiscretizedTrajectory::EvaluateStation(double station) const {
  auto iter = QueryLowerBoundStationPoint(station);

  if (iter == data_.begin()) {
    iter = std::next(iter);
  }

  auto prev = std::prev(iter, 1);

  return LinearInterpolateTrajectory(*prev, *iter, station);
}

DiscretizedTrajectory::DataType::const_iterator
DiscretizedTrajectory::QueryLowerBoundTimePoint(double timestamp) const {
  if (timestamp >= data_.back().t) {
    return data_.end() - 1;
  }

  return std::lower_bound(
      data_.begin(), data_.end(), timestamp,
      [](const TrajectoryPoint &t, double time) {
        return t.t < time;
      });
}

DiscretizedTrajectory::DataType::const_iterator
DiscretizedTrajectory::QueryNearestPoint(const Vec2d &point, double *out_distance) const {
  auto nearest_iter = data_.begin();
  double nearest_distance = std::numeric_limits<double>::max();

  for (auto iter = data_.begin(); iter != data_.end(); iter++) {
    double dx = iter->x - point.x(), dy = iter->y - point.y();
    double distance = dx * dx + dy * dy;
    if (distance < nearest_distance) {
      nearest_iter = iter;
      nearest_distance = distance;
    }
  }

  if (out_distance != nullptr) {
    *out_distance = sqrt(nearest_distance);
  }
  return nearest_iter;
}

Vec2d DiscretizedTrajectory::GetProjection(const Vec2d &xy) const {
  long point_idx = std::distance(data_.begin(), QueryNearestPoint(xy));
  auto project_point = data_[point_idx];
  auto index_start = std::max(0l, point_idx - 1);
  auto index_end = std::min(data_.size() - 1, (ulong)point_idx + 1);

  if (index_start < index_end) {
    double v0x = xy.x() - data_[index_start].x;
    double v0y = xy.y() - data_[index_start].y;

    double v1x = data_[index_end].x - data_[index_start].x;
    double v1y = data_[index_end].y - data_[index_start].y;

    double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
    double dot = v0x * v1x + v0y * v1y;

    double delta_s = dot / v1_norm;
    project_point = LinearInterpolateTrajectory(data_[index_start], data_[index_end], data_[index_start].s + delta_s);
  }

  double nr_x = xy.x() - project_point.x, nr_y = xy.y() - project_point.y;
  double lateral = copysign(hypot(nr_x, nr_y), nr_y * cos(project_point.theta) - nr_x * sin(project_point.theta));
  return {project_point.s, lateral};
}

Vec2d DiscretizedTrajectory::ToCartesian(double station, double lateral) const {
  auto ref = EvaluateStation(station);
  return {ref.x - lateral * sin(ref.theta), ref.y + lateral * cos(ref.theta)};
}

void DiscretizedTrajectory::PatchStations() {
  double s = 0.0;
  for (size_t i = 0; i < data_.size(); i++) {
    data_[i].s = s;
    if (i < data_.size() - 1) {
      s += hypot(data_[i + 1].x - data_[i].x, data_[i + 1].y - data_[i].y);
    }
  }
}

}  // namespace data
}  // namespace common