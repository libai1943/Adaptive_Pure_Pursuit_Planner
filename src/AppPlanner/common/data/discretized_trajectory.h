#pragma once

#include <cassert>
#include <utility>
#include <vector>

#include "common/math/trajectory1d.h"
#include "common/math/vec2d.h"
#include "common/math/math_utils.h"
#include "common/math/linear_interpolation.h"
#include "common/util/point_cloud.h"
#include "common/visualization_plot.h"

namespace common {
namespace data {

using common::math::kMathEpsilon;
using common::math::SVProfile;
using common::math::Vec2d;

struct TrajectoryPoint {
  double s = 0.0;
  double t = 0.0;

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double v = 0.0;
  double phi = 0.0;
  double a = 0.0;
  double omega = 0.0;

  double left_bound = 0.0;
  double right_bound = 0.0;

  int road_id;

  int gear = 1;
};

inline TrajectoryPoint LinearInterpolateStation(
    const TrajectoryPoint &p0, const TrajectoryPoint &p1, double s0, double s1, double s) {
  if (std::abs(s1 - s0) < kMathEpsilon) {
    return p0;
  }

  TrajectoryPoint pt;
  double weight = (s - s0) / (s1 - s0);
  pt.s = s;
  pt.x = (1 - weight) * p0.x + weight * p1.x;
  pt.y = (1 - weight) * p0.y + weight * p1.y;
  pt.theta = common::math::slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  pt.v = (1 - weight) * p0.v + weight * p1.v;
  pt.phi = (1 - weight) * p0.phi + weight * p1.phi;
  pt.a = (1 - weight) * p0.a + weight * p1.a;
  pt.omega = (1 - weight) * p0.omega + weight * p1.omega;

  return pt;
}

typedef std::vector<TrajectoryPoint> Trajectory;

constexpr double kResolution = 0.05;

/**
 * Discretized Trajectory
 */
class DiscretizedTrajectory {
 public:
  typedef std::vector<TrajectoryPoint> DataType;

  DiscretizedTrajectory() = default;

  DiscretizedTrajectory(const DiscretizedTrajectory &rhs, size_t begin, size_t end = -1);

  template <typename Iter>
  DiscretizedTrajectory(const Iter begin, const Iter end) : data_(begin, end) {
    ResampleTrajectory(kResolution);
  }

  explicit DiscretizedTrajectory(std::vector<TrajectoryPoint> points, double resolution) : data_(std::move(points)) {
    ResampleTrajectory(resolution);
  }

  explicit DiscretizedTrajectory(std::vector<TrajectoryPoint> points) : data_(std::move(points)) {
    ResampleTrajectory(kResolution);
  }

  void set_trajectory(const std::vector<common::data::TrajectoryPoint> &tps);

  void ResampleWithCount(int count) {
    assert(data_.size() >= 2);
    double step = (data_.back().s - data_.front().s) / (count - 1);
    ResampleTrajectory(step);
  }

  inline const DataType &data() const { return data_; }

  inline void delete_data(size_t begin, size_t end = -1) { data_.erase(data_.begin() + begin, data_.begin() + end); }

  inline DataType &mut_data() { return data_; }

  DiscretizedTrajectory SubTrajectory(size_t begin, size_t end = -1) const;

  void Clip(size_t begin, size_t end = -1);

  DataType::const_iterator QueryLowerBoundStationPoint(double station) const;

  DataType::const_iterator QueryLowerBoundTimePoint(double timestamp) const;

  DataType::const_iterator QueryNearestPoint(const Vec2d &point, double *out_distance = nullptr) const;

  TrajectoryPoint EvaluateStation(double station) const;

  SVProfile GetSVProfile();

  void SetVelocityProfile(const std::vector<double> &profile);

  void SampleTime();

  Vec2d GetProjection(const Vec2d &xy) const;

  Vec2d ToCartesian(double station, double lateral) const;

  int GetId() { return idx_; }

  bool Empty() {
    if (data_.empty()) {
      return true;
    }
    return false;
  }

  void Clear() { data_.clear(); }

  void set_idx(double idx) { idx_ = idx; }

  void PatchStations();

 protected:
  std::vector<TrajectoryPoint> data_;
  double resample_step_ = 0.0;
  int idx_ = 0.0;

  void ResampleTrajectory(double step);
};

}  // namespace data
}  // namespace common
