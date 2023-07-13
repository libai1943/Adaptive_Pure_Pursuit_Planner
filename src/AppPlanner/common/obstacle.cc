//
// Created by yenkn on 4/20/22.
//

#include "obstacle.h"

#include <algorithm>

#include "common/math/linear_interpolation.h"
#include "common/math/vec2d.h"

namespace common {
common::math::Vec2d RotatePt(const common::math::Vec2d &tp, double angle) {
  double x = tp.x() * cos(angle) - tp.y() * sin(angle);
  double y = tp.x() * sin(angle) + tp.y() * cos(angle);
  return {x, y};
}

math::Polygon2d Obstacle::GetPolygon(double relative_time) const {
  // if(trajectory_.empty()) {
  //   return polygon_;
  // }

  // double time = relative_time;
  // math::PoseStamped pose = trajectory_.back();
  // if(time < trajectory_.back().time) {
  //   auto iter = std::lower_bound(trajectory_.begin(), trajectory_.end(), time, [](const math::PoseStamped &a, double val) {
  //     return a.time < val;
  //   });

  //   if(iter > trajectory_.begin()) {
  //     auto prev = std::prev(iter);

  //     pose.time = time;
  //     double weight = (time - prev->time) / (iter->time - prev->time);
  //     pose.pose.set_x((1 - weight) * prev->pose.x() + weight * iter->pose.x());
  //     pose.pose.set_y((1 - weight) * prev->pose.y() + weight * iter->pose.y());
  //     pose.pose.set_theta(math::slerp(prev->pose.theta(),prev->time, iter->pose.theta(), iter->time, time));
  //   }
  // }

  // std::vector<math::Vec2d> points;
  // points.reserve(polygon_.num_points());
  // math::Vec2d transform(0.0, 0.0);
  // // math::Vec2d transform(pose.pose.x() - trajectory_.front().pose.x(), pose.pose.y() - trajectory_.back().pose.y());
  // for(auto &pt: polygon_.points()) {
  //   points.push_back(pt + transform);
  // }
  // return math::Polygon2d(points);

  if (trajectory_.empty()) {
    return polygon_;
  }

  double time = relative_time;
  math::PoseStamped relative_pose;
  // relative_pose.pose.set_x(trajectory_.back().pose.x() - trajectory_.front().pose.x());
  // relative_pose.pose.set_y(trajectory_.back().pose.y() - trajectory_.front().pose.y());
  relative_pose.pose.set_x(0.0);
  relative_pose.pose.set_y(0.0);

  if (time < trajectory_.back().time) {
    auto iter = std::lower_bound(trajectory_.begin(), trajectory_.end(), time, [](const math::PoseStamped &a, double val) {
      return a.time < val;
    });

    if (iter > trajectory_.begin()) {
      auto prev = std::prev(iter);

      relative_pose.time = time;
      double weight = (time - prev->time) / (iter->time - prev->time);
      relative_pose.pose.set_x((1 - weight) * prev->pose.x() + weight * iter->pose.x() - trajectory_.front().pose.x());
      relative_pose.pose.set_y((1 - weight) * prev->pose.y() + weight * iter->pose.y() - trajectory_.front().pose.y());
      relative_pose.pose.set_theta(math::slerp(prev->pose.theta(), prev->time, iter->pose.theta(), iter->time, time) - trajectory_.front().pose.theta());
    }
  } else {
    relative_pose.pose.set_x(trajectory_.back().pose.x() - trajectory_.front().pose.x());
    relative_pose.pose.set_y(trajectory_.back().pose.y() - trajectory_.front().pose.y());
  }

  std::vector<math::Vec2d> points;
  points.reserve(polygon_.num_points());
  math::Vec2d transform(relative_pose.pose.x(), relative_pose.pose.y());
  for (auto &pt : polygon_.points()) {
    auto tmp_pt = pt - polygon_.center();
    tmp_pt = RotatePt(tmp_pt, relative_pose.pose.theta());
    tmp_pt = tmp_pt + polygon_.center();
    points.push_back(tmp_pt + transform);
  }
  return math::Polygon2d(points);
  // Frame frame;
  // auto tmp_pose = Pose(trajectory_.front().pose.x() + relative_pose.pose.x(), trajectory_.front().pose.y() + relative_pose.pose.y(), trajectory_.front().pose.theta() + relative_pose.pose.theta());
  // return Polygon2d(frame.env()->vehicle.GenerateBox(tmp_pose));
}

math::Pose Obstacle::GetVecPose(double relative_time) const {
  // if(trajectory_.empty()) {
  //   return polygon_;
  // }

  // double time = relative_time;
  // math::PoseStamped pose = trajectory_.back();
  // if(time < trajectory_.back().time) {
  //   auto iter = std::lower_bound(trajectory_.begin(), trajectory_.end(), time, [](const math::PoseStamped &a, double val) {
  //     return a.time < val;
  //   });

  //   if(iter > trajectory_.begin()) {
  //     auto prev = std::prev(iter);

  //     pose.time = time;
  //     double weight = (time - prev->time) / (iter->time - prev->time);
  //     pose.pose.set_x((1 - weight) * prev->pose.x() + weight * iter->pose.x());
  //     pose.pose.set_y((1 - weight) * prev->pose.y() + weight * iter->pose.y());
  //     pose.pose.set_theta(math::slerp(prev->pose.theta(),prev->time, iter->pose.theta(), iter->time, time));
  //   }
  // }

  // std::vector<math::Vec2d> points;
  // points.reserve(polygon_.num_points());
  // math::Vec2d transform(0.0, 0.0);
  // // math::Vec2d transform(pose.pose.x() - trajectory_.front().pose.x(), pose.pose.y() - trajectory_.back().pose.y());
  // for(auto &pt: polygon_.points()) {
  //   points.push_back(pt + transform);
  // }
  // return math::Polygon2d(points);

  // if (trajectory_.empty()) {
  //   return polygon_;
  // }

  double time = relative_time;
  math::PoseStamped relative_pose;
  // relative_pose.pose.set_x(trajectory_.back().pose.x() - trajectory_.front().pose.x());
  // relative_pose.pose.set_y(trajectory_.back().pose.y() - trajectory_.front().pose.y());
  relative_pose.pose.set_x(0.0);
  relative_pose.pose.set_y(0.0);

  if (time < trajectory_.back().time) {
    auto iter = std::lower_bound(trajectory_.begin(), trajectory_.end(), time, [](const math::PoseStamped &a, double val) {
      return a.time < val;
    });

    if (iter > trajectory_.begin()) {
      auto prev = std::prev(iter);

      relative_pose.time = time;
      double weight = (time - prev->time) / (iter->time - prev->time);
      relative_pose.pose.set_x((1 - weight) * prev->pose.x() + weight * iter->pose.x() - trajectory_.front().pose.x());
      relative_pose.pose.set_y((1 - weight) * prev->pose.y() + weight * iter->pose.y() - trajectory_.front().pose.y());
      relative_pose.pose.set_theta(math::slerp(prev->pose.theta(), prev->time, iter->pose.theta(), iter->time, time) - trajectory_.front().pose.theta());
    }
  } else {
    relative_pose.pose.set_x(trajectory_.back().pose.x() - trajectory_.front().pose.x());
    relative_pose.pose.set_y(trajectory_.back().pose.y() - trajectory_.front().pose.y());
  }

  // std::vector<math::Vec2d> points;
  // points.reserve(polygon_.num_points());
  // math::Vec2d transform(relative_pose.pose.x(), relative_pose.pose.y());
  // for (auto &pt : polygon_.points()) {
  //   auto tmp_pt = pt - polygon_.center();
  //   tmp_pt = RotatePt(tmp_pt, relative_pose.pose.theta());
  //   tmp_pt = tmp_pt + polygon_.center();
  //   points.push_back(tmp_pt + transform);
  // }
  // return math::Polygon2d(points);
  // Frame frame;
  auto tmp_pose = common::math::Pose(trajectory_.front().pose.x() + relative_pose.pose.x(), trajectory_.front().pose.y() + relative_pose.pose.y(), trajectory_.front().pose.theta() + relative_pose.pose.theta());
  return tmp_pose;
  // return Polygon2d(frame.env()->vehicle.GenerateBox(tmp_pose));
}
}  // namespace common