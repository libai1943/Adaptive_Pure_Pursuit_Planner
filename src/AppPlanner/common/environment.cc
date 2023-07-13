//
// Created by yenkn on 4/21/22.
//
#include "environment.h"

#include "math/box2d.h"
#include "math/polygon2d.h"
#include "visualization_plot.h"

bool Environment::CheckPoseCollision(const common::math::Pose& pose, double radius_buffer) {
  common::math::AABox2d initial_box({-vehicle.radius - radius_buffer, -vehicle.radius - radius_buffer},
                                    {vehicle.radius + radius_buffer, vehicle.radius + radius_buffer});

  double xr, yr, xf, yf;
  // std::tie(xr, yr, xf, yf) = vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());

  auto f_box = initial_box, r_box = initial_box;
  f_box.Shift({xf, yf});
  r_box.Shift({xr, yr});

  // VisualizationPlot::PlotPolygon(common::math::Polygon2d(common::math::Box2d(f_box)), 0.1, Color::Red, 1, "f_box");
  // VisualizationPlot::PlotPolygon(common::math::Polygon2d(common::math::Box2d(r_box)), 0.1, Color::Red, 1, "r_box");
  // VisualizationPlot::Trigger();

  if (CheckCollision(common::math::Box2d(f_box)) || CheckCollision(common::math::Box2d(r_box))) {
    return true;
  }
  return false;
}

bool Environment::CheckCollision(const common::math::Box2d& box) {
  if (points.CheckInBox(box)) {
    return true;
  }

  return std::any_of(obstacles.begin(), obstacles.end(), [&](common::Obstacle& ob) {
    if (ob.is_moving()) {
      return false;
    }
    return ob.GetPolygon().HasOverlap(box);
  });
}

void Environment::Visualize() {
  // for (int i = 0; i < obstacles.size(); i++) {
  //   // TODO: plot dynamic obstacle
  //   if (obstacles[i].is_moving()) {
  //     VisualizationPlot::PlotPolygon(obstacles[i].GetPolygon(), 0.2, Color::Blue, i, "Obstacles");
  //   } else {
  //     VisualizationPlot::PlotPolygon(obstacles[i].GetPolygon(), 0.2, Color::Magenta, i, "Obstacles");
  //   }
  //   VisualizationPlot::Trigger();
  // }

  std::vector<double> xs, ys, thetas;
  for (auto& pt : points.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  VisualizationPlot::PlotPoints(xs, ys, 0.01, Color::Yellow, 9999, "Border");

  // xs.clear();
  // ys.clear();
  // for (auto& guidance : guidances) {
  //   for (auto& pt : guidance.data()) {
  //     xs.push_back(pt.x);
  //     ys.push_back(pt.y);
  //     thetas.push_back(pt.theta);
  //   }
  // }
  // // VisualizationPlot::PlotPath(xs, ys, thetas, 250, 0.1, 1, "guidances");
  // VisualizationPlot::PlotPoints(xs, ys, 0.1, Color::White, 1, "guidances");

  // xs.clear();
  // ys.clear();
  // thetas.clear();
  // for (auto& pt : reference.data()) {
    // xs.push_back(pt.x);
    // ys.push_back(pt.y);
    // thetas.push_back(pt.theta);
  // }
  // VisualizationPlot::PlotPath(xs, ys, thetas, 0, 0.1, 1, "references");
  // VisualizationPlot::PlotPoints(xs, ys, 0.1, Color::Yellow, 1, "references");

  VisualizationPlot::Trigger();
}