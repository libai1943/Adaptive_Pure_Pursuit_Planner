#include "map_generator.h"

using namespace common::math;
using namespace common::util;

namespace map_generator {
MapGenerator::MapGenerator() {
  map_width_ = 2.0;
  map_height_ = 3.6;
  road_width_ = 0.3;
  min_turn_radiu_ = 0.45;
  // 可以调整这个改变整个地图
  x_straight_ = 0.1;
  y_straight_ = 0.05;

  // block_width_ = 0.5 * (map_width_ - 2.0 * road_width_);
  // block_height_ = 0.5 * (map_height_ - 2.0 * road_width_);

  cross_ = std::tuple<double, double, double, double>(
      -0.5 * road_width_ - min_turn_radiu_ + 0.01,
      0.5 * road_width_ + min_turn_radiu_ - 0.01, -min_turn_radiu_ + 0.01,
      min_turn_radiu_ - 0.01);
}

Vec2d RotatePt(const common::math::Vec2d &tp, double angle) {
  double x = tp.x() * cos(angle) - tp.y() * sin(angle);
  double y = tp.x() * sin(angle) + tp.y() * cos(angle);
  return {x, y};
}

void MapGenerator::GenerateBlocks() {
  // -1.15 0.75 || -0.35, 0.75 || -0.35 0.25 || -1.15 0.25
  // 1.15 0.75 || 0.35, 0.75 || 0.35 0.25 || -1.15 0.25
  Polygon2d block_1 = Polygon2d(Box2d(Vec2d(-0.75, 0.5), 0, 0.8, 0.5));
  Polygon2d block_2 = Polygon2d(Box2d(Vec2d(0.75, 0.5), 0, 0.8, 0.5));

  blocks_.emplace_back(common::Obstacle(block_1));
  blocks_.emplace_back(common::Obstacle(block_2));
}

void MapGenerator::GenerateRoadNodes() {
  // road points
  common::data::TrajectoryPoint node_1;
  node_1.x = road_width_ * 0.5;
  node_1.y = min_turn_radiu_;
  node_1.theta = M_PI_2;

  common::data::TrajectoryPoint node_2;
  node_2.x = road_width_ * 0.5;
  node_2.y = min_turn_radiu_ + y_straight_;
  node_2.theta = M_PI;

  common::data::TrajectoryPoint node_3;
  node_3.x = node_2.x + min_turn_radiu_;
  node_3.y = node_2.y + min_turn_radiu_;
  node_3.theta = 0;

  common::data::TrajectoryPoint node_4;
  node_4.x = node_3.x + x_straight_;
  node_4.y = node_3.y;
  node_4.theta = 0;

  common::data::TrajectoryPoint node_5;
  node_5.x = node_4.x + min_turn_radiu_;
  node_5.y = node_4.y - min_turn_radiu_;
  node_5.theta = -M_PI_2;

  common::data::TrajectoryPoint node_6;
  node_6.x = node_1.x + min_turn_radiu_ + x_straight_ + min_turn_radiu_;
  node_6.y = node_1.y;
  node_6.theta = -M_PI_2;

  common::data::TrajectoryPoint node_7;
  node_7.x = node_6.x - min_turn_radiu_;
  node_7.y = node_6.y - min_turn_radiu_;
  node_7.theta = -M_PI;

  common::data::TrajectoryPoint node_8;
  node_8.x = node_1.x + min_turn_radiu_;
  node_8.y = 0;
  node_8.theta = -M_PI;

  road_node_.emplace_back(node_1);
  road_node_.emplace_back(node_2);
  road_node_.emplace_back(node_3);
  road_node_.emplace_back(node_4);
  road_node_.emplace_back(node_5);
  road_node_.emplace_back(node_6);
  road_node_.emplace_back(node_7);
  road_node_.emplace_back(node_8);

  common::data::TrajectoryPoint node_9;
  node_9.x = -node_1.x;
  node_9.y = node_1.y;
  node_9.theta = -M_PI_2;

  common::data::TrajectoryPoint node_10;
  node_10.x = -node_2.x;
  node_10.y = node_2.y;
  node_10.theta = -M_PI_2;

  common::data::TrajectoryPoint node_11;
  node_11.x = -node_3.x;
  node_11.y = node_3.y;
  node_11.theta = 0;

  common::data::TrajectoryPoint node_12;
  node_12.x = -node_4.x;
  node_12.y = node_4.y;
  node_12.theta = 0;

  common::data::TrajectoryPoint node_13;
  node_13.x = -node_5.x;
  node_13.y = node_5.y;
  node_13.theta = M_PI_2;

  common::data::TrajectoryPoint node_14;
  node_14.x = -node_6.x;
  node_14.y = node_6.y;
  node_14.theta = M_PI_2;

  common::data::TrajectoryPoint node_15;
  node_15.x = -node_7.x;
  node_15.y = node_7.y;
  node_15.theta = M_PI;

  common::data::TrajectoryPoint node_16;
  node_16.x = -node_8.x;
  node_16.y = node_8.y;
  node_16.theta = M_PI;

  road_node_.emplace_back(node_9);
  road_node_.emplace_back(node_10);
  road_node_.emplace_back(node_11);
  road_node_.emplace_back(node_12);
  road_node_.emplace_back(node_13);
  road_node_.emplace_back(node_14);
  road_node_.emplace_back(node_15);
  road_node_.emplace_back(node_16);

  common::data::TrajectoryPoint node_17;
  node_17.x = road_width_ * 0.5 + min_turn_radiu_;
  node_17.y = -road_width_;
  node_17.theta = 0;

  common::data::TrajectoryPoint node_18;
  node_18.x = node_17.x + x_straight_;
  node_18.y = node_17.y;
  node_18.theta = 0;

  common::data::TrajectoryPoint node_19;
  node_19.x = node_18.x + min_turn_radiu_ + road_width_;
  node_19.y = node_18.y + min_turn_radiu_ + road_width_;
  node_19.theta = M_PI_2;

  common::data::TrajectoryPoint node_20;
  node_20.x = node_19.x;
  node_20.y = node_19.y + y_straight_;
  node_20.theta = M_PI_2;

  common::data::TrajectoryPoint node_21;
  node_21.x = node_18.x;
  node_21.y = node_20.y + min_turn_radiu_ + road_width_;
  node_21.theta = M_PI;

  common::data::TrajectoryPoint node_22;
  node_22.x = -node_21.x;
  node_22.y = node_21.y;
  node_22.theta = M_PI;

  common::data::TrajectoryPoint node_23;
  node_23.x = -node_20.x;
  node_23.y = node_20.y;
  node_23.theta = M_PI * 1.5;

  common::data::TrajectoryPoint node_24;
  node_24.x = -node_19.x;
  node_24.y = node_19.y;
  node_24.theta = M_PI * 1.5;

  common::data::TrajectoryPoint node_25;
  node_25.x = -node_18.x;
  node_25.y = node_18.y;
  node_25.theta = M_PI * 2.0;

  common::data::TrajectoryPoint node_26;
  node_26.x = -node_17.x;
  node_26.y = node_17.y;
  node_26.theta = M_PI * 2.0;

  road_node_.emplace_back(node_17);
  road_node_.emplace_back(node_18);
  road_node_.emplace_back(node_19);
  road_node_.emplace_back(node_20);
  road_node_.emplace_back(node_21);
  road_node_.emplace_back(node_22);
  road_node_.emplace_back(node_23);
  road_node_.emplace_back(node_24);
  road_node_.emplace_back(node_25);
  road_node_.emplace_back(node_26);
}

void MapGenerator::GenerateCurves() {
  // spirit curve 1
  // right down
  apollo::planning::QuinticSpiralPath spi_path(0, 0, 0, M_PI_2, 0, 0, 0.765);
  std::vector<double> ss = common::util::LinSpaced(0.765, 0, 35);
  std::vector<double> xs1(ss.size()), ys1(ss.size()), theta1(ss.size());
  for (int i = 0; i < ss.size(); i++) {
    xs1[i] = spi_path.ComputeCartesianDeviationX<5>(ss[i]);
    ys1[i] = spi_path.ComputeCartesianDeviationY<5>(ss[i]);
    theta1[i] = spi_path.Evaluate(0, ss[i]);
  }
  // VisualizationPlot::PlotPath(xs1, ys1, theta1, 120, 0.01, 4, "Spiral");
  // VisualizationPlot::Trigger();

  common::data::Trajectory curve_1;
  for (int i = 0; i < xs1.size(); i++) {
    common::data::TrajectoryPoint tp;
    tp.x = xs1[i];
    tp.y = ys1[i];
    tp.theta = theta1[i];
    tp.s = ss[xs1.size() - 1 - i];
    curve_1.emplace_back(tp);
  }
  spirit_curve_.emplace_back(curve_1);

  // right up
  std::vector<double> xs2(xs1.size()), ys2(xs1.size()), theta2(xs1.size());
  for (int i = 0; i < xs1.size(); i++) {
    auto tp = RotatePt(Vec2d(xs1[i], ys1[i]), M_PI_2);
    xs2[i] = tp.x();
    ys2[i] = tp.y();
    theta2[i] = theta1[i] + M_PI_2;
  }
  // VisualizationPlot::PlotPath(xs2, ys2, theta2, 120, 0.01, 2, "Spiral");

  common::data::Trajectory curve_2;
  for (int i = 0; i < xs2.size(); i++) {
    common::data::TrajectoryPoint tp;
    tp.x = xs2[i];
    tp.y = ys2[i];
    tp.theta = theta2[i];
    tp.s = ss[xs1.size() - 1 - i];
    curve_2.emplace_back(tp);
  }
  spirit_curve_.emplace_back(curve_2);

  // left up
  std::vector<double> xs3(xs1.size()), ys3(xs1.size()), theta3(xs1.size());
  for (int i = 0; i < xs1.size(); i++) {
    auto tp = RotatePt(Vec2d(xs1[i], ys1[i]), M_PI);
    xs3[i] = tp.x();
    ys3[i] = tp.y();
    theta3[i] = theta1[i] + M_PI;
  }
  // VisualizationPlot::PlotPath(xs3, ys3, theta3, 120, 0.01, 3, "Spiral");

  common::data::Trajectory curve_3;
  for (int i = 0; i < xs3.size(); i++) {
    common::data::TrajectoryPoint tp;
    tp.x = xs3[i];
    tp.y = ys3[i];
    tp.theta = theta3[i];
    tp.s = ss[xs1.size() - 1 - i];
    curve_3.emplace_back(tp);
  }
  spirit_curve_.emplace_back(curve_3);

  // left down
  std::vector<double> xs4(xs1.size()), ys4(xs1.size()), theta4(xs1.size());
  for (int i = 0; i < xs1.size(); i++) {
    auto tp = RotatePt(Vec2d(xs1[i], ys1[i]), M_PI * 1.5);
    xs4[i] = tp.x();
    ys4[i] = tp.y();
    theta4[i] = theta1[i] + M_PI * 1.5;
  }
  // VisualizationPlot::PlotPath(xs4, ys4, theta4, 120, 0.01, 4, "Spiral");

  common::data::Trajectory curve_4;
  for (int i = 0; i < xs4.size(); i++) {
    common::data::TrajectoryPoint tp;
    tp.x = xs4[i];
    tp.y = ys4[i];
    tp.theta = theta4[i];
    tp.s = ss[xs1.size() - 1 - i];
    curve_4.emplace_back(tp);
  }
  spirit_curve_.emplace_back(curve_4);

  // spirit curve 2
  // right down
  apollo::planning::QuinticSpiralPath spi_path2(0, 0, 0, M_PI_2, 0, 0, 1.275);
  ss = common::util::LinSpaced(0, 1.275, 60);
  std::vector<double> xs5(ss.size()), ys5(ss.size()), theta5(ss.size());
  for (int i = 0; i < ss.size(); i++) {
    xs5[i] = spi_path2.ComputeCartesianDeviationX<5>(ss[i]);
    ys5[i] = spi_path2.ComputeCartesianDeviationY<5>(ss[i]);
    theta5[i] = spi_path2.Evaluate(0, ss[i]);
  }
  // VisualizationPlot::PlotPath(xs5, ys5, theta5, 120, 0.01, 5, "Spiral");

  common::data::Trajectory curve_5;
  for (int i = 0; i < xs5.size(); i++) {
    common::data::TrajectoryPoint tp;
    tp.x = xs5[i];
    tp.y = ys5[i];
    tp.theta = theta5[i];
    tp.s = ss[i];
    curve_5.emplace_back(tp);
  }
  spirit_curve_.emplace_back(curve_5);

  // right up
  std::vector<double> xs6(xs5.size()), ys6(xs5.size()), theta6(xs5.size());
  for (int i = 0; i < xs5.size(); i++) {
    auto tp = RotatePt(Vec2d(xs5[i], ys5[i]), M_PI_2);
    xs6[i] = tp.x();
    ys6[i] = tp.y();
    theta6[i] = theta5[i] + M_PI_2;
  }
  // VisualizationPlot::PlotPath(xs6, ys6, theta6, 120, 0.01, 6, "Spiral");

  common::data::Trajectory curve_6;
  for (int i = 0; i < xs6.size(); i++) {
    common::data::TrajectoryPoint tp;
    tp.x = xs6[i];
    tp.y = ys6[i];
    tp.theta = theta6[i];
    tp.s = ss[i];
    curve_6.emplace_back(tp);
  }
  spirit_curve_.emplace_back(curve_6);

  // left up
  std::vector<double> xs7(xs5.size()), ys7(xs5.size()), theta7(xs5.size());
  for (int i = 0; i < xs5.size(); i++) {
    auto tp = RotatePt(Vec2d(xs5[i], ys5[i]), M_PI);
    xs7[i] = tp.x();
    ys7[i] = tp.y();
    theta7[i] = theta5[i] + M_PI;
  }
  // VisualizationPlot::PlotPath(xs7, ys7, theta7, 120, 0.01, 7, "Spiral");

  common::data::Trajectory curve_7;
  for (int i = 0; i < xs7.size(); i++) {
    common::data::TrajectoryPoint tp;
    tp.x = xs7[i];
    tp.y = ys7[i];
    tp.theta = theta7[i];
    tp.s = ss[i];
    curve_7.emplace_back(tp);
  }
  spirit_curve_.emplace_back(curve_7);

  // left down
  std::vector<double> xs8(xs5.size()), ys8(xs5.size()), theta8(xs5.size());
  for (int i = 0; i < xs8.size(); i++) {
    auto tp = RotatePt(Vec2d(xs5[i], ys5[i]), M_PI * 1.5);
    xs8[i] = tp.x();
    ys8[i] = tp.y();
    theta8[i] = theta5[i] + M_PI * 1.5;
  }
  // VisualizationPlot::PlotPath(xs8, ys8, theta8, 120, 0.01, 8, "Spiral");

  common::data::Trajectory curve_8;
  for (int i = 0; i < xs8.size(); i++) {
    common::data::TrajectoryPoint tp;
    tp.x = xs8[i];
    tp.y = ys8[i];
    tp.theta = theta8[i];
    tp.s = ss[i];
    curve_8.emplace_back(tp);
  }
  spirit_curve_.emplace_back(curve_8);
}

void MapGenerator::GenerateRoads() {
  // road 1
  // road segment 1
  double resolution = 0.02;
  std::vector<common::data::TrajectoryPoint> road_seg_1;
  std::vector<double> xs1, ys1, thetas1, ss1;
  double len = std::sqrt(std::pow(road_node_[0].x - road_node_[1].x, 2) +
                         std::pow(road_node_[0].y - road_node_[1].y, 2));
  int n = len / resolution;

  xs1 = LinSpaced(road_node_[0].x, road_node_[1].x, n);
  ys1 = LinSpaced(road_node_[0].y, road_node_[1].y, n);
  thetas1 = std::vector<double>(n, road_node_[0].theta);
  ss1 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs1, ys1, 0.05, Color::Red, 1234,
  // "road_segment"); VisualizationPlot::Trigger();

  road_seg_1.resize(xs1.size());
  for (int i = 0; i < road_seg_1.size(); i++) {
    road_seg_1[i].x = xs1[i];
    road_seg_1[i].y = ys1[i];
    road_seg_1[i].theta = thetas1[i];
    road_seg_1[i].s = ss1[i];
  }

  // road segment 2
  std::vector<common::data::TrajectoryPoint> road_seg_2 = spirit_curve_[2];
  // std::reverse(road_seg_2.begin(), road_seg_2.end());
  std::vector<double> xs2(road_seg_2.size()), ys2(road_seg_2.size()),
      thetas2(road_seg_2.size());
  for (int i = 0; i < road_seg_2.size(); i++) {
    road_seg_2[i].x += min_turn_radiu_ + road_width_ * 0.5;
    road_seg_2[i].y += 2.0 * min_turn_radiu_ + y_straight_;
    road_seg_2[i].s += road_seg_1.back().s;
    road_seg_2[i].theta -= M_PI;

    xs2[i] = road_seg_2[i].x;
    ys2[i] = road_seg_2[i].y;
    thetas2[i] = road_seg_2[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs2, ys2, 0.01, Color::White, 2,
  // "road_segment");

  // road segment 3
  std::vector<common::data::TrajectoryPoint> road_seg_3;
  std::vector<double> xs3, ys3, thetas3, ss3;
  len = std::sqrt(std::pow(road_node_[2].x - road_node_[3].x, 2) +
                  std::pow(road_node_[2].y - road_node_[3].y, 2));
  n = len / resolution;

  xs3 = LinSpaced(road_node_[2].x, road_node_[3].x, n);
  ys3 = LinSpaced(road_node_[2].y, road_node_[3].y, n);
  thetas3 = std::vector<double>(n, road_node_[2].theta);
  ss3 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs3, ys3, 0.01, Color::White, 3,
  // "road_segment");

  road_seg_3.resize(xs3.size());
  for (int i = 0; i < road_seg_3.size(); i++) {
    road_seg_3[i].x = xs3[i];
    road_seg_3[i].y = ys3[i];
    road_seg_3[i].theta = thetas3[i];
    road_seg_3[i].s = ss3[i] + road_seg_2.back().s;
  }

  // road segment 4
  std::vector<common::data::TrajectoryPoint> road_seg_4 = spirit_curve_[1];
  // std::reverse(road_seg_4.begin(), road_seg_4.end());
  std::vector<double> xs4(road_seg_4.size()), ys4(road_seg_4.size()),
      thetas4(road_seg_4.size());
  for (int i = 0; i < road_seg_4.size(); i++) {
    road_seg_4[i].x += 0.5 * road_width_ + 2 * min_turn_radiu_ + x_straight_;
    road_seg_4[i].y += min_turn_radiu_ + y_straight_;
    road_seg_4[i].s += road_seg_3.back().s;
    road_seg_4[i].theta -= M_PI;

    xs4[i] = road_seg_4[i].x;
    ys4[i] = road_seg_4[i].y;
    thetas4[i] = road_seg_4[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs4, ys4, 0.01, Color::White, 4,
  // "road_segment");

  // road segment 5
  std::vector<common::data::TrajectoryPoint> road_seg_5;
  std::vector<double> xs5, ys5, thetas5, ss5;
  len = std::sqrt(std::pow(road_node_[4].x - road_node_[5].x, 2) +
                  std::pow(road_node_[4].y - road_node_[5].y, 2));
  n = len / resolution;

  xs5 = LinSpaced(road_node_[4].x, road_node_[5].x, n);
  ys5 = LinSpaced(road_node_[4].y, road_node_[5].y, n);
  thetas5 = std::vector<double>(n, road_node_[5].theta);
  ss5 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs5, ys5, 0.01, Color::White, 5,
  // "road_segment");

  road_seg_5.resize(xs5.size());
  for (int i = 0; i < road_seg_5.size(); i++) {
    road_seg_5[i].x = xs5[i];
    road_seg_5[i].y = ys5[i];
    road_seg_5[i].theta = thetas5[i];
    road_seg_5[i].s = ss5[i] + road_seg_4.back().s;
  }

  // road segment 6
  std::vector<common::data::TrajectoryPoint> road_seg_6 = spirit_curve_[0];
  // std::reverse(road_seg_6.begin(), road_seg_6.end());
  std::vector<double> xs6(road_seg_6.size()), ys6(road_seg_6.size()),
      thetas6(road_seg_6.size());
  for (int i = 0; i < road_seg_6.size(); i++) {
    road_seg_6[i].x += 0.5 * road_width_ + min_turn_radiu_ + x_straight_;
    // road_seg_6[i].y += ;
    road_seg_6[i].s += road_seg_5.back().s;
    road_seg_6[i].theta -= M_PI;

    xs6[i] = road_seg_6[i].x;
    ys6[i] = road_seg_6[i].y;
    thetas6[i] = road_seg_6[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs6, ys6, 0.01, Color::White, 6,
  // "road_segment");

  // road segment 7
  std::vector<common::data::TrajectoryPoint> road_seg_7;
  std::vector<double> xs7, ys7, thetas7, ss7;
  len = std::sqrt(std::pow(road_node_[6].x - road_node_[7].x, 2) +
                  std::pow(road_node_[6].y - road_node_[7].y, 2));
  n = len / resolution;

  xs7 = LinSpaced(road_node_[6].x, road_node_[7].x, n);
  ys7 = LinSpaced(road_node_[6].y, road_node_[7].y, n);
  thetas7 = std::vector<double>(n, road_node_[6].theta);
  ss7 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs7, ys7, 0.01, Color::White, 7,
  // "road_segment");

  road_seg_7.resize(xs7.size());
  for (int i = 0; i < road_seg_7.size(); i++) {
    road_seg_7[i].x = xs7[i];
    road_seg_7[i].y = ys7[i];
    road_seg_7[i].theta = thetas7[i];
    road_seg_7[i].s = ss7[i] + road_seg_6.back().s;
  }

  // road segment 8
  std::vector<common::data::TrajectoryPoint> road_seg_8 = spirit_curve_[3];
  std::vector<double> xs8(road_seg_8.size()), ys8(road_seg_8.size()),
      thetas8(road_seg_8.size());
  for (int i = 0; i < road_seg_8.size(); i++) {
    road_seg_8[i].x += road_width_ * 0.5;
    road_seg_8[i].y += min_turn_radiu_;
    road_seg_8[i].s += 0;
    road_seg_8[i].theta -= 3 * M_PI;

    xs8[i] = road_seg_8[i].x;
    ys8[i] = road_seg_8[i].y;
    thetas8[i] = road_seg_8[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs8, ys8, 0.05, Color::Red, 128,
  // "road_segment"); VisualizationPlot::Trigger();

  std::vector<common::data::TrajectoryPoint> points_1;
  points_1.insert(points_1.end(), road_seg_1.begin(), road_seg_1.end());
  points_1.insert(points_1.end(), road_seg_2.begin(), road_seg_2.end());
  points_1.insert(points_1.end(), road_seg_3.begin(), road_seg_3.end());
  points_1.insert(points_1.end(), road_seg_4.begin(), road_seg_4.end());
  points_1.insert(points_1.end(), road_seg_5.begin(), road_seg_5.end());
  points_1.insert(points_1.end(), road_seg_6.begin(), road_seg_6.end());
  points_1.insert(points_1.end(), road_seg_7.begin(), road_seg_7.end());
  // points_1.insert(points_1.end(), road_seg_8.begin(), road_seg_8.end());

  Road road_1;
  road_1.points = points_1;
  for (auto &tp : road_1.points) {
    tp.road_id = 0;
  }
  road_1.next_seg = {7, 8};
  roads_.emplace_back(road_1);

  // check
  std::vector<double> road_1_xs(points_1.size()), road_1_ys(points_1.size()),
      road_1_thetas(points_1.size()), road_1_ss(points_1.size());
  for (int i = 0; i < points_1.size(); i++) {
    road_1_xs[i] = points_1[i].x;
    road_1_ys[i] = points_1[i].y;
    road_1_thetas[i] = points_1[i].theta;
    road_1_ss[i] = points_1[i].s;
  }

  // road 2
  // road segment 9
  std::vector<common::data::TrajectoryPoint> road_seg_9;
  std::vector<double> xs9, ys9, thetas9, ss9;
  len = std::sqrt(std::pow(road_node_[15].x - road_node_[14].x, 2) +
                  std::pow(road_node_[15].y - road_node_[14].y, 2));
  n = len / resolution;

  xs9 = LinSpaced(road_node_[15].x, road_node_[14].x, n);
  ys9 = LinSpaced(road_node_[15].y, road_node_[14].y, n);
  thetas9 = std::vector<double>(n, road_node_[15].theta);
  ss9 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs9, ys9, 0.01, Color::White, 9,
  // "road_segment");

  road_seg_9.resize(xs9.size());
  for (int i = 0; i < road_seg_9.size(); i++) {
    road_seg_9[i].x = xs9[i];
    road_seg_9[i].y = ys9[i];
    road_seg_9[i].theta = thetas9[i];
    road_seg_9[i].s = ss9[i];
  }

  // road segment 10
  std::vector<common::data::TrajectoryPoint> road_seg_10 = spirit_curve_[3];
  std::vector<double> xs10(road_seg_10.size()), ys10(road_seg_10.size()),
      thetas10(road_seg_10.size());
  for (int i = 0; i < road_seg_10.size(); i++) {
    road_seg_10[i].x += -road_width_ * 0.5 - 2 * min_turn_radiu_ - x_straight_;
    road_seg_10[i].y += min_turn_radiu_;
    road_seg_10[i].s += road_seg_9.back().s;
    road_seg_10[i].theta -= M_PI;

    xs10[i] = road_seg_10[i].x;
    ys10[i] = road_seg_10[i].y;
    thetas10[i] = road_seg_10[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs10, ys10, 0.01, Color::White, 10,
  // "road_segment");

  // road segment 11
  std::vector<common::data::TrajectoryPoint> road_seg_11;
  std::vector<double> xs11, ys11, thetas11, ss11;
  len = std::sqrt(std::pow(road_node_[13].x - road_node_[12].x, 2) +
                  std::pow(road_node_[13].y - road_node_[12].y, 2));
  n = len / resolution;

  xs11 = LinSpaced(road_node_[13].x, road_node_[12].x, n);
  ys11 = LinSpaced(road_node_[13].y, road_node_[12].y, n);
  thetas11 = std::vector<double>(n, road_node_[13].theta);
  ss11 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs11, ys11, 0.01, Color::White, 11,
  // "road_segment");

  road_seg_11.resize(xs11.size());
  for (int i = 0; i < road_seg_11.size(); i++) {
    road_seg_11[i].x = xs11[i];
    road_seg_11[i].y = ys11[i];
    road_seg_11[i].theta = thetas11[i];
    road_seg_11[i].s = ss11[i] + road_seg_10.back().s;
  }

  // road segment 12
  std::vector<common::data::TrajectoryPoint> road_seg_12 = spirit_curve_[2];
  std::vector<double> xs12(road_seg_12.size()), ys12(road_seg_12.size()),
      thetas12(road_seg_12.size());
  for (int i = 0; i < road_seg_12.size(); i++) {
    road_seg_12[i].x += -road_width_ * 0.5 - min_turn_radiu_ - x_straight_;
    road_seg_12[i].y += 2.0 * min_turn_radiu_ + y_straight_;
    road_seg_12[i].s += road_seg_11.back().s;
    road_seg_12[i].theta -= M_PI;

    xs12[i] = road_seg_12[i].x;
    ys12[i] = road_seg_12[i].y;
    thetas12[i] = road_seg_12[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs12, ys12, 0.01, Color::White, 12,
  // "road_segment");

  // road segment 13
  std::vector<common::data::TrajectoryPoint> road_seg_13;
  std::vector<double> xs13, ys13, thetas13, ss13;
  len = std::sqrt(std::pow(road_node_[11].x - road_node_[10].x, 2) +
                  std::pow(road_node_[11].y - road_node_[10].y, 2));
  n = len / resolution;

  xs13 = LinSpaced(road_node_[11].x, road_node_[10].x, n);
  ys13 = LinSpaced(road_node_[11].y, road_node_[10].y, n);
  thetas13 = std::vector<double>(n, road_node_[11].theta);
  ss13 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs13, ys13, 0.01, Color::White, 13,
  // "road_segment");

  road_seg_13.resize(xs13.size());
  for (int i = 0; i < road_seg_13.size(); i++) {
    road_seg_13[i].x = xs13[i];
    road_seg_13[i].y = ys13[i];
    road_seg_13[i].theta = thetas13[i];
    road_seg_13[i].s = ss13[i] + road_seg_12.back().s;
  }

  // road segment 14
  std::vector<common::data::TrajectoryPoint> road_seg_14 = spirit_curve_[1];
  std::vector<double> xs14(road_seg_14.size()), ys14(road_seg_14.size()),
      thetas14(road_seg_14.size());
  for (int i = 0; i < road_seg_14.size(); i++) {
    road_seg_14[i].x += -road_width_ * 0.5;
    road_seg_14[i].y += min_turn_radiu_ + y_straight_;
    road_seg_14[i].theta -= M_PI;
    road_seg_14[i].s += road_seg_13.back().s;

    xs14[i] = road_seg_14[i].x;
    ys14[i] = road_seg_14[i].y;
    thetas14[i] = road_seg_14[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs14, ys14, 0.01, Color::White, 14,
  // "road_segment");

  // road segment 15
  std::vector<common::data::TrajectoryPoint> road_seg_15;
  std::vector<double> xs15, ys15, thetas15, ss15;
  len = std::sqrt(std::pow(road_node_[9].x - road_node_[8].x, 2) +
                  std::pow(road_node_[9].y - road_node_[8].y, 2));
  n = len / resolution;

  xs15 = LinSpaced(road_node_[9].x, road_node_[8].x, n);
  ys15 = LinSpaced(road_node_[9].y, road_node_[8].y, n);
  thetas15 = std::vector<double>(n, road_node_[9].theta);
  ss15 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs15, ys15, 0.01, Color::White, 15,
  // "road_segment");

  road_seg_15.resize(xs15.size());
  for (int i = 0; i < road_seg_15.size(); i++) {
    road_seg_15[i].x = xs15[i];
    road_seg_15[i].y = ys15[i];
    road_seg_15[i].theta = thetas15[i];
    road_seg_15[i].s = ss15[i] + road_seg_14.back().s;
  }

  // road segment 16
  std::vector<common::data::TrajectoryPoint> road_seg_16 = spirit_curve_[0];
  std::vector<double> xs16(road_seg_16.size()), ys16(road_seg_16.size()),
      thetas16(road_seg_16.size());
  for (int i = 0; i < road_seg_16.size(); i++) {
    road_seg_16[i].x += -road_width_ * 0.5 - min_turn_radiu_;
    road_seg_16[i].y += 0;
    road_seg_16[i].theta += -M_PI;
    road_seg_16[i].s += 0;

    xs16[i] = road_seg_16[i].x;
    ys16[i] = road_seg_16[i].y;
    thetas16[i] = road_seg_16[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs16, ys16, 0.01, Color::White, 16,
  // "road_segment");

  std::vector<common::data::TrajectoryPoint> points_2;
  points_2.insert(points_2.end(), road_seg_9.begin(), road_seg_9.end());
  points_2.insert(points_2.end(), road_seg_10.begin(), road_seg_10.end());
  points_2.insert(points_2.end(), road_seg_11.begin(), road_seg_11.end());
  points_2.insert(points_2.end(), road_seg_12.begin(), road_seg_12.end());
  points_2.insert(points_2.end(), road_seg_13.begin(), road_seg_13.end());
  points_2.insert(points_2.end(), road_seg_14.begin(), road_seg_14.end());
  points_2.insert(points_2.end(), road_seg_15.begin(), road_seg_15.end());

  Road road_2;
  road_2.points = points_2;
  for (auto &tp : road_2.points) {
    tp.road_id = 1;
  }
  road_2.next_seg = {5, 6};
  roads_.emplace_back(road_2);

  std::vector<double> road_2_xs(points_2.size()), road_2_ys(points_2.size()),
      road_2_thetas(points_2.size()), road_2_ss(points_2.size());
  for (int i = 0; i < points_2.size(); i++) {
    road_2_xs[i] = points_2[i].x;
    road_2_ys[i] = points_2[i].y;
    road_2_thetas[i] = points_2[i].theta;
    road_2_ss[i] = points_2[i].s;
  }

  // road 3
  // road segment 17
  std::vector<common::data::TrajectoryPoint> road_seg_17;
  std::vector<double> xs17, ys17, thetas17, ss17;
  len = std::sqrt(std::pow(road_node_[16].x - road_node_[17].x, 2) +
                  std::pow(road_node_[16].y - road_node_[17].y, 2));
  n = len / resolution;

  xs17 = LinSpaced(road_node_[16].x, road_node_[17].x, n);
  ys17 = LinSpaced(road_node_[16].y, road_node_[17].y, n);
  thetas17 = std::vector<double>(n, road_node_[16].theta);
  ss17 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs17, ys17, 0.05, Color::White, 127,
  // "road_segment");

  road_seg_17.resize(xs17.size());
  for (int i = 0; i < road_seg_17.size(); i++) {
    road_seg_17[i].x = xs17[i];
    road_seg_17[i].y = ys17[i];
    road_seg_17[i].theta = thetas17[i];
    road_seg_17[i].s = ss17[i];
  }

  // road segment 18
  std::vector<common::data::TrajectoryPoint> road_seg_18 = spirit_curve_[4];
  std::vector<double> xs18(road_seg_18.size()), ys18(road_seg_18.size()),
      thetas18(road_seg_18.size());
  for (int i = 0; i < road_seg_18.size(); i++) {
    road_seg_18[i].x += road_width_ * 0.5 + min_turn_radiu_ + x_straight_;
    road_seg_18[i].y += -road_width_;
    road_seg_18[i].theta += 0;
    road_seg_18[i].s += road_seg_17.back().s;

    xs18[i] = road_seg_18[i].x;
    ys18[i] = road_seg_18[i].y;
    thetas18[i] = road_seg_18[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs18, ys18, 0.01, Color::White, 18,
  // "road_segment");

  // road segment 19
  std::vector<common::data::TrajectoryPoint> road_seg_19;
  std::vector<double> xs19, ys19, thetas19, ss19;
  len = std::sqrt(std::pow(road_node_[18].x - road_node_[19].x, 2) +
                  std::pow(road_node_[18].y - road_node_[19].y, 2));
  n = len / resolution;

  xs19 = LinSpaced(road_node_[18].x, road_node_[19].x, n);
  ys19 = LinSpaced(road_node_[18].y, road_node_[19].y, n);
  thetas19 = std::vector<double>(n, road_node_[18].theta);
  ss19 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs19, ys19, 0.01, Color::White, 19,
  // "road_segment");

  road_seg_19.resize(xs19.size());
  for (int i = 0; i < road_seg_19.size(); i++) {
    road_seg_19[i].x = xs19[i];
    road_seg_19[i].y = ys19[i];
    road_seg_19[i].theta = thetas19[i];
    road_seg_19[i].s = ss19[i] + road_seg_18.back().s;
  }

  // road segment 20
  std::vector<common::data::TrajectoryPoint> road_seg_20 = spirit_curve_[5];
  std::vector<double> xs20(road_seg_20.size()), ys20(road_seg_20.size()),
      thetas20(road_seg_20.size());
  for (int i = 0; i < road_seg_20.size(); i++) {
    road_seg_20[i].x +=
        road_width_ * 0.5 + 2.0 * min_turn_radiu_ + x_straight_ + road_width_;
    road_seg_20[i].y += min_turn_radiu_ + y_straight_;
    road_seg_20[i].theta += 0;
    road_seg_20[i].s += road_seg_19.back().s;

    xs20[i] = road_seg_20[i].x;
    ys20[i] = road_seg_20[i].y;
    thetas20[i] = road_seg_20[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs20, ys20, 0.01, Color::White, 20,
  // "road_segment");

  // road segment 21
  std::vector<common::data::TrajectoryPoint> road_seg_21;
  std::vector<double> xs21, ys21, thetas21, ss21;
  len = std::sqrt(std::pow(road_node_[20].x - road_node_[21].x, 2) +
                  std::pow(road_node_[20].y - road_node_[21].y, 2));
  n = len / resolution;

  xs21 = LinSpaced(road_node_[20].x, road_node_[21].x, n);
  ys21 = LinSpaced(road_node_[20].y, road_node_[21].y, n);
  thetas21 = std::vector<double>(n, road_node_[21].theta);
  ss21 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs21, ys21, 0.01, Color::White, 21,
  // "road_segment");

  road_seg_21.resize(xs21.size());
  for (int i = 0; i < road_seg_21.size(); i++) {
    road_seg_21[i].x = xs21[i];
    road_seg_21[i].y = ys21[i];
    road_seg_21[i].theta = thetas21[i];
    road_seg_21[i].s = ss21[i] + road_seg_20.back().s;
  }

  // road segment 22
  std::vector<common::data::TrajectoryPoint> road_seg_22 = spirit_curve_[6];
  std::vector<double> xs22(road_seg_22.size()), ys22(road_seg_22.size()),
      thetas22(road_seg_22.size());
  for (int i = 0; i < road_seg_22.size(); i++) {
    road_seg_22[i].x += -road_width_ * 0.5 - min_turn_radiu_ - x_straight_;
    road_seg_22[i].y += 2.0 * min_turn_radiu_ + y_straight_ + road_width_;
    road_seg_22[i].theta += 0;
    road_seg_22[i].s += road_seg_21.back().s;

    xs22[i] = road_seg_22[i].x;
    ys22[i] = road_seg_22[i].y;
    thetas22[i] = road_seg_22[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs22, ys22, 0.01, Color::White, 22,
  // "road_segment");

  // road segment 23
  std::vector<common::data::TrajectoryPoint> road_seg_23;
  std::vector<double> xs23, ys23, thetas23, ss23;
  len = std::sqrt(std::pow(road_node_[22].x - road_node_[23].x, 2) +
                  std::pow(road_node_[22].y - road_node_[23].y, 2));
  n = len / resolution;

  xs23 = LinSpaced(road_node_[22].x, road_node_[23].x, n);
  ys23 = LinSpaced(road_node_[22].y, road_node_[23].y, n);
  thetas23 = std::vector<double>(n, road_node_[22].theta);
  ss23 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs23, ys23, 0.01, Color::White, 23,
  // "road_segment");

  road_seg_23.resize(xs23.size());
  for (int i = 0; i < road_seg_23.size(); i++) {
    road_seg_23[i].x = xs23[i];
    road_seg_23[i].y = ys23[i];
    road_seg_23[i].theta = thetas23[i];
    road_seg_23[i].s = ss23[i] + road_seg_22.back().s;
  }

  // road segment 24
  std::vector<common::data::TrajectoryPoint> road_seg_24 = spirit_curve_[7];
  std::vector<double> xs24(road_seg_24.size()), ys24(road_seg_24.size()),
      thetas24(road_seg_24.size());
  for (int i = 0; i < road_seg_24.size(); i++) {
    road_seg_24[i].x +=
        -road_width_ * 1.5 - min_turn_radiu_ * 2.0 - x_straight_;
    road_seg_24[i].y += min_turn_radiu_;
    road_seg_24[i].theta += 0;
    road_seg_24[i].s += road_seg_23.back().s;

    xs24[i] = road_seg_24[i].x;
    ys24[i] = road_seg_24[i].y;
    thetas24[i] = road_seg_24[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs24, ys24, 0.01, Color::White, 24,
  // "road_segment");

  // road segment 25
  std::vector<common::data::TrajectoryPoint> road_seg_25;
  std::vector<double> xs25, ys25, thetas25, ss25;
  len = std::sqrt(std::pow(road_node_[24].x - road_node_[25].x, 2) +
                  std::pow(road_node_[24].y - road_node_[25].y, 2));
  n = len / resolution;

  xs25 = LinSpaced(road_node_[24].x, road_node_[25].x, n);
  ys25 = LinSpaced(road_node_[24].y, road_node_[25].y, n);
  thetas25 = std::vector<double>(n, road_node_[25].theta);
  ss25 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs25, ys25, 0.01, Color::White, 25,
  // "road_segment");

  road_seg_25.resize(xs25.size());
  for (int i = 0; i < road_seg_25.size(); i++) {
    road_seg_25[i].x = xs25[i];
    road_seg_25[i].y = ys25[i];
    road_seg_25[i].theta = thetas25[i];
    road_seg_25[i].s = ss25[i] + road_seg_24.back().s;
  }

  // road segment 26
  std::vector<common::data::TrajectoryPoint> road_seg_26;
  std::vector<double> xs26, ys26, thetas26, ss26;
  len = std::sqrt(std::pow(road_node_[25].x - road_node_[16].x, 2) +
                  std::pow(road_node_[25].y - road_node_[16].y, 2));
  n = len / resolution;

  xs26 = LinSpaced(road_node_[25].x, road_node_[16].x, n);
  ys26 = LinSpaced(road_node_[25].y, road_node_[16].y, n);
  thetas26 = std::vector<double>(n, road_node_[25].theta);
  ss26 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs26, ys26, 0.01, Color::White, 26,
  // "road_segment");

  road_seg_26.resize(xs26.size());
  for (int i = 0; i < road_seg_26.size(); i++) {
    road_seg_26[i].x = xs26[i];
    road_seg_26[i].y = ys26[i];
    road_seg_26[i].theta = thetas26[i];
    road_seg_26[i].s = ss26[i];
  }

  std::vector<common::data::TrajectoryPoint> points_3;
  points_3.insert(points_3.end(), road_seg_17.begin(), road_seg_17.end());
  points_3.insert(points_3.end(), road_seg_18.begin(), road_seg_18.end());
  points_3.insert(points_3.end(), road_seg_19.begin(), road_seg_19.end());
  points_3.insert(points_3.end(), road_seg_20.begin(), road_seg_20.end());
  points_3.insert(points_3.end(), road_seg_21.begin(), road_seg_21.end());
  points_3.insert(points_3.end(), road_seg_22.begin(), road_seg_22.end());
  points_3.insert(points_3.end(), road_seg_23.begin(), road_seg_23.end());
  points_3.insert(points_3.end(), road_seg_24.begin(), road_seg_24.end());
  points_3.insert(points_3.end(), road_seg_25.begin(), road_seg_25.end());

  Road road_3;
  road_3.points = points_3;
  for (auto &tp : road_3.points) {
    tp.road_id = 2;
  }
  road_3.next_seg = {3, 4};
  roads_.emplace_back(road_3);

  std::vector<double> road_3_xs(points_3.size()), road_3_ys(points_3.size()),
      road_3_thetas(points_3.size()), road_3_ss(points_3.size());
  for (int i = 0; i < points_3.size(); i++) {
    road_3_xs[i] = points_3[i].x;
    road_3_ys[i] = points_3[i].y;
    road_3_thetas[i] = points_3[i].theta;
    road_3_ss[i] = points_3[i].s;
  }
  // VisualizationPlot::PlotPoints(road_3_xs, road_3_ys, 0.05, Color::White,
  // 2226, "road_segment"); VisualizationPlot::Trigger();

  // road 4
  // road_seg_24为左下弧线
  Road road_4;
  road_4.points = road_seg_26;
  for (auto &tp : road_4.points) {
    tp.road_id = 3;
  }
  road_4.next_seg = {2};
  roads_.emplace_back(road_4);
  std::vector<double> road_4_xs(road_seg_26.size()),
      road_4_ys(road_seg_26.size()), road_4_thetas(road_seg_26.size()),
      road_4_ss(road_seg_26.size());
  for (int i = 0; i < road_seg_26.size(); i++) {
    road_4_xs[i] = road_seg_26[i].x;
    road_4_ys[i] = road_seg_26[i].y;
    road_4_thetas[i] = road_seg_26[i].theta;
    road_4_ss[i] = road_seg_26[i].s;
  }
  // VisualizationPlot::PlotPoints(road_4_xs, road_4_ys, 0.05, Color::White, 2226,
  //                               "road_segment");
  // VisualizationPlot::Trigger();
  // road 5
  // road segment 27
  std::vector<common::data::TrajectoryPoint> road_seg_27 = spirit_curve_[4];
  std::vector<double> xs27(road_seg_27.size()), ys27(road_seg_27.size()),
      thetas27(road_seg_27.size());
  for (int i = 0; i < road_seg_27.size(); i++) {
    road_seg_27[i].x += -road_width_ * 0.5 - min_turn_radiu_;
    road_seg_27[i].y += -road_width_;
    road_seg_27[i].theta += 0;
    road_seg_27[i].s += 0;

    xs27[i] = road_seg_27[i].x;
    ys27[i] = road_seg_27[i].y;
    thetas27[i] = road_seg_27[i].theta;
  }
  // VisualizationPlot::PlotPoints(xs27, ys27, 0.01, Color::White, 27,
  // "road_segment");

  Road road_5;
  road_5.points = road_seg_27;
  for (auto &tp : road_5.points) {
    tp.road_id = 4;
  }
  road_5.next_seg = {0};
  roads_.emplace_back(road_5);

  std::vector<double> road_5_xs(road_seg_27.size()),
      road_5_ys(road_seg_27.size()), road_5_thetas(road_seg_27.size()),
      road_5_ss(road_seg_27.size());
  for (int i = 0; i < road_seg_27.size(); i++) {
    road_5_xs[i] = road_seg_27[i].x;
    road_5_ys[i] = road_seg_27[i].y;
    road_5_thetas[i] = road_seg_27[i].theta;
    road_5_ss[i] = road_seg_27[i].s;
  }

  // road 6
  Road road_6;
  road_6.points = road_seg_16;
  for (auto &tp : road_6.points) {
    tp.road_id = 5;
  }
  road_6.next_seg = {1};
  roads_.emplace_back(road_6);

  std::vector<double> road_6_xs(road_seg_16.size()),
      road_6_ys(road_seg_16.size()), road_6_thetas(road_seg_16.size()),
      road_6_ss(road_seg_16.size());
  for (int i = 0; i < road_seg_16.size(); i++) {
    road_6_xs[i] = road_seg_16[i].x;
    road_6_ys[i] = road_seg_16[i].y;
    road_6_thetas[i] = road_seg_16[i].theta;
    road_6_ss[i] = road_seg_16[i].s;
  }

  // road 7
  // road segment 28
  std::vector<common::data::TrajectoryPoint> road_seg_28 = spirit_curve_[7];
  std::vector<double> xs28(road_seg_28.size()), ys28(road_seg_28.size()),
      thetas28(road_seg_28.size());
  for (int i = 0; i < road_seg_28.size(); i++) {
    road_seg_28[i].x += -road_width_ * 0.5;
    road_seg_28[i].y += min_turn_radiu_;
    road_seg_28[i].theta += 0;

    xs28[i] = road_seg_28[i].x;
    ys28[i] = road_seg_28[i].y;
    thetas28[i] = road_seg_28[i].theta;
  }
  // for (int i = 0, j = road_seg_28.size() - 1; i < j; i++, j--) {
  //   double tmp = road_seg_28[i].s;
  //   road_seg_28[i].s = road_seg_28[j].s;
  //   road_seg_28[j].s = tmp;
  // }
  // VisualizationPlot::PlotPoints(xs28, ys28, 0.01, Color::White, 28,
  // "road_segment");

  Road road_7;
  road_7.points = road_seg_28;
  for (auto &tp : road_7.points) {
    tp.road_id = 6;
  }
  road_7.next_seg = {2};
  roads_.emplace_back(road_7);

  std::vector<double> road_7_xs(road_seg_28.size()),
      road_7_ys(road_seg_28.size()), road_7_thetas(road_seg_28.size()),
      road_7_ss(road_seg_28.size());
  for (int i = 0; i < road_seg_28.size(); i++) {
    road_7_xs[i] = road_seg_28[i].x;
    road_7_ys[i] = road_seg_28[i].y;
    road_7_thetas[i] = road_seg_28[i].theta;
    road_7_ss[i] = road_seg_28[i].s;
  }

  // road 8
  Road road_8;
  road_8.points = road_seg_8;
  for (auto &tp : road_8.points) {
    tp.road_id = 7;
  }
  road_8.next_seg = {0};
  roads_.emplace_back(road_8);

  std::vector<double> road_8_xs(road_seg_8.size()),
      road_8_ys(road_seg_8.size()), road_8_thetas(road_seg_8.size()),
      road_8_ss(road_seg_8.size());
  for (int i = 0; i < road_seg_8.size(); i++) {
    road_8_xs[i] = road_seg_8[i].x;
    road_8_ys[i] = road_seg_8[i].y;
    road_8_thetas[i] = road_seg_8[i].theta;
    road_8_ss[i] = road_seg_8[i].s;
  }

  // road 9
  // road segment 29
  std::vector<common::data::TrajectoryPoint> road_seg_29;
  std::vector<double> xs29, ys29, thetas29, ss29;
  len = std::sqrt(std::pow(road_node_[7].x - road_node_[15].x, 2) +
                  std::pow(road_node_[7].y - road_node_[15].y, 2));
  n = len / resolution;

  xs29 = LinSpaced(road_node_[7].x, road_node_[15].x, n);
  ys29 = LinSpaced(road_node_[7].y, road_node_[15].y, n);
  thetas29 = std::vector<double>(n, road_node_[15].theta);
  ss29 = LinSpaced(0, len, n);
  // VisualizationPlot::PlotPoints(xs29, ys29, 0.01, Color::White, 29,
  // "road_segment");

  road_seg_29.resize(xs29.size());
  for (int i = 0; i < road_seg_29.size(); i++) {
    road_seg_29[i].x = xs29[i];
    road_seg_29[i].y = ys29[i];
    road_seg_29[i].theta = thetas29[i];
    road_seg_29[i].s = ss29[i];
  }

  Road road_9;
  road_9.points = road_seg_29;
  for (auto &tp : road_9.points) {
    tp.road_id = 8;
  }
  road_9.next_seg = {1};
  roads_.emplace_back(road_9);

  std::vector<double> road_9_xs(road_seg_29.size()),
      road_9_ys(road_seg_29.size()), road_9_thetas(road_seg_29.size()),
      road_9_ss(road_seg_29.size());
  for (int i = 0; i < road_seg_29.size(); i++) {
    road_9_xs[i] = road_seg_29[i].x;
    road_9_ys[i] = road_seg_29[i].y;
    road_9_thetas[i] = road_seg_29[i].theta;
    road_9_ss[i] = road_seg_29[i].s;
  }

  for (int i = 0; i < roads_.size(); i++) {
    std::vector<int> repeat_record;
    for (int j = 0; j < roads_[i].points.size() - 1; j++) {
      if (hypot(roads_[i].points[j + 1].x - roads_[i].points[j].x,
                roads_[i].points[j + 1].y - roads_[i].points[j].y) < 0.001) {
        repeat_record.emplace_back(j + 1);
      }
    }

    std::vector<common::data::TrajectoryPoint> points;
    for (int j = 0; j < roads_[i].points.size(); j++) {
      bool flag = false;
      for (auto id : repeat_record) {
        if (j == id) {
          flag = true;
          break;
        }
      }
      if (!flag) points.emplace_back(roads_[i].points[j]);
    }
    roads_[i].points = points;
  }

  road_segment_.emplace_back(road_seg_1);
  road_segment_.emplace_back(road_seg_2);
  road_segment_.emplace_back(road_seg_3);
  road_segment_.emplace_back(road_seg_4);
  road_segment_.emplace_back(road_seg_5);
  road_segment_.emplace_back(road_seg_6);
  road_segment_.emplace_back(road_seg_7);
  road_segment_.emplace_back(road_seg_8);
  road_segment_.emplace_back(road_seg_9);
  road_segment_.emplace_back(road_seg_10);
  road_segment_.emplace_back(road_seg_11);
  road_segment_.emplace_back(road_seg_12);
  road_segment_.emplace_back(road_seg_13);
  road_segment_.emplace_back(road_seg_14);
  road_segment_.emplace_back(road_seg_15);
  road_segment_.emplace_back(road_seg_16);
  road_segment_.emplace_back(road_seg_17);
  road_segment_.emplace_back(road_seg_18);
  road_segment_.emplace_back(road_seg_19);
  road_segment_.emplace_back(road_seg_20);
  road_segment_.emplace_back(road_seg_21);
  road_segment_.emplace_back(road_seg_22);
  road_segment_.emplace_back(road_seg_23);
  road_segment_.emplace_back(road_seg_24);
  road_segment_.emplace_back(road_seg_25);
  road_segment_.emplace_back(road_seg_26);
  road_segment_.emplace_back(road_seg_27);
  road_segment_.emplace_back(road_seg_28);
  road_segment_.emplace_back(road_seg_29);

  // VisualizationPlot::Trigger();
}

void MapGenerator::GenerateMap() {
  GenerateBlocks();
  GenerateRoadNodes();
  GenerateCurves();
  GenerateRoads();
}

void MapGenerator::Visualize() {
  // blocks
  for (int i = 0; i < blocks_.size(); i++) {
    VisualizationPlot::PlotPolygon(blocks_[i].GetPolygon(), 0.01, Color::Blue,
                                   i, "blocks");
  }

  // road_nodes
  std::vector<double> xs, ys;
  for (int i = 0; i < road_node_.size(); i++) {
    xs.push_back(road_node_[i].x);
    ys.push_back(road_node_[i].y);
  }
  VisualizationPlot::PlotPoints(xs, ys, 0.01, Color::White, 1, "road_node_");

  // field
  Polygon2d field =
      Polygon2d(Box2d(Vec2d(0, 0.5), M_PI_2, map_width_, map_height_));
  VisualizationPlot::PlotPolygon(field, 0.01, Color::Blue, 1, "map");

  // roads
  for (int i = 0; i < roads_.size(); i++) {
    xs.clear(), ys.clear();
    for (int j = 0; j < roads_[i].points.size(); j++) {
      xs.push_back(roads_[i].points[j].x);
      ys.push_back(roads_[i].points[j].y);
    }

    Color c = Color::White;
    if (i >= 3) c = Color::Green;
    c.set_a(0.8);
    VisualizationPlot::Plot(xs, ys, 0.01, c, i, "road");
  }

  VisualizationPlot::Trigger();
}

}  // namespace map_generator