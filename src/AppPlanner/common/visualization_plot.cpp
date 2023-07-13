//
// Created by yenkn on 4/13/21.
//
#include <common/math/math_utils.h>
#include <tf/transform_datatypes.h>

#include "visualization_plot.h"

namespace VisualizationPlot {
namespace {
std::string frame_ = "map";

ros::Publisher publisher_;
visualization_msgs::MarkerArray arr_;
}
}

void VisualizationPlot::Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic) {
  frame_ = frame;
  publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 1000, true);
  sleep(1);
}

void
VisualizationPlot::Plot(const Vector &xs, const Vector &ys, double width, Color color, int id, const std::string &ns) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    pt.z = 0.1 * id;
    msg.points.push_back(pt);
  }

  arr_.markers.push_back(msg);
}

void VisualizationPlot::Plot(const VisualizationPlot::Vector &xs, const VisualizationPlot::Vector &ys, double width,
                             const std::vector <Color> &color, int id, const std::string &ns) {
  assert(xs.size() == color.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(color[i].toColorRGBA());
  }

  arr_.markers.push_back(msg);
}


void VisualizationPlot::PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color, int id,
                                    const std::string &ns) {
  auto xxs = xs;
  auto yys = ys;
  xxs.push_back(xxs[0]);
  yys.push_back(yys[0]);
  Plot(xxs, yys, width, color, id, ns);
}

void VisualizationPlot::PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                                    const std::string &ns) {
  std::vector<double> xs, ys;
  for (auto &pt: polygon.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  PlotPolygon(xs, ys, width, color, id, ns);
}

void VisualizationPlot::PlotPath(const Vector &xs, const Vector &ys, const Vector &thetas, double bias, double width, int id,
                                 const std::string &ns) {
  std::vector<Color> colors(xs.size());
  auto gears = common::math::GetPathGears(xs, ys, thetas);

  for (size_t i = 0; i < xs.size()-1; i++) {
    colors[i] = Color::fromHSV(2 * bias - (gears[i] ? 1.0 : -1.0) * bias, 1.0, 1.0);
    colors[i].set_a(std::min(1.0, width / 0.02));
  }
  colors[colors.size()-1] = colors.back();

  Plot(xs, ys, width, colors, id, ns);
}

void VisualizationPlot::PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity,
                                       double width, int id, const std::string &ns) {
  std::vector<Color> colors(xs.size());

  for (size_t i = 0; i < xs.size(); i++) {
    double percent = (vs[i] / max_velocity);
    colors[i] = Color::fromHSV(120.0f - percent * 120, 1.0, 1.0);
  }

  Plot(xs, ys, width, colors, id, ns);
}

void VisualizationPlot::PlotPoints(const Vector &xs, const Vector &ys, double width, const Color &color, int id,
                                   const std::string &ns) {
  assert(xs.size() == ys.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = !xs.empty() ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETE;
  msg.type = visualization_msgs::Marker::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
  }

  arr_.markers.push_back(msg);
}

void VisualizationPlot::PlotPose(const Pose &pose, double width, const Color &color, int id,
                                   const std::string &ns) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns.empty() ? "Pose" : ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::ARROW;
  msg.pose.position.x = pose.x();
  msg.pose.position.y = pose.y();
  msg.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());
  msg.scale.x = width * 2; // arrow length
  msg.scale.y = width * 0.2; // arrow width
  msg.scale.z = width; // arrow height
  msg.color = color.toColorRGBA();

  arr_.markers.push_back(msg);
}

void VisualizationPlot::Trigger() {
  // double start_time = ros::Time::now().toSec();
  // while (ros::Time::now().toSec() - start_time < 1.0) {
  //   publisher_.publish(arr_);
  //   sleep(0.1);
  // }
  publisher_.publish(arr_);
  arr_.markers.clear();
  // ros::spinOnce();
}

void VisualizationPlot::Clear() {
  arr_.markers.clear();

  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.ns = "Markers";

  msg.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(msg);
  publisher_.publish(arr);
}
