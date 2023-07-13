#include <chrono>
#include "ros/ros.h"
#include <iostream>
#include "algorithm"
#include "std_msgs/String.h"
#include "nlohmann/json.hpp"
#include "common/math/vec2d.h"
#include "map/map_generator.h"
#include "common/util/vector.h"
#include "planning/app_planner.h"
#include "common/math/polygon2d.h"
#include "sandbox_msgs/Trajectory.h"
#include "common/visualization_plot.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include "planning/decider/dp_path_decider.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
using json = nlohmann::json;
class APP {
 public:
  APP(const ros::NodeHandle& nh) : node_handle_(nh) {
    VisualizationPlot::Init(node_handle_, "world", "markers");
    traj_pub_ = node_handle_.advertise<sandbox_msgs::Trajectory>("/traj", 1);
    info_sub_ =
        node_handle_.subscribe("/nokov_info", 1, &APP::InfoCallback, this);
    clicked_sub_ = node_handle_.subscribe("/clicked_point", 1,
                                          &APP::GetClickedPointCallback, this);
    ros::Duration(2).sleep();
    map_generator_.GenerateMap();
    dp_path_decider_ = std::make_shared<dp_path_decider::DpPathDecider>(
        dp_decider_config_, frame_.env());
    GetRefLine(map_generator_.roads());
    timer_ = node_handle_.createTimer(ros::Duration(0.2), &APP::Plan, this);
    virtual_locating_timer_ = node_handle_.createTimer(
        ros::Duration(0.1), &APP::GenerateVirtualLocating, this);
  }
  void Plan(const ros::TimerEvent& evt) {
    // test
    current_pose_received_ = true;
    if (current_pose_received_) {
      int index = 0;
      if (FindTrajIndex(index)) {
        if (index > int(pre_traj_msg_.points.size() * 0.8)) {
          is_replan_ = true;
        }
      }
      if (CheckCollison(frame_, app_ref_) || is_replan_) {
        ProcesByDP();
        PublishInfoToAPP();
        ProcessAPPRef();
      }
    }
  }
  void ProcesByDP() {
    common::data::Trajectory dp_result;
    Pose start_pose(traj_ref_[5].x, traj_ref_[5].y, traj_ref_[5].theta);
    Pose goal_pose(traj_ref_[int(traj_ref_.size() * 0.95)].x,
                   traj_ref_[int(traj_ref_.size() * 0.95)].y,
                   traj_ref_[int(traj_ref_.size() * 0.95)].theta);
    auto input_reference = common::data::DiscretizedTrajectory(traj_ref_);
    if (dp_path_decider_->Plan(input_reference, start_pose, goal_pose, 0,
                               dp_result)) {
      auto xx =
          common::util::LinSpaced(dp_result.back().x, dp_result.front().x, 20);
      auto yy =
          common::util::LinSpaced(dp_result.back().y, dp_result.front().y, 20);
      for (int i = 0; i < xx.size(); i++) {
        common::data::TrajectoryPoint tp;
        tp.x = xx[i];
        tp.y = yy[i];
        if (i < xx.size() - 1) {
          tp.theta = std::atan2(yy[i + 1] - yy[i], xx[i + 1] - xx[i]);
        } else {
          tp.theta = std::atan2(yy[i] - yy[i - 1], xx[i] - xx[i - 1]);
        }
        dp_result.push_back(tp);
      }
      std::vector<double> xs, ys;
      for (auto& i : dp_result) {
        xs.push_back(i.x);
        ys.push_back(i.y);
      }
      VisualizationPlot::Plot(xs, ys, 0.01, Color::Red, 6, "dp_result");
      VisualizationPlot::Trigger();
      dp_ref_ = dp_result;
      is_replan_ = false;
    }
  }
  void GenerateVirtualLocating(const ros::TimerEvent& evt) {
    return;
    if (pre_traj_msg_.points.empty()) {
      return;
    }
    count_ += 1;
    double dt = 1 * count_;
    current_pose_.set_x(pre_traj_msg_.points[dt].x);
    current_pose_.set_y(pre_traj_msg_.points[dt].y);
    current_pose_.set_theta(pre_traj_msg_.points[dt].yaw);
    Polygon2d vec;
    vec = Polygon2d(frame_.env()->vehicle.GenerateBox(
        Pose(pre_traj_msg_.points[dt].x, pre_traj_msg_.points[dt].y,
             pre_traj_msg_.points[dt].yaw)));
    VisualizationPlot::PlotPolygon(vec, 0.01, Color::Cyan, 8, "vehicle");
    VisualizationPlot::Trigger();
  }

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher traj_pub_, path_pub_;
  ros::Subscriber info_sub_, clicked_sub_;
  map_generator::MapGenerator map_generator_;
  std::vector<Polygon2d> obstacles_;
  common::data::Trajectory traj_ref_, dp_ref_, app_ref_;
  sandbox_msgs::Trajectory pre_traj_msg_;
  ros::Timer timer_, virtual_locating_timer_;
  std::vector<Vec2d> clicked_polygon_;
  DpDeciderConfig dp_decider_config_;
  std::shared_ptr<dp_path_decider::DpPathDecider> dp_path_decider_;
  Frame frame_;
  Pose current_pose_;
  bool current_pose_received_ = false;
  bool is_replan_ = true;
  double nominal_v_ = 0.1;
  double start_time_ = 0.0;
  int count_ = 0;

  void GetClickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg) {
    Vec2d pt(msg->point.x, msg->point.y);
    bool is_closed =
        std::any_of(clicked_polygon_.begin(), clicked_polygon_.end(),
                    [&](Vec2d& x) { return x.DistanceTo(pt) < 0.2; });
    if (!is_closed) {
      clicked_polygon_.push_back(pt);
      std::vector<double> xs, ys;
      for (auto& i : clicked_polygon_) {
        xs.push_back(i.x());
        ys.push_back(i.y());
      }
      std::cout << "clicked_polygon_:" << clicked_polygon_.size() << std::endl;
      VisualizationPlot::Plot(xs, ys, 0.05, Color::Yellow, 9, "Temp");
      VisualizationPlot::Trigger();
    } else {
      ROS_INFO("closing polygon");
      obstacles_.clear();
      GetStaticInfo();
      Polygon2d obs = Polygon2d(clicked_polygon_);
      obstacles_.push_back(obs);
      clicked_polygon_.clear();
      VisualizationPlot::PlotPoints({}, {}, 0.1, Color::Yellow, 9,
                                    "road_node_");
    }
  }
  void ProcessAPPRef() {
    common::data::Trajectory pubulish_traj = {};
    if (!app_ref_.empty()) {
      int index = -1;
      FindStitchIndex(app_ref_, index);
      double s_integral = 0;
      double time = 0;
      for (int i = index; i < app_ref_.size(); ++i) {
        common::data::TrajectoryPoint tp;
        tp.x = app_ref_[i].x;
        tp.y = app_ref_[i].y;
        tp.s = s_integral;
        double theta = 0;
        if (i < app_ref_.size() - 1) {
          theta = std::atan2(app_ref_[i + 1].y - app_ref_[i].y,
                             app_ref_[i + 1].x - app_ref_[i].x);
          s_integral += std::hypot(app_ref_[i + 1].y - app_ref_[i].y,
                                   app_ref_[i + 1].x - app_ref_[i].x);
        } else {
          theta = std::atan2(app_ref_[i].y - app_ref_[i - 1].y,
                             app_ref_[i].x - app_ref_[i - 1].x);
        }
        tp.theta = theta;
        tp.v = nominal_v_;
        tp.t = tp.s / tp.v;
        pubulish_traj.emplace_back(tp);
      }
      SendTrajectory(1, pubulish_traj);
    }
  }
  void PublishInfoToAPP() {
    int cur_index = -1;
    if (!FindStitchIndex(dp_ref_, cur_index)) {
      ROS_WARN("cannot find proper stich index!");
    }
    common::data::Trajectory pubulish_ref = {};
    pubulish_ref.insert(pubulish_ref.end(), dp_ref_.begin() + cur_index,
                        dp_ref_.end());
    pubulish_ref.insert(pubulish_ref.end(), dp_ref_.begin(),
                        dp_ref_.begin() + cur_index);
    for (int i = 0; i < pubulish_ref.size(); ++i) {
      if (i < pubulish_ref.size() - 1) {
        pubulish_ref[i].theta =
            std::atan2(pubulish_ref[i + 1].y - pubulish_ref[i].y,
                       pubulish_ref[i + 1].x - pubulish_ref[i].x);
      } else {
        pubulish_ref[i].theta =
            std::atan2(pubulish_ref[i].y - pubulish_ref[i - 1].y,
                       pubulish_ref[i].x - pubulish_ref[i - 1].x);
      }
    }
    APP_Planner app_planner(pubulish_ref, pubulish_ref.front(),
                            pubulish_ref.back(), frame_, 10);
    using namespace std::chrono;
    auto start = system_clock::now();
    if (app_planner.Plan(app_ref_)) {
      ROS_INFO("success!");
    } else {
      ROS_WARN("failed!");
    }
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    std::cout << "app_planner time:"
         << double(duration.count()) * microseconds::period::num /
                microseconds::period::den
         << "s" << std::endl;
    std::vector<double> xx, yy, theta, obs_xs, obs_ys;
    for (auto& pt : app_ref_) {
      xx.emplace_back(pt.x);
      yy.emplace_back(pt.y);
      theta.emplace_back(pt.theta);
    }
  }
  bool FindPathIndex(int& index, const std::vector<Pose>& path) {
    double min_distance =
        hypot(current_pose_.x() - path[0].x(), current_pose_.y() - path[0].y());
    int station_index = 0;

    for (int i = 0; i < path.size(); i++) {
      double distance = hypot(current_pose_.x() - path[i].x(),
                              current_pose_.y() - path[i].y());
      if (distance < min_distance) {
        station_index = i;
        min_distance = distance;
      }
    }

    if (min_distance > 0.5) {
      return false;
    }
    index = station_index;
    return true;
  }
  bool FindTrajIndex(int& index) {
    if (pre_traj_msg_.points.size() != 0) {
      double min_distance =
          hypot(current_pose_.x() - pre_traj_msg_.points[0].x,
                current_pose_.y() - pre_traj_msg_.points[0].x);
      int station_index = 0;
      for (int i = 0; i < pre_traj_msg_.points.size(); i++) {
        double distance = hypot(current_pose_.x() - pre_traj_msg_.points[i].x,
                                current_pose_.y() - pre_traj_msg_.points[i].y);
        if (distance < min_distance) {
          station_index = i;
          min_distance = distance;
        }
      }
      if (min_distance > 0.5) {
        return false;
      }
      index = station_index;
      return true;
    } else {
      return false;
    }
  }
  bool FindStitchIndex(common::data::Trajectory dp_ref, int& index) {
    double min_distance =
        hypot(current_pose_.x() - dp_ref[0].x, current_pose_.y() - dp_ref[0].y);
    int station_index = 0;

    for (int i = 0; i < dp_ref.size(); i++) {
      double distance = hypot(current_pose_.x() - dp_ref[i].x,
                              current_pose_.y() - dp_ref[i].y);
      if (distance < min_distance) {
        station_index = i;
        min_distance = distance;
      }
    }
    if (min_distance > 0.5) {
      return false;
    }
    index = station_index;

    return true;
  }
  void GetRefLine(const std::vector<map_generator::Road>& roads_) {
    std::vector<double> ref_xs, ref_ys;
    for (int i = 0; i < roads_.size(); i++) {
      if (i == 2 || i == 3) {
        for (int j = 0; j < roads_[i].points.size(); j++) {
          ref_xs.push_back(roads_[i].points[j].x);
          ref_ys.push_back(roads_[i].points[j].y);
        }
      }
    }
    double s_integral = 0;
    for (int i = 0; i < roads_.size(); i++) {
      if (i == 2 || i == 3) {
        for (int j = 0; j < roads_[i].points.size(); j++) {
          common::data::TrajectoryPoint pt;
          pt.x = (roads_[i].points[j].x);
          pt.y = (roads_[i].points[j].y);
          pt.theta = (roads_[i].points[j].theta);
          pt.v = (nominal_v_);
          pt.s = s_integral;
          if (j < roads_[i].points.size() - 1) {
            s_integral +=
                std::hypot(roads_[i].points[j + 1].y - roads_[i].points[j].y,
                           roads_[i].points[j + 1].x - roads_[i].points[j].x);
          }

          traj_ref_.emplace_back(pt);
        }
      }
    }
    dp_ref_ = traj_ref_;
    current_pose_ = Pose(traj_ref_[0].x, traj_ref_[0].y, traj_ref_[0].theta);
    Color c = Color::White;
    c.set_a(0.8);
    VisualizationPlot::Plot(ref_xs, ref_ys, 0.01, c, 0, "road");

    VisualizationPlot::Trigger();
  }
  void VisualizeObstacle(const std::vector<Polygon2d>& obstacles) {
    int count = 0;
    for (auto obs : obstacles) {
      VisualizationPlot::PlotPolygon(obs, 0.01, Color::Yellow, count + 11,
                                     "blocks");
      count++;
    }
    VisualizationPlot::Trigger();
  }
  bool CheckCollison(Frame& frame, const common::data::Trajectory& path) {
    if (obstacles_.empty()) {
      GetStaticInfo();
    }
    std::vector<common::Obstacle> obs;
    for (auto& ob : obstacles_) {
      obs.emplace_back(common::Obstacle(ob));
    }
    Polygon2d field =
        Polygon2d(common::math::Box2d(Vec2d(0, 0.5), M_PI_2, 2.0, 3.0));
    frame_.env()->obstacles = obs;
    frame_.env()->Visualize();
    frame.env()->obstacles = obs;
    for (int i = 0; i < path.size(); ++i) {
      auto vec_box = frame_.env()->vehicle.GenerateBox(
          Pose(path[i].x, path[i].y, path[i].theta));
      if (frame_.env()->CheckCollision(vec_box)) {
        return true;
      }
    }
    return false;
  }

  void GetStaticInfo() {
    // add virtual obs
 Polygon2d vir_obs =
        Polygon2d(common::math::Box2d(Vec2d(1.6, 0.15), 0.1, 0.2, 0.2));
    Polygon2d vir_obs1 =
        Polygon2d(common::math::Box2d(Vec2d(1.65, 0.5), 0.2, 0.2, 0.1));
    Polygon2d vir_obs2 =
        Polygon2d(common::math::Box2d(Vec2d(1.5, 1.2), 0.3, 0.2, 0.25));
    Polygon2d vir_obs3 =
        Polygon2d(common::math::Box2d(Vec2d(0.7, 1.1), -0.1, 0.1, 0.2));
    Polygon2d vir_obs4 =
        Polygon2d(common::math::Box2d(Vec2d(0, 1.45), -0.2, 0.1, 0.2));
    Polygon2d vir_obs5 =
        Polygon2d(common::math::Box2d(Vec2d(-1.5, -0.2), 0.3, 0.2, 0.2));
    Polygon2d vir_obs6 =
        Polygon2d(common::math::Box2d(Vec2d(-1.65, 0.5), 0.4, 0.2, 0.1));
    Polygon2d vir_obs7 =
        Polygon2d(common::math::Box2d(Vec2d(-1.5, 1.2), -0.2, 0.2, 0.25));
    Polygon2d vir_obs8 =
        Polygon2d(common::math::Box2d(Vec2d(-0.7, 1.1), 0.3, 0.1, 0.2));
    Polygon2d vir_obs9 =
        Polygon2d(common::math::Box2d(Vec2d(-0.3, -0.1), -0.4, 0.1, 0.25));
    Polygon2d vir_obs10 =
        Polygon2d(common::math::Box2d(Vec2d(0.2, -0.5), 0.2, 0.1, 0.2));
    obstacles_.push_back(vir_obs1);
    obstacles_.push_back(vir_obs2);
    obstacles_.push_back(vir_obs3);
    obstacles_.push_back(vir_obs4);
    obstacles_.push_back(vir_obs5);
    obstacles_.push_back(vir_obs6);
    obstacles_.push_back(vir_obs7);
    obstacles_.push_back(vir_obs8);
    obstacles_.push_back(vir_obs9);
    obstacles_.push_back(vir_obs10);
    VisualizeObstacle(obstacles_);
  }
  void InfoCallback(const std_msgs::StringConstPtr& msg) {
    json obj;
    try {
      obj = json::parse(msg->data);
    } catch (std::exception& ex) {
      ROS_ERROR("parsing json: %s", ex.what());
      return;
    }
    if (obj["vehicles"].contains("1")) {
      std::vector<double> pose;
      obj["vehicles"]["1"].get_to(pose);
      current_pose_ = Pose(pose[0], pose[1], pose[2]);
      current_pose_received_ = true;
    }
    obstacles_.clear();
    for (auto& ob : obj["obstacles"]) {
      Polygon2d ob_poly;
      ob.get_to(ob_poly);
      obstacles_.push_back(ob_poly);
    }
    GetStaticInfo();
  }
  void SendTrajectory(int ID, common::data::Trajectory& traj) {
    int index = -1;
    if (FindTrajIndex(index)) {
      start_time_ = pre_traj_msg_.points[index].time + 0.1;
    }
    sandbox_msgs::Trajectory traj_msg;
    std::vector<double> xs, ys;
    traj_msg.target = ID;
    traj_msg.header.frame_id = "world";
    traj_msg.header.stamp = ros::Time::now();
    double s_integral = 0;
    for (int i = 0; i < traj.size(); i++) {
      sandbox_msgs::TrajectoryPoint tp;
      xs.emplace_back(traj[i].x);
      ys.emplace_back(traj[i].y);
      tp.x = traj[i].x;
      tp.y = traj[i].y;
      tp.yaw = traj[i].theta;
      tp.velocity = traj[i].v;
      tp.acceleration = traj[i].a;
      tp.time = s_integral / tp.velocity + start_time_;
      if (i < traj.size() - 1) {
        s_integral +=
            std::hypot(traj[i + 1].y - traj[i].y, traj[i + 1].x - traj[i].x);
      }
      traj_msg.points.push_back(tp);
    }
    pre_traj_msg_ = traj_msg;
    traj_pub_.publish(traj_msg);
    Polygon2d vec;
    vec = Polygon2d(frame_.env()->vehicle.GenerateBox(
        Pose(traj_msg.points.front().x, traj_msg.points.front().y,
             traj_msg.points.front().yaw)));
    VisualizationPlot::Plot(xs, ys, 0.01, Color::Green, 7, "vehicle traj");
    VisualizationPlot::Trigger();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "APP_experiments");
  ros::NodeHandle node;
  APP planner(node);
  ros::spin();
  return 0;
}
