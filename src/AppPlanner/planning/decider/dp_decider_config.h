//
// Created by Tsq on 2022/3/14.
//
#pragma once

struct DpDeciderConfig {
  // 障碍物代价，应设置的较大来避免与障碍物碰撞
  double w_obstacle = 10000;

  // 横向位移代价，越大轨迹越贴近指引线
  double w_lateral = 10;
  // 横向偏移代价
  double w_lateral_change = 0.2;

  // 横向速度代价
  double w_lateral_velocity_change = 1.0;

  // 障碍物远离代价
  double w_obstacle_evasion = 5.0;
  double w_obstacle_evasion_exp_denom = 10.0;  // larger exp means slower decreasing

  double collision_check_resolution = 0.02;
  double cost_out_lane = 50;
  double safe_distance = 0.3;
  double s_resolution = 0.05;
  // 最大处理长度
  double horizon = 12;
  // 速度变化代价
  double w_v_change = 0.2;

  // 加速度代价
  double w_a = 1;

  // 效率代价
  double w_efficiency = 0.2;

  // 违反最大加速度的代价
  double w_invalid_a = 1000;
};
