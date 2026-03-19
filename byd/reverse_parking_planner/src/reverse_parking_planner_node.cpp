// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "reverse_parking_planner/reverse_parking_planner_node.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>

namespace reverse_parking_planner
{

/**
 * @brief ReverseParkingPlannerNode 构造函数。
 * @details 负责：
 * - 声明并读取 ROS2 参数
 * - 初始化 Reeds-Shepp 路径规划器
 * - 初始化 TF buffer/listener、订阅/发布器、服务、定时器
 */
ReverseParkingPlannerNode::ReverseParkingPlannerNode(const rclcpp::NodeOptions & options)
: Node("reverse_parking_planner", options)
{
  // 声明并获取参数
  wheel_base_ = declare_parameter<double>("wheel_base", 1.0);
  min_turning_radius_ = declare_parameter<double>("min_turning_radius", 2.0);
  vehicle_length_ = declare_parameter<double>("vehicle_length", 1.8);
  vehicle_width_ = declare_parameter<double>("vehicle_width", 1.2);
  path_resolution_ = declare_parameter<double>("path_resolution", 0.1);
  velocity_forward_ = declare_parameter<double>("velocity_forward", 0.5);
  velocity_reverse_ = declare_parameter<double>("velocity_reverse", -0.3);
  publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
  enable_reverse_only_ = declare_parameter<bool>("enable_reverse_only", false);
  
  // 初始化 Reeds-Shepp Planner。
  // 注意：这里 planner 内部会以 turning_radius_ = min_turning_radius_ 为尺度做坐标归一化与反归一化。
  planner_ = std::make_unique<ReedsSheppPlanner>(min_turning_radius_);
  
  // 初始化 TF：当前代码中主要是 getYaw 等工具函数处理“朝向”，本节点也保留了 TF buffer/listener，
  // 以便后续扩展（例如目标位姿从其他坐标系变换到 map）。
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // 订阅里程计作为“当前车辆位姿”来源。
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&ReverseParkingPlannerNode::onOdometry, this, std::placeholders::_1));
  
  // 发布规划轨迹与可视化 Marker。
  traj_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/trajectory", 1);
    
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/output/path_markers", 1);
  
  // 服务 1：接收目标位姿后立刻触发规划。
  set_goal_srv_ = create_service<SetGoalPose>(
    "~/set_goal_pose",
    std::bind(&ReverseParkingPlannerNode::onSetGoalPose, this,
              std::placeholders::_1, std::placeholders::_2));

  // 服务 2：仅在已经设置过 goal 的前提下触发重新规划（例如目标不变、车辆状态更新）。
  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/trigger_planning",
    std::bind(&ReverseParkingPlannerNode::onTriggerPlanning, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 周期性定时器：只要已经规划出路径，就持续发布轨迹与 Marker。
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ReverseParkingPlannerNode::onTimer, this));
  
  RCLCPP_INFO(get_logger(), "Reverse Parking Planner initialized");
  RCLCPP_INFO(get_logger(), "  - Turning radius: %.2f m", min_turning_radius_);
  RCLCPP_INFO(get_logger(), "  - Path resolution: %.2f m", path_resolution_);
  RCLCPP_INFO(get_logger(), "  - Forward velocity: %.2f m/s", velocity_forward_);
  RCLCPP_INFO(get_logger(), "  - Reverse velocity: %.2f m/s", velocity_reverse_);
}

/**
 * @brief 定时器回调。
 * @details 周期性检查是否已经收到里程计、是否已有规划路径；
 * 若有路径则持续发布到话题。
 */
void ReverseParkingPlannerNode::onTimer()
{
  if (!current_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry...");
    return;
  }
  
  // 如果有规划好的路径，持续发布
  if (!current_path_.empty()) {
    // 每次发布都基于“当前缓存路径点”生成新的 Trajectory，
    // 这样即使下游需要更高频率的轨迹刷新，也不需要重复规划。
    auto trajectory = convertToTrajectory(current_path_);
    traj_pub_->publish(trajectory);
    publishVisualization(current_path_);
  }
}

/**
 * @brief 里程计回调，将当前车辆位姿缓存到 `current_odom_`。
 */
void ReverseParkingPlannerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // 直接缓存最新的 odometry，用于后续 onSetGoalPose/onTriggerPlanning 规划。
  current_odom_ = msg;
}

/**
 * @brief 接收目标位姿服务回调：保存 goal 并立即规划到该目标。
 */
void ReverseParkingPlannerNode::onSetGoalPose(
  const std::shared_ptr<SetGoalPose::Request> request,
  std::shared_ptr<SetGoalPose::Response> response)
{
  goal_pose_ = request->goal_pose;
  has_goal_ = true;
  
  RCLCPP_INFO(get_logger(), "Service: Received goal pose: (%.2f, %.2f, %.2f)",
              goal_pose_.pose.position.x,
              goal_pose_.pose.position.y,
              tf2::getYaw(goal_pose_.pose.orientation));
  
  if (!current_odom_) {
    // 车辆状态尚未就绪：保存 goal，但规划延后到 odometry 可用后触发。
    response->success = false;
    response->message = "No odometry available, goal saved but planning deferred";
    response->path_points_num = 0;
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  
  // odometry 已可用：尝试从当前位姿规划到目标位姿。
  if (planPath()) {
    response->success = true;
    response->message = "Path planned successfully";
    response->path_points_num = static_cast<uint32_t>(current_path_.size());
    RCLCPP_INFO(get_logger(), "Path planned successfully with %zu points", current_path_.size());
  } else {
    response->success = false;
    response->message = "Failed to plan path to goal";
    response->path_points_num = 0;
    RCLCPP_WARN(get_logger(), "Failed to plan path to goal");
  }
}

/**
 * @brief 触发规划服务回调：在有 goal 且有 odometry 的前提下重新规划。
 */
void ReverseParkingPlannerNode::onTriggerPlanning(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!current_odom_) {
    response->success = false;
    response->message = "No odometry available";
    return;
  }
  
  if (!has_goal_) {
    response->success = false;
    response->message = "No goal pose set";
    return;
  }
  
  // 重新规划并返回规划结果。
  if (planPath()) {
    response->success = true;
    response->message = "Path planned successfully with " + 
                        std::to_string(current_path_.size()) + " points";
  } else {
    response->success = false;
    response->message = "Failed to plan path";
  }
}

/**
 * @brief 核心规划流程：从当前 odometry 计算到 goal 的 Reeds-Shepp 路径，并采样生成离散路径点。
 * @return 是否成功得到非空路径。
 */
bool ReverseParkingPlannerNode::planPath()
{
  if (!current_odom_ || !has_goal_) {
    return false;
  }
  
  // 获取当前位姿（作为 Reeds-Shepp 的起点）。
  double x0 = current_odom_->pose.pose.position.x;
  double y0 = current_odom_->pose.pose.position.y;
  double yaw0 = tf2::getYaw(current_odom_->pose.pose.orientation);
  
  // 获取目标位姿（作为 Reeds-Shepp 的终点）。
  double x1 = goal_pose_.pose.position.x;
  double y1 = goal_pose_.pose.position.y;
  double yaw1 = tf2::getYaw(goal_pose_.pose.orientation);
  
  RCLCPP_INFO(get_logger(), "Planning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
              x0, y0, yaw0, x1, y1, yaw1);
  
  // 计算 Reeds-Shepp 路径段（不含插值采样）。
  ReedsSheppPath rs_path = planner_->planPath(x0, y0, yaw0, x1, y1, yaw1);
  
  // 若解析求解得到的路径无效（例如所有分支均不可行），则规划失败。
  if (!rs_path.valid()) {
    RCLCPP_ERROR(get_logger(), "No valid Reeds-Shepp path found");
    return false;
  }
  
  // 打印路径长度用于调试：rs_path.length() 内部是归一化空间长度，
  // 因此这里乘 min_turning_radius_ 还原到米单位。
  RCLCPP_INFO(get_logger(), "Reeds-Shepp path length: %.2f m", 
              rs_path.length() * min_turning_radius_);
  
  // 采样：把路径弧长离散化为离散点序列，后续用于生成 trajectory 与 marker。
  current_path_ = planner_->samplePath(rs_path, x0, y0, yaw0, path_resolution_);
  
  // 统计正向/倒车点数量（注意：这里统计的是“点级别是否倒车”，
  // 用于快速确认路径是否符合“逆向规划”预期）。
  int reverse_count = 0;
  int forward_count = 0;
  for (const auto& pt : current_path_) {
    if (pt.is_reverse) {
      reverse_count++;
    } else {
      forward_count++;
    }
  }
  
  RCLCPP_INFO(get_logger(), "Path segments: %d forward, %d reverse points",
              forward_count, reverse_count);
  
  // 只要采样后非空即可认为规划成功。
  return !current_path_.empty();
}

/**
 * @brief 将离散 `PathPoint` 序列转换为 Autoware `Trajectory` 消息。
 * @details 当前实现使用简化模型：
 * - 用 `pt.is_reverse` 决定速度正负（倒车速度为负）
 * - 路径末端逐步降速到 0
 * - 根据相邻两个路径点估算航向变化，再用前轮模型的简化几何关系求前轮转角
 */
autoware_planning_msgs::msg::Trajectory ReverseParkingPlannerNode::convertToTrajectory(
  const std::vector<PathPoint>& path_points) const
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = now();
  trajectory.header.frame_id = "map";
  
  for (size_t i = 0; i < path_points.size(); ++i) {
    const auto& pt = path_points[i];
    
    autoware_planning_msgs::msg::TrajectoryPoint traj_pt;
    // 位置直接取样点坐标（z 默认 0）。
    traj_pt.pose.position.x = pt.x;
    traj_pt.pose.position.y = pt.y;
    traj_pt.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    // 使用 yaw 构造朝向四元数：roll/pitch 固定为 0。
    q.setRPY(0, 0, pt.yaw);
    traj_pt.pose.orientation = tf2::toMsg(q);
    
    // 根据是否倒车设置速度（负值表示倒车）。
    traj_pt.longitudinal_velocity_mps = pt.is_reverse ? velocity_reverse_ : velocity_forward_;
    
    // 最后几个点线性减速到 0（避免终点处速度突变）。
    if (i >= path_points.size() - 5) {
      double ratio = static_cast<double>(path_points.size() - 1 - i) / 5.0;
      traj_pt.longitudinal_velocity_mps *= ratio;
    }
    
    traj_pt.lateral_velocity_mps = 0.0;
    traj_pt.acceleration_mps2 = 0.0;
    traj_pt.heading_rate_rps = 0.0;
    
    // 计算前轮转角（简化计算）：
    // - 用相邻两点的方向估计“路径目标航向”
    // - 与当前点航向求差得到 yaw_diff
    // - 用自行车模型几何关系估算转角（并对距离做下限保护，避免除 0）
    if (i + 1 < path_points.size()) {
      double dx = path_points[i + 1].x - pt.x;
      double dy = path_points[i + 1].y - pt.y;
      double path_yaw = std::atan2(dy, dx);
      double yaw_diff = normalizeAngle(path_yaw - pt.yaw);
      // 简化的转向角计算（与 wheel_base_ 及车辆前轮几何相关）
      traj_pt.front_wheel_angle_rad = std::atan(2.0 * wheel_base_ * std::sin(yaw_diff) / 
                                                 std::max(std::sqrt(dx*dx + dy*dy), 0.01));
    } else {
      traj_pt.front_wheel_angle_rad = 0.0;
    }
    
    trajectory.points.push_back(traj_pt);
  }
  
  return trajectory;
}

/**
 * @brief 发布可视化 Marker（路径折线+方向箭头）。
 */
void ReverseParkingPlannerNode::publishVisualization(const std::vector<PathPoint>& path_points)
{
  auto markers = createPathMarkers(path_points);
  marker_pub_->publish(markers);
}

/**
 * @brief 根据路径点创建可视化 MarkerArray。
 * @details：
 * - `LINE_STRIP`：绘制路径线，并根据 is_reverse（倒车/前进）使用不同颜色
 * - `ARROW`：每隔若干点绘制一次箭头，箭头方向在倒车时做反转显示
 * - 起点与终点使用 `SPHERE` 标记
 */
visualization_msgs::msg::MarkerArray ReverseParkingPlannerNode::createPathMarkers(
  const std::vector<PathPoint>& path_points) const
{
  visualization_msgs::msg::MarkerArray markers;
  
  // 路径线
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.stamp = now();
  line_marker.header.frame_id = "map";
  line_marker.ns = "path_line";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.05;
  line_marker.color.a = 1.0;
  
  for (const auto& pt : path_points) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.1;
    line_marker.points.push_back(p);
    
    // 倒车段用红色，前进段用绿色（便于在 RViz 中直观看到逆向段分布）。
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    if (pt.is_reverse) {
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
    } else {
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
    }
    line_marker.colors.push_back(color);
  }
  
  markers.markers.push_back(line_marker);
  
  // 方向箭头（每隔几个点画一个）
  int arrow_id = 0;
  for (size_t i = 0; i < path_points.size(); i += 10) {
    const auto& pt = path_points[i];
    
    visualization_msgs::msg::Marker arrow;
    arrow.header.stamp = now();
    arrow.header.frame_id = "map";
    arrow.ns = "path_arrows";
    arrow.id = arrow_id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    
    arrow.pose.position.x = pt.x;
    arrow.pose.position.y = pt.y;
    arrow.pose.position.z = 0.1;
    
    tf2::Quaternion q;
    // 倒车时箭头反向显示：显示为“车辆实际移动方向”而不是车辆朝向。
    double display_yaw = pt.is_reverse ? pt.yaw + M_PI : pt.yaw;
    q.setRPY(0, 0, display_yaw);
    arrow.pose.orientation = tf2::toMsg(q);
    
    arrow.scale.x = 0.3;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    
    arrow.color.a = 1.0;
    if (pt.is_reverse) {
      arrow.color.r = 1.0;
      arrow.color.g = 0.3;
      arrow.color.b = 0.0;
    } else {
      arrow.color.r = 0.0;
      arrow.color.g = 0.8;
      arrow.color.b = 0.2;
    }
    
    markers.markers.push_back(arrow);
  }
  
  // 起点标记
  visualization_msgs::msg::Marker start_marker;
  start_marker.header.stamp = now();
  start_marker.header.frame_id = "map";
  start_marker.ns = "start_goal";
  start_marker.id = 0;
  start_marker.type = visualization_msgs::msg::Marker::SPHERE;
  start_marker.action = visualization_msgs::msg::Marker::ADD;
  // 起点颜色：绿色
  start_marker.pose.position.x = path_points.front().x;
  start_marker.pose.position.y = path_points.front().y;
  start_marker.pose.position.z = 0.2;
  start_marker.scale.x = 0.3;
  start_marker.scale.y = 0.3;
  start_marker.scale.z = 0.3;
  start_marker.color.r = 0.0;
  start_marker.color.g = 1.0;
  start_marker.color.b = 0.0;
  start_marker.color.a = 1.0;
  markers.markers.push_back(start_marker);
  
  // 终点标记
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.stamp = now();
  goal_marker.header.frame_id = "map";
  goal_marker.ns = "start_goal";
  goal_marker.id = 1;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  // 终点颜色：红色
  goal_marker.pose.position.x = path_points.back().x;
  goal_marker.pose.position.y = path_points.back().y;
  goal_marker.pose.position.z = 0.2;
  goal_marker.scale.x = 0.3;
  goal_marker.scale.y = 0.3;
  goal_marker.scale.z = 0.3;
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 0.0;
  goal_marker.color.a = 1.0;
  markers.markers.push_back(goal_marker);
  
  return markers;
}

/**
 * @brief 将角度归一化到 [-pi, pi]。
 */
double ReverseParkingPlannerNode::normalizeAngle(double angle) const
{
  // 通过循环加/减 2pi，把角度推回到目标区间。
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * @brief 获取当前 odometry 对应的 Pose。
 * @return 若当前还没有 odometry，则返回 `std::nullopt`；否则返回 Pose。
 */
std::optional<geometry_msgs::msg::Pose> ReverseParkingPlannerNode::getCurrentPose() const
{
  if (!current_odom_) {
    return std::nullopt;
  }
  return current_odom_->pose.pose;
}

}  // namespace reverse_parking_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(reverse_parking_planner::ReverseParkingPlannerNode)
