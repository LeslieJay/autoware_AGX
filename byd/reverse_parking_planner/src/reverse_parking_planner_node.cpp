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
  
  // 初始化规划器
  planner_ = std::make_unique<ReedsSheppPlanner>(min_turning_radius_);
  
  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // 创建订阅者
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&ReverseParkingPlannerNode::onOdometry, this, std::placeholders::_1));
  
  // 创建发布者
  traj_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/trajectory", 1);
    
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/output/path_markers", 1);
  
  // 创建服务：设置目标位姿并规划
  set_goal_srv_ = create_service<SetGoalPose>(
    "~/set_goal_pose",
    std::bind(&ReverseParkingPlannerNode::onSetGoalPose, this,
              std::placeholders::_1, std::placeholders::_2));

  // 创建服务：仅触发重新规划
  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/trigger_planning",
    std::bind(&ReverseParkingPlannerNode::onTriggerPlanning, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 创建定时器
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

void ReverseParkingPlannerNode::onTimer()
{
  if (!current_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry...");
    return;
  }
  
  // 如果有规划好的路径，持续发布
  if (!current_path_.empty()) {
    auto trajectory = convertToTrajectory(current_path_);
    traj_pub_->publish(trajectory);
    publishVisualization(current_path_);
  }
}

void ReverseParkingPlannerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_odom_ = msg;
}

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
    response->success = false;
    response->message = "No odometry available, goal saved but planning deferred";
    response->path_points_num = 0;
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  
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
  
  if (planPath()) {
    response->success = true;
    response->message = "Path planned successfully with " + 
                        std::to_string(current_path_.size()) + " points";
  } else {
    response->success = false;
    response->message = "Failed to plan path";
  }
}

bool ReverseParkingPlannerNode::planPath()
{
  if (!current_odom_ || !has_goal_) {
    return false;
  }
  
  // 获取当前位姿
  double x0 = current_odom_->pose.pose.position.x;
  double y0 = current_odom_->pose.pose.position.y;
  double yaw0 = tf2::getYaw(current_odom_->pose.pose.orientation);
  
  // 获取目标位姿
  double x1 = goal_pose_.pose.position.x;
  double y1 = goal_pose_.pose.position.y;
  double yaw1 = tf2::getYaw(goal_pose_.pose.orientation);
  
  RCLCPP_INFO(get_logger(), "Planning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
              x0, y0, yaw0, x1, y1, yaw1);
  
  // 计算Reeds-Shepp路径
  ReedsSheppPath rs_path = planner_->planPath(x0, y0, yaw0, x1, y1, yaw1);
  
  if (!rs_path.valid()) {
    RCLCPP_ERROR(get_logger(), "No valid Reeds-Shepp path found");
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "Reeds-Shepp path length: %.2f m", 
              rs_path.length() * min_turning_radius_);
  
  // 采样路径点
  current_path_ = planner_->samplePath(rs_path, x0, y0, yaw0, path_resolution_);
  
  // 输出路径段信息
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
  
  return !current_path_.empty();
}

autoware_planning_msgs::msg::Trajectory ReverseParkingPlannerNode::convertToTrajectory(
  const std::vector<PathPoint>& path_points) const
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = now();
  trajectory.header.frame_id = "map";
  
  for (size_t i = 0; i < path_points.size(); ++i) {
    const auto& pt = path_points[i];
    
    autoware_planning_msgs::msg::TrajectoryPoint traj_pt;
    traj_pt.pose.position.x = pt.x;
    traj_pt.pose.position.y = pt.y;
    traj_pt.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, pt.yaw);
    traj_pt.pose.orientation = tf2::toMsg(q);
    
    // 根据是否倒车设置速度（负值表示倒车）
    traj_pt.longitudinal_velocity_mps = pt.is_reverse ? velocity_reverse_ : velocity_forward_;
    
    // 最后几个点减速到0
    if (i >= path_points.size() - 5) {
      double ratio = static_cast<double>(path_points.size() - 1 - i) / 5.0;
      traj_pt.longitudinal_velocity_mps *= ratio;
    }
    
    traj_pt.lateral_velocity_mps = 0.0;
    traj_pt.acceleration_mps2 = 0.0;
    traj_pt.heading_rate_rps = 0.0;
    
    // 计算前轮转角（简化计算）
    if (i + 1 < path_points.size()) {
      double dx = path_points[i + 1].x - pt.x;
      double dy = path_points[i + 1].y - pt.y;
      double path_yaw = std::atan2(dy, dx);
      double yaw_diff = normalizeAngle(path_yaw - pt.yaw);
      // 简化的转向角计算
      traj_pt.front_wheel_angle_rad = std::atan(2.0 * wheel_base_ * std::sin(yaw_diff) / 
                                                 std::max(std::sqrt(dx*dx + dy*dy), 0.01));
    } else {
      traj_pt.front_wheel_angle_rad = 0.0;
    }
    
    trajectory.points.push_back(traj_pt);
  }
  
  return trajectory;
}

void ReverseParkingPlannerNode::publishVisualization(const std::vector<PathPoint>& path_points)
{
  auto markers = createPathMarkers(path_points);
  marker_pub_->publish(markers);
}

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
    
    // 倒车段用红色，前进段用绿色
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
    // 倒车时箭头反向
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

double ReverseParkingPlannerNode::normalizeAngle(double angle) const
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

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
