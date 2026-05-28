// Copyright 2025 BYD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "trailer_pedestrian_stop_module.hpp"

#include "collision.hpp"
#include "debug.hpp"
#include "footprint.hpp"
#include "object_filtering.hpp"
#include "object_stop_decision.hpp"
#include "types.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/stop_watch.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
namespace
{
void declare_trailer_pedestrian_stop_params(
  rclcpp::Node & node, trailer_pedestrian_stop::PlannerParam & p, const std::string & ns)
{
  using autoware_utils::get_or_declare_parameter;
  auto & trailer = p.trailer;
  trailer.count = static_cast<size_t>(
    get_or_declare_parameter<int64_t>(node, ns + ".trailer.count"));
  trailer.length = get_or_declare_parameter<double>(node, ns + ".trailer.length");
  trailer.width = get_or_declare_parameter<double>(node, ns + ".trailer.width");
  trailer.hitch_gap = get_or_declare_parameter<double>(node, ns + ".trailer.hitch_gap");
  trailer.side_margin = get_or_declare_parameter<double>(node, ns + ".trailer.side_margin");
  trailer.rear_margin = get_or_declare_parameter<double>(node, ns + ".trailer.rear_margin");

  auto & pedestrian = p.pedestrian;
  pedestrian.enable_pedestrian =
    get_or_declare_parameter<bool>(node, ns + ".pedestrian.enable_pedestrian");
  pedestrian.enable_bicycle =
    get_or_declare_parameter<bool>(node, ns + ".pedestrian.enable_bicycle");
  pedestrian.enable_unknown =
    get_or_declare_parameter<bool>(node, ns + ".pedestrian.enable_unknown");
  pedestrian.time_horizon = get_or_declare_parameter<double>(node, ns + ".pedestrian.time_horizon");
  pedestrian.radius_margin =
    get_or_declare_parameter<double>(node, ns + ".pedestrian.radius_margin");
  pedestrian.min_predicted_path_confidence = get_or_declare_parameter<double>(
    node, ns + ".pedestrian.min_predicted_path_confidence");
  pedestrian.min_approach_speed =
    get_or_declare_parameter<double>(node, ns + ".pedestrian.min_approach_speed");
  pedestrian.use_predicted_path =
    get_or_declare_parameter<bool>(node, ns + ".pedestrian.use_predicted_path");
  pedestrian.require_lateral_approach =
    get_or_declare_parameter<bool>(node, ns + ".pedestrian.require_lateral_approach");
  pedestrian.max_lateral_approach_distance = get_or_declare_parameter<double>(
    node, ns + ".pedestrian.max_lateral_approach_distance");

  auto & stop = p.stop;
  stop.stop_distance_buffer =
    get_or_declare_parameter<double>(node, ns + ".stop.stop_distance_buffer");
  stop.add_duration_buffer =
    get_or_declare_parameter<double>(node, ns + ".stop.add_stop_duration_buffer");
  stop.remove_duration_buffer =
    get_or_declare_parameter<double>(node, ns + ".stop.remove_stop_duration_buffer");
  stop.hysteresis = get_or_declare_parameter<double>(node, ns + ".stop.hysteresis");
  stop.ignore_unavoidable_collisions =
    get_or_declare_parameter<bool>(node, ns + ".stop.ignore_unavoidable_collisions");

  p.sampling.trajectory_step_length =
    get_or_declare_parameter<double>(node, ns + ".sampling.trajectory_step_length");
}
}  // namespace

void TrailerPedestrianStopModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  logger_ = node.get_logger().get_child(ns_);
  clock_ = node.get_clock();

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "trailer_pedestrian_stop");

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_diag_publisher_ = std::make_shared<autoware_utils_debug::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms_diag");
  processing_time_publisher_ =
    node.create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/" + ns_ + "/processing_time_ms", 1);

  declare_trailer_pedestrian_stop_params(node, params_, ns_);

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  params_.ego_lateral_offset =
    std::max(std::abs(vehicle_info.min_lateral_offset_m), vehicle_info.max_lateral_offset_m);
  params_.ego_longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
  params_.ego_rear_longitudinal_offset =
    std::abs(vehicle_info.min_longitudinal_offset_m);
}

void TrailerPedestrianStopModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  auto & p = params_;
  int64_t trailer_count = static_cast<int64_t>(p.trailer.count);
  update_param(parameters, ns_ + ".trailer.count", trailer_count);
  p.trailer.count = static_cast<size_t>(trailer_count);
  update_param(parameters, ns_ + ".trailer.length", p.trailer.length);
  update_param(parameters, ns_ + ".trailer.width", p.trailer.width);
  update_param(parameters, ns_ + ".trailer.hitch_gap", p.trailer.hitch_gap);
  update_param(parameters, ns_ + ".trailer.side_margin", p.trailer.side_margin);
  update_param(parameters, ns_ + ".trailer.rear_margin", p.trailer.rear_margin);
  update_param(parameters, ns_ + ".pedestrian.enable_pedestrian", p.pedestrian.enable_pedestrian);
  update_param(parameters, ns_ + ".pedestrian.enable_bicycle", p.pedestrian.enable_bicycle);
  update_param(parameters, ns_ + ".pedestrian.enable_unknown", p.pedestrian.enable_unknown);
  update_param(parameters, ns_ + ".pedestrian.time_horizon", p.pedestrian.time_horizon);
  update_param(parameters, ns_ + ".pedestrian.radius_margin", p.pedestrian.radius_margin);
  update_param(
    parameters, ns_ + ".pedestrian.min_predicted_path_confidence",
    p.pedestrian.min_predicted_path_confidence);
  update_param(parameters, ns_ + ".pedestrian.min_approach_speed", p.pedestrian.min_approach_speed);
  update_param(parameters, ns_ + ".pedestrian.use_predicted_path", p.pedestrian.use_predicted_path);
  update_param(
    parameters, ns_ + ".pedestrian.require_lateral_approach", p.pedestrian.require_lateral_approach);
  update_param(
    parameters, ns_ + ".pedestrian.max_lateral_approach_distance",
    p.pedestrian.max_lateral_approach_distance);
  update_param(parameters, ns_ + ".stop.stop_distance_buffer", p.stop.stop_distance_buffer);
  update_param(parameters, ns_ + ".stop.add_stop_duration_buffer", p.stop.add_duration_buffer);
  update_param(parameters, ns_ + ".stop.remove_stop_duration_buffer", p.stop.remove_duration_buffer);
  update_param(parameters, ns_ + ".stop.hysteresis", p.stop.hysteresis);
  update_param(
    parameters, ns_ + ".stop.ignore_unavoidable_collisions", p.stop.ignore_unavoidable_collisions);
  update_param(
    parameters, ns_ + ".sampling.trajectory_step_length", p.sampling.trajectory_step_length);
}

void TrailerPedestrianStopModule::publish_processing_time(const double processing_time_ms)
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = clock_->now();
  processing_time_msg.data = processing_time_ms;
  processing_time_publisher_->publish(processing_time_msg);
}

/**
 * @brief 根据输入的轨迹点与感知数据，对挂车/牵引车列和横向接近行人进行碰撞预测，并在规划轨迹上插入停止点。
 * 
 * 主要流程：
 * 1. 对轨迹进行重采样与去重，获得自车与挂车的轨迹/包络。
 * 2. 结合感知/预测目标，筛选可能导致碰撞的行人，并为每个行人生成扫掠区域。
 * 3. 检查行人扫掠区是否与车辆列车包络相交，记录碰撞点，并应用防抖逻辑。
 * 4. 若有早于当前运动状态能够停车的碰撞点，则在对应位置插入停止点，并发布调试信息与虚拟墙。
 * 5. 发布各阶段处理耗时信息。
 * 
 * @param raw_trajectory_points    原始（未平滑）轨迹点，未使用
 * @param smoothed_trajectory_points  平滑后的轨迹点，用于重采样和包络生成
 * @param planner_data            当前自车、感知、控制等相关动态信息
 * @return VelocityPlanningResult  输出插入了停止点的规划结果
 */
VelocityPlanningResult TrailerPedestrianStopModule::plan(
  [[maybe_unused]] const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> &
    raw_trajectory_points,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  VelocityPlanningResult result;
  debug_data_.reset_data();
  autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  if (smoothed_trajectory_points.size() < 2) {
    publish_processing_time(stopwatch.toc() / 1000);
    return result;
  }

  stopwatch.tic();
  stopwatch.tic("preprocessing");
  trailer_pedestrian_stop::EgoData ego_data;
  ego_data.pose = planner_data->current_odometry.pose.pose;
  ego_data.trajectory = trailer_pedestrian_stop::resample_trajectory(
    smoothed_trajectory_points, params_.sampling.trajectory_step_length);
  ego_data.trajectory = autoware::motion_utils::removeOverlapPoints(ego_data.trajectory);
  ego_data.first_trajectory_idx =
    autoware::motion_utils::findNearestSegmentIndex(ego_data.trajectory, ego_data.pose.position);
  ego_data.longitudinal_offset_to_first_trajectory_idx =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(
      ego_data.trajectory, ego_data.first_trajectory_idx, ego_data.pose.position);
  const auto min_stop_distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
                                   planner_data->current_odometry.twist.twist.linear.x, 0.0,
                                   planner_data->current_acceleration.accel.accel.linear.x,
                                   planner_data->velocity_smoother_->getMinDecel(),
                                   planner_data->velocity_smoother_->getMaxJerk(),
                                   planner_data->velocity_smoother_->getMinJerk())
                                   .value_or(0.0);
  ego_data.earliest_stop_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    ego_data.trajectory, ego_data.pose.position, min_stop_distance);

  trailer_pedestrian_stop::make_train_trailer_footprint_rtree(ego_data, params_);
  const double hysteresis =
    std::find_if(
      object_map_.begin(), object_map_.end(),
      [](const auto & pair) { return pair.second.should_be_avoided(); }) == object_map_.end()
      ? 0.0
      : params_.stop.hysteresis;
  const auto pedestrians = trailer_pedestrian_stop::filter_predicted_objects(
    planner_data->objects, ego_data, params_, hysteresis);

  const auto preprocessing_duration_us = stopwatch.toc("preprocessing");

  stopwatch.tic("footprints");
  const auto pedestrian_footprints =
    trailer_pedestrian_stop::make_pedestrian_footprints(pedestrians, params_, hysteresis);
  const auto footprints_duration_us = stopwatch.toc("footprints");
  stopwatch.tic("collisions");
  auto collisions = trailer_pedestrian_stop::find_collisions(
    ego_data, pedestrians, pedestrian_footprints);
  trailer_pedestrian_stop::update_object_map(
    object_map_, collisions, clock_->now(), ego_data.trajectory, params_);
  std::optional<geometry_msgs::msg::Point> earliest_collision =
    trailer_pedestrian_stop::find_earliest_collision(object_map_, ego_data);
  const auto collisions_duration_us = stopwatch.toc("collisions");
  if (earliest_collision) {
    const auto arc_length_diff = autoware::motion_utils::calcSignedArcLength(
      ego_data.trajectory, *earliest_collision, ego_data.pose.position);
    const auto can_stop_before_limit = arc_length_diff < min_stop_distance -
                                                           params_.ego_longitudinal_offset -
                                                           params_.stop.stop_distance_buffer;
    const auto stop_pose = can_stop_before_limit
                             ? autoware::motion_utils::calcLongitudinalOffsetPose(
                                 ego_data.trajectory, *earliest_collision,
                                 -params_.stop.stop_distance_buffer -
                                   params_.ego_longitudinal_offset)
                             : ego_data.earliest_stop_pose;
    debug_data_.stop_pose = stop_pose;
    if (stop_pose) {
      result.stop_points.push_back(stop_pose->position);
      planning_factor_interface_->add(
        smoothed_trajectory_points, ego_data.pose, *stop_pose, PlanningFactor::STOP,
        SafetyFactorArray{});
      create_virtual_walls();
    }
  }

  debug_data_.ego_footprints = ego_data.trajectory_footprints;
  debug_data_.pedestrian_footprints = pedestrian_footprints;
  debug_data_.z = ego_data.pose.position.z;
  debug_publisher_->publish(create_debug_marker_array());
  virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers());

  const auto total_time_us = stopwatch.toc();
  RCLCPP_DEBUG(
    logger_,
    "Total time = %2.2fus\n\tpreprocessing = %2.2fus\n\tfootprints = "
    "%2.2fus\n\tcollisions = %2.2fus\n",
    total_time_us, preprocessing_duration_us, footprints_duration_us, collisions_duration_us);
  std::map<std::string, double> processing_times;
  processing_times["preprocessing"] = preprocessing_duration_us / 1000;
  processing_times["footprints"] = footprints_duration_us / 1000;
  processing_times["collisions"] = collisions_duration_us / 1000;
  processing_times["Total"] = total_time_us / 1000;
  processing_diag_publisher_->publish(processing_times);
  publish_processing_time(processing_times["Total"]);
  return result;
}

visualization_msgs::msg::MarkerArray TrailerPedestrianStopModule::create_debug_marker_array()
{
  const auto z = debug_data_.z;
  visualization_msgs::msg::MarkerArray array;
  std::string ns = "collisions";
  const auto collision_markers =
    trailer_pedestrian_stop::debug::make_collision_markers(object_map_, ns, z, clock_->now());
  trailer_pedestrian_stop::debug::add_markers(
    array, debug_data_.prev_collisions_nb, collision_markers, ns);
  ns = "pedestrian_footprints";
  const auto pedestrian_footprint_markers = trailer_pedestrian_stop::debug::make_polygon_markers(
    debug_data_.pedestrian_footprints, ns, z, 1.0f, 0.2f, 0.2f);
  trailer_pedestrian_stop::debug::add_markers(
    array, debug_data_.prev_pedestrian_footprints_nb, pedestrian_footprint_markers, ns);
  ns = "train_trailer_footprints";
  const auto ego_footprint_markers = trailer_pedestrian_stop::debug::make_polygon_markers(
    debug_data_.ego_footprints, ns, z, 0.1f, 0.6f, 1.0f, 0.25);
  trailer_pedestrian_stop::debug::add_markers(
    array, debug_data_.prev_ego_footprints_nb, ego_footprint_markers, ns);
  return array;
}

void TrailerPedestrianStopModule::create_virtual_walls()
{
  if (debug_data_.stop_pose) {
    autoware::motion_utils::VirtualWall virtual_wall;
    virtual_wall.text = "trailer_pedestrian_stop";
    virtual_wall.longitudinal_offset = params_.ego_longitudinal_offset;
    virtual_wall.style = autoware::motion_utils::VirtualWallType::stop;
    virtual_wall.pose = *debug_data_.stop_pose;
    virtual_wall_marker_creator.add_virtual_wall(virtual_wall);
  }
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::TrailerPedestrianStopModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
