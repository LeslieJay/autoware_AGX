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

#ifndef TRAILER_PEDESTRIAN_STOP_MODULE_HPP_
#define TRAILER_PEDESTRIAN_STOP_MODULE_HPP_

#include "object_stop_decision.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
class TrailerPedestrianStopModule : public PluginModuleInterface
{
public:
  void init(rclcpp::Node & node, const std::string & module_name) override;
  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo info;
    info.predicted_objects = true;
    return info;
  }
  void publish_planning_factor() override { planning_factor_interface_->publish(); };
  std::string get_short_module_name() const override { return "trailer_pedestrian_stop"; }
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  void publish_processing_time(const double processing_time_ms);
  VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw_trajectory_points,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;
  std::string get_module_name() const override { return module_name_; }

private:
  visualization_msgs::msg::MarkerArray create_debug_marker_array();
  void create_virtual_walls();

  inline static const std::string ns_ = "trailer_pedestrian_stop";
  std::string module_name_;
  rclcpp::Clock::SharedPtr clock_;

  trailer_pedestrian_stop::PlannerParam params_;
  trailer_pedestrian_stop::ObjectStopDecisionMap object_map_;

  mutable trailer_pedestrian_stop::DebugData debug_data_;
};
}  // namespace autoware::motion_velocity_planner

#endif  // TRAILER_PEDESTRIAN_STOP_MODULE_HPP_
