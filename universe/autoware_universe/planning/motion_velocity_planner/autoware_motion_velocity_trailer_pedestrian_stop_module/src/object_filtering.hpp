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

#ifndef OBJECT_FILTERING_HPP_
#define OBJECT_FILTERING_HPP_

#include "types.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>

#include <vector>

namespace autoware::motion_velocity_planner::trailer_pedestrian_stop
{
bool is_target_pedestrian(const autoware_perception_msgs::msg::PredictedObject & object);

bool is_in_range(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const PlannerParam & params, const double hysteresis);

bool is_not_too_close(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const PlannerParam & params);

bool is_approaching_laterally(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const PlannerParam & params);

bool is_unavoidable(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const PlannerParam & params);

double max_lateral_range(const PlannerParam & params, const double hysteresis);

std::vector<autoware_perception_msgs::msg::PredictedObject> filter_predicted_objects(
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects, const EgoData & ego_data,
  const PlannerParam & params, const double hysteresis);
}  // namespace autoware::motion_velocity_planner::trailer_pedestrian_stop

#endif  // OBJECT_FILTERING_HPP_
