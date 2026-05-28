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

#include "object_filtering.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::motion_velocity_planner::trailer_pedestrian_stop
{
bool is_target_pedestrian(const autoware_perception_msgs::msg::PredictedObject & object)
{
  if (object.classification.empty()) return false;
  const auto label = object.classification.front().label;
  return label == autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN ||
         label == autoware_perception_msgs::msg::ObjectClassification::BICYCLE ||
         label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
}

bool is_pedestrian_enabled(
  const autoware_perception_msgs::msg::PredictedObject & object, const PlannerParam & params)
{
  if (object.classification.empty()) return false;
  const auto label = object.classification.front().label;
  if (label == autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN)
    return params.pedestrian.enable_pedestrian;
  if (label == autoware_perception_msgs::msg::ObjectClassification::BICYCLE)
    return params.pedestrian.enable_bicycle;
  if (label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN)
    return params.pedestrian.enable_unknown;
  return false;
}

double max_lateral_range(const PlannerParam & params, const double hysteresis)
{
  const auto & trailer = params.trailer;
  const auto train_length =
    trailer.count * (trailer.length + trailer.hitch_gap) + trailer.rear_margin;
  return params.ego_lateral_offset + trailer.width / 2.0 + trailer.side_margin +
         train_length * 0.1 + params.pedestrian.max_lateral_approach_distance + hysteresis;
}

bool is_in_range(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const PlannerParam & params, const double hysteresis)
{
  const auto lateral_distance = std::abs(autoware::motion_utils::calcLateralOffset(
    ego_data.trajectory, object.kinematics.initial_pose_with_covariance.pose.position));
  const auto object_radius =
    std::hypot(object.shape.dimensions.x, object.shape.dimensions.y) / 2.0 +
    params.pedestrian.radius_margin;
  return lateral_distance <= max_lateral_range(params, hysteresis) + object_radius;
}

bool is_not_too_close(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const PlannerParam & params)
{
  const auto obj_arc_length = autoware::motion_utils::calcSignedArcLength(
    ego_data.trajectory, ego_data.pose.position,
    object.kinematics.initial_pose_with_covariance.pose.position);
  const auto & trailer = params.trailer;
  const auto train_rear_length =
    params.ego_rear_longitudinal_offset + trailer.count * (trailer.length + trailer.hitch_gap) +
    trailer.rear_margin;
  // Allow pedestrians alongside the train; only ignore objects clearly behind the last trailer.
  return obj_arc_length >= -(train_rear_length + object.shape.dimensions.x / 2.0);
}

bool is_approaching_laterally(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const PlannerParam & params)
{
  if (!params.pedestrian.require_lateral_approach) return true;

  const auto nearest_idx = autoware::motion_utils::findNearestIndex(
    ego_data.trajectory, object.kinematics.initial_pose_with_covariance.pose.position);
  const auto traj_yaw = tf2::getYaw(ego_data.trajectory[nearest_idx].pose.orientation);
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  const auto lateral_speed =
    std::abs(-std::sin(traj_yaw) * twist.linear.x + std::cos(traj_yaw) * twist.linear.y);
  return lateral_speed >= params.pedestrian.min_approach_speed;
}

bool is_unavoidable(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const PlannerParam & params)
{
  const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto lateral_offset = autoware::motion_utils::calcLateralOffset(ego_data.trajectory, pose.position);
  const auto nearest_idx =
    autoware::motion_utils::findNearestIndex(ego_data.trajectory, pose.position);
  const auto traj_yaw = tf2::getYaw(ego_data.trajectory[nearest_idx].pose.orientation);
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  const auto lateral_velocity =
    -std::sin(traj_yaw) * twist.linear.x + std::cos(traj_yaw) * twist.linear.y;
  const auto moving_away_from_train =
    lateral_offset * lateral_velocity > 0.0 &&
    std::abs(lateral_velocity) >= params.pedestrian.min_approach_speed;

  const auto collision_arc_length = autoware::motion_utils::calcSignedArcLength(
    ego_data.trajectory, ego_data.pose.position, pose.position);
  const auto already_behind_ego = collision_arc_length < 0.0;

  return moving_away_from_train || already_behind_ego;
}

std::vector<autoware_perception_msgs::msg::PredictedObject> filter_predicted_objects(
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects, const EgoData & ego_data,
  const PlannerParam & params, const double hysteresis)
{
  std::vector<autoware_perception_msgs::msg::PredictedObject> filtered_objects;
  for (const auto & object : objects) {
    const auto & predicted_object = object->predicted_object;
    if (
      is_target_pedestrian(predicted_object) && is_pedestrian_enabled(predicted_object, params) &&
      is_in_range(predicted_object, ego_data, params, hysteresis) &&
      is_not_too_close(predicted_object, ego_data, params) &&
      is_approaching_laterally(predicted_object, ego_data, params) &&
      (!params.stop.ignore_unavoidable_collisions ||
       !is_unavoidable(predicted_object, ego_data, params)))
      filtered_objects.push_back(predicted_object);
  }
  return filtered_objects;
}
}  // namespace autoware::motion_velocity_planner::trailer_pedestrian_stop
