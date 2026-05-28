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

#ifndef FOOTPRINT_HPP_
#define FOOTPRINT_HPP_

#include "types.hpp"

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

namespace autoware::motion_velocity_planner::trailer_pedestrian_stop
{
TrajectoryPoints resample_trajectory(
  const TrajectoryPoints & trajectory, const double step_length);

autoware_utils::Polygon2d make_trailer_footprint_at_offset(
  const geometry_msgs::msg::Pose & tractor_pose, const double backward_offset_from_base,
  const TrailerParam & trailer);

std::vector<autoware_utils::Polygon2d> make_footprint_parts_at_pose(
  const geometry_msgs::msg::Pose & pose, const PlannerParam & params);

/** Axis-aligned bounding envelope of all parts (for tests / coarse checks). */
autoware_utils::Polygon2d make_combined_footprint_at_pose(
  const geometry_msgs::msg::Pose & pose, const PlannerParam & params);

void make_train_trailer_footprint_rtree(EgoData & ego_data, const PlannerParam & params);
}  // namespace autoware::motion_velocity_planner::trailer_pedestrian_stop

#endif  // FOOTPRINT_HPP_
