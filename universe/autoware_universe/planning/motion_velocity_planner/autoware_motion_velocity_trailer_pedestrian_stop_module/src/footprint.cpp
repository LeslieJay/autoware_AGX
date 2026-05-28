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

#include "footprint.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry/algorithms/convert.hpp>
#include <boost/geometry/algorithms/envelope.hpp>

#include <tf2/utils.h>

#include <cmath>
#include <vector>

namespace autoware::motion_velocity_planner::trailer_pedestrian_stop
{
TrajectoryPoints resample_trajectory(
  const TrajectoryPoints & trajectory, const double step_length)
{
  if (trajectory.size() < 2 || step_length <= 0.0) return trajectory;
  TrajectoryPoints resampled;
  resampled.push_back(trajectory.front());
  double accumulated = 0.0;
  for (size_t i = 1; i < trajectory.size(); ++i) {
    const auto segment_length = autoware_utils::calc_distance2d(
      trajectory[i - 1].pose.position, trajectory[i].pose.position);
    if (segment_length < 1e-6) continue;
    accumulated += segment_length;
    while (accumulated >= step_length) {
      const auto ratio = 1.0 - (accumulated - step_length) / segment_length;
      autoware_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x = trajectory[i - 1].pose.position.x +
                          ratio * (trajectory[i].pose.position.x - trajectory[i - 1].pose.position.x);
      p.pose.position.y = trajectory[i - 1].pose.position.y +
                          ratio * (trajectory[i].pose.position.y - trajectory[i - 1].pose.position.y);
      p.pose.position.z = trajectory[i - 1].pose.position.z +
                          ratio * (trajectory[i].pose.position.z - trajectory[i - 1].pose.position.z);
      const auto yaw_prev = tf2::getYaw(trajectory[i - 1].pose.orientation);
      const auto yaw_next = tf2::getYaw(trajectory[i].pose.orientation);
      const auto yaw = yaw_prev + ratio * autoware_utils::normalize_radian(yaw_next - yaw_prev);
      p.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
      resampled.push_back(p);
      accumulated -= step_length;
    }
  }
  if (resampled.back().pose.position.x != trajectory.back().pose.position.x ||
      resampled.back().pose.position.y != trajectory.back().pose.position.y) {
    resampled.push_back(trajectory.back());
  }
  return resampled;
}

autoware_utils::Polygon2d make_trailer_footprint_at_offset(
  const geometry_msgs::msg::Pose & tractor_pose, const double backward_offset_from_base,
  const TrailerParam & trailer)
{
  const auto yaw = tf2::getYaw(tractor_pose.orientation);
  geometry_msgs::msg::Pose trailer_pose = tractor_pose;
  trailer_pose.position.x -= std::cos(yaw) * backward_offset_from_base;
  trailer_pose.position.y -= std::sin(yaw) * backward_offset_from_base;
  const auto width = trailer.width + 2.0 * trailer.side_margin;
  const auto half_length = trailer.length / 2.0;
  return autoware_utils::to_footprint(trailer_pose, half_length, half_length, width);
}

std::vector<autoware_utils::Polygon2d> make_footprint_parts_at_pose(
  const geometry_msgs::msg::Pose & pose, const PlannerParam & params)
{
  const auto & trailer = params.trailer;
  const auto tractor_width = params.ego_lateral_offset * 2.0 + 2.0 * trailer.side_margin;
  std::vector<autoware_utils::Polygon2d> parts;
  parts.push_back(autoware_utils::to_footprint(
    pose, params.ego_longitudinal_offset, params.ego_rear_longitudinal_offset, tractor_width));

  double backward_offset =
    params.ego_rear_longitudinal_offset + trailer.hitch_gap + trailer.length / 2.0;
  for (size_t i = 0; i < trailer.count; ++i) {
    parts.push_back(make_trailer_footprint_at_offset(pose, backward_offset, trailer));
    backward_offset += trailer.length + trailer.hitch_gap;
  }

  if (trailer.rear_margin > 0.0) {
    TrailerParam rear_only = trailer;
    rear_only.length = trailer.rear_margin;
    const double rear_center_offset =
      backward_offset - trailer.hitch_gap - trailer.length / 2.0 + trailer.rear_margin / 2.0;
    parts.push_back(make_trailer_footprint_at_offset(pose, rear_center_offset, rear_only));
  }

  return parts;
}

autoware_utils::Polygon2d make_combined_footprint_at_pose(
  const geometry_msgs::msg::Pose & pose, const PlannerParam & params)
{
  const auto parts = make_footprint_parts_at_pose(pose, params);
  if (parts.empty()) {
    return {};
  }
  autoware_utils::Box2d envelope;
  boost::geometry::envelope(parts.front(), envelope);
  for (size_t i = 1; i < parts.size(); ++i) {
    autoware_utils::Box2d part_box;
    boost::geometry::envelope(parts[i], part_box);
    boost::geometry::expand(envelope, part_box);
  }
  autoware_utils::Polygon2d combined;
  boost::geometry::convert(envelope, combined);
  return combined;
}

void make_train_trailer_footprint_rtree(EgoData & ego_data, const PlannerParam & params)
{
  ego_data.trajectory_footprints.clear();
  ego_data.trajectory_footprint_traj_indices.clear();
  std::vector<BoxIndexPair> rtree_nodes;
  for (auto i = 0UL; i < ego_data.trajectory.size(); ++i) {
    const auto parts = make_footprint_parts_at_pose(ego_data.trajectory[i].pose, params);
    for (const auto & part : parts) {
      const auto footprint_idx = ego_data.trajectory_footprints.size();
      ego_data.trajectory_footprints.push_back(part);
      ego_data.trajectory_footprint_traj_indices.push_back(i);
      const auto box = boost::geometry::return_envelope<autoware_utils::Box2d>(part);
      rtree_nodes.emplace_back(box, footprint_idx);
    }
  }
  ego_data.rtree = Rtree(rtree_nodes);
}

}  // namespace autoware::motion_velocity_planner::trailer_pedestrian_stop
