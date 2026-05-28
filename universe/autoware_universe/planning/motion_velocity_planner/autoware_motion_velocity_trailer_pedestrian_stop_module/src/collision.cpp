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

#include "collision.hpp"

#include "footprint.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/union.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::trailer_pedestrian_stop
{
namespace
{
autoware_utils::Polygon2d make_circle_polygon(
  const geometry_msgs::msg::Point & center, const double radius, const size_t num_points = 12)
{
  autoware_utils::Polygon2d polygon;
  for (size_t i = 0; i <= num_points; ++i) {
    const auto angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points);
    polygon.outer().emplace_back(
      center.x + radius * std::cos(angle), center.y + radius * std::sin(angle));
  }
  return polygon;
}

}  // namespace

autoware_utils::Polygon2d make_pedestrian_swept_polygon(
  const autoware_perception_msgs::msg::PredictedObject & object, const PlannerParam & params,
  const double hysteresis)
{
  const auto & pedestrian = params.pedestrian;
  const auto & shape = object.shape.dimensions;
  const auto radius =
    std::hypot(shape.x, shape.y) / 2.0 + pedestrian.radius_margin + hysteresis;
  const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  const auto speed = std::hypot(twist.linear.x, twist.linear.y);

  if (
    pedestrian.use_predicted_path && !object.kinematics.predicted_paths.empty() &&
    object.kinematics.predicted_paths.front().confidence >=
      pedestrian.min_predicted_path_confidence) {
    const auto & path = object.kinematics.predicted_paths.front();
    autoware_utils::Polygon2d accumulated;
    bool has_accumulated = false;
    // Predicted path poses are in map frame (same as other motion_velocity_planner modules).
    for (const auto & path_pose : path.path) {
      const auto circle = make_circle_polygon(path_pose.position, radius);
      if (!has_accumulated) {
        accumulated = circle;
        has_accumulated = true;
      } else {
        autoware_utils::MultiPolygon2d union_result;
        boost::geometry::union_(accumulated, circle, union_result);
        if (!union_result.empty()) accumulated = union_result.front();
      }
    }
    if (has_accumulated) return accumulated;
  }

  const auto horizon_distance = std::max(speed * pedestrian.time_horizon, radius);
  const auto yaw = speed > pedestrian.min_approach_speed
                     ? std::atan2(twist.linear.y, twist.linear.x)
                     : tf2::getYaw(pose.orientation);
  geometry_msgs::msg::Pose end_pose = pose;
  end_pose.position.x += std::cos(yaw) * horizon_distance;
  end_pose.position.y += std::sin(yaw) * horizon_distance;
  end_pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);

  autoware_utils::Polygon2d start_circle = make_circle_polygon(pose.position, radius);
  autoware_utils::Polygon2d end_circle = make_circle_polygon(end_pose.position, radius);
  autoware_utils::MultiPolygon2d union_result;
  boost::geometry::union_(start_circle, end_circle, union_result);
  if (!union_result.empty()) return union_result.front();

  return make_circle_polygon(pose.position, radius);
}

autoware_utils::MultiPolygon2d make_pedestrian_footprints(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & pedestrians,
  const PlannerParam & params, const double hysteresis)
{
  autoware_utils::MultiPolygon2d footprints;
  footprints.reserve(pedestrians.size());
  for (const auto & pedestrian : pedestrians)
    footprints.push_back(make_pedestrian_swept_polygon(pedestrian, params, hysteresis));
  return footprints;
}

std::optional<geometry_msgs::msg::Point> find_closest_collision_point(
  const EgoData & ego_data, const autoware_utils::Polygon2d & pedestrian_footprint)
{
  std::optional<geometry_msgs::msg::Point> closest_collision_point;
  auto closest_dist = std::numeric_limits<double>::max();
  std::vector<BoxIndexPair> rough_collisions;
  ego_data.rtree.query(
    boost::geometry::index::intersects(pedestrian_footprint), std::back_inserter(rough_collisions));
  for (const auto & rough_collision : rough_collisions) {
    const auto footprint_idx = rough_collision.second;
    if (footprint_idx >= ego_data.trajectory_footprints.size()) continue;

    const auto & ego_footprint = ego_data.trajectory_footprints[footprint_idx];
    if (!boost::geometry::intersects(ego_footprint, pedestrian_footprint)) continue;

    const auto traj_idx = ego_data.trajectory_footprint_traj_indices[footprint_idx];

    autoware_utils::MultiPoint2d collision_points;
    boost::geometry::intersection(
      ego_footprint.outer(), pedestrian_footprint.outer(), collision_points);
    for (const auto & coll_p : collision_points) {
      auto p = geometry_msgs::msg::Point().set__x(coll_p.x()).set__y(coll_p.y());
      const auto dist_to_ego = autoware::motion_utils::calcSignedArcLength(
        ego_data.trajectory, ego_data.pose.position, p);
      if (dist_to_ego < closest_dist) {
        closest_dist = dist_to_ego;
        closest_collision_point = p;
      }
    }
    if (!closest_collision_point) {
      const auto & ego_pose = ego_data.trajectory[traj_idx].pose.position;
      const auto dist_to_ego = autoware::motion_utils::calcSignedArcLength(
        ego_data.trajectory, ego_data.pose.position, ego_pose);
      if (dist_to_ego < closest_dist) {
        closest_dist = dist_to_ego;
        closest_collision_point = ego_pose;
      }
    }
  }
  return closest_collision_point;
}

std::vector<Collision> find_collisions(
  const EgoData & ego_data,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & pedestrians,
  const autoware_utils::MultiPolygon2d & pedestrian_footprints)
{
  std::vector<Collision> collisions;
  for (auto object_idx = 0UL; object_idx < pedestrians.size(); ++object_idx) {
    const auto & pedestrian_footprint = pedestrian_footprints[object_idx];
    const auto collision = find_closest_collision_point(ego_data, pedestrian_footprint);
    if (collision) {
      Collision c;
      c.object_uuid = autoware_utils::to_hex_string(pedestrians[object_idx].object_id);
      c.point = *collision;
      collisions.push_back(c);
    }
  }
  return collisions;
}

}  // namespace autoware::motion_velocity_planner::trailer_pedestrian_stop
