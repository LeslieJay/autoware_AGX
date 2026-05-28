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

#include "../src/collision.hpp"
#include "../src/footprint.hpp"
#include "../src/object_filtering.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include "../src/object_stop_decision.hpp"
#include "../src/types.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <boost/geometry/algorithms/area.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

using autoware::motion_velocity_planner::trailer_pedestrian_stop::EgoData;
using autoware::motion_velocity_planner::trailer_pedestrian_stop::ObjectStopDecision;
using autoware::motion_velocity_planner::trailer_pedestrian_stop::PlannerParam;
using autoware::motion_velocity_planner::trailer_pedestrian_stop::TrajectoryPoints;

namespace
{
autoware_perception_msgs::msg::PredictedObject make_pedestrian(const double x, const double y)
{
  autoware_perception_msgs::msg::PredictedObject object;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
  object.classification = {classification};
  object.kinematics.initial_pose_with_covariance.pose.position.x = x;
  object.kinematics.initial_pose_with_covariance.pose.position.y = y;
  object.shape.dimensions.x = 0.6;
  object.shape.dimensions.y = 0.6;
  object.object_id = autoware_utils_uuid::generate_uuid();
  return object;
}

TrajectoryPoints make_straight_trajectory(const size_t n = 20)
{
  TrajectoryPoints trajectory;
  for (size_t i = 0; i < n; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = static_cast<double>(i);
    p.pose.position.y = 0.0;
    p.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
    trajectory.push_back(p);
  }
  return trajectory;
}

PlannerParam make_default_params()
{
  PlannerParam params;
  params.trailer.count = 5;
  params.trailer.length = 2.0;
  params.trailer.width = 1.2;
  params.trailer.hitch_gap = 0.3;
  params.trailer.side_margin = 0.5;
  params.trailer.rear_margin = 0.5;
  params.pedestrian.enable_pedestrian = true;
  params.pedestrian.time_horizon = 2.0;
  params.pedestrian.radius_margin = 0.5;
  params.pedestrian.max_lateral_approach_distance = 8.0;
  params.ego_lateral_offset = 1.0;
  params.ego_longitudinal_offset = 2.0;
  params.ego_rear_longitudinal_offset = 1.0;
  params.sampling.trajectory_step_length = 1.0;
  return params;
}

double sum_footprint_areas(const std::vector<autoware_utils::Polygon2d> & parts)
{
  double area = 0.0;
  for (const auto & part : parts) {
    area += boost::geometry::area(part);
  }
  return area;
}
}  // namespace

TEST(TrailerPedestrianStop, isTargetPedestrian)
{
  using autoware::motion_velocity_planner::trailer_pedestrian_stop::is_target_pedestrian;
  auto pedestrian = make_pedestrian(0.0, 2.0);
  EXPECT_TRUE(is_target_pedestrian(pedestrian));
  pedestrian.classification.front().label =
    autoware_perception_msgs::msg::ObjectClassification::CAR;
  EXPECT_FALSE(is_target_pedestrian(pedestrian));
}

TEST(TrailerPedestrianStop, trailersExtendBehindTractor)
{
  const auto params = make_default_params();
  geometry_msgs::msg::Pose pose;
  pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);

  const auto parts =
    autoware::motion_velocity_planner::trailer_pedestrian_stop::make_footprint_parts_at_pose(
      pose, params);
  ASSERT_GT(parts.size(), 1UL);

  double tractor_min_x = std::numeric_limits<double>::max();
  double trailer_min_x = std::numeric_limits<double>::max();
  for (size_t i = 0; i < parts.size(); ++i) {
    double min_x = std::numeric_limits<double>::max();
    for (const auto & p : parts[i].outer()) {
      min_x = std::min(min_x, p.x());
    }
    if (i == 0) {
      tractor_min_x = min_x;
    } else {
      trailer_min_x = std::min(trailer_min_x, min_x);
    }
  }

  EXPECT_LT(trailer_min_x, tractor_min_x);
  EXPECT_LT(trailer_min_x, -params.ego_rear_longitudinal_offset);
}

TEST(TrailerPedestrianStop, footprintPartsHaveArea)
{
  const auto params = make_default_params();
  geometry_msgs::msg::Pose pose;
  pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
  const auto parts =
    autoware::motion_velocity_planner::trailer_pedestrian_stop::make_footprint_parts_at_pose(
      pose, params);
  EXPECT_GT(sum_footprint_areas(parts), 10.0);
}

TEST(TrailerPedestrianStop, collisionDetectedForLateralPedestrian)
{
  auto params = make_default_params();
  EgoData ego_data;
  ego_data.trajectory = make_straight_trajectory();
  ego_data.pose = ego_data.trajectory.front().pose;
  autoware::motion_velocity_planner::trailer_pedestrian_stop::make_train_trailer_footprint_rtree(
    ego_data, params);

  const auto pedestrian = make_pedestrian(3.0, 1.0);
  const auto pedestrian_footprints =
    autoware::motion_velocity_planner::trailer_pedestrian_stop::make_pedestrian_footprints(
      {pedestrian}, params, 0.0);
  const auto collisions = autoware::motion_velocity_planner::trailer_pedestrian_stop::find_collisions(
    ego_data, {pedestrian}, pedestrian_footprints);
  EXPECT_FALSE(collisions.empty());
}

TEST(TrailerPedestrianStop, noCollisionWhenPedestrianFarAway)
{
  auto params = make_default_params();
  EgoData ego_data;
  ego_data.trajectory = make_straight_trajectory();
  ego_data.pose = ego_data.trajectory.front().pose;
  autoware::motion_velocity_planner::trailer_pedestrian_stop::make_train_trailer_footprint_rtree(
    ego_data, params);

  const auto pedestrian = make_pedestrian(5.0, 20.0);
  const auto pedestrian_footprints =
    autoware::motion_velocity_planner::trailer_pedestrian_stop::make_pedestrian_footprints(
      {pedestrian}, params, 0.0);
  const auto collisions = autoware::motion_velocity_planner::trailer_pedestrian_stop::find_collisions(
    ego_data, {pedestrian}, pedestrian_footprints);
  EXPECT_TRUE(collisions.empty());
}

TEST(TrailerPedestrianStop, ignoreUnavoidableWhenMovingAway)
{
  using autoware::motion_velocity_planner::trailer_pedestrian_stop::filter_predicted_objects;
  auto params = make_default_params();
  params.stop.ignore_unavoidable_collisions = true;
  EgoData ego_data;
  ego_data.trajectory = make_straight_trajectory();
  ego_data.pose = ego_data.trajectory.front().pose;

  auto pedestrian = make_pedestrian(5.0, 4.0);
  pedestrian.kinematics.initial_twist_with_covariance.twist.linear.y = 1.0;

  auto obj_ptr = std::make_shared<autoware::motion_velocity_planner::PlannerData::Object>();
  obj_ptr->predicted_object = pedestrian;
  const auto filtered = filter_predicted_objects({obj_ptr}, ego_data, params, 0.0);
  EXPECT_TRUE(filtered.empty());
}

TEST(TrailerPedestrianStop, rearMarginExpandsFootprint)
{
  auto params = make_default_params();
  params.trailer.rear_margin = 2.0;
  geometry_msgs::msg::Pose pose;
  pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
  const auto with_rear =
    autoware::motion_velocity_planner::trailer_pedestrian_stop::make_footprint_parts_at_pose(
      pose, params);
  params.trailer.rear_margin = 0.0;
  const auto baseline =
    autoware::motion_velocity_planner::trailer_pedestrian_stop::make_footprint_parts_at_pose(
      pose, params);
  EXPECT_GT(sum_footprint_areas(with_rear), sum_footprint_areas(baseline));
  EXPECT_GT(with_rear.size(), baseline.size());
}

TEST(TrailerPedestrianStop, objectStopDecisionBuffers)
{
  ObjectStopDecision decision;
  const rclcpp::Time t0{0, 0, RCL_ROS_TIME};
  decision.collision_detected = true;
  decision.update_timers(t0, 0.2, 1.0);
  EXPECT_FALSE(decision.should_be_avoided());
  const rclcpp::Time t1{0, 300000000, RCL_ROS_TIME};
  decision.update_timers(t1, 0.2, 1.0);
  EXPECT_TRUE(decision.should_be_avoided());
  decision.collision_detected = false;
  const rclcpp::Time t2{1, 0, RCL_ROS_TIME};
  decision.update_timers(t2, 0.2, 1.0);
  EXPECT_TRUE(decision.should_be_avoided());
  const rclcpp::Time t3{2, 100000000, RCL_ROS_TIME};
  decision.update_timers(t3, 0.2, 1.0);
  EXPECT_FALSE(decision.should_be_avoided());
}
