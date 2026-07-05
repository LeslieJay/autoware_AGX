#!/usr/bin/env python3
"""Publish a static UNKNOWN obstacle for static_obstacle_avoidance testing."""

import argparse
import math

import rclpy
from rclpy.node import Node

from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjects,
    ObjectClassification,
    Shape,
)
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LiveObstaclePublisher(Node):
    def __init__(
        self,
        mode: str,
        ahead_m: float,
        lateral_m: float,
        fixed_x: float,
        fixed_y: float,
        fixed_z: float,
        length_m: float,
        width_m: float,
        height_m: float,
    ):
        super().__init__("live_obstacle_publisher")
        self.mode = mode
        self.ahead_m = ahead_m
        self.lateral_m = lateral_m
        self.fixed_x = fixed_x
        self.fixed_y = fixed_y
        self.fixed_z = fixed_z
        self.length_m = length_m
        self.width_m = width_m
        self.height_m = height_m
        self.ego_odom = None

        self.publisher_ = self.create_publisher(
            DetectedObjects,
            "/perception/object_recognition/detection/objects",
            10,
        )
        if self.mode == "ahead":
            self.create_subscription(
                Odometry, "/localization/kinematic_state", self._on_odom, 10
            )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f"Publishing UNKNOWN obstacle mode={mode} ahead={ahead_m}m lateral={lateral_m}m"
        )

    def _on_odom(self, msg: Odometry):
        self.ego_odom = msg

    def _obstacle_pose(self):
        pose = DetectedObject().kinematics.pose_with_covariance.pose
        if self.mode == "fixed":
            pose.position.x = self.fixed_x
            pose.position.y = self.fixed_y
            pose.position.z = self.fixed_z
            pose.orientation.z = 0.707
            pose.orientation.w = -0.707
            return pose

        if self.ego_odom is None:
            return None

        ego = self.ego_odom.pose.pose
        yaw = yaw_from_quaternion(ego.orientation)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        pose.position.x = ego.position.x + self.ahead_m * cos_y - self.lateral_m * sin_y
        pose.position.y = ego.position.y + self.ahead_m * sin_y + self.lateral_m * cos_y
        pose.position.z = ego.position.z
        pose.orientation = ego.orientation
        return pose

    def timer_callback(self):
        pose = self._obstacle_pose()
        if pose is None:
            return

        msg = DetectedObjects()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        obj = DetectedObject()
        obj.existence_probability = 1.0

        class_msg = ObjectClassification()
        class_msg.label = ObjectClassification.UNKNOWN
        class_msg.probability = 1.0
        obj.classification.append(class_msg)

        obj.kinematics.pose_with_covariance.pose = pose
        covariance = [0.0] * 36
        covariance[0] = 0.0009
        covariance[7] = 0.0009
        covariance[14] = 0.0009
        covariance[35] = 0.0076
        obj.kinematics.pose_with_covariance.covariance = covariance
        obj.kinematics.twist_with_covariance.twist.linear.x = 0.0

        obj.shape.type = Shape.BOUNDING_BOX
        obj.shape.dimensions.x = self.length_m
        obj.shape.dimensions.y = self.width_m
        obj.shape.dimensions.z = self.height_m

        msg.objects.append(obj)
        self.publisher_.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description="Publish static obstacle for avoidance test")
    parser.add_argument(
        "--mode",
        choices=["ahead", "fixed"],
        default="ahead",
        help="ahead: place obstacle in front of ego; fixed: use map coordinates",
    )
    parser.add_argument("--ahead", type=float, default=15.0, help="longitudinal offset [m]")
    parser.add_argument(
        "--lateral",
        type=float,
        default=1.5,
        help="lateral offset from ego heading [m], helps parked_vehicle ratio",
    )
    parser.add_argument("--fixed-x", type=float, default=22.0)
    parser.add_argument("--fixed-y", type=float, default=-66.9)
    parser.add_argument("--fixed-z", type=float, default=-0.41)
    parser.add_argument("--length", type=float, default=4.5)
    parser.add_argument("--width", type=float, default=2.0)
    parser.add_argument("--height", type=float, default=1.5)
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = LiveObstaclePublisher(
        mode=cli_args.mode,
        ahead_m=cli_args.ahead,
        lateral_m=cli_args.lateral,
        fixed_x=cli_args.fixed_x,
        fixed_y=cli_args.fixed_y,
        fixed_z=cli_args.fixed_z,
        length_m=cli_args.length,
        width_m=cli_args.width,
        height_m=cli_args.height,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
