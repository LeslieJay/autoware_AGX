#!/usr/bin/env python3
"""
发布一个前方倒车接近的 PredictedObject，用于验证 obstacle_cruise 的 horn_request 话题。
"""

import argparse
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
    Shape,
)
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistWithCovariance


PREDICTED_OBJECTS_TOPIC = "/perception/object_recognition/objects"
FRAME_ID = "map"
PUBLISH_HZ = 10.0
VEHICLE_YAW = -math.pi / 2  # 绕 Z 轴朝向 [rad]


class PassiveCollisionHornPublisher(Node):
    def __init__(self, x: float, y: float, z: float, reverse_speed: float):
        super().__init__("passive_collision_horn_publisher")
        self._x = x
        self._y = y
        self._z = z
        self._reverse_speed = reverse_speed
        self._publisher = self.create_publisher(PredictedObjects, PREDICTED_OBJECTS_TOPIC, 10)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info(
            f"Publishing reversing front obstacle at ({x:.2f}, {y:.2f}, {z:.2f}), "
            f"reverse_speed={reverse_speed:.2f} m/s"
        )

    def _publish(self) -> None:
        msg = PredictedObjects()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID

        obj = PredictedObject()
        obj.existence_probability = 1.0

        classification = ObjectClassification()
        classification.label = ObjectClassification.CAR
        classification.probability = 1.0
        obj.classification.append(classification)

        pose = obj.kinematics.initial_pose_with_covariance.pose
        pose.position.x = self._x
        pose.position.y = self._y
        pose.position.z = self._z
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(VEHICLE_YAW / 2)
        pose.orientation.w = math.cos(VEHICLE_YAW / 2)

        # 倒车：map 系速度沿车头反方向
        speed = abs(self._reverse_speed)
        forward_x = math.cos(VEHICLE_YAW)
        forward_y = math.sin(VEHICLE_YAW)
        reverse_x = -forward_x
        reverse_y = -forward_y

        covariance = [0.0] * 36
        covariance[0] = 0.0009
        covariance[7] = 0.0009
        covariance[14] = 0.0009
        covariance[35] = 0.0076
        obj.kinematics.initial_pose_with_covariance.covariance = covariance

        twist = TwistWithCovariance()
        twist.twist.linear.x = reverse_x * speed
        twist.twist.linear.y = reverse_y * speed
        obj.kinematics.initial_twist_with_covariance = twist

        obj.shape.type = Shape.BOUNDING_BOX
        obj.shape.dimensions.x = 4.5
        obj.shape.dimensions.y = 2.0
        obj.shape.dimensions.z = 1.8

        predicted_path = PredictedPath()
        predicted_path.time_step = Duration(seconds=0.2).to_msg()
        predicted_path.confidence = 1.0
        for i in range(20):
            path_pose = Pose()
            path_pose.position.x = self._x + reverse_x * speed * 0.2 * i
            path_pose.position.y = self._y + reverse_y * speed * 0.2 * i
            path_pose.position.z = self._z
            path_pose.orientation.x = 0.0
            path_pose.orientation.y = 0.0
            path_pose.orientation.z = math.sin(VEHICLE_YAW / 2)
            path_pose.orientation.w = math.cos(VEHICLE_YAW / 2)
            predicted_path.path.append(path_pose)
        obj.kinematics.predicted_paths.append(predicted_path)

        msg.objects.append(obj)
        self._publisher.publish(msg)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish a reversing front obstacle")
    parser.add_argument("--x", type=float, default=23.241851806640625, help="map x [m]")
    parser.add_argument("--y", type=float, default=-25.649909973144531, help="map y [m]")
    parser.add_argument("--z", type=float, default=-1.6219378753823483, help="map z [m]")
    parser.add_argument("--reverse-speed", type=float, default=1.0, help="reverse speed [m/s]")
    return parser.parse_args()


def main(args=None) -> None:
    parsed = parse_args()
    rclpy.init(args=args)
    node = PassiveCollisionHornPublisher(parsed.x, parsed.y, parsed.z, parsed.reverse_speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
