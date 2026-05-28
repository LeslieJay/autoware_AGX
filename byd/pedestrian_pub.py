#!/usr/bin/env python3
"""
向 motion_velocity_planner 发布 PEDESTRIAN，用于测试 trailer_pedestrian_stop 模块。

Planning Simulator 默认将规划器输入改到 /test/pedestrian_objects，
避免与 map_based_prediction 在 /perception/object_recognition/objects 上竞争。
"""

import argparse
import uuid

import rclpy
from rclpy.node import Node

from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    PredictedPath,
    Shape,
)
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, TwistWithCovariance

DEFAULT_TOPIC = "/test/pedestrian_objects"
FRAME_ID = "map"
PUBLISH_HZ = 10.0

# 默认：num_3-num_8-map 起点，车头朝 -Y，挂车沿 +Y 向后延伸
# 起点 ego≈(23.24,-11.65)：Y=-11.5 牵引车侧；Y=-8 挂车列侧面；Y≈0 列车尾部
DEFAULT_X = 25.0
DEFAULT_Y = -8.0
DEFAULT_Z = -1.62
DEFAULT_VX = 0.0
DEFAULT_VY = 0.0


class PedestrianPublisher(Node):
    def __init__(
        self,
        topic: str,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
    ):
        super().__init__("pedestrian_publisher")
        self._x = x
        self._y = y
        self._z = z
        self._vx = vx
        self._vy = vy
        self._object_uuid = list(uuid.uuid4().bytes)
        self._publisher = self.create_publisher(PredictedObjects, topic, 10)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info(
            f"Publishing PEDESTRIAN at ({x:.2f}, {y:.2f}, {z:.2f}) "
            f"vel=({vx:.2f}, {vy:.2f}) on {topic}"
        )

    def _make_predicted_path(self) -> PredictedPath:
        """横穿路径：从当前位置向列车方向延伸，便于 use_predicted_path 判撞。"""
        path = PredictedPath()
        path.confidence = 1.0
        path.time_step = Duration(sec=0, nanosec=500_000_000)

        speed = (self._vx**2 + self._vy**2) ** 0.5
        if speed >= 0.1:
            dx = self._vx / speed
            dy = self._vy / speed
        else:
            # 静止行人：默认向 -X（列车方向）横穿
            dx, dy = -1.0, 0.0

        for i in range(9):
            pose = Pose()
            step = i * 0.5
            pose.position.x = self._x + dx * step
            pose.position.y = self._y + dy * step
            pose.position.z = self._z
            pose.orientation.w = 1.0
            path.path.append(pose)

        return path

    def _publish(self) -> None:
        msg = PredictedObjects()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID

        obj = PredictedObject()
        obj.object_id.uuid = self._object_uuid
        obj.existence_probability = 1.0

        classification = ObjectClassification()
        classification.label = ObjectClassification.PEDESTRIAN
        classification.probability = 1.0
        obj.classification.append(classification)

        pose = obj.kinematics.initial_pose_with_covariance.pose
        pose.position.x = self._x
        pose.position.y = self._y
        pose.position.z = self._z
        pose.orientation.w = 1.0

        covariance = [0.0] * 36
        covariance[0] = 0.0009
        covariance[7] = 0.0009
        covariance[14] = 0.0009
        covariance[35] = 0.0076
        obj.kinematics.initial_pose_with_covariance.covariance = covariance

        twist = TwistWithCovariance()
        twist.twist.linear.x = self._vx
        twist.twist.linear.y = self._vy
        obj.kinematics.initial_twist_with_covariance = twist

        obj.kinematics.predicted_paths.append(self._make_predicted_path())

        try:
            obj.shape.type = Shape.BOUNDING_BOX
        except AttributeError:
            obj.shape.type = 1
        obj.shape.dimensions.x = 0.6
        obj.shape.dimensions.y = 0.6
        obj.shape.dimensions.z = 1.7

        msg.objects.append(obj)
        self._publisher.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish a PEDESTRIAN for trailer_pedestrian_stop tests")
    parser.add_argument("--topic", type=str, default=DEFAULT_TOPIC, help="PredictedObjects topic")
    parser.add_argument("--x", type=float, default=DEFAULT_X, help="map x [m]")
    parser.add_argument("--y", type=float, default=DEFAULT_Y, help="map y [m]")
    parser.add_argument("--z", type=float, default=DEFAULT_Z, help="map z [m]")
    parser.add_argument("--vx", type=float, default=DEFAULT_VX, help="velocity x [m/s]")
    parser.add_argument("--vy", type=float, default=DEFAULT_VY, help="velocity y [m/s]")
    return parser.parse_args()


def main(args: list[str] | None = None) -> None:
    parsed = parse_args()
    rclpy.init(args=args)
    node = PedestrianPublisher(parsed.topic, parsed.x, parsed.y, parsed.z, parsed.vx, parsed.vy)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
