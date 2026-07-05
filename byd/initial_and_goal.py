#!/usr/bin/env python3
"""
通过话题发布初始位姿和终点位姿。
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_vehicle_msgs.msg import Engage


# ── 起始位姿 ──────────────────────────────────────────────────────────────────
START_X = 23.241851806640625
START_Y = -11.649909973144531
START_Z = -1.6219378753823483
START_OX = -0.012247327209749626
START_OY = -0.012271131496340485
START_OZ = -0.7063137630176388
START_OW = 0.7076865764446006

# ── 终点位姿 ──────────────────────────────────────────────────────────────────
GOAL_X = 29.286663055419922
GOAL_Y = -186.04461669921875
GOAL_Z = 4.4608073702486974
GOAL_OX = 0.0
GOAL_OY = 0.0
GOAL_OZ = -0.7210580500984284
GOAL_OW = 0.6928746556111375

FRAME_ID = "map"
INITIAL_POSE_TOPIC = "/initialpose"
INITIAL_POSE_3D_TOPIC = "/initialpose3d"
GOAL_TOPIC = "/planning/mission_planning/goal"
WAIT_SUBSCRIBER_TIMEOUT_SEC = 10.0
PUBLISH_SETTLE_SEC = 0.5

# 位置协方差（对角线，单位 m²）
COV_XY = 1.0
COV_Z = 0.01
COV_ROLL_PITCH = 0.01
COV_YAW = 10.0


class InitAndGoalNode(Node):
    def __init__(self):
        super().__init__("init_and_goal_node")

        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, INITIAL_POSE_TOPIC, 10
        )
        self._initial_pose_3d_pub = self.create_publisher(
            PoseWithCovarianceStamped, INITIAL_POSE_3D_TOPIC, 10
        )
        self._goal_pub = self.create_publisher(PoseStamped, GOAL_TOPIC, 10)

    def _wait_for_subscribers(self, publisher, topic_name: str) -> bool:
        self.get_logger().info(f"等待话题 {topic_name} 的订阅者 ...")
        deadline = self.get_clock().now() + Duration(seconds=WAIT_SUBSCRIBER_TIMEOUT_SEC)

        while rclpy.ok() and self.get_clock().now() < deadline:
            if publisher.get_subscription_count() > 0:
                self.get_logger().info(f"话题 {topic_name} 已有订阅者")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            f"话题 {topic_name} 在 {WAIT_SUBSCRIBER_TIMEOUT_SEC}s 内没有订阅者，退出"
        )
        return False

    def _build_initial_pose_msg(self) -> PoseWithCovarianceStamped:
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = FRAME_ID
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.pose.pose.position.x = START_X
        initial_pose_msg.pose.pose.position.y = START_Y
        initial_pose_msg.pose.pose.position.z = START_Z
        initial_pose_msg.pose.pose.orientation.x = START_OX
        initial_pose_msg.pose.pose.orientation.y = START_OY
        initial_pose_msg.pose.pose.orientation.z = START_OZ
        initial_pose_msg.pose.pose.orientation.w = START_OW

        initial_pose_msg.pose.covariance[0] = COV_XY
        initial_pose_msg.pose.covariance[7] = COV_XY
        initial_pose_msg.pose.covariance[14] = COV_Z
        initial_pose_msg.pose.covariance[21] = COV_ROLL_PITCH
        initial_pose_msg.pose.covariance[28] = COV_ROLL_PITCH
        initial_pose_msg.pose.covariance[35] = COV_YAW
        return initial_pose_msg

    def _publish_initial_pose(self) -> bool:
        if not self._wait_for_subscribers(self._initial_pose_pub, INITIAL_POSE_TOPIC):
            return False

        initial_pose_msg = self._build_initial_pose_msg()
        self._initial_pose_pub.publish(initial_pose_msg)
        self.get_logger().info(
            f"已发布初始位姿到 {INITIAL_POSE_TOPIC}: "
            f"pos=({START_X:.2f}, {START_Y:.2f}, {START_Z:.2f})"
        )

        if self._wait_for_subscribers(self._initial_pose_3d_pub, INITIAL_POSE_3D_TOPIC):
            self._initial_pose_3d_pub.publish(initial_pose_msg)
            self.get_logger().info(f"已发布初始位姿到 {INITIAL_POSE_3D_TOPIC}")
        return True

    def _publish_goal(self) -> bool:
        if not self._wait_for_subscribers(self._goal_pub, GOAL_TOPIC):
            return False

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = FRAME_ID
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = GOAL_X
        goal_msg.pose.position.y = GOAL_Y
        goal_msg.pose.position.z = GOAL_Z
        goal_msg.pose.orientation.x = GOAL_OX
        goal_msg.pose.orientation.y = GOAL_OY
        goal_msg.pose.orientation.z = GOAL_OZ
        goal_msg.pose.orientation.w = GOAL_OW

        self._goal_pub.publish(goal_msg)
        self.get_logger().info(
            f"已发布终点位姿到 {GOAL_TOPIC}: "
            f"pos=({GOAL_X:.2f}, {GOAL_Y:.2f}, {GOAL_Z:.2f})"
        )
        return True

    def run(self):
        if not self._publish_initial_pose():
            return

        rclpy.spin_once(self, timeout_sec=PUBLISH_SETTLE_SEC)
        self._publish_goal()
        # 等待规划/诊断就绪后再切换 AUTONOMOUS
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=1.0)
        self._engage_autonomous()

    def _engage_autonomous(self) -> None:
        client = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_autonomous")
        for attempt in range(5):
            if not client.wait_for_service(timeout_sec=3.0):
                continue
            future = client.call_async(ChangeOperationMode.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() and future.result().status.success:
                self.get_logger().info("已切换至 AUTONOMOUS 模式")
                break
            self.get_logger().warn(
                f"切换 AUTONOMOUS 失败 (attempt {attempt + 1}/5)，等待后重试..."
            )
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=1.0)
        else:
            self.get_logger().warn("切换 AUTONOMOUS 模式最终失败，请手动调用 change_to_autonomous")

        engage_pub = self.create_publisher(Engage, "/vehicle/engage", 10)
        deadline = self.get_clock().now() + Duration(seconds=3.0)
        while rclpy.ok() and self.get_clock().now() < deadline:
            if engage_pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        engage_pub.publish(Engage(engage=True))
        self.get_logger().info("已发布 /vehicle/engage: true")


def main():
    rclpy.init()
    node = InitAndGoalNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()