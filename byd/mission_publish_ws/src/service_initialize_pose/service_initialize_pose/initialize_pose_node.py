#!/usr/bin/env python3
"""
ROS2 Node for initializing pose via /api/localization/initialize service.

This node calls the Autoware AD API service to initialize the vehicle's pose.
Supports setting initial pose from configuration parameters.
"""

import rclpy
from rclpy.node import Node
from autoware_adapi_v1_msgs.srv import InitializeLocalization
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header


class InitializePoseNode(Node):
    """Node for calling the initialize_localization service to set initial pose."""

    def __init__(self):
        super().__init__('initialize_pose_node')

        # Declare parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('orientation_x', 0.0)
        self.declare_parameter('orientation_y', 0.0)
        self.declare_parameter('orientation_z', 0.0)
        self.declare_parameter('orientation_w', 1.0)
        self.declare_parameter('auto_initialize', False)  # If True, use auto initialization (no pose needed)

        # Get parameters
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        auto_initialize = self.get_parameter('auto_initialize').get_parameter_value().bool_value

        # Create service client
        self.client = self.create_client(
            InitializeLocalization,
            '/api/localization/initialize'
        )

        self.get_logger().info('Waiting for /api/localization/initialize service...')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service is available!')

        # Send initial pose
        if auto_initialize:
            self.send_auto_initialize()
        else:
            self.send_initialize_pose()

    def send_initialize_pose(self):
        """Send initial pose via the initialize_localization service."""
        # Get pose parameters
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        z = self.get_parameter('z').get_parameter_value().double_value
        ox = self.get_parameter('orientation_x').get_parameter_value().double_value
        oy = self.get_parameter('orientation_y').get_parameter_value().double_value
        oz = self.get_parameter('orientation_z').get_parameter_value().double_value
        ow = self.get_parameter('orientation_w').get_parameter_value().double_value

        request = InitializeLocalization.Request()

        # Create pose with covariance
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = z
        pose_msg.pose.pose.orientation.x = ox
        pose_msg.pose.pose.orientation.y = oy
        pose_msg.pose.pose.orientation.z = oz
        pose_msg.pose.pose.orientation.w = ow

        # Set default covariance (6x6 matrix)
        # Position covariance (x, y, z)
        pose_msg.pose.covariance[0] = 1.0   # x
        pose_msg.pose.covariance[7] = 1.0    # y
        pose_msg.pose.covariance[14] = 0.01  # z
        # Orientation covariance (roll, pitch, yaw)
        pose_msg.pose.covariance[21] = 0.01  # roll
        pose_msg.pose.covariance[28] = 0.01  # pitch
        pose_msg.pose.covariance[35] = 10.0  # yaw

        request.pose = [pose_msg]

        self.get_logger().info(
            f'Sending initial pose: position=({x:.2f}, {y:.2f}, {z:.2f}), '
            f'orientation=({ox:.4f}, {oy:.4f}, {oz:.4f}, {ow:.4f})'
        )

        # Send request asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.initialize_response_callback)

    def send_auto_initialize(self):
        """Send auto initialization request (uses GNSS position)."""
        request = InitializeLocalization.Request()
        # Empty pose list means auto-initialize using GNSS
        request.pose = []

        self.get_logger().info('Sending auto-initialize request (using GNSS position)...')

        # Send request asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.initialize_response_callback)

    def initialize_response_callback(self, future):
        """Handle the service response."""
        try:
            response = future.result()
            if response.status.success:
                self.get_logger().info('Pose initialized successfully!')
            else:
                error_code = response.status.code
                error_message = response.status.message
                self.get_logger().error(
                    f'Failed to initialize pose: code={error_code}, message={error_message}'
                )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = InitializePoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

