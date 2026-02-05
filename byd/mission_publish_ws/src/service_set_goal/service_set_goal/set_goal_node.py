#!/usr/bin/env python3
"""
ROS2 Node for setting multiple route points via /api/routing/set_route_points service.

This node calls the Autoware AD API service to set route points for autonomous navigation.
Supports multiple goals with sequential execution and arrival detection.
"""

import rclpy
from rclpy.node import Node
from autoware_adapi_v1_msgs.srv import SetRoutePoints
from autoware_adapi_v1_msgs.msg import RouteState
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class SetGoalNode(Node):
    """Node for calling the set_route_points service to send multiple goal points."""

    # Routing state constants
    ROUTE_STATE_UNKNOWN = 0
    ROUTE_STATE_UNSET = 1
    ROUTE_STATE_SET = 2
    ROUTE_STATE_ARRIVED = 3
    ROUTE_STATE_CHANGING = 4

    def __init__(self):
        super().__init__('set_goal_node')

        # Declare parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('allow_goal_modification', True)
        self.declare_parameter('loop', False)  # Whether to loop through goals
        self.declare_parameter('goal_names', ['A', 'B'])  # List of goal names

        # Get goal names
        goal_names = self.get_parameter('goal_names').get_parameter_value().string_array_value

        # Declare parameters for each goal
        self.goals = []
        for name in goal_names:
            self.declare_parameter(f'goals.{name}.x', 0.0)
            self.declare_parameter(f'goals.{name}.y', 0.0)
            self.declare_parameter(f'goals.{name}.z', 0.0)
            self.declare_parameter(f'goals.{name}.orientation_x', 0.0)
            self.declare_parameter(f'goals.{name}.orientation_y', 0.0)
            self.declare_parameter(f'goals.{name}.orientation_z', 0.0)
            self.declare_parameter(f'goals.{name}.orientation_w', 1.0)
            self.declare_parameter(f'goals.{name}.wait_time', 5.0)  # Wait time after arrival

        # Load goals from parameters
        self._load_goals(goal_names)

        # State management
        self.current_goal_index = 0
        self.is_waiting = False
        self.wait_timer = None
        self.route_state = self.ROUTE_STATE_UNKNOWN
        self.goal_sent = False

        # Create service client
        self.client = self.create_client(
            SetRoutePoints,
            '/api/routing/set_route_points'
        )

        # Subscribe to routing state
        self.route_state_sub = self.create_subscription(
            RouteState,
            '/api/routing/state',
            self.route_state_callback,
            10
        )

        self.get_logger().info('Waiting for /api/routing/set_route_points service...')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service is available!')
        self.get_logger().info(f'Loaded {len(self.goals)} goals: {[g["name"] for g in self.goals]}')

        # Send first goal
        self.send_current_goal()

    def _load_goals(self, goal_names):
        """Load goal points from parameters."""
        for name in goal_names:
            goal = {
                'name': name,
                'x': self.get_parameter(f'goals.{name}.x').get_parameter_value().double_value,
                'y': self.get_parameter(f'goals.{name}.y').get_parameter_value().double_value,
                'z': self.get_parameter(f'goals.{name}.z').get_parameter_value().double_value,
                'orientation_x': self.get_parameter(f'goals.{name}.orientation_x').get_parameter_value().double_value,
                'orientation_y': self.get_parameter(f'goals.{name}.orientation_y').get_parameter_value().double_value,
                'orientation_z': self.get_parameter(f'goals.{name}.orientation_z').get_parameter_value().double_value,
                'orientation_w': self.get_parameter(f'goals.{name}.orientation_w').get_parameter_value().double_value,
                'wait_time': self.get_parameter(f'goals.{name}.wait_time').get_parameter_value().double_value,
            }
            self.goals.append(goal)
            self.get_logger().info(
                f'Loaded goal {name}: ({goal["x"]:.2f}, {goal["y"]:.2f}), '
                f'wait_time={goal["wait_time"]}s'
            )

    def route_state_callback(self, msg: RouteState):
        """Handle routing state changes."""
        prev_state = self.route_state
        self.route_state = msg.state

        # Log state changes
        state_names = {
            self.ROUTE_STATE_UNKNOWN: 'UNKNOWN',
            self.ROUTE_STATE_UNSET: 'UNSET',
            self.ROUTE_STATE_SET: 'SET',
            self.ROUTE_STATE_ARRIVED: 'ARRIVED',
            self.ROUTE_STATE_CHANGING: 'CHANGING'
        }

        if prev_state != self.route_state:
            self.get_logger().info(
                f'Route state changed: {state_names.get(prev_state, "?")} -> '
                f'{state_names.get(self.route_state, "?")}'
            )

        # Check if arrived at current goal
        if self.route_state == self.ROUTE_STATE_ARRIVED and self.goal_sent and not self.is_waiting:
            self.on_goal_arrived()

    def on_goal_arrived(self):
        """Handle arrival at current goal."""
        current_goal = self.goals[self.current_goal_index]
        wait_time = current_goal['wait_time']

        self.get_logger().info(
            f'Arrived at goal {current_goal["name"]}! '
            f'Waiting {wait_time} seconds before next goal...'
        )

        self.is_waiting = True
        self.goal_sent = False

        # Start wait timer
        self.wait_timer = self.create_timer(wait_time, self.on_wait_complete)

    def on_wait_complete(self):
        """Handle wait timer completion."""
        # Cancel timer (one-shot)
        if self.wait_timer:
            self.wait_timer.cancel()
            self.wait_timer = None

        self.is_waiting = False

        # Move to next goal
        self.current_goal_index += 1
        loop = self.get_parameter('loop').get_parameter_value().bool_value

        if self.current_goal_index >= len(self.goals):
            if loop:
                self.current_goal_index = 0
                self.get_logger().info('Looping back to first goal...')
            else:
                self.get_logger().info('All goals completed!')
                return

        # Send next goal
        self.send_current_goal()

    def send_current_goal(self):
        """Send the current goal in the sequence."""
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('No more goals to send.')
            return

        goal = self.goals[self.current_goal_index]
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        allow_goal_modification = self.get_parameter('allow_goal_modification').get_parameter_value().bool_value

        self.get_logger().info(
            f'Sending goal {self.current_goal_index + 1}/{len(self.goals)}: {goal["name"]}'
        )

        self.send_goal(
            goal['x'], goal['y'], goal['z'],
            goal['orientation_x'], goal['orientation_y'],
            goal['orientation_z'], goal['orientation_w'],
            frame_id, allow_goal_modification
        )

    def send_goal(self, x: float, y: float, z: float,
                  ox: float, oy: float, oz: float, ow: float,
                  frame_id: str = 'map', allow_goal_modification: bool = True):
        """
        Send a goal point via the set_route_points service.

        Args:
            x: Position x coordinate
            y: Position y coordinate
            z: Position z coordinate
            ox: Orientation x (quaternion)
            oy: Orientation y (quaternion)
            oz: Orientation z (quaternion)
            ow: Orientation w (quaternion)
            frame_id: Frame ID for the pose header
            allow_goal_modification: Whether to allow goal modification
        """
        request = SetRoutePoints.Request()

        # Set header
        request.header = Header()
        request.header.frame_id = frame_id
        request.header.stamp = self.get_clock().now().to_msg()

        # Set route option
        request.option.allow_goal_modification = allow_goal_modification

        # Set goal pose
        request.goal = Pose()
        request.goal.position.x = x
        request.goal.position.y = y
        request.goal.position.z = z
        request.goal.orientation.x = ox
        request.goal.orientation.y = oy
        request.goal.orientation.z = oz
        request.goal.orientation.w = ow

        # waypoints is empty (direct navigation to goal)
        request.waypoints = []

        self.get_logger().info(
            f'Sending goal: position=({x:.2f}, {y:.2f}, {z:.2f}), '
            f'orientation=({ox:.4f}, {oy:.4f}, {oz:.4f}, {ow:.4f})'
        )

        # Send request asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the service response."""
        try:
            response = future.result()
            if response.status.success:
                self.get_logger().info('Goal set successfully!')
                self.goal_sent = True
            else:
                self.get_logger().error(
                    f'Failed to set goal: {response.status.message}'
                )
                self.goal_sent = False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            self.goal_sent = False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = SetGoalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
