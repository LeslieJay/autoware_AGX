#!/usr/bin/env python3
"""Launch file for initialize_pose_node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for initialize_pose_node."""

    # Get package share directory
    pkg_share = get_package_share_directory('service_initialize_pose')

    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'initial_pose.yaml')

    # Declare launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the initial pose configuration file'
    )

    # Create the node - parameters come from YAML file only
    initialize_pose_node = Node(
        package='service_initialize_pose',
        executable='initialize_pose_node',
        name='initialize_pose_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        initialize_pose_node,
    ])

