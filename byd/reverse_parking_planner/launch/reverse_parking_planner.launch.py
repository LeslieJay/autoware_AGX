import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


"""
reverse_parking_planner.launch.py

该 Launch 文件负责：
- 声明并解析与倒车入库规划器相关的启动参数（配置文件路径、输入里程计话题、输出轨迹话题）
- 创建并启动 `reverse_parking_planner` 节点，并完成话题 remap 与参数加载
"""


def generate_launch_description():
    """
    生成 ROS2 LaunchDescription。

    返回的 LaunchDescription 包含：
    - 3 个可配置的启动参数：
      - config_file：节点参数配置文件路径
      - input_odom：输入里程计话题名
      - output_trajectory：输出轨迹话题名
    - 1 个 ReverseParkingPlannerNode 节点实例
    """
    # 获取当前 package 的 share 目录（用于定位默认参数 YAML）
    pkg_share = get_package_share_directory('reverse_parking_planner')
    
    # 默认配置文件路径：安装目录下的 config/reverse_parking_planner.param.yaml
    config_file = os.path.join(pkg_share, 'config', 'reverse_parking_planner.param.yaml')
    
    # 声明配置文件路径参数，允许在启动时通过 `config_file:=xxx` 覆盖默认值
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the config file'
    )
    
    # 声明输入里程计话题参数（默认接入车辆定位模块的 kinematic_state）
    declare_input_odom = DeclareLaunchArgument(
        'input_odom',
        default_value='/localization/kinematic_state',
        description='Input odometry topic'
    )
    
    # 声明输出轨迹话题参数（默认接入 Autoware 停车轨迹接口）
    declare_output_trajectory = DeclareLaunchArgument(
        'output_trajectory',
        default_value='/planning/scenario_planning/parking/trajectory',
        description='Output trajectory topic'
    )
    
    # 创建倒车入库规划节点：
    # - parameters：从 config_file 中加载节点参数
    # - remappings：将节点内部的私有话题 remap 到全局/上游/下游话题
    reverse_parking_planner_node = Node(
        package='reverse_parking_planner',
        executable='reverse_parking_planner_node_exe',
        name='reverse_parking_planner',
        namespace='',
        output='screen',
        # 使用 LaunchConfiguration 绑定启动参数，运行时会替换为实际值
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # 里程计输入 remap：节点内使用 ~/input/odometry，外部由 input_odom 提供
            ('~/input/odometry', LaunchConfiguration('input_odom')),
            # 轨迹输出 remap：节点内使用 ~/output/trajectory，外部由 output_trajectory 接收
            ('~/output/trajectory', LaunchConfiguration('output_trajectory')),
            # 可视化 Marker 输出 remap：固定映射到停车规划可视化话题
            ('~/output/path_markers', '/planning/parking/path_markers'),
        ],
    )
    
    # 将所有 Launch 动作组织成一个 LaunchDescription 返回。
    return LaunchDescription([
        declare_config_file,
        declare_input_odom,
        declare_output_trajectory,
        reverse_parking_planner_node,
    ])
