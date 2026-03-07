1. param
/planning/scenario_planning/velocity_smoother:

normal.max_acc normal.max_jerk normal.min_acc max_velocity

/control/trajectory_follower/controller_node_exe

/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner 

/planning/scenario_planning/lane_driving/motion_planning/elastic_band_smoother/output/trajectory

关闭转向角速率限制
ros2 param set /planning/scenario_planning/velocity_smoother enable_steering_rate_limit false

### main function flowchart

1. onCurrentTrajectory
1.1 receive data odometry acceleration external_velocity_limit operation_mode
1.2 calculate trajectory velocity // convertToTrajectoryPointArray
1.3 calculate distance to insert external velocity limit
1.4 calcTrajectoryVelocity
1.5 resampleTrajectory

2. calcTrajectoryVelocity
2.1 findNearestIndexFromEgo

### 离当前位置最近点的最大速度

～/closest_max_velocity