<!--
 * @Author: leslie leslie@byd.com
 * @Date: 2026-02-26 10:14:33
 * @LastEditors: leslie leslie@byd.com
 * @LastEditTime: 2026-03-23 15:24:28
 * @FilePath: /autoware/src/core/autoware_core/planning/autoware_velocity_smoother/note.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
1. param
/planning/scenario_planning/velocity_smoother:

normal.max_acc normal.max_jerk normal.min_acc max_velocity

/control/trajectory_follower/controller_node_exe

/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner 

/planning/scenario_planning/lane_driving/motion_planning/elastic_band_smoother/output/trajectory

关闭转向角速率限制
ros2 param set /planning/scenario_planning/velocity_smoother enable_steering_rate_limit false

### main function flowchart

1. 主函数 onCurrentTrajectory
1.1 接收里程计/加速度/外部速度限制/操作模式等数据 
1.2 将输入轨迹转为轨迹点队列 // convertToTrajectoryPointArray
1.3 计算当前位置在上一输出轨迹中的索引
1.4 计算外部速度限制并更新
1.5 计算轨迹点上的速度 // calcTrajectoryVelocity(input_points)
1.6 重采样轨迹点 // resampleTrajectory
1.7 发布轨迹点 // publishTrajectory(output_resampled)

2. calcTrajectoryVelocity
2.1 轨迹点中距离当前位置最近点的索引 //findNearestIndexFromEgo
2.2 在当前索引前后提取一定长度的路径 // extractPathAroundIndex
2.3 如果有停止点，则将距离停止点stopping_distance距离内的点所有速度设为stopping_velocity // applyStopApproachingVelocity
2.4 平滑速度 // smoothVelocity

3. smoothVelocity
3.1 根据当前车速、上一帧规划结果等，决定优化的初始速度/加速度 // calcInitialMotion(input, input_closest)
3.2 使用横向加速度滤波限制速度，在弯道处降低速度，防止横向加速度超限
3.3 使用转向角速率限制速度
3.4 重采样轨迹
3.5 设置优化器约束 （setMaxAccel，setMaxJerk）
3.6 优化求解+后处理

### 离当前位置最近点的最大速度

～/closest_max_velocity

### 横向加速度过滤限制速度  applyLateralAccelerationFilter

1. 设置轨迹点间隔，重采样轨迹点
2. 计算每个轨迹点曲率计算需要用到的索引距离
3. 根据索引距离计算曲率
4. 选择轨迹点i处前后距离内的曲率
5. 选择i点处一段距离内的最大曲率作为该点的速度限制曲率
6. 根据曲率和横向加速度限制速度

latacc_min_vel_arr 车辆动力学所允许的每个点最小的速度

### 转向速率限制  applySteeringRateLimit

1. 