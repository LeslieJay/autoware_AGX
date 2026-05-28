<!--
 * @Author: leslie
 * @Date: 2026-02-08 16:27:56
 * @LastEditors: leslie
 * @LastEditTime: 2026-04-21 20:21:54
 * @FilePath: /autoware/src/byd/reverse_parking_planner/note.md
 * @Description:  
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
-->
# 通过服务设置目标位姿并立即触发规划
ros2 service call /reverse_parking_planner/set_goal_pose \
  reverse_parking_planner/srv/SetGoalPose \
  "{goal_pose: {header: {frame_id: 'map'}, pose: {position: {x: 57257.78125, y: 59082.53125, z: 89.7237}, orientation: {x: 0.0, y: 0.0, z: -0.6680717263004464, w: 0.7440968811370878}}}}"

# 使用已有目标重新规划
ros2 service call /reverse_parking_planner/trigger_planning std_srvs/srv/Trigger

    x: 57257.78125
    y: 59082.53125
    z: 89.7237
  orientation:
    x: -0.0013419518860451885
    y: -0.00032467946726269064
    z: -0.9719555816159812
    w: 0.2351604582278212
