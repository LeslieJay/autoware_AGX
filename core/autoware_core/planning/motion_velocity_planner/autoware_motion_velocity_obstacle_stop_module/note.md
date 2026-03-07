###  主函数逻辑 plan()

1. initial variables (x_offset_to_bump)
2. preprocess 轨迹离散化
3. 可疑障碍物过滤(感知)
4. 可疑障碍物过滤(点云)
5. 合并可疑障碍物，判断是否需要stop // plan_stop() 

### 可疑障碍物过滤（感知）filter_stop_obstacle_for_predicted_object()

1. 粗过滤
1.1 检查障碍物是否在车辆前方
1.2 检查纵向距离是否小于阈值

2. 精过滤
2.1 从感知的物体中筛选目标障碍物 // pick_stop_obstacle_from_predicted_object
2.1.1 通过label过滤(check_inside==true)
2.1.2 检查障碍物多边形和轨迹多边形的横向间距

2.1.3 检查是否会真的发生碰撞
2.1.3.1 获取检测区域多边形
2.1.3.2 检查处于当前轨迹内的障碍物是否与检测区域 detection_polygon 相交
2.1.3.3 检查处于当前轨迹外侧的障碍物是否会切入检测区域 detection_polygon

2.1.4 过滤马上会穿过轨迹远离的障碍物
2.1.5 检查该障碍物是否要固定停止
2.1.6 如果使用rss的话，使用rss stop来触发停车

2.2 检查持续性: 障碍物是否消失

### 可疑障碍物过滤（点云）filter_stop_obstacle_for_point_cloud()

1. 获取带横向裕度的检测区域多边形
2. 获取点云与多边形的最近碰撞点
3. 根据最新的点云时间来更新 pointcloud_stop_candidates
4. 从 pointcloud_stop_candidates 中，过滤障碍物
4.1 速度估计 stop_candidate.vel_lpf.getValue() 通过两帧点云之间的距离除以时间得到低通滤波的速度估计
4.2 时间补偿 （考虑动态物体）
4.3 静态障碍物选 normal stop
    动态障碍物选 rss stop
4.4 rss stop == reaction_distance + braking distance


### 根据障碍物规划停止点  plan_stop()

1. 没有障碍物，直接返回空
2. 获取最近的停止障碍物集合 // get_closest_stop_obstacles
2.1 类型比较
2.2 距离比较 (dist_to_collide + braking_dist)

3. 计算碰撞距离（在参考轨迹上）
** 之前计算的距离都是基于裁减轨迹（从自车位置到障碍物）
碰撞距离 = 从参考轨迹起点到自车的距离 + 从自车（剪裁轨迹起点）到障碍物的距离 + (可选) 障碍物的制动距离或其他修正

4. 计算期望停止边距 //calc_desired_stop_margin

5. 计算候选零速度距离 // calc_candidate_zero_vel_dist
5.1 如果抑制 急停(suppress_sudden_stop_=true) 
5.1.1 选择合适的加速度
5.1.1.1 计算突然性判断的距离
5.1.1.2 取突然性判断的距离和阈值中的较小值
5.1.2 如果 stop_planning_param_.get_param(stop_obstacle.classification).abandon_to_stop为true，
放弃停止，改为巡航
5.1.3 存在可接受的停止加速度，根据加速度计算可接受的停止位置

5.2 不抑制 直接返回 dist_to_collide_on_ref_traj - desired_stop_margin

6. 如果已有确定的停止障碍物，比较并决定是否替换
6.1 检查是否应该跳过当前障碍物（基于距离和类型）

7. 计算停止点 calc_stop_point
7.1 计算停止点插入位置的索引
7.2 创建虚拟墙标记用于调试可视化
7.3 更新规划因子，向外部报告停车原因(type, position, is_safe)
7.4 添加规划因子接口，包含轨迹，当前位置等
7.5 记录当前的停止信息，用于下一帧的“保持停止”（hold stop）逻辑判断

8. 根据情况，选择返回 stop_point or buffered_stop

stop_point 是当前帧根据当前时刻感知到的障碍物和自车状态，实时计算出的理想停车位置；不稳定，直接使用可能会频繁急刹或者减速。
buffered_stop 是经过时间维度滤波/缓冲处理后的停车点建议。

9. 如果有buffered_stop，优先使用buffered_stop并设置调试信息