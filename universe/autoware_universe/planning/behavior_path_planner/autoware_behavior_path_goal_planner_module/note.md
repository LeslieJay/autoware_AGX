### 关键信息

1. 终点不允许修改（哪怕允许修改到终点附件，工程上最好也别用），所以必须使用 fixed_goal_planner

2. fixed_goal_planner 的触发条件：
- Route is set with `allow_goal_modification=false`. This is the default.
- The goal is set on `road` lanes.

3. fixed_goal_planner 的特点
没有"goal search", "collision check", "safety check", etc等功能.

### 整体流程

1. 初始化 goal_searcher
2. 生成 goal candidates
3. 判断是否触发规划线程
4. 如果模块未激活 → 退出
5. 重置路径
6. 安全检测（静态+动态障碍物）
7. 状态机更新（核心）
8. 同步子线程（lane / freespace）
9. 更新 context_data（核心数据）
10. 决策完成 → 处理速度
11. 如果已激活 → 推进路径执行

# 仿真结果

## no path candidate

1. 现象：接近终点时，显示 no path candidate 停止状态
![no path candidate](./images/no_path_candidate.png)

"no path candidate" 消息产生于 goal_planner_module.cpp:1554，条件是：

goal_candidates_ 不为空（已找到有效的 goal 候选点）
但 pull_over_path_candidates 为空（所有规划器都无法为这些 goal 候选点生成有效路径）
这意味着 三个路径规划器（SHIFT、ARC_FORWARD、ARC_BACKWARD）对所有 goal candidate 全部规划失败。

## 常见原因及排查方法


1. 没有 shoulder lane（路肩车道）— 最常见原因
getPullOverLanes() 会查找 goal 附近的 shoulder lane。如果地图中 goal 位置附近没有 shoulder lane，返回的是最外侧的 route lane。此时 goal planner 试图在行车道内做 pull over，shift 距离为 0 或非常小，路径规划很容易因为 lane departure check 失败而返回空。

排查：在 RViz 中检查 goal 附近是否有 shoulder lanelet。

2. parking_policy 方向不匹配
当前配置为 parking_policy: "right_side"。如果你的地图中 goal 靠左侧或 shoulder lane 在左侧，需要改为 "left_side"。

3. goal 搜索范围太小
当前配置：
forward_goal_search_length: 15.0   # 向前搜索 15m
backward_goal_search_length: 5.0   # 向后搜索 5m

如果车辆接近 goal 时距离较近，可能搜索范围不够。

4. Shift planner 的 lane departure check 过于严格
Shift planner 在 shift_pull_over.cpp:282 中会检查生成的路径是否超出 departure_check_lane。如果 goal 在道路边缘或 lane 较窄，shifted path 会因为超出车道边界而被拒绝。

5. margin_from_boundary 太大
margin_from_boundary: 0.75 意味着 goal 候选点必须距离车道边界至少 0.75m。对于窄道，这个值可能太大。

6. collision_check_soft_margins 太大

collision_check_soft_margins: [5.0, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0]

最大 5m 的碰撞检测余量，如果 goal 附近有任何障碍物，几乎所有路径都会被判定不安全。

推荐的实时调试命令
# 1. 查看 goal planner 的 debug 信息
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/goal_planner --once 2>/dev/null | head -100

# 2. 检查是否找到了 goal candidates
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/goal_planner --once 2>/dev/null | head -50

# 3. 查看 pull over path candidate 的数量
ros2 topic echo /planning/path_candidate/goal_planner --once 2>/dev/null | head -10

# 4. 检查当前参数
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.forward_goal_search_length
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.margin_from_boundary
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.pull_over.shift_parking.enable_shift_parking

推荐的参数调整（即时生效）
# 方案A: 增大搜索范围
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.forward_goal_search_length 30.0
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.backward_goal_search_length 15.0

# 方案B: 减小边界余量（适用于窄道）
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.margin_from_boundary 0.3

# 方案C: 放宽碰撞检测margins
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.object_recognition.collision_check_soft_margins [2.0,1.5,1.0,0.8,0.6]

# 方案D: 放宽lane departure检测
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.pull_over.lane_departure_check_expansion_margin 0.5

# 方案E: 增加lateral搜索范围
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner goal_planner.goal_search.max_lateral_offset 2.0


如果是地图问题（最根本的解决方案）
如果你的 goal 设置在没有 shoulder lane 的普通车道终点，goal planner 的 pull over 逻辑本身就不太适用。这种情况下有两个方向：

1. 在地图中为 goal 附近添加 shoulder lanelet，让 pull over 有合法的目标车道

2. 考虑 goal 是否应该设在车道中心线上，而不是需要 pull over 的位置