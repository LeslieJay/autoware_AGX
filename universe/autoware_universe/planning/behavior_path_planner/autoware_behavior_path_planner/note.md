## 查看各个组件的状态

### 查看已激活组件
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/internal_state --once --full-length 2>/dev/null > /tmp/internal_state_full.txt 2>&1; wc -c /tmp/internal_state_full.txt; python3 -c "
with open('/tmp/internal_state_full.txt','rb') as f:
    raw = f.read()
print(repr(raw[:3000]))
"
**注意**：要加 --full-length

### 查看障碍物信息
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/static_obstacle_avoidance --once 2>/dev/null > /tmp/avoidance_info.txt && grep -E "text:|ns:" /tmp/avoidance_info.txt | head -40

'''
  ns: avoidable_target_objects_cube
  text: ''
  ns: avoidable_target_objects_info_reason_reason
  text: NONE
  ns: avoidable_target_objects_envelope_polygon
  text: ''
  ns: stop_target
  text: ''
'''

ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array --once 2>/dev/null

'''
avoidance_info:
- object_id: 34214b5d66b3823720d280f8943059c9
  allow_avoidance: false
  text: 
  longitudinal_distance: 6.032962607889342
  lateral_distance_from_centerline: 0.0
  to_furthest_linestring_distance: 0.0
  max_shift_length: 0.0
  required_jerk: 0.0
  maximum_jerk: 0.0
  failed_reason: ''
---
'''


ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/static_obstacle_avoidance --once 2>/dev/null > /tmp/avd.txt && wc -l /tmp/avd.txt && grep "ns:\|text:" /tmp/avd.txt

'''
  text: 'ratio:0.00 [-]

    lateral:0.00 [m]

    clip:true [-]

    necessity:true [-]

    stoppable:true [-]

    stop_factor:inf [m]

    move_time:0.00 [s]

    stop_...'
  ns: avoidable_target_objects_info
'''

**关键信息**: 

- `avoidable_target_objects_info`: 障碍物被分类为 avoidable

- `necessity`: 系统判定是否需要绕障 `check_necessity()`

- UNKNOWN 类型（非车辆类型）的关键判定在 `isSatisfiedWithNonVehicleCondition()`

```c++
if (std::abs(object.to_centerline) < parameters->threshold_distance_object_is_on_center) {
    object.info = ObjectInfo::TOO_NEAR_TO_CENTERLINE;
    return false;  // 直接忽略！
}
```
### 查看 obstacle_stop 是否把车速设置为 0 

ros2 topic echo /planning/planning_factors/obstacle_stop --once 2>/dev/null | head -30

### 诊断总结
根本原因找到了！ 整个调试链路如下：

问题链路（5 层）
Layer 1 ✅ 障碍物未被分类为 parked vehicle → 已修复 th_offset_from_centerline=0.0, th_shiftable_ratio=0.0
Layer 2 ✅ necessity=false（UNKNOWN 类型 hard_margin=-0.2 太小）→ 已修复 hard_margin=1.5
Layer 3 ✅ allow_avoidance: false → 发现是死代码，不影响实际功能
Layer 4 ✅ Avoidance 模块确已生效 (RUNNING)，shift lines 已注册，behavior_path 输出 v=8.33m/s
Layer 5 (真正的根因) 🎯 obstacle_stop 模块在 motion_velocity_planner 中仍然检测到碰撞并插入停止点
真正的根因


### 了解 ratio:0.00 和 lateral:0.00 意味着什么以及为什么 allow_avoidance: false

## 检查map中是否有相邻车道
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/lane_change_left --once 2>/dev/null > /tmp/lc_debug.txt && grep "text:" /tmp/lc_debug.txt | grep -v "text: ''" | head -1

## 查看 avoidance_by_lane_change 的info信息
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/avoidance_by_lane_change --once 2>/dev/null > /tmp/ablc_info.txt && wc -l /tmp/ablc_info.txt && cat /tmp/ablc_info.txt | head -20

## 诊断数据总结

| 指标 | 当前值 | 含义 |
|---|---|---|
| allow_avoidance | false | 障碍物不被允许绕行 |
| lateral_distance_from_centerline | 0.0 m | 障碍物正好在车道中心线上 |
| shiftable_ratio | 0.00 | 障碍物没有偏向路边 |
| longitudinal_distance | 6.04 m | 障碍物在前方约6m |
| necessity | false | 系统判定不需要绕障 |
| stoppable | false | 已来不及刹停 |
| failed_reason | 空 | 没有显式的失败原因 |
| 已激活模块 | static_obstacle_avoidance(RUNNING) | 仅绕障模块在运行 |


## 原因分析

核心原因：障碍物在车道中心线上（lateral: 0.00），不满足"停靠车辆"判定条件

Autoware 的避障逻辑要求障碍物必须被分类为**停靠车辆（parked vehicle）**才能触发绕障。判定条件如下：

shiftable_ratio > th_shiftable_ratio(0.1) — 当前值 0.00 < 0.1 ❌
|to_centerline| >= th_offset_from_centerline(0.3m) — 当前值 0.0 < 0.3 ❌
两个条件都不满足，因此障碍物不被认为是停靠车辆，不触发避障。

通俗理解：系统认为一个停在车道正中央的障碍物不是"停靠"的车辆（停靠应该偏向路边），而是一个"正常行驶中"或"模糊意图"的车辆，默认行为是等待它离开。

为什么 avoidance_by_lane_change 也没有触发？
avoidance_by_lane_change 模块的 info 为空 (markers: [])，因为它和 static_obstacle_avoidance 共享相同的障碍物过滤逻辑。障碍物没有通过过滤，换道绕障也不会被触发

## 解决方案

### 降低停靠车辆判定门槛

1. 将centerline偏移阈值改为0，使得中心线上的障碍物也能被判定为停靠
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.parked_vehicle.th_offset_from_centerline 0.0

2. 将shiftable_ratio阈值改为0，任何位置的障碍物都能被判为停靠
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.parked_vehicle.th_shiftable_ratio 0.0

### 让 ambiguous vehicle 策略也能覆盖这种情况

已经设置了 policy: "auto"，但障碍物可能还没满足停止时间要求，降低ambiguous vehicle的停止时间要求

ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.avoidance_for_ambiguous_vehicle.condition.th_stopped_time 0.5

### 两个都设置

ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.parked_vehicle.th_offset_from_centerline 0.0
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.parked_vehicle.th_shiftable_ratio 0.0
ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.avoidance_for_ambiguous_vehicle.condition.th_stopped_time 0.5