### 调试步骤

1. 检查路径是否存在

话题：ros2 topic echo /planning/path_candidate/static_obstacle_avoidance --once 2>/dev/null | head -5
可视化：

2. 查看障碍物的信息
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/static_obstacle_avoidance --once 2>/dev/null | head -100
Type: visualization_msgs/msg/MarkerArray

ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array --once 2>/dev/null | head -200
Type: tier4_planning_msgs/msg/AvoidanceDebugMsgArray
tier4_planning_msgs/AvoidanceDebugMsg[] avoidance_info

ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/static_obstacle_avoidance --once 2>/dev/null | grep -A 20 "text:"
Type: visualization_msgs/msg/MarkerArray

3. 由于不满足th_offset_from_centerline参数要求，判定为ambiguous_vehicle可疑车辆，wait and see而可疑车辆的规则是需要手动启动避障，所以停车等待手动批准，因此修改参数配置，减小判定阈值，更容易判断为可执行避障的物体，且自动通过审批

ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.parked_vehicle.th_offset_from_centerline 2>/dev/null

ros2 param set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner avoidance.target_filtering.avoidance_for_ambiguous_vehicle.policy "auto"

ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array --once 2>/dev/null

ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/static_obstacle_avoidance --once 2>/dev/null | grep -E "ratio:|lateral:|clip:|stoppable:|parked|is_on"


### 判断障碍物为 parked_vehicle 的条件

```c++
...
    bool isParkedVehicle(
  const ObjectData & object, const std::shared_ptr<AvoidanceParameters> & parameters)
```


1. 不在交叉路口
2. shiftable_ratio 大于阈值
3. 距离中心线足够远

### wait and see 功能说明

**满足3个条件才会触发**

1. 障碍物必须是模糊车辆，模糊车辆的判断条件

停止时间 > th_stopped_time (配置为 3.0秒)
移动距离 < th_moving_distance (配置为 1.0米)
不在忽略区域内（交通灯前20米、人行横道前20米）

2. 障碍物行为属于配置的目标行为

wait_and_see:
  target_behaviors: ["MERGING", "DEVIATING"]

```c++
...
    ObjectData::Behavior getObjectBehavior(
  const ObjectData & object, const std::shared_ptr<AvoidanceParameters> & parameters)
```
返回车辆行为 none/merging/deviating

3. 距离条件判断

```c++
...
    const bool enough_distance =
  object.longitudinal < prepare_distance + constant_distance + avoidance_distance +
                        parameters_->wait_and_see_th_closest_distance;
```

只有当障碍物距离足够远，才会触发 wait and see

**结果**

1. 延迟开始绕障，插入wait点，开始等待

**配置参数位置**
<static_obstacle_avoidance.param.yaml>
wait_and_see:
  target_behaviors: ["MERGING", "DEVIATING"]  # 要等待观察的行为类型
  th_closest_distance: 10.0                    # 最近距离阈值[m]


### RTC(request to cooperate)

**触发条件**

data.request_operator == true

**rtc_interface.hpp**

1. RTC 接口使用的命名空间
std::string cooperate_status_namespace_ = "/planning/cooperate_status";
std::string cooperate_commands_namespace_ = "/planning/cooperate_commands";
std::string enable_auto_mode_namespace_ = "/planning/enable_auto_mode";

2. 状态发布: 
/planning/cooperate_status/static_obstacle_avoidance_left 和 /planning/cooperate_status/
static_obstacle_avoidance_right

3. 命令服务: 
/planning/cooperate_commands/static_obstacle_avoidance_left 和 /planning/cooperate_commands/static_obstacle_avoidance_right

# 查看左侧绕避状态
ros2 topic echo /planning/cooperate_status/static_obstacle_avoidance_left

# 查看右侧绕避状态
ros2 topic echo /planning/cooperate_status/static_obstacle_avoidance_right

# 批准左侧绕避（ACTIVATE命令）
ros2 service call /planning/cooperate_commands/static_obstacle_avoidance_left \
  tier4_rtc_msgs/srv/CooperateCommands \
  "{
    stamp: {sec: 0, nanosec: 0},
    commands: [
      {
        module: {type: 10},  // module: {type: 11} 批准右侧绕避
        command: {type: 1},
        uuid: {uuid: [你的UUID]}
      }
    ]
  }"

# 实时查看UUID
ros2 topic echo /planning/cooperate_status/static_obstacle_avoidance_left

# 输出示例：
# statuses:
# - uuid: {uuid: [12, 34, 56, 78, ...]}  # 这就是UUID
#   requested: true
#   safe: true
#   ...


### 查看有那些组件被激活
ros2 topic echo /planning/.../behavior_path_planner/debug/module_status

### 算法流程

1. 更新数据 getPreviousModuleOutput().reference_path， PreviousSplineShiftPath， PreviousLinearShiftPath // updateData
1.1 更新基础路径和目标障碍物 // fillFundamentalData
1.1.1 获取自车参考位姿和车道信息
1.1.2 根据是否存在红灯信号扩展可行驶车道
1.1.3 计算可行驶区域的边界 （data.drivable_lanes_same_direction/data.drivable_lanes）
1.1.4 重采样参考路径

1.1.5 过滤最新检测到的障碍物  // fillAvoidanceTargetObjects
1.1.5.1 基于障碍物位于车道线内/外，划分为线内/外障碍物
1.1.5.2 从线内障碍物集合过滤得到目标障碍物  // filterTargetObjects

1.1.6 补偿感知丢失的障碍物 // compensateLostTargetObjects