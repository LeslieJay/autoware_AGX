# 静态绕障模块测试排障指南

本文档说明 `autoware_behavior_path_static_obstacle_avoidance_module` 新增调试日志的用法，以及常见现象的定位步骤。

## 开启 DEBUG 日志

在 launch 时传入 `log_level:=debug`（作用于 `behavior_planning_container`，包含 `behavior_path_planner` 与静态绕障模块）：

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/num_3-num_8-map/ \
  vehicle_model:=byd_vehicle \
  sensor_model:=byd_sensor_kit \
  log_level:=debug
```

可选级别：`debug` / `info` / `warn` / `error` / `fatal`（默认 `info`）。

模块日志命名空间：

```text
planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner.static_obstacle_avoidance
```

说明：本次新增的 `[AVOIDANCE_*]` 排障日志大多为 **WARN** 节流输出，默认 `info` 级别即可在控制台看到，不一定需要 `debug`。

排障时优先 grep 以下前缀（均为 `WARN` 节流 1s，便于控制台直接查看）：

```bash
grep -E 'AVOIDANCE_(DEBUG|TARGET|FILTER|OUTLINE|PREPROCESS|CANDIDATE|RETURN|NEW_SL|SAFETY|EGO|GENERATOR)' your.log
```

## 日志字段速查

| 前缀 | 含义 |
|------|------|
| `[AVOIDANCE_FILTER]` | 目标过滤结果：`targets` 进入绕障候选，`others` 被过滤 |
| `[AVOIDANCE_TARGET]` | 单个目标摘要：纵向/横向距离、是否需绕障、是否可绕、过滤原因 `info` |
| `[AVOIDANCE_OUTLINE]` | 为何未生成避让 outline，或成功生成（含 return=1） |
| `[AVOIDANCE_PREPROCESS]` | outline → raw shift line 各阶段数量 |
| `[AVOIDANCE_CANDIDATE]` | merge/trim 后候选 shift line 数量 |
| `[AVOIDANCE_RETURN]` | 回正线添加/跳过原因 |
| `[AVOIDANCE_NEW_SL]` | 最终提取的新 shift line（RTC 待审批） |
| `[AVOIDANCE_DEBUG]` | 总览：`safe/comfortable/valid/ready/request_op` 及路径规模 |
| `[AVOIDANCE_SAFETY]` | 安全检查：左右偏移、碰撞对象、 hysteresis |
| `[AVOIDANCE_EGO]` | 状态机决策：`reason` 字段说明当前分支 |
| `[AVOIDANCE_GENERATOR]` | `update()` 完成后的 outline/raw 数量 |

### `[AVOIDANCE_DEBUG]` 扩展字段

- `targets`：当前 target_objects 数量
- `found_path`：是否存在新 shift 或已注册 shift
- `candidate_pts`：候选路径点数
- `registered_sl`：path_shifter 中已注册 shift 数
- `ego_shift`：当前 ego 横向偏移

### `[AVOIDANCE_EGO] reason` 取值

| reason | 说明 |
|--------|------|
| `safe_path` | 安全检查通过，正常执行 |
| `yield_disabled` | yield 关闭，unsafe 也继续 |
| `cannot_yield` | 已在绕障中，无法切 yield |
| `force_deactivated` | RTC 强制停用 |
| `unsafe_yield_maneuver` | 不安全，切 yield/停车 |
| `unsafe_cancel_approved` | 取消已批准路径 |
| `output_path_locked` | 外部锁定输出路径 |

## 按现象排查

### 1. 无目标（`targets=0`）

1. 看 `[AVOIDANCE_FILTER] others` 是否很大
2. 看 `[AVOIDANCE_TARGET] info=` 常见值：
   - `TOO_NEAR_TO_GOAL`：距 goal 太近 → 调 `object_check_goal_distance`
   - `FURTHER_THAN_THRESHOLD`：超出检测距离 → 调 `max_forward_distance`
   - `TOO_NEAR_TO_CENTERLINE` / `ENOUGH_LATERAL_DISTANCE`：横向无需绕障
   - `MOVING_OBJECT` / `UNSTABLE_OBJECT`：动态或未稳定
3. RViz 对照：`/planning/.../debug/static_obstacle_avoidance` marker

### 2. 有目标但 `new_sl_size=0`

按顺序检查：

1. `[AVOIDANCE_OUTLINE]`：`infeasible_shift_profile` / `same_direction_shift` / `no_avoid_margin`
2. `[AVOIDANCE_PREPROCESS]`：`out raw_lines=0` → outline 在 merge/combine 阶段被清掉
3. `[AVOIDANCE_CANDIDATE] after trim processed=0` → trim 去噪过强
4. `[AVOIDANCE_NEW_SL] no usable shift line` → 低于 `lateral_execution_threshold` 或 prepare 距离不足
5. `[AVOIDANCE_DEBUG] comfortable=0` → 横向加速度/jerk 超限，调 `lateral_accel_limit` / `lateral_jerk_limit`

### 3. 有 shift 但 `ready=0`

- 检查 `registered_sl`：可能 shift 尚未 RTC 批准
- `request_op=1`：需手动/自动 RTC 批准
- 对象 `is_avoidable=0` 且 `avoid_req=1`：可能处于 wait-and-see

### 4. `request_op=1`

```bash
# 查看 RTC 状态
ros2 topic echo /planning/.../rtc_status --once

# 自动批准脚本（模块目录下）
python3 rtc_auto_approve.py
```

ambiguous 车辆默认需审批：调低 `parked_vehicle.th_offset_from_centerline` 或改 `policy_ambiguous_vehicle`。

### 5. `safe=0`

看 `[AVOIDANCE_SAFETY] result=0`：

- `fail_id` / `class` / `front` / `oncoming`：哪类对象导致碰撞
- 若 `targets=0` 且 `left_shift=1`：相邻车道无检查对象但仍 unsafe → 查 hysteresis
- 临时验证：设 `enable_safety_check: false`（仅调试）

### 6. 有避让线但没有 return shift

看 `[AVOIDANCE_OUTLINE] skip_return_shift` 或 `[AVOIDANCE_RETURN] skip reason=`：

- `object_near_goal` / `last_shift_near_goal`
- `no_enough_distance`：`to_return_point` 不足
- `unavoidable_object`：前方存在不可绕对象

### 7. 车辆停在障碍物前不动

综合链路：

1. `[AVOIDANCE_EGO] reason=unsafe_yield_maneuver` → 安全否掉 + yield
2. `[AVOIDANCE_DEBUG] comfortable=0 valid=0` → 路径未生成或不舒服
3. 下游 `obstacle_stop` / `planning_factor` 有 STOP → 规划层停车，不一定是本模块 approved

## 常用命令

```bash
# 候选路径是否为空
ros2 topic echo /planning/path_candidate/static_obstacle_avoidance --once 2>/dev/null | head -5

# debug marker
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/static_obstacle_avoidance --once 2>/dev/null | grep -A 5 "text:"

# avoidance debug array
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/avoidance_debug_message_array --once 2>/dev/null | head -50

# 关键参数
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner \
  avoidance.target_filtering.object_check_goal_distance
```

## 仿真实例（comfortable=0）

日志示例：

```text
[AVOIDANCE_DEBUG] safe=1 comfortable=0 valid=1 ready=0 request_op=0 new_sl_size=1 targets=1 ...
[AVOIDANCE_RETURN] skip reason=no_enough_distance ...
```

解读：

- 目标已识别，shift line 已生成（`new_sl_size=1`）
- `comfortable=0`：减速度/横向 jerk 超限 → 优先调 `lateral_accel_limit`、`nominal_deceleration`
- return shift 因剩余距离不足被跳过 → 查 `to_return_point`、goal 距离参数

## 编译验证

```bash
cd /home/byd/autoware
source install/setup.bash  # 若已有 underlay
colcon build --packages-select autoware_behavior_path_static_obstacle_avoidance_module \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

启动 dummy 场景后，控制台应能按周期看到上述 `[AVOIDANCE_*]` 日志链，无需依赖 RViz 即可判断卡在哪一层。
