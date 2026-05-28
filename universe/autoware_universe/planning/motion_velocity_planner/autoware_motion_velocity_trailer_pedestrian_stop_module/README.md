# autoware_motion_velocity_trailer_pedestrian_stop_module

面向室外牵引车-挂车列的停止规划模块：当行人从侧向接近时触发停车。

## 概述

本模块是 `motion_velocity_planner` 的插件。当行人的预测/扫掠区域与下列组合包络相交时，在轨迹上插入停车点：

- 牵引车（由 `vehicle_info` 车辆尺寸确定），以及
- 沿平滑轨迹向后串联的 `trailer.count` 节矩形挂车。

实际平滑减速与停车由下游的 velocity smoother 和控制器完成。

## 什么情况下会停车？

本模块在轨迹上插入 **停车点**（`stop_points`），车辆真正刹停由下游 velocity smoother 与纵向控制器执行。

### 处理流程

1. 对平滑后的轨迹重采样（`trajectory_step_length`，默认 0.5 m）。
2. 沿轨迹构建组合包络：牵引车 + `trailer.count` 节挂车 + `rear_margin`。
3. 筛选感知 `PredictedObjects`（见下文）。
4. 为每个剩余行人生成扫掠区域。
5. 若扫掠区域与任一列车包络相交 → 记录碰撞点。
6. 经防抖确认后，在最早碰撞点前方插入停车点。

若轨迹点数少于 2，模块直接返回，不输出停车。

### 行人筛选条件

`PredictedObject` 需 **同时满足** 以下全部条件才会被考虑：

| 检查项 | 默认行为 |
|--------|----------|
| 目标类别 | 启用 `PEDESTRIAN`；默认关闭 `BICYCLE` / `UNKNOWN`（`enable_*`） |
| 横向范围 | 到规划轨迹的横向距离 ≤ `max_lateral_range` + 目标半径。范围含牵引车半宽、挂车半宽 + `side_margin`、`train_length * 0.1`、`max_lateral_approach_distance`（默认 6.0 m） |
| 不在列车后方过远 | 轨迹纵向弧长 ≥ `-(train_rear_length + object_radius/2)`。明显在最后一节挂车后方的目标忽略；列车侧面并行允许 |
| 横向接近 | `require_lateral_approach` 为 true 时，相对轨迹的横向速度须 ≥ `min_approach_speed`（默认 0.1 m/s）。默认 **false**（静止行人也会参与） |
| 可避免碰撞 | `ignore_unavoidable_collisions` 为 **false**（默认）时，忽略已在自车后方、或正横向远离列车的行人（横向偏移 × 横向速度 > 0 且 \|横向速度\| ≥ `min_approach_speed`） |

若已有目标正在触发停车，`hysteresis`（默认 1.0 m）会扩大横向范围与行人扫掠半径，减少抖动。

### 碰撞几何

**自车（列车）包络** — 在每个重采样轨迹位姿上：

- 由 `vehicle_info` 得到的牵引车矩形，两侧各加 `side_margin`。
- `trailer.count` 节矩形挂车沿轨迹向后串联（`length`、`width`、`hitch_gap`）。
- 最后一节挂车后再加 `rear_margin` 多边形。

挂车航向跟随轨迹（无铰接角反馈）。

**行人扫掠区域**：

- 若 `use_predicted_path` 为 true，且置信度最高的预测路径 confidence ≥ `min_predicted_path_confidence`：沿预测路径各点放置圆（半径 = 目标半尺寸 + `radius_margin` + hysteresis），再求并集。
- 否则：在当前位姿与 `位姿 + 速度 × time_horizon`（默认 4.0 s）处各放一个圆并求并集。低速时用当前朝向作为终点方向。

**碰撞判定** — 行人扫掠多边形与未来轨迹上任一列车包络多边形相交。每个目标保留沿轨迹弧长距自车 **最近** 的碰撞点。

### 停车决策防抖

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `add_stop_duration_buffer` | 0.2 s | 碰撞须持续此时长才下发停车 |
| `remove_stop_duration_buffer` | 1.0 s | 碰撞消失后须等待此时长才从停车映射中移除该目标 |
| `hysteresis` | 1.0 m | 已有停车生效时，额外扩大横向/扫掠裕度 |

在通过防抖（`should_be_avoided`）的多个目标中，取沿轨迹 **最早** 的碰撞点。

### 停车点位置

在得到有效碰撞点后：

1. 根据当前速度、加速度和 velocity smoother 的 jerk/减速度限制，计算最短制动距离 `min_stop_distance`。
2. 若能在碰撞前停下（`arc_length_to_collision < min_stop_distance - 车头纵向偏移 - stop_distance_buffer`）：
   - 停车位姿 = 沿轨迹从碰撞点向后退 `stop_distance_buffer`（默认 1.0 m）+ 车头纵向偏移。
3. 否则：
   - 停车位姿 = 当前动力学下最早可停位姿（`earliest_stop_pose`）。

### 典型会停车场景（默认参数）

- 行人从列车侧面横穿，预测路径或 4 s 扫掠区与牵引车 + 5 节挂车包络相交。
- 行人站在轨迹横向约 6 m 内，其扫掠区与未来路径上的列车包络相交。
- 威胁存在时通过 hysteresis 保持停车；碰撞几何消失约 1 s 后解除。

### 典型不会停车场景

| 场景 | 原因 |
|------|------|
| 自行车 / 未知类别 | 默认未启用 |
| 行人在最后一节挂车后方较远 | `is_not_too_close` |
| 行人超出横向范围（约 6 m + 裕度） | `is_in_range` |
| 正横向远离列车 | `is_unavoidable`（`ignore_unavoidable_collisions` 为 false 时） |
| 已在自车后方（相对轨迹弧长 < 0） | `is_unavoidable` |
| 碰撞仅持续不足 0.2 s | `add_stop_duration_buffer` |
| 轨迹点少于 2 个 | 提前返回 |
| 急弯 / 倒车 | 挂车几何仅按轨迹近似，精度不足（见限制） |

## 参数

详见 `config/trailer_pedestrian_stop.param.yaml`。

| 分组 | 键 | 说明 |
|------|-----|------|
| trailer | count, length, width, hitch_gap | 挂车列几何 |
| trailer | side_margin, rear_margin | 横向/尾部安全裕度 |
| pedestrian | enable_* | 目标类别开关 |
| pedestrian | time_horizon, radius_margin | 无预测时的扫掠区域 |
| stop | stop_distance_buffer | 碰撞点前的停车距离 |
| stop | add/remove_stop_duration_buffer | 触发/解除停车的防抖时间 |
| stop | ignore_unavoidable_collisions | 是否忽略已在后方或横向远离的行人 |
| sampling | trajectory_step_length | 沿轨迹的包络采样步长 |

## 调试话题

- `~/trailer_pedestrian_stop/debug_markers` — 列车包络（蓝）、行人扫掠区（红）、碰撞标记
- `~/trailer_pedestrian_stop/virtual_walls` — 停车虚拟墙
- `~/debug/trailer_pedestrian_stop/processing_time_ms` — 处理耗时

**说明：** 本模块**没有**单独的挂车 3D 模型。挂车是沿规划轨迹采样的**蓝色线框**（`train_trailer_footprints`），牵引车+各节挂车合并为一个多边形轮廓。

### RViz 显示

1. 左侧面板展开：**Planning → Debug → MotionVelocityPlanner**
2. 勾选 **TrailerPedestrianStop**（话题 `.../trailer_pedestrian_stop/debug_markers`）
3. 在 Namespaces 中确认勾选 `train_trailer_footprints`（蓝线）、`pedestrian_footprints`（红线）
4. 线框沿**整条未来轨迹**绘制（约每 0.5 m 一个），需沿路径缩放查看，不是只画在当前车位
5. 使用 `byd_planning_sim.rviz` 或更新后的 `autoware.rviz` 已预置上述显示项

若仍看不见：确认 **Fixed Frame = map**，并检查父级 **Planning / Debug** 分组已启用。

## 启动

在 `autoware_launch` 规划 preset 中通过 `launch_trailer_pedestrian_stop_module` 启用（默认：`true`）。

参数文件：`autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trailer_pedestrian_stop.param.yaml`

## 限制

- 挂车位姿由牵引车轨迹近似（无铰接角反馈）。
- 面向低速前进工况；急弯与倒车时模型不准确。