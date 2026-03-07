# 换道绕障模块未激活 - 故障排查指南

## 问题现象
车辆检测到障碍物但未激活换道绕障模块，直接穿过障碍物。

## 可能原因及排查方法

### ✅ 排查清单

#### 1. 检查是否存在可换道车道
**现象**: `LC_required=0` 在日志中
**原因**:
- 当前车道旁没有相邻车道
- 相邻车道禁止换道（实线、路缘石等）
- 地图数据问题

**排查**:
```bash
# 查看日志
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/avoidance_by_lane_change

# 检查RViz中的lanelet可视化
```

#### 2. 检查障碍物检测
**现象**: 日志显示 `No target objects detected` 或 `No avoidance target found`

**可能原因**:
- 障碍物类型未启用
- 障碍物不在自车车道内  
- 障碍物距离中心线太近
- 障碍物被判定为移动物体

**排查**:
```yaml
# 检查配置文件: config/avoidance_by_lane_change.param.yaml
avoidance_by_lane_change:
  target_object:
    target_filtering:
      target_type:
        car: true      # 确保目标类型已启用
        truck: true
        bus: true
```

**调整**:
- 降低 `th_moving_speed` (默认1.0 m/s) - 提高静态物体判定阈值
- 调整 `threshold_distance_object_is_on_center` - 减小中心线距离阈值

#### 3. 检查距离是否充足 ⚠️ **最常见原因**
**现象**: 日志显示 `Insufficient distance (obj_dist=X < required=Y)`

**原因**:
- 障碍物距离 < 最小换道距离
- 障碍物距离 < 最小绕障距离
- 车速过高导致所需距离增加

**计算公式**:
```
required_distance = max(minimum_lane_change_length, minimum_avoid_length)

- minimum_lane_change_length: 取决于车速、加速度、换道参数
- minimum_avoid_length: 取决于横向偏移量、最大横向加速度
```

**解决方案**:
```yaml
# 方案1: 降低距离要求（不推荐，可能不安全）
avoidance_by_lane_change:
  execute_object_longitudinal_margin: 50.0  # 默认80.0

# 方案2: 提高换道速度参数（需谨慎调整）
lane_change:
  lane_change_prepare_duration: 2.0  # 减小准备时间
  min_lane_changing_velocity: 5.0    # 提高最小换道速度
```

#### 4. 检查路径有效性
**现象**: `valid_path=0` 在日志中

**原因**:
- 换道路径生成失败
- 碰撞检测未通过
- 目标车道空间不足

**排查**:
```bash
# 查看换道模块的详细日志
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path

# 检查碰撞检测参数
```

#### 5. 检查模块注册和优先级
**原因**:
- 换道绕障模块未加载
- 其他模块（如普通绕障）优先级更高

**排查**:
```bash
# 检查模块是否加载
ros2 param list | grep avoidance_by_lane_change

# 查看behavior_path_planner的模块列表
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/modules_info
```

#### 6. 检查障碍物筛选条件
**原因**: 障碍物被以下条件过滤
- 距离goal太近 (`object_check_goal_distance`)
- 目标物体消失时间超过阈值 (`max_compensation_time`)
- 不在可绕障区域内 (`th_shiftable_ratio`)

**调整**:
```yaml
avoidance:
  target_filtering:
    object_check_goal_distance: 20.0        # 增大此值
    max_compensation_time: 2.0              # 延长补偿时间
    parked_vehicle:
      th_shiftable_ratio: 0.6               # 降低可绕障比例要求
```

## 调试日志说明

已在代码中添加详细日志输出：

### 正常激活日志
```
[INFO] [AvoidByLC] Module ACTIVATED: targets=2, obj_dist=95.50 > required=85.30
```

### 未激活日志示例

#### 情况1: 无目标物体
```
[WARN] [AvoidByLC] Module NOT activated: No target objects detected
```

#### 情况2: 无需绕障目标
```
[WARN] [AvoidByLC] Module NOT activated: No avoidance target found (total_objects=3, avoidance_required=0)
```

#### 情况3: 距离不足
```
[WARN] [AvoidByLC] Module NOT activated: Insufficient distance (obj_dist=45.20 < required=85.30 [LC=85.30, Avoid=62.15])
```

#### 情况4: 换道条件不满足
```
[DEBUG] [AvoidByLC] Execution NOT requested: LC_required=0, special_check=1, valid_path=1
```

## 快速诊断命令

```bash
# 1. 实时查看模块状态
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/avoidance_by_lane_change

# 2. 查看检测到的障碍物
ros2 topic echo /perception/object_recognition/objects

# 3. 查看当前激活的行为模块
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/current_module

# 4. 重启规划器（如果修改了参数）
ros2 lifecycle set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner configure
ros2 lifecycle set /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner activate
```

## 参数调优建议

### 提高模块激活率（谨慎使用）
```yaml
avoidance_by_lane_change:
  execute_object_longitudinal_margin: 50.0  # 降低距离要求
  execute_only_when_lane_change_finish_before_object: false  # 允许换道中经过障碍物
  
  target_object:
    car:
      th_moving_speed: 2.0  # 提高静止判定阈值
      lateral_margin:
        hard_margin: 0.3    # 减小安全余量（不推荐）
```

### 保守安全配置（推荐）
```yaml
avoidance_by_lane_change:
  execute_object_longitudinal_margin: 100.0  # 增加安全距离
  execute_only_when_lane_change_finish_before_object: true  # 必须在障碍物前完成换道
  
  target_object:
    car:
      lateral_margin:
        hard_margin: 0.5    # 增大安全余量
        soft_margin: 0.2
```

## 与普通绕障模块的关系

换道绕障模块和普通静态障碍物绕障模块可能同时存在，优先级由behavior_path_planner决定：
- 如果换道绕障条件不满足，会fallback到普通绕障（横向偏移）
- 如果两者都不满足，车辆可能会减速或停车

## 联系支持

如果以上方法都无法解决问题，请收集以下信息：
1. ROS日志文件
2. rosbag录制的场景数据
3. 参数配置文件
4. RViz截图显示地图和障碍物位置
