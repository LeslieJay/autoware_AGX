# AvoidanceByLaneChange 模块执行流程详解

## 1. 模块激活流程概览

```
┌─ isExecutionRequested() ─────────────────────────────────────┐
│  检查是否应该执行换道避障                                    │
│  1. 调用 specialRequiredCheck()                              │
│  2. 检查路线有效性                                          │
│  3. 返回执行请求结果                                        │
└─────────────┬──────────────────────────────────────────────┘
              │
        [YES/NO]
              │
    ┌─────────┴─────────┐
    │                   │
  [YES]               [NO]
    │                   │
    ▼                   ▼
 激活模块          取消执行
```

## 2. specialRequiredCheck() 详细流程

**作用**：检查模块是否满足激活条件

### 2.1 检查流程

| 步骤 | 检查条件 | 结果 | 日志 |
|------|---------|------|------|
| 1 | `data.target_objects.empty()` | ❌ 返回false | "[AvoidByLC] No target objects detected" |
| 2 | 计数避碰目标 `num_of_avoidance_targets` | 若为0 ❌ | "[AvoidByLC] No avoidance target found" |
| 3 | 获取最近对象 `nearest_object` | 继续检查 | - |
| 4 | 计算避碰所需距离 `minimum_avoid_length` | 使用 | calcMinAvoidanceLength() |
| 5 | 计算换道所需距离 `minimum_lane_change_length` | 使用 | calc_minimum_dist_buffer() |
| 6 | 比较纵向距离 `nearest_object.longitudinal > required_distance` | ✅ true/❌ false | 激活/未激活 |

### 2.2 关键参数检查

```cpp
// 激活条件 (全部满足才能激活)
✓ data.target_objects 不为空
✓ num_of_avoidance_targets >= 1
✓ nearest_object.longitudinal > max(minimum_avoid_length, minimum_lane_change_length)
```

## 3. updateSpecialData() 详细流程

**作用**：更新避碰数据和对象信息

### 3.1 执行步骤

```
┌─ updateSpecialData() ───────────────────────┐
│                                             │
├─> ① 清空分析数据                           │
│   avoidance_debug_data_ = DebugData()      │
│                                             │
├─> ② 计算避碰planning数据                   │
│   calcAvoidancePlanningData()              │
│   └─> fillAvoidanceTargetObjects()         │
│       └─> createObjectData() for each      │
│                                             │
├─> ③ 判断避碰方向                           │
│   if not empty: isOnRight()?               │
│   YES → Direction::LEFT                    │
│   NO  → Direction::RIGHT                   │
│                                             │
├─> ④ 补偿丢失对象                           │
│   compensateLostTargetObjects()            │
│                                             │
├─> ⑤ 更新已注册对象                         │
│   updateStoredObjects()                    │
│                                             │
└─> ⑥ 排序目标对象                           │
    std::sort(by longitudinal distance)     │
                                             │
└─────────────────────────────────────────────┘
```

## 4. calcAvoidancePlanningData() 详细流程

**作用**：生成避碰规划所需的数据

```cpp
AvoidancePlanningData calcAvoidancePlanningData(AvoidanceDebugData & debug)
{
  AvoidancePlanningData data;
  
  // ① 获取参考位姿
  data.reference_pose = getEgoPose();
  
  // ② 获取上一模块输出的路径
  data.reference_path_rough = prev_module_output_.path;
  
  // ③ 重采样路径 (间隔: resample_interval_for_planning)
  data.reference_path = utils::resamplePathWithSpline(
    data.reference_path_rough, 
    resample_interval
  );
  
  // ④ 获取当前车道集
  data.current_lanelets = get_current_lanes();
  
  // ⑤ 填充目标对象 (关键!)
  fillAvoidanceTargetObjects(data, debug);
  
  return data;
}
```

## 5. fillAvoidanceTargetObjects() 对象过滤流程

**作用**：过滤可避碰的目标对象

### 5.1 过滤步骤

```
┌─ fillAvoidanceTargetObjects() ──────────────────────┐
│                                                     │
├─> ① 分离车道内外对象                               │
│   separateObjectsByLanelets()                      │
│   ├─> object_within_target_lane                    │
│   └─> object_outside_target_lane → other_objects   │
│                                                     │
├─> ② 遍历车道内对象                                 │
│   for (const auto & obj : object_within_target_lane)│
│   {                                                 │
│     // ③ 检查 target_type 过滤                     │
│     obj_type = getHighestProbLabel()               │
│     switch(obj_type):                              │
│       CAR:        → check params.target_type.car   │
│       TRUCK:      → check params.target_type.truck │
│       BUS:        → check params.target_type.bus   │
│       TRAILER:    → check params.target_type.trailer
│       UNKNOWN:    → check params.target_type.unknown
│       BICYCLE:    → check params.target_type.bicycle
│       MOTORCYCLE: → check params.target_type.motorcycle
│       PEDESTRIAN: → check params.target_type.pedestrian
│                                                     │
│     if NOT enabled:                                 │
│       continue; // 跳过此对象                       │
│                                                     │
│     // ④ 为对象创建数据                            │
│     target_lane_object = createObjectData(obj)     │
│     if (invalid):                                   │
│       continue; // 对象被过滤                       │
│                                                     │
│     // ⑤ 添加到目标列表                            │
│     target_lane_objects.push_back(object_data)     │
│   }                                                 │
│                                                     │
└─ data.target_objects = target_lane_objects ────────┘
```

### 5.2 target_type 参数检查 (重要!)

```yaml
# avoidance_by_lane_change.param.yaml
target_filtering:
  target_type:
    car: false           # ❌ 禁用汽车
    truck: false         # ❌ 禁用卡车
    bus: false           # ❌ 禁用公交车
    trailer: false       # ❌ 禁用拖车
    unknown: true        # ✅ 启用未知物体
    bicycle: true        # ✅ 启用自行车
    motorcycle: false    # ❌ 禁用摩托车
    pedestrian: true     # ✅ 启用行人
```

## 6. createObjectData() 对象数据创建流程

**作用**：为单个对象创建完整的避碰数据

### 6.1 对象处理步骤

```cpp
std::optional<ObjectData> createObjectData(const AvoidancePlanningData & data, 
                                          const PredictedObject & object)
{
  ObjectData object_data{};
  
  // ① 查找最近路径点
  object_closest_index = findNearestIndex(path_points, object_pose);
  
  // ② 计算到中心线的距离
  object_data.to_centerline = getArcCoordinates(...).distance;
  
  // ③ 检查中心线距离阈值
  if (abs(object_data.to_centerline) < threshold_distance_object_is_on_center) {
    return std::nullopt; // 对象太接近中心线，过滤
  }
  
  // ④ 计算距离因子 (用于包络多边形扩展)
  object_data.distance_factor = ...;
  
  // ⑤ 填充对象包络多边形
  fillObjectEnvelopePolygon(object_data, ...);
  
  // ⑥ 计算质心
  object_data.centroid = return_centroid(object_data.envelope_poly);
  
  // ⑦ 填充移动时间
  fillObjectMovingTime(object_data, stopped_objects_, param);
  
  // ⑧ 判断避碰方向 (左/右)
  object_data.direction = (lateral_deviation > 0) ? Direction::LEFT : Direction::RIGHT;
  
  // ⑨ 计算超悬距离
  object_data.overhang_points = calcEnvelopeOverhangDistance(...);
  
  // ⑩ 检查是否需要避碰
  fillAvoidanceNecessity(object_data, registered_objects_, vehicle_width, param);
  
  // ⑪ 填充纵向距离和长度
  fillLongitudinalAndLengthByClosestEnvelopeFootprint(...);
  
  return object_data; // 返回有效的对象数据
}
```

### 6.2 过滤条件 (对象被过滤的情况)

| 条件 | 原因 | 结果 |
|------|------|------|
| `abs(to_centerline) < threshold` | 对象过于接近中心线 | ❌ 返回nullopt |
| 其他createObjectData检查失败 | 例如: 计算错误 | ❌ 返回nullopt |

## 7. 模块激活的完整条件检查

激活模块需要满足以下**所有**条件：

```
┌─ 激活条件检查 ─────────────────────────────┐
│                                           │
│ ✓ detected_objects != empty              │
│   └─> 至少有一个被感知的对象             │
│                                           │
│ ✓ target_objects != empty                │
│   └─> 至少一个对象在当前车道内           │
│                                           │
│ ✓ target_type enabled                    │
│   └─> 对象类型在参数中启用              │
│                                           │
│ ✓ createObjectData success               │
│   └─> 对象满足各种检查条件               │
│                                           │
│ ✓ avoid_required == true                 │
│   └─> 对象标记为需要避碰                │
│                                           │
│ ✓ num_of_avoidance_targets >= 1         │
│   └─> 至少有一个需要避碰的对象          │
│                                           │
│ ✓ nearest_object.longitudinal >          │
│      required_distance                   │
│   └─> 纵向距离足够进行换道               │
│                                           │
└────────────────────────────────────────────┘
```

## 8. 调试日志和信息

### 8.1 关键日志消息

```
DEBUG:
  "[AvoidByLC] Objects: total=X, in_lane=Y, out_lane=Z"
  "[AvoidByLC] Object type XYZ filtered by target_type config"
  "[AvoidByLC] Object filtered by createObjectData check"
  "[AvoidByLC] Target objects after filtering: N"

INFO:
  "[AvoidByLC] Module ACTIVATED: targets=X, obj_dist=Y.ZZ > required=W.WW"

WARN/INFO (THROTTLED):
  "[AvoidByLC] Module NOT activated: No target objects detected"
  "[AvoidByLC] Module NOT activated: No avoidance target found (total_objects=X, avoidance_required=0)"
  "[AvoidByLC] Module NOT activated: Insufficient distance (obj_dist=X.XX < required=Y.YY [LC=Z.ZZ, Avoid=W.WW])"
```

### 8.2 启用调试日志

```bash
# 方式1: 启用所有DEBUG日志
ros2 launch ... --log-level debug

# 方式2: 只看avoidance_by_lane_change的日志
ros2 launch ... --log-level autoware_behavior_path_avoidance_by_lane_change_module:=debug
```

## 9. 常见问题排查

### 问题: "No target objects detected"

**可能原因**:
1. ❌ 感知未检测到障碍物
   - 检查: `/perception/object_recognition/objects` 话题
   
2. ❌ 对象不在当前车道内
   - 检查: `in_lane` vs `out_lane` 对象数量
   
3. ❌ 对象类型被参数过滤
   - 检查: `target_type` 参数中该类型是否enabled

### 问题: "No avoidance target found"

**可能原因**:
1. ❌ 所有对象都 `avoid_required = false`
   - 可能原因: 对象已停止,不需要避碰
   
2. ❌ `to_centerline` 距离过小
   - 对象太接近路径中心线

### 问题: "Insufficient distance"

**可能原因**:
1. ❌ 对象距离太近 (纵向)
   - 需要 `execute_object_longitudinal_margin` 距离
   
2. ❌ 换道或避碰需要的空间不足
   - 可能需要调整相关参数

## 10. 参数调整建议 (针对低速AGV 4.2m/s)

```yaml
avoidance_by_lane_change:
  # 减小纵向间距
  execute_object_longitudinal_margin: 30.0  # 原: 80.0
  
  # 更严格的安全检查
  execute_only_when_lane_change_finish_before_object: true
  
  target_filtering:
    target_type:
      car: false
      truck: false
      bus: false
      trailer: false
      unknown: true
      bicycle: true
      motorcycle: false
      pedestrian: true
```

---

**模块完全激活流程总结**: 
感知检测 → 对象过滤(lane/type) → 创建对象数据 → 检查避碰必要性 → 计数目标对象 → 计算距离 → 比较纵向距离 → **激活模块**
