# 模块激活诊断快速表

## 🔴 模块未激活？快速诊断表

当你看到 "[AvoidByLC] Module NOT activated" 时，按以下顺序检查：

### 检查清单

| # | 检查项 | 日志关键词 | 检查方法 | 修复方案 |
|---|--------|----------|--------|---------|
| 1️⃣ | **感知有无检测到对象** | `Objects: total=0` | `ros2 topic echo /perception/object_recognition/objects` | 检查感知管道配置 |
| 2️⃣ | **对象是否在车道内** | `in_lane=0` | 查看日志中 `in_lane` 数量 | 调整lane_expansion参数 |
| 3️⃣ | **对象类型是否启用** | `filtered by target_type` | 查看参数 `target_type.XXX` | 在参数中启用该物体类型 |
| 4️⃣ | **对象到中心线距离** | `filtered by createObjectData` | 查看对象位置 | 调整 `threshold_distance_object_is_on_center` |
| 5️⃣ | **对象是否标记为需避碰** | `avoidance_required=0` | 查看对象状态信息 | 检查avoid_required逻辑 |
| 6️⃣ | **纵向距离是否足够** | `Insufficient distance` | 查看 `obj_dist` vs `required` | 增加距离或调整参数 |

---

## 📊 激活逻辑决策树

```
有感知对象吗?
  │
  ├─NO──> ❌ "未检测到目标对象"
  │
  └─YES─> 对象在车道内吗?
            │
            ├─NO──> ❌ 移至 other_objects
            │
            └─YES─> 类型在target_type中启用吗?
                      │
                      ├─NO──> ❌ "被target_type过滤"
                      │
                      └─YES─> createObjectData成功吗?
                                │
                                ├─NO──> ❌ "被createObjectData过滤"
                                │       (to_centerline太小等)
                                │
                                └─YES─> avoid_required=true吗?
                                          │
                                          ├─NO──> ❌ "没有需要避碰的目标"
                                          │
                                          └─YES─> ✅ 有效目标对象
                                                    │
                                                    └─> 计算距离
                                                        │
                                                        ├─不足──> ⚠️ "距离不足"
                                                        │
                                                        └─足够──> ✅✅✅ 模块激活!
```

---

## 🔧 参数配置要点

### target_type 参数

```yaml
# ❌ 如果只启用了这些，园区AGV检测不到对象
target_type:
  car: true
  truck: true
  bus: true
  
# ✅ 园区AGV应该启用这些
target_type:
  unknown: true      # 未知/小物体
  bicycle: true      # 自行车
  pedestrian: true   # 行人
```

### 距离参数

```yaml
# 纵向间距 (米)
execute_object_longitudinal_margin: 30.0   # 物体纵向距离
                                           # AGV需要这个距离才能执行避碰

# 所需距离计算: required_distance = max(
#   calcMinAvoidanceLength(),        # 避碰需要的长度
#   calc_minimum_dist_buffer()       # 换道需要的长度
# )
```

---

## 📈 精确诊断：启用DEBUG日志

### 方法1: 修改日志级别

```bash
# 在终端启动planning_simulator时加入log-level参数
ros2 launch ... --log-level autoware_behavior_path_avoidance_by_lane_change_module:=debug
```

### 方法2: 查看日志文件

```bash
# 查看最新日志
tail -f ~/.ros/log/latest/*/logger_*/avoidance_by_lane_change.log

# 搜索AvoidByLC相关消息
grep -i "AvoidByLC" ~/.ros/log/latest/**/*/logger*
```

### 关键DEBUG输出

```
DEBUG: "[AvoidByLC] Objects: total=5, in_lane=2, out_lane=3"
       └─> 表示有5个总对象，2个在当前车道，3个在其他车道

DEBUG: "[AvoidByLC] Object type 1 filtered by target_type config"
       └─> 表示类型为1的对象被target_type配置过滤了

DEBUG: "[AvoidByLC] Object filtered by createObjectData check"
       └─> 表示对象在createObjectData中被过滤（如to_centerline太小）

DEBUG: "[AvoidByLC] Target objects after filtering: 1"
       └─> 最终有1个有效的目标对象
```

---

## 🎯 针对你的情况的调试步骤

### 🔍 Step 1: 验证感知有无工作
```bash
ros2 topic echo /perception/object_recognition/objects | head -20
# 查看是否有 classification (car, pedestrian, bicycle 等)
```

### 🔍 Step 2: 检查参数配置
```bash
# 查看当前的target_type配置
ros2 param get /planning_simulator avoidance_by_lane_change.target_filtering.target_type
```

### 🔍 Step 3: 启用调试日志并查看完整流程
```bash
# 在一个终端启动planning_simulator并启用DEBUG日志
ros2 launch planning_simulator planning_simulator.launch.xml --log-level debug

# 在另一个终端查看日志输出
# 或将日志保存到文件后分析
```

### 🔍 Step 4: 基于DEBUG日志诊断

对照以下DEBUG输出诊断:

| 日志输出 | 含义 | 可能原因 | 修复方案 |
|---------|------|---------|---------|
| `total=0` | 无感知对象 | 感知关闭 | 启动感知模块 |
| `in_lane=0` | 无对象在车道 | 对象在车道外 | 调整lane_expansion |
| `filtered by target_type` | 类型被过滤 | 参数禁用该类型 | 启用该类型参数 |
| `filtered by createObjectData` | 被对象检查过滤 | 检查条件不满足 | 查看to_centerline等 |
| `Target objects... : 0` | 最终无目标 | 以上问题合计 | 排查上述项 |
| `Insufficient distance` | 距离不足 | 对象太近 | 增加纵向间距参数 |

---

## 💡 常见场景解决方案

### 场景1: 有行人/自行车但模块未激活
```yaml
# 检查点:
1. target_type.pedestrian/bicycle 是否为 true?
2. 对象是否在当前车道内? (检查lane_expansion)
3. 对象到中心线距离是否过小?

# 调整:
target_filtering:
  target_type:
    pedestrian: true
    bicycle: true
    
# 或调整lane_expansion增大检测范围
lane_expansion:
  left_offset: 1.0
  right_offset: 1.0
```

### 场景2: 模块激活但距离不足警告
```yaml
# 问题: Insufficient distance 警告

# 原因: 对象太近，无法在execute_object_longitudinal_margin内完成避碰

# 解决方案1: 增加纵向间距 (不推荐，可能导致过度保守)
execute_object_longitudinal_margin: 20.0  # 从30.0减小

# 解决方案2: 提高激活距离 (推荐)
# 让障碍物检测更早触发
min_forward_distance: 5.0   # 减小前向检测距离
max_forward_distance: 15.0  # 减小后向检测距离
```

### 场景3: 多个对象但只选择最近的
```
# 模块特性: 每次只针对最近的对象规划
# 这是正常行为，一次完成一个对象避碰

nearest_object.longitudinal > required_distance
└─> 检查最近对象的距离是否足够
```

---

## 🎓 理解关键参数

### execute_object_longitudinal_margin (纵向间距)
- **含义**: AGV需要保持到障碍物的最小纵向距离
- **单位**: 米 (m)
- **推荐值**: 
  - 低速AGV (4.2m/s): 20-30m
  - 中速 (10-15m/s): 30-50m  
  - 高速 (>20m/s): 50-80m
- **影响**: 太小→任务无法完成, 太大→环路效率低

### execute_only_when_lane_change_finish_before_object
- **true**: 只在换道完全完成后且在对象前执行 (安全，但可能耗时)
- **false**: 在换道过程中边避碰 (快速，但风险高)
- **推荐**: AGV应设置为 **true**

---

## 📞 进一步诊断

如果按上述步骤仍无法解决，请收集以下信息：

```bash
# 1. 完整的参数文件
cat avoidance_by_lane_change.param.yaml

# 2. 启用DEBUG的完整日志
ros2 launch ... --log-level debug 2>&1 | tee debug.log

# 3. 感知对象信息
ros2 topic echo /perception/object_recognition/objects

# 4. 规划路径信息
ros2 topic echo /planning/path

# 5. AGV位置信息
ros2 topic echo /localization/kinematic_state
```

