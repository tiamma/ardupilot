# VTOL偏航增益参数说明

## 概述

已添加 `VTOL_YAW_GAIN` 参数，用于调整VTOL模式下偏航控制的输出幅度。

## 参数详情

### 参数名称
```
VTOL_YAW_GAIN
```

### 参数信息

| 属性 | 值 |
|------|-----|
| **显示名称** | VTOL Yaw Control Gain |
| **描述** | VTOL偏航稳定输出到副翼的增益。更高的值增加偏航纠正强度 |
| **默认值** | 1.0 |
| **范围** | 0.0 - 2.0 |
| **增量** | 0.1 |
| **用户级别** | Advanced |

## 功能说明

### 控制流程

```
偏航角度误差 → 归一化(-1到1) ┐
                              ├→ 混合 → 限幅 → × SERVO_MAX × VTOL_YAW_GAIN → 副翼输出
遥控器偏航输入 → 归一化(-1到1) ┘
```

### 计算公式

```cpp
// 1. 归一化偏航误差
yaw_error_normalized = yaw_error / 90.0  // -1 到 1

// 2. 获取遥控器输入
yaw_input = rudder_input / rudder_range  // -1 到 1

// 3. 混合控制
combined = yaw_input + yaw_error_normalized

// 4. 限幅
combined = constrain(combined, -1.0, 1.0)

// 5. 应用增益并输出
ail_out = combined × SERVO_MAX × VTOL_YAW_GAIN
```

## 使用方法

### 在Mission Planner中设置

```
1. 连接飞控
2. 进入 CONFIG/TUNING → Full Parameter List
3. 搜索 "VTOL_YAW_GAIN"
4. 设置期望值 (0.0 - 2.0)
5. 点击 "Write Params"
```

### 通过MAVProxy设置

```bash
param set VTOL_YAW_GAIN 1.0
param write
```

## 参数调整指南

### 推荐设置

**保守设置** (弱偏航控制):
```
VTOL_YAW_GAIN = 0.5
```
- 适用场景：初次测试、轻载、低风速
- 特点：响应温和、不易过调

**标准设置** (默认):
```
VTOL_YAW_GAIN = 1.0
```
- 适用场景：正常飞行、中等载重
- 特点：平衡的响应和稳定性

**激进设置** (强偏航控制):
```
VTOL_YAW_GAIN = 1.5
```
- 适用场景：重载、强风、快速机动
- 特点：强力纠正、快速响应

**最大设置**:
```
VTOL_YAW_GAIN = 2.0
```
- 适用场景：极端条件、测试
- 特点：最大纠正力度
- ⚠️ 注意：可能导致振荡

### 调整步骤

**步骤1**: 从默认值开始
```
VTOL_YAW_GAIN = 1.0
```

**步骤2**: 地面测试
- 手动改变偏航角目标
- 观察副翼响应
- 检查是否有振荡

**步骤3**: 根据表现调整

| 现象 | 调整方向 | 建议值 |
|------|---------|--------|
| 偏航纠正太弱 | 增加 | +0.2 |
| 偏航纠正太强 | 减少 | -0.2 |
| 出现振荡 | 减少 | -0.3 |
| 响应迟钝 | 增加 | +0.3 |

**步骤4**: 空中验证
- 悬停测试
- 慢速前飞测试
- 观察偏航保持能力

**步骤5**: 微调优化
- 每次调整 ±0.1
- 记录每次调整的效果
- 找到最佳值

## 效果对比

### 不同增益值的效果

| VTOL_YAW_GAIN | 偏航纠正强度 | 响应速度 | 稳定性 | 适用场景 |
|---------------|-------------|---------|--------|---------|
| **0.3** | 很弱 | 慢 | 很稳定 | 初次测试 |
| **0.5** | 弱 | 较慢 | 稳定 | 轻载/低风 |
| **0.8** | 中等 | 中等 | 良好 | 一般飞行 |
| **1.0** | 标准 | 正常 | 平衡 | 推荐默认 |
| **1.2** | 较强 | 较快 | 良好 | 中等载重 |
| **1.5** | 强 | 快 | 可能振荡 | 重载/强风 |
| **2.0** | 很强 | 很快 | 易振荡 | 极端条件 |

### 示例场景

**场景1**: 轻载悬停
```
条件: 无风、轻载、平稳悬停
推荐: VTOL_YAW_GAIN = 0.8
原因: 不需要强力纠正，避免过调
```

**场景2**: 正常飞行
```
条件: 微风、正常载重、一般机动
推荐: VTOL_YAW_GAIN = 1.0
原因: 平衡的性能
```

**场景3**: 强风环境
```
条件: 强风、需要保持航向
推荐: VTOL_YAW_GAIN = 1.3
原因: 需要更强的纠正力度
```

**场景4**: 重载起飞
```
条件: 满载、起飞阶段
推荐: VTOL_YAW_GAIN = 1.5
原因: 惯性大，需要强力控制
```

## 与其他参数的关系

### 相关参数

**1. Q_TILT_P_ANG_P** (俯仰角度P增益)
```
如果增加 VTOL_YAW_GAIN，可能需要相应调整俯仰控制
保持协调的控制响应
```

**2. Q_TILT_MAX_RATE** (最大角速度)
```
VTOL_YAW_GAIN 影响输出幅度
MAX_RATE 限制角速度
两者配合使用
```

**3. SERVO_MAX** (舵机最大值)
```
VTOL_YAW_GAIN 是基于 SERVO_MAX 的倍数
SERVO_MAX = 4500 时:
  VTOL_YAW_GAIN = 1.0 → 最大输出 ±4500
  VTOL_YAW_GAIN = 0.5 → 最大输出 ±2250
```

## 故障排查

### 问题1: 偏航控制无效

**症状**: 设置参数后偏航控制没有变化

**检查**:
```
1. 确认参数已写入: param show VTOL_YAW_GAIN
2. 确认在QHOVER模式
3. 检查stabilize_vtol_yaw是否被调用
4. 查看日志中的副翼输出
```

### 问题2: 偏航振荡

**症状**: 飞机左右摇摆

**解决**:
```
1. 立即减小 VTOL_YAW_GAIN
   VTOL_YAW_GAIN = current_value × 0.7
   
2. 检查PID参数是否过大
   Q_TILT_Y_ANG_P
   Q_TILT_Y_RAT_P
   
3. 增加阻尼
   Q_TILT_Y_ANG_D
```

### 问题3: 响应太慢

**症状**: 偏航纠正缓慢

**解决**:
```
1. 增加 VTOL_YAW_GAIN
   每次增加 0.2
   
2. 检查是否达到限幅
   查看日志中的输出值
   
3. 确认遥控器输入正常
   检查 channel_rudder 范围
```

### 问题4: 参数不显示

**症状**: Mission Planner中找不到参数

**解决**:
```
1. 刷新参数列表
2. 重新上传固件
3. 搜索 "VTOL" 或 "YAW"
4. 检查固件版本
```

## 代码实现

### 参数定义位置

**Parameters.cpp** (第552行):
```cpp
// @Param: VTOL_YAW_GAIN
// @DisplayName: VTOL Yaw Control Gain
// @Description: Gain for VTOL yaw stabilization output to aileron
// @Range: 0.0 2.0
// @Increment: 0.1
// @User: Advanced
GSCALAR(vtol_yaw_gain, "VTOL_YAW_GAIN", 1.0),
```

**Parameters.h** (枚举):
```cpp
k_param_vtol_yaw_gain,  // 第223行
```

**Parameters.h** (变量声明):
```cpp
AP_Float vtol_yaw_gain;  // 第448行
```

**Attitude.cpp** (使用):
```cpp
float ail_out = combined_output * (float)SERVO_MAX * g.vtol_yaw_gain;
```

## 日志分析

### 查看参数效果

**日志字段**:
```
PARM.Name = VTOL_YAW_GAIN
PARM.Value = 当前设置值

RCOU.C1 = 副翼输出 (受VTOL_YAW_GAIN影响)
```

**分析方法**:
```
1. 绘制 RCOU.C1 vs 时间
2. 观察输出幅度
3. 检查是否达到限幅 (±4500)
4. 对比不同增益值的效果
```

## 安全建议

### ⚠️ 重要提示

1. **首次使用**: 从低值开始 (0.5)
2. **逐步增加**: 每次 +0.1 或 +0.2
3. **地面测试**: 先在地面验证
4. **空中测试**: 保持足够高度
5. **准备降落**: 如果振荡立即降落
6. **记录数值**: 记录每次调整
7. **备份参数**: 保存工作配置

### 推荐流程

```
1. 地面测试 (VTOL_YAW_GAIN = 0.5)
   ↓
2. 低空悬停 (VTOL_YAW_GAIN = 0.8)
   ↓
3. 正常飞行 (VTOL_YAW_GAIN = 1.0)
   ↓
4. 根据需要微调 (±0.1)
   ↓
5. 保存最佳配置
```

## 总结

### 关键点

✅ **默认值**: 1.0 (适合大多数情况)  
✅ **调整范围**: 0.0 - 2.0  
✅ **调整步长**: 0.1  
✅ **主要作用**: 控制偏航纠正强度  
✅ **影响输出**: 副翼通道 (k_aileron)  

### 快速参考

```
弱控制:   VTOL_YAW_GAIN = 0.5
标准控制: VTOL_YAW_GAIN = 1.0  ← 推荐
强控制:   VTOL_YAW_GAIN = 1.5
```

现在你可以通过调整 `VTOL_YAW_GAIN` 参数来优化双旋翼的偏航控制性能！
