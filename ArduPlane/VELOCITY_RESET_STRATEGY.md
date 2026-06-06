# 速度积分重置策略说明

## 📋 问题描述

### **漂移问题**
```
加速度积分计算速度时的问题：
- 加速度计零偏：±0.01 m/s²
- 积分累积误差：每秒 ±0.01 m/s
- 3秒后漂移：±0.03 m/s
- 10秒后漂移：±0.1 m/s
- 30秒后漂移：±0.3 m/s ❌ 不可接受
```

---

## 🔧 解决方案

### **定期重置策略**
```cpp
// 每隔3秒重置积分速度
static uint32_t last_velocity_reset_ms = 0;

if (now_ms - last_velocity_reset_ms >= 3000) {
    integrated_velocity_mps = 0.0f;
    last_velocity_reset_ms = now_ms;
}
```

---

## 📊 重置周期对比

### **1秒重置**
```
优点：
✅ 漂移最小
✅ 误差不超过 ±0.01 m/s

缺点：
❌ 速度控制不连续
❌ 无法保持匀速
❌ 频繁归零影响控制
```

### **3秒重置（推荐）⭐**
```
优点：
✅ 平衡漂移和连续性
✅ 误差控制在 ±0.03 m/s
✅ 允许短时间加速/减速
✅ 控制相对平滑

缺点：
⚠️ 3秒时会有速度跳变
```

### **5秒重置**
```
优点：
✅ 控制更连续
✅ 适合长时间匀速

缺点：
❌ 漂移较大（±0.05 m/s）
❌ 长时间累积误差
```

### **10秒重置**
```
优点：
✅ 控制最连续

缺点：
❌ 漂移严重（±0.1 m/s）
❌ 不推荐
```

---

## 🎯 工作原理

### **时间轴示例（3秒重置）**
```
时间 | 积分速度 | 真实速度 | 漂移 | 操作
-----|---------|---------|------|------
0.0s | 0.00    | 0.00    | 0.00 | 重置
0.5s | 0.50    | 0.48    | +0.02| 加速中
1.0s | 1.00    | 0.97    | +0.03| 
1.5s | 1.20    | 1.15    | +0.05| 
2.0s | 1.00    | 0.98    | +0.02| 减速中
2.5s | 0.50    | 0.48    | +0.02| 
3.0s | 0.00    | 0.00    | 0.00 | 重置 ← 清除漂移
3.5s | 0.30    | 0.28    | +0.02| 新周期
4.0s | 0.60    | 0.57    | +0.03| 
...
```

---

## 💡 重置时机选择

### **固定周期重置（当前方案）**
```cpp
if (now_ms - last_velocity_reset_ms >= 3000) {
    integrated_velocity_mps = 0.0f;
    last_velocity_reset_ms = now_ms;
}
```

**特点**：
- 简单可靠
- 周期固定
- 易于预测

---

### **智能重置（可选改进）**
```cpp
// 检测静止状态
bool is_stationary = (fabsf(forward_accel_mps2) < 0.05f) && 
                     (fabsf(rc_pitch_input) < 0.05f);

if (is_stationary) {
    // 静止时快速重置
    integrated_velocity_mps *= 0.9f;
    if (fabsf(integrated_velocity_mps) < 0.01f) {
        integrated_velocity_mps = 0.0f;
    }
} else if (now_ms - last_velocity_reset_ms >= 3000) {
    // 运动时定期重置
    integrated_velocity_mps = 0.0f;
    last_velocity_reset_ms = now_ms;
}
```

**优点**：
- 静止时快速归零
- 运动时保持连续
- 减少跳变

---

## 📈 性能分析

### **速度误差对比**

| 重置周期 | 最大漂移 | 平均漂移 | 控制连续性 | 推荐度 |
|---------|---------|---------|-----------|--------|
| 1秒 | ±0.01 m/s | ±0.005 m/s | ⭐⭐ | ⭐⭐⭐ |
| 3秒 | ±0.03 m/s | ±0.015 m/s | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 5秒 | ±0.05 m/s | ±0.025 m/s | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| 10秒 | ±0.1 m/s | ±0.05 m/s | ⭐⭐⭐⭐⭐ | ⭐⭐ |

---

### **飞行场景适配**

#### **场景1：悬停**
```
操作：摇杆中位
速度：0 m/s
重置影响：无影响（速度本就为0）
推荐周期：3秒
```

#### **场景2：短时加速**
```
操作：前推1秒，回中
速度：0 → 1 m/s → 0
重置影响：小（1秒内完成）
推荐周期：3秒
```

#### **场景3：匀速飞行**
```
操作：前推保持2秒
速度：0 → 2 m/s → 保持
重置影响：中（2秒时可能重置）
推荐周期：5秒
```

#### **场景4：长时间飞行**
```
操作：前推保持5秒+
速度：0 → 3 m/s → 保持
重置影响：大（会被重置）
推荐周期：不适合用积分速度
建议：改用EKF速度
```

---

## 🔧 参数调整

### **修改重置周期**

#### **1秒重置（快速归零）**
```cpp
if (now_ms - last_velocity_reset_ms >= 1000) {  // 1秒
    integrated_velocity_mps = 0.0f;
    last_velocity_reset_ms = now_ms;
}
```

#### **3秒重置（默认推荐）**
```cpp
if (now_ms - last_velocity_reset_ms >= 3000) {  // 3秒
    integrated_velocity_mps = 0.0f;
    last_velocity_reset_ms = now_ms;
}
```

#### **5秒重置（长时间控制）**
```cpp
if (now_ms - last_velocity_reset_ms >= 5000) {  // 5秒
    integrated_velocity_mps = 0.0f;
    last_velocity_reset_ms = now_ms;
}
```

---

## ⚠️ 注意事项

### **1. 重置时的速度跳变**
```
问题：重置瞬间速度归零，可能导致控制跳变
影响：俯仰角突变

缓解方法：
- 使用较长的重置周期（3-5秒）
- 添加速度平滑滤波
- 使用渐进式重置
```

### **2. 重置周期选择**
```
太短（<1秒）：
❌ 无法保持速度
❌ 控制不连续

太长（>10秒）：
❌ 漂移严重
❌ 误差累积

推荐：3-5秒
```

### **3. 与速度控制PID配合**
```
速度PID参数需要考虑重置影响：
- I项：不要太大（避免重置后超调）
- D项：可以适当增加（抑制跳变）
```

---

## 🧪 测试方法

### **测试1：悬停漂移**
```
1. 起飞悬停
2. 摇杆中位
3. 观察3秒周期内的速度变化
4. 检查重置时是否有跳变

预期：
- 速度在 ±0.03 m/s 内波动
- 每3秒归零一次
- 无明显跳变
```

### **测试2：加速响应**
```
1. 悬停
2. 前推摇杆1秒
3. 回中摇杆
4. 观察速度变化

预期：
- 速度快速增加
- 回中后快速减小
- 3秒内归零
```

### **测试3：匀速飞行**
```
1. 悬停
2. 前推摇杆保持2秒
3. 观察速度是否稳定

预期：
- 速度逐渐增加
- 2秒时达到期望速度
- 3秒时被重置（会有跳变）
```

---

## 📊 GCS调试输出

### **建议添加重置标志**
```cpp
static uint32_t last_velocity_reset_ms = 0;
static bool velocity_was_reset = false;

if (now_ms - last_velocity_reset_ms >= 3000) {
    integrated_velocity_mps = 0.0f;
    last_velocity_reset_ms = now_ms;
    velocity_was_reset = true;
}

// 调试输出
static uint32_t last_debug_ms = 0;
if (now_ms - last_debug_ms >= 100) {  // 每100ms
    last_debug_ms = now_ms;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                 "VEL: %.2f acc:%.2f rst:%d", 
                 (double)integrated_velocity_mps,
                 (double)forward_accel_mps2,
                 velocity_was_reset ? 1 : 0);
    velocity_was_reset = false;
}
```

### **输出示例**
```
VEL: 0.50 acc:0.20 rst:0
VEL: 0.70 acc:0.20 rst:0
VEL: 0.90 acc:0.20 rst:0
VEL: 0.00 acc:0.00 rst:1  ← 重置
VEL: 0.10 acc:0.10 rst:0
```

---

## 💡 可选改进方案

### **方案1：渐进式重置**
```cpp
// 不是突然归零，而是逐渐衰减
if (now_ms - last_velocity_reset_ms >= 3000) {
    integrated_velocity_mps *= 0.5f;  // 衰减50%
    last_velocity_reset_ms = now_ms;
}
```

**优点**：减少跳变  
**缺点**：漂移清除不彻底

---

### **方案2：条件重置**
```cpp
// 只在速度较小时重置
if (now_ms - last_velocity_reset_ms >= 3000) {
    if (fabsf(integrated_velocity_mps) < 0.5f) {
        integrated_velocity_mps = 0.0f;
    }
    last_velocity_reset_ms = now_ms;
}
```

**优点**：避免高速时重置  
**缺点**：高速时漂移无法清除

---

### **方案3：自适应重置周期**
```cpp
// 根据速度大小调整重置周期
uint32_t reset_period_ms = 3000;
if (fabsf(integrated_velocity_mps) > 2.0f) {
    reset_period_ms = 5000;  // 高速时延长周期
}

if (now_ms - last_velocity_reset_ms >= reset_period_ms) {
    integrated_velocity_mps = 0.0f;
    last_velocity_reset_ms = now_ms;
}
```

**优点**：自适应调整  
**缺点**：逻辑复杂

---

## 📝 总结

### **当前方案（3秒固定重置）**
```cpp
if (now_ms - last_velocity_reset_ms >= 3000) {
    integrated_velocity_mps = 0.0f;
    last_velocity_reset_ms = now_ms;
}
```

### **优点**
✅ 简单可靠  
✅ 有效防止漂移累积  
✅ 误差控制在 ±0.03 m/s  
✅ 适合短时间速度控制

### **缺点**
⚠️ 每3秒有速度跳变  
⚠️ 不适合长时间匀速飞行

### **适用场景**
- ⭐ 室内悬停
- ⭐ 短时间机动（<3秒）
- ⭐ 频繁加速/减速
- ⭐ GPS拒止环境

### **不适用场景**
- ❌ 长时间匀速飞行（>5秒）
- ❌ 高精度速度控制
- ❌ GPS可用环境（建议用EKF）

---

**版本**: 1.0  
**日期**: 2026-06-05  
**重置周期**: 3秒  
**最大漂移**: ±0.03 m/s
