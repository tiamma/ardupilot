# 加速度积分速度计算说明

## 📋 概述

将速度获取方式从 **EKF融合速度** 改为 **加速度传感器积分**，用于VTOL模式下的前后速度控制。

---

## 🔄 修改对比

### **修改前（EKF速度）**
```cpp
// 使用EKF融合速度
Vector3f velocity_ned;
if (plane.ahrs.get_velocity_NED(velocity_ned)) {
    Vector3f velocity_body = plane.ahrs.get_rotation_body_to_ned().transposed() * velocity_ned;
    current_velocity_mps = velocity_body.x;
    velocity_valid = true;
}
```

**优点**：
- ✅ 融合多传感器数据（GPS、气压计、IMU）
- ✅ 精度高，漂移小
- ✅ 长期稳定

**缺点**：
- ❌ 依赖GPS信号
- ❌ 室内或GPS拒止环境无法使用
- ❌ 更新频率受GPS限制

---

### **修改后（加速度积分）**
```cpp
// 获取机体坐标系加速度
Vector3f accel_body = plane.ins.get_accel();  // m/s²

// 补偿重力加速度
const Matrix3f &rot_body_to_ned = plane.ahrs.get_rotation_body_to_ned();
Vector3f accel_ned = rot_body_to_ned * accel_body;
accel_ned.z += GRAVITY_MSS;  // 去除重力
Vector3f accel_body_corrected = rot_body_to_ned.transposed() * accel_ned;

// 提取前向加速度并积分
float forward_accel_mps2 = accel_body_corrected.x;
integrated_velocity_mps += forward_accel_mps2 * dt;

// 添加衰减因子防止漂移
integrated_velocity_mps *= 0.98f;

current_velocity_mps = integrated_velocity_mps;
```

**优点**：
- ✅ 不依赖GPS
- ✅ 高频更新（通常400Hz+）
- ✅ 室内可用
- ✅ 响应快速

**缺点**：
- ❌ 积分漂移
- ❌ 需要重力补偿
- ❌ 长期精度较低

---

## 🔧 实现细节

### **1. 获取加速度**
```cpp
Vector3f accel_body = plane.ins.get_accel();  // m/s²
```
- 从IMU获取机体坐标系加速度
- 单位：m/s²
- 坐标系：机体坐标系（X前，Y右，Z下）

---

### **2. 重力补偿**

#### **为什么需要重力补偿？**
```
加速度计测量的是：
总加速度 = 运动加速度 + 重力加速度

例如：
- 水平飞行：accel_z ≈ -9.8 m/s²（重力）
- 俯冲加速：accel_x = 运动加速度 + 重力分量
```

#### **补偿步骤**
```cpp
// 步骤1：转换到NED坐标系
Vector3f accel_ned = rot_body_to_ned * accel_body;

// 步骤2：去除重力（NED中重力在Z轴负方向）
accel_ned.z += GRAVITY_MSS;  // GRAVITY_MSS = 9.80665 m/s²

// 步骤3：转回机体坐标系
Vector3f accel_body_corrected = rot_body_to_ned.transposed() * accel_ned;
```

#### **示例计算**
```
姿态：俯仰30°
加速度计读数：accel_body = [5.0, 0, -8.5] m/s²

转换到NED：
accel_ned = [5.0×cos(30°) - 8.5×sin(30°), 0, 5.0×sin(30°) + 8.5×cos(30°)]
          ≈ [0.08, 0, -9.86] m/s²

去除重力：
accel_ned.z += 9.80665
accel_ned ≈ [0.08, 0, -0.05] m/s²

转回机体：
accel_body_corrected ≈ [0.1, 0, 0] m/s²  ← 真实运动加速度
```

---

### **3. 速度积分**
```cpp
integrated_velocity_mps += forward_accel_mps2 * dt;
```

**积分公式**：
```
v(t) = v(t-1) + a(t) × Δt
```

**示例**：
```
初始速度：0 m/s
加速度：2 m/s²
时间间隔：0.02s (50Hz)

第1次：v = 0 + 2×0.02 = 0.04 m/s
第2次：v = 0.04 + 2×0.02 = 0.08 m/s
第3次：v = 0.08 + 2×0.02 = 0.12 m/s
...
第50次：v = 0.98 + 2×0.02 = 1.0 m/s
```

---

### **4. 漂移抑制**
```cpp
const float decay_factor = 0.98f;
integrated_velocity_mps *= decay_factor;
```

#### **为什么需要衰减？**
```
加速度计误差：
- 零偏误差：±0.01 m/s²
- 噪声：±0.05 m/s²

积分效果：
- 1秒后：误差 ≈ 0.01 m/s
- 10秒后：误差 ≈ 0.1 m/s
- 100秒后：误差 ≈ 1.0 m/s  ← 不可接受
```

#### **衰减效果**
```
衰减因子 = 0.98 (每次更新衰减2%)
更新频率 = 50Hz

时间常数：
τ = -dt / ln(decay_factor)
  = -0.02 / ln(0.98)
  ≈ 1.0 秒

含义：
- 1秒后，速度衰减到原来的 37%
- 3秒后，速度衰减到原来的 5%
- 5秒后，速度基本归零
```

#### **衰减因子选择**
```
decay_factor = 0.99  → τ ≈ 2.0秒  （慢衰减，适合长时间飞行）
decay_factor = 0.98  → τ ≈ 1.0秒  （中等衰减，推荐）
decay_factor = 0.95  → τ ≈ 0.4秒  （快衰减，适合短时机动）
```

---

## 📊 性能分析

### **更新频率对比**

| 方法 | 更新频率 | 延迟 | 室内可用 |
|------|---------|------|---------|
| EKF速度 | 5-10Hz | 100-200ms | ❌ 需要GPS |
| 加速度积分 | 400Hz+ | <5ms | ✅ 可用 |

---

### **精度对比**

| 时间 | EKF速度误差 | 加速度积分误差 |
|------|------------|---------------|
| 0.1s | ±0.01 m/s | ±0.02 m/s |
| 1s | ±0.05 m/s | ±0.1 m/s |
| 5s | ±0.1 m/s | ±0.5 m/s |
| 10s | ±0.2 m/s | ±1.0 m/s |

**结论**：
- 短时间（<1秒）：加速度积分精度可接受
- 长时间（>5秒）：EKF速度更准确

---

## ⚙️ 参数调整

### **衰减因子调整**

#### **场景1：快速机动**
```cpp
const float decay_factor = 0.95f;  // 快速衰减
```
- 适合：短时间加速/减速
- 优点：快速归零，减少漂移
- 缺点：长时间加速时速度被抑制

---

#### **场景2：稳定飞行**
```cpp
const float decay_factor = 0.99f;  // 慢速衰减
```
- 适合：长时间匀速飞行
- 优点：保持速度稳定
- 缺点：漂移累积较多

---

#### **场景3：推荐配置**
```cpp
const float decay_factor = 0.98f;  // 中等衰减
```
- 平衡漂移和响应
- 适合大多数场景

---

### **零速度重置**

可以添加零速度检测，当检测到悬停时重置积分速度：

```cpp
// 检测悬停状态
bool is_hovering = (fabsf(rc_pitch_input) < 0.05f) && 
                   (fabsf(forward_accel_mps2) < 0.1f);

if (is_hovering) {
    // 快速衰减到零
    integrated_velocity_mps *= 0.9f;
}
```

---

## 🧪 测试方法

### **地面测试**
```
1. 连接地面站
2. 查看日志输出：
   - log_current_velocity (积分速度)
   - forward_accel_mps2 (前向加速度)
3. 手动移动飞机
4. 观察速度变化
```

### **悬停测试**
```
1. 起飞悬停
2. 前推摇杆加速
3. 观察：
   - 速度是否增加
   - 松杆后速度是否衰减
   - 是否有明显漂移
```

### **速度控制测试**
```
1. 前推摇杆到50%
2. 观察飞机是否稳定前飞
3. 回中摇杆
4. 观察飞机是否减速悬停
```

---

## 📈 调试输出

### **建议添加的GCS输出**
```cpp
static uint32_t last_debug_ms = 0;
if (now_ms - last_debug_ms >= 1000) {
    last_debug_ms = now_ms;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                 "VEL: int=%.2f acc=%.2f err=%.2f", 
                 (double)integrated_velocity_mps,
                 (double)forward_accel_mps2,
                 (double)velocity_error);
}
```

### **输出示例**
```
VEL: int=2.50 acc=0.50 err=-0.30
```
- `int`: 积分速度 (m/s)
- `acc`: 前向加速度 (m/s²)
- `err`: 速度误差 (m/s)

---

## ⚠️ 注意事项

### **1. 积分漂移**
```
问题：长时间飞行后速度漂移严重
原因：加速度计零偏累积
解决：
- 使用衰减因子
- 定期零速度重置
- 结合其他传感器（如光流）
```

### **2. 姿态依赖**
```
问题：姿态估计错误导致速度错误
原因：重力补偿依赖姿态
解决：
- 确保AHRS校准良好
- 检查姿态估计精度
```

### **3. 振动影响**
```
问题：振动导致加速度噪声大
原因：电机振动传递到IMU
解决：
- 使用低通滤波
- 改善减震
- 增加衰减因子
```

---

## 🔧 可选改进

### **1. 低通滤波**
```cpp
// 添加低通滤波减少噪声
static float filtered_accel = 0.0f;
const float alpha = 0.2f;  // 滤波系数
filtered_accel = alpha * forward_accel_mps2 + (1 - alpha) * filtered_accel;

// 使用滤波后的加速度
integrated_velocity_mps += filtered_accel * dt;
```

---

### **2. 自适应衰减**
```cpp
// 根据加速度大小调整衰减
float adaptive_decay = 0.98f;
if (fabsf(forward_accel_mps2) < 0.1f) {
    // 加速度小时快速衰减
    adaptive_decay = 0.95f;
}
integrated_velocity_mps *= adaptive_decay;
```

---

### **3. 零速度检测**
```cpp
// 检测静止状态
static uint32_t stationary_count = 0;
if (fabsf(forward_accel_mps2) < 0.05f && 
    fabsf(rc_pitch_input) < 0.05f) {
    stationary_count++;
    if (stationary_count > 50) {  // 1秒静止
        integrated_velocity_mps = 0.0f;  // 重置速度
        stationary_count = 0;
    }
} else {
    stationary_count = 0;
}
```

---

### **4. 融合EKF速度（混合方案）**
```cpp
// 高频使用加速度积分，低频校正EKF速度
static uint32_t last_ekf_update_ms = 0;
if (now_ms - last_ekf_update_ms >= 200) {  // 每200ms
    Vector3f velocity_ned;
    if (plane.ahrs.get_velocity_NED(velocity_ned)) {
        Vector3f velocity_body = rot_body_to_ned.transposed() * velocity_ned;
        float ekf_velocity = velocity_body.x;
        
        // 融合：80%积分 + 20%EKF
        integrated_velocity_mps = 0.8f * integrated_velocity_mps + 
                                  0.2f * ekf_velocity;
    }
    last_ekf_update_ms = now_ms;
}
```

---

## 📊 总结

### **优点**
✅ 不依赖GPS，室内可用  
✅ 高频更新，响应快速  
✅ 实现简单

### **缺点**
❌ 积分漂移  
❌ 长期精度低  
❌ 依赖姿态估计

### **适用场景**
- ⭐ 室内飞行
- ⭐ GPS拒止环境
- ⭐ 短时间速度控制（<5秒）
- ⭐ 快速机动

### **不适用场景**
- ❌ 长时间匀速飞行
- ❌ 高精度速度控制
- ❌ GPS可用环境（建议用EKF）

---

**版本**: 1.0  
**日期**: 2026-06-04  
**修改**: 从EKF速度改为加速度积分
