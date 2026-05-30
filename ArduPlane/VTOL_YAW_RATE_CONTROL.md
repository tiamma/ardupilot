# VTOL偏航角速度控制说明

## 📋 概述

实现了**纯角速度控制模式**，方向舵直接控制偏航角速度，无角度环延迟。

---

## 🎯 控制方案对比

### **方案1：角度控制（原有）**
```
方向舵 → 角速度 → 累积角度 → 角度误差 → [外环P] → 期望角速度 → [内环PID] → 输出
```
**特点**：
- ✅ 可以保持角度
- ❌ 响应慢（双重积分）
- ❌ 手感不直观
- 适合：自动保持航向

---

### **方案2：角速度控制（新增）⭐**
```
方向舵 → 期望角速度 → [内环PID] → 输出
```
**特点**：
- ✅ 响应快速
- ✅ 手感直观
- ✅ 适合手动控制
- ❌ 不保持角度（松杆停转）
- 适合：特技飞行、手动控制

---

## 🔧 实现细节

### **新增函数**

#### **Plane.h**
```cpp
void stabilize_vtol_yaw_rate(float desired_yaw_rate_dps);
```

#### **Attitude.cpp**
```cpp
void Plane::stabilize_vtol_yaw_rate(float desired_yaw_rate_dps)
{
    // 1. 读取当前角速度
    float current_yaw_rate_dps = ahrs.get_gyro().z * RAD_TO_DEG;
    
    // 2. 计算误差
    float yaw_rate_error_dps = desired_yaw_rate_dps - current_yaw_rate_dps;
    
    // 3. PID控制
    P = error × VTOL_YAW_RT_P
    I = ∫(error × dt) × VTOL_YAW_RT_I
    D = d(error)/dt × VTOL_YAW_RT_D
    
    // 4. 输出到副翼
    output = (P + I + D) → aileron
}
```

---

### **mode_qstabilize.cpp 修改**

#### **修改前（角度控制）**
```cpp
// 计算角速度
const float yaw_rate_dps = rudder_input × rate;

// 累积成角度
max_change = yaw_rate_dps × dt;
nav_yaw_cd = offset + max_change;

// 计算角度误差
angle_error = current_yaw - nav_yaw_cd;

// 传入角度误差（会再次转换为角速度）
stabilize_vtol_yaw(angle_error);
```

#### **修改后（角速度控制）**
```cpp
// 计算期望角速度
const float desired_yaw_rate_dps = rudder_input × rate;

// 直接传入期望角速度
stabilize_vtol_yaw_rate(desired_yaw_rate_dps);
```

---

## 📊 控制流程

### **输入处理**
```cpp
rudder_input = channel_rudder->get_control_in() / range  // [-1, 1]
rate = quadplane.command_model_pilot.get_rate()          // 默认90 deg/s
desired_yaw_rate = rudder_input × rate / 100             // deg/s
```

**示例**：
| 方向舵位置 | rudder_input | rate | desired_yaw_rate |
|-----------|--------------|------|------------------|
| 中位 | 0.0 | 90 | 0°/s |
| 右满杆 | +1.0 | 90 | +90°/s |
| 左满杆 | -1.0 | 90 | -90°/s |
| 右半杆 | +0.5 | 90 | +45°/s |

---

### **PID控制**
```
期望角速度: 45°/s
当前角速度: 10°/s
误差: 45 - 10 = 35°/s

P输出 = 35 × 0.5 = 17.5
I输出 = ∫(35 × dt) × 0.1
D输出 = d(35)/dt × 0.02

总输出 = P + I + D → 副翼
```

---

## 🎮 飞行体验

### **方向舵操作**

#### **向右打舵**
```
方向舵 → 右
期望角速度 → +90°/s
飞机 → 向右偏航
速度 → 与打舵量成正比
```

#### **回中**
```
方向舵 → 中位
期望角速度 → 0°/s
飞机 → 停止转动
保持 → 当前角度（有小漂移）
```

#### **反向打舵**
```
方向舵 → 左
期望角速度 → -90°/s
飞机 → 向左偏航
```

---

## ⚙️ 参数调整

### **使用的参数**

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `VTOL_YAW_RT_P` | 0.5 | 角速度跟踪响应速度 |
| `VTOL_YAW_RT_I` | 0.1 | 消除稳态误差 |
| `VTOL_YAW_RT_D` | 0.02 | 抑制振荡 |
| `VTOL_YAW_RT_IMAX` | 10.0 | 积分限幅 |
| `VTOL_YAW_RT_MAX` | 90.0 | 最大角速度（未使用） |

**注意**：不使用 `VTOL_YAW_ANG_P`（无外环）

---

### **调参建议**

#### **响应太慢**
```
现象：打舵后转动慢
解决：增加 VTOL_YAW_RT_P
VTOL_YAW_RT_P = 0.7  (从0.5增加)
```

#### **抖动**
```
现象：转动时振荡
解决：减小 P，增加 D
VTOL_YAW_RT_P = 0.3  (从0.5减小)
VTOL_YAW_RT_D = 0.05 (从0.02增加)
```

#### **跟踪误差**
```
现象：期望90°/s，实际只有70°/s
解决：增加 I
VTOL_YAW_RT_I = 0.15 (从0.1增加)
```

---

## 📈 GCS调试输出

### **输出格式**
```
VTOL_YAW_RATE: desired=45.00 current=38.50 err=6.50
```

### **字段说明**
| 字段 | 说明 | 单位 |
|------|------|------|
| `desired` | 期望角速度（方向舵输入） | deg/s |
| `current` | 当前角速度（陀螺仪） | deg/s |
| `err` | 角速度误差 | deg/s |

---

### **分析方法**

#### **1. 响应速度**
```
打满舵：desired = 90°/s
观察：current 从 0 → 90 的时间
理想：< 0.5秒
```

#### **2. 跟踪精度**
```
稳定后：err 应该接近 0
理想：err < 5°/s
```

#### **3. 超调**
```
松杆：desired = 0°/s
观察：current 是否越过 0 并反向
理想：无超调或 < 10°/s
```

---

## 🆚 两种模式对比

### **何时使用角度控制**
```cpp
// QHOVER模式 - 保持航向
plane.stabilize_vtol_yaw(angle_error);
```
**场景**：
- 自动悬停
- 保持航向
- GPS导航

---

### **何时使用角速度控制**
```cpp
// QSTABILIZE模式 - 手动控制
plane.stabilize_vtol_yaw_rate(desired_yaw_rate_dps);
```
**场景**：
- 手动飞行
- 特技动作
- 快速响应

---

## 🔄 模式切换

### **QHOVER模式**
```cpp
// mode_qhover.cpp
float angle_error = current_yaw - target_yaw;
plane.stabilize_vtol_yaw(angle_error);  // 角度控制
```

### **QSTABILIZE模式**
```cpp
// mode_qstabilize.cpp
float desired_rate = rudder_input × max_rate;
plane.stabilize_vtol_yaw_rate(desired_rate);  // 角速度控制
```

---

## 💡 优化建议

### **1. 添加死区**
```cpp
// 避免小抖动
if (fabsf(rudder_input) < 0.05f) {
    desired_yaw_rate_dps = 0.0f;
}
```

### **2. 速率曲线**
```cpp
// 非线性响应，低速精细，高速快速
float expo = 0.5f;  // 指数曲线
desired_yaw_rate_dps = rudder_input × (1 - expo) + 
                       rudder_input³ × expo;
```

### **3. 动态限速**
```cpp
// 根据倾转角度调整最大角速度
float tilt_factor = constrain_float(tilt_angle / 90.0f, 0.3f, 1.0f);
max_rate = 90.0f × tilt_factor;
```

---

## 🧪 测试步骤

### **地面测试**
```
1. 连接地面站
2. 进入QSTABILIZE模式
3. 打方向舵
4. 观察GCS输出：
   - desired 应该响应方向舵
   - current 应该跟踪 desired
   - err 应该较小
```

### **悬停测试**
```
1. 起飞悬停
2. 缓慢打方向舵
3. 观察：
   - 飞机是否平滑转动
   - 松杆是否立即停转
   - 无明显抖动
```

### **动态测试**
```
1. 快速左右打舵
2. 观察：
   - 响应是否快速
   - 是否超调
   - 是否振荡
```

---

## ⚠️ 注意事项

### **1. 不保持角度**
- 松杆后飞机会停止转动
- 但不会保持当前角度
- 可能有小漂移

### **2. 风扰动**
- 侧风会导致角度漂移
- 需要持续修正方向舵
- 建议大风天使用角度控制

### **3. PID状态**
- 两个函数有独立的PID状态
- 切换模式时积分项会保留
- 建议切换后等待1秒稳定

---

## 📝 总结

### **优点**
✅ 响应快速，无延迟  
✅ 手感直观，易控制  
✅ 适合特技飞行  
✅ 代码简洁

### **缺点**
❌ 不保持角度  
❌ 需要持续操作  
❌ 风扰动影响大

### **推荐使用场景**
- ⭐ QSTABILIZE模式手动飞行
- ⭐ 特技动作
- ⭐ 快速机动

### **不推荐场景**
- ❌ 自动悬停（用角度控制）
- ❌ GPS导航（用角度控制）
- ❌ 大风环境（用角度控制）

---

**版本**: 1.0  
**日期**: 2026-05-30  
**作者**: VTOL Team  
**修改**: 新增纯角速度控制模式
