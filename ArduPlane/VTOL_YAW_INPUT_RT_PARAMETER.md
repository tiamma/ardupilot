# VTOL_YAW_INPUT_RT 参数说明

## 📋 新增参数

### **VTOL_YAW_INPUT_RT**

**参数信息**：
```
参数名：VTOL_YAW_INPUT_RT
索引：45
默认值：90.0 deg/s
范围：10 - 360 deg/s
单位：deg/s（度/秒）
增量：5
用户级别：Standard
```

**说明**：
定义VTOL模式下方向舵满杆时的最大偏航角速度。这个参数控制飞机在垂直起降模式下的偏航响应速度。

---

## 🎯 为什么添加这个参数

### **替代原有方案**

#### **修改前**
```cpp
// 使用QuadPlane的通用参数
const float desired_yaw_rate_dps = rudder_input * quadplane.command_model_pilot.get_rate();
```

**问题**：
- ❌ 参数名称不直观（Q_A_RAT_YAW_RATE）
- ❌ 与其他模式共享，调整会影响其他功能
- ❌ 不是专门为VTOL偏航设计

---

#### **修改后**
```cpp
// 使用专用的VTOL偏航输入参数
const float desired_yaw_rate_dps = rudder_input * plane.g2.vtol_yaw_input_rate;
```

**优点**：
- ✅ 参数名称清晰（VTOL_YAW_INPUT_RT）
- ✅ 独立控制，不影响其他功能
- ✅ 专门为VTOL偏航设计
- ✅ 与其他VTOL_YAW_*参数一致

---

## 📊 参数作用

### **控制流程**
```
方向舵输入 → rudder_input [-1, 1]
                    ↓
         × VTOL_YAW_INPUT_RT (90 deg/s)
                    ↓
         desired_yaw_rate_dps
                    ↓
              PID控制器
                    ↓
              副翼输出
```

### **计算示例**

| 方向舵位置 | rudder_input | VTOL_YAW_INPUT_RT | 期望角速度 |
|-----------|--------------|-------------------|-----------|
| 中位 | 0.0 | 90 | 0°/s |
| 右满杆 | +1.0 | 90 | +90°/s |
| 左满杆 | -1.0 | 90 | -90°/s |
| 右半杆 | +0.5 | 90 | +45°/s |
| 右1/4杆 | +0.25 | 90 | +22.5°/s |

---

## ⚙️ 使用场景

### **QSTABILIZE模式**
```cpp
// mode_qstabilize.cpp
const float desired_yaw_rate_dps = rudder_input * plane.g2.vtol_yaw_input_rate;
plane.stabilize_vtol_yaw_rate(desired_yaw_rate_dps);
```

**用途**：手动控制偏航角速度

---

### **QHOVER模式**
```cpp
// mode_qhover.cpp
const float yaw_rate_dps = rudder_input * plane.g2.vtol_yaw_input_rate;
// 累积到目标角度
max_change = yaw_rate_dps * dt;
```

**用途**：通过角速度累积控制目标角度

---

## 🔧 调参指南

### **默认值（90°/s）**
```
满杆转一圈：360° / 90°/s = 4秒
适合：大多数VTOL飞机
```

### **增加速率（120°/s）**
```
满杆转一圈：360° / 120°/s = 3秒
适合：
- 小型敏捷飞机
- 特技飞行
- 需要快速响应
```

### **减小速率（60°/s）**
```
满杆转一圈：360° / 60°/s = 6秒
适合：
- 大型稳定飞机
- 载重飞行
- 新手练习
```

### **极限值**

#### **最小值（10°/s）**
```
满杆转一圈：36秒
用途：超稳定模式，新手训练
```

#### **最大值（360°/s）**
```
满杆转一圈：1秒
用途：极限特技，专业飞手
```

---

## 📈 与其他参数的关系

### **VTOL偏航参数族**

| 参数 | 作用 | 默认值 |
|------|------|--------|
| `VTOL_YAW_INPUT_RT` | 方向舵输入最大角速度 ⭐ | 90 deg/s |
| `VTOL_YAW_ANG_P` | 角度环P增益 | 2.0 |
| `VTOL_YAW_RT_P` | 角速度环P增益 | 0.5 |
| `VTOL_YAW_RT_I` | 角速度环I增益 | 0.1 |
| `VTOL_YAW_RT_D` | 角速度环D增益 | 0.02 |
| `VTOL_YAW_RT_IMAX` | 积分限幅 | 10.0 |
| `VTOL_YAW_RT_MAX` | 最大角速度（未使用） | 90.0 |

---

### **参数协同**

#### **VTOL_YAW_INPUT_RT vs VTOL_YAW_RT_MAX**
```
VTOL_YAW_INPUT_RT：方向舵输入的最大角速度
VTOL_YAW_RT_MAX：  角度环输出的最大角速度（QHOVER模式）

建议：设置为相同值（90°/s）
```

---

## 🎮 地面站配置

### **Mission Planner**
```
1. Config/Tuning → Full Parameter List
2. 搜索：VTOL_YAW_INPUT_RT
3. 修改值（10 - 360）
4. Write Params
```

### **QGroundControl**
```
1. Vehicle Setup → Parameters
2. 搜索：VTOL_YAW_INPUT_RT
3. 修改并保存
```

---

## 🧪 测试方法

### **地面测试**
```
1. 连接地面站
2. 进入QSTABILIZE模式
3. 打方向舵满杆
4. 观察GCS输出：
   VTOL_YAW_RATE: desired=90.00 ...
   
期望值应该等于 VTOL_YAW_INPUT_RT
```

### **悬停测试**
```
1. 起飞悬停
2. 打满舵，计时转360°
3. 计算实际角速度：
   实际速度 = 360° / 时间(秒)
   
理想：实际速度 ≈ VTOL_YAW_INPUT_RT
```

---

## 💡 调参建议

### **场景1：转速太慢**
```
现象：满杆转动慢，不够灵活
原因：VTOL_YAW_INPUT_RT 太小
解决：增加参数值

VTOL_YAW_INPUT_RT = 120  (从90增加)
```

### **场景2：转速太快**
```
现象：满杆转动太快，难以控制
原因：VTOL_YAW_INPUT_RT 太大
解决：减小参数值

VTOL_YAW_INPUT_RT = 60  (从90减小)
```

### **场景3：精细控制**
```
现象：需要更精细的偏航控制
原因：角速度范围太大
解决：减小参数值，增加控制精度

VTOL_YAW_INPUT_RT = 45  (从90减半)
```

### **场景4：快速机动**
```
现象：需要快速转向
原因：角速度不够
解决：增加参数值

VTOL_YAW_INPUT_RT = 180  (从90加倍)
```

---

## 📝 代码修改记录

### **Parameters.h**
```cpp
// 添加成员变量
AP_Float vtol_yaw_input_rate;   // 方向舵输入最大角速度 (deg/s)
```

### **Parameters.cpp**
```cpp
// 注册参数
AP_GROUPINFO("VTOL_YAW_INPUT_RT", 45, ParametersG2, vtol_yaw_input_rate, 90.0),
```

### **mode_qstabilize.cpp**
```cpp
// 修改前
const float desired_yaw_rate_dps = rudder_input * quadplane.command_model_pilot.get_rate();

// 修改后
const float desired_yaw_rate_dps = rudder_input * plane.g2.vtol_yaw_input_rate;
```

### **mode_qhover.cpp**
```cpp
// 修改前
const float yaw_rate_dps = rudder_input * quadplane.command_model_pilot.get_rate();

// 修改后
const float yaw_rate_dps = rudder_input * plane.g2.vtol_yaw_input_rate;
```

---

## ⚠️ 注意事项

### **1. 参数范围**
```
最小值：10 deg/s  （非常慢，超稳定）
最大值：360 deg/s （非常快，需谨慎）
推荐值：60 - 120 deg/s
```

### **2. 与PID配合**
```
VTOL_YAW_INPUT_RT 定义期望值
VTOL_YAW_RT_P/I/D 控制跟踪性能

两者需要配合调整
```

### **3. 安全测试**
```
首次修改：
1. 从默认值90开始
2. 每次调整±10
3. 悬停测试稳定性
4. 逐步调整到理想值
```

---

## 🔄 版本历史

**v1.0 - 2026-05-31**
- 新增 VTOL_YAW_INPUT_RT 参数
- 替代 quadplane.command_model_pilot.get_rate()
- 应用于 QSTABILIZE 和 QHOVER 模式
- 默认值设置为 90 deg/s

---

**作者**: VTOL Team  
**日期**: 2026-05-31  
**参数索引**: 45
