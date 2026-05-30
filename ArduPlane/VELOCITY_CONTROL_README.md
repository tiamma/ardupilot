# 双旋翼速度控制实现说明

## 📋 概述

在垂直飞行模式下，新增了基于EKF速度反馈的PID控制，通过遥控器通道2控制前后速度。

## 🎯 控制架构

```
遥控器CH2 → 期望速度 → [速度PID] → 归一化前馈 ──┐
                                              ├→ 电机差分 → 电机输出
导航指令 → [角度PID] → [角速度PID] → 电机差分 ──┘
```

### 前馈 + 双环串级控制

1. **速度控制环（前馈路径）**
   - 输入：遥控器通道2 + EKF速度
   - 输出：归一化前馈量 [-1, 1]
   - **直接叠加到电机差分，绕过角度环**

2. **角度控制环（稳定路径）**
   - 输入：导航指令（通常为0，保持悬停）
   - 输出：期望角速度（度/秒）

3. **角速度控制环（内环）**
   - 输入：期望角速度 + 当前角速度
   - 输出：电机差分量

### 优势
- **响应快速**：速度控制直接作用于电机，无角度环延迟
- **稳定性好**：角度环仍然工作，提供姿态稳定
- **解耦控制**：速度控制和姿态控制独立工作

---

## ⚙️ 新增参数

| 参数名 | 默认值 | 单位 | 说明 |
|--------|--------|------|------|
| `Q_TILT_VEL_P` | 0.5 | - | 速度P增益 |
| `Q_TILT_VEL_I` | 0.1 | - | 速度I增益 |
| `Q_TILT_VEL_D` | 0.05 | - | 速度D增益 |
| `Q_TILT_VEL_IM` | 5.0 | m/s | 速度积分限幅 |
| `Q_TILT_VEL_MAX` | 2.0 | m/s | 最大期望速度 |
| `Q_TILT_VEL_P_M` | 15.0 | 度 | 速度控制最大俯仰角 |
| `Q_TILT_VEL_P_S` | 1 | - | 速度俯仰方向（1=正向，-1=反向） |

---

## 🔧 实现细节

### 1. 速度获取
```cpp
// 使用EKF融合速度（GPS + IMU + 气压计）
if (plane.ahrs.get_velocity_NED(velocity_ned)) {
    Vector3f velocity_body = plane.ahrs.get_rotation_body_to_ned().transposed() * velocity_ned;
    current_velocity_mps = velocity_body.x;  // 机体前向速度
}
```

### 2. 遥控器映射
```cpp
// 遥控器通道2归一化输入 [-1, 1]
rc_pitch_input = plane.channel_pitch->norm_input();

// 死区处理（±5%）
if (fabsf(rc_pitch_input) < 0.05f) {
    rc_pitch_input = 0.0f;
}

// 映射到期望速度
desired_velocity_mps = rc_pitch_input * bicopter_max_vel_mps;
```

### 3. PID计算
```cpp
velocity_error = desired_velocity_mps - current_velocity_mps;

// P项
vel_p = bicopter_vel_p * velocity_error;

// I项（带抗饱和和衰减）
bicopter_vel_integral += velocity_error * dt_s;
if (fabsf(velocity_error) < 0.1f) {
    bicopter_vel_integral *= 0.95f;  // 小误差衰减
}

// D项
vel_d = bicopter_vel_d * (velocity_error - last_vel_error) / dt_s;

// 输出限幅（俯仰角）
velocity_pitch_cmd = constrain(vel_p + vel_i + vel_d, -15°, +15°);

// 归一化为前馈量 [-1, 1]
velocity_feedforward = velocity_pitch_cmd / bicopter_max_vel_pitch_deg;
```

### 4. 前馈输出
```cpp
// 速度前馈直接叠加到电机差分
velocity_feedforward_diff = velocity_feedforward * pitch_range;
pitch_diff += velocity_feedforward_diff;

// 最终电机输出
left_motor = base_output + pitch_diff - yaw_diff;
right_motor = base_output + pitch_diff + yaw_diff;
```

### 4. 积分抗饱和策略
- **误差 < 0.1 m/s**：积分项衰减至95%
- **误差 < 0.2 m/s**：积分项衰减至98%
- **误差 ≥ 0.2 m/s**：正常累加
- **速度无效时**：积分项清零

---

## 📊 调试参数建议

### 初始调试步骤

1. **仅P控制**（I=0, D=0）
   ```
   Q_TILT_VEL_P = 0.5
   Q_TILT_VEL_I = 0
   Q_TILT_VEL_D = 0
   ```
   观察响应速度，如果太慢增加P，太快减小P

2. **添加I控制**
   ```
   Q_TILT_VEL_I = 0.1
   ```
   消除稳态误差，如果振荡减小I

3. **添加D控制**
   ```
   Q_TILT_VEL_D = 0.05
   ```
   抑制超调和振荡

### 参数调整指南

| 现象 | 可能原因 | 调整方法 |
|------|----------|----------|
| 响应太慢 | P增益太小 | 增加 `VEL_P` |
| 振荡/超调 | P增益太大 | 减小 `VEL_P` |
| 稳态误差 | I增益太小 | 增加 `VEL_I` |
| 低频振荡 | I增益太大 | 减小 `VEL_I` |
| 高频抖动 | D增益太大 | 减小 `VEL_D` |
| 速度过快 | 最大速度太大 | 减小 `VEL_MAX` |
| 姿态过激 | 俯仰角限幅太大 | 减小 `VEL_P_M` |

---

## 🔍 日志变量

新增以下日志变量用于调试：

- `log_velocity_error`：速度误差（m/s）
- `log_current_velocity`：当前速度（m/s）
- `log_desired_velocity`：期望速度（m/s）
- `log_velocity_pitch_cmd`：速度控制输出的俯仰角（度）

---

## ⚠️ 注意事项

### 1. EKF健康检查
- 速度数据来自EKF，需要GPS定位良好
- 室内或GPS信号差时，速度控制可能失效
- 速度无效时自动清空积分项，防止失控

### 2. 遥控器失控保护
- 无有效遥控输入时，期望速度自动设为0
- 飞机会尝试悬停（速度趋于0）

### 3. 坐标系说明
- **机体坐标系X轴**：机头方向为正（前进）
- **遥控器前推**：正输入，期望前飞
- **遥控器后拉**：负输入，期望后退

### 4. 方向配置（重要）⭐
- **`VEL_P_S = 1`**（默认）：遥控器前推 → 前飞（正常）
- **`VEL_P_S = -1`**：遥控器前推 → 后退（反向）
- 如果测试时发现速度控制方向相反，修改此参数即可
- 类似于 `left_pitch_sign` 和 `right_pitch_sign` 的作用

### 5. 参数限制
- 速度控制输出的俯仰角限制在 `±VEL_P_M`
- 防止速度控制要求过大的姿态角度
- 默认±15度，可根据飞机性能调整

---

## 🧪 测试流程

### 地面测试
1. 连接地面站，监控速度参数
2. 检查EKF速度是否有效
3. 手动移动飞机，观察速度反馈
4. 调整遥控器，观察期望速度变化

### 悬停测试
1. 起飞到安全高度（5-10米）
2. 遥控器通道2保持中位
3. 观察飞机是否能稳定悬停
4. 记录速度误差和俯仰角

### 速度控制测试
1. 缓慢前推遥控器
2. 观察飞机是否平稳加速
3. 松开遥控器回中位
4. 观察飞机是否能减速悬停
5. 测试后拉遥控器（后退）

### 参数调优
1. 记录飞行日志
2. 分析速度响应曲线
3. 根据调试指南调整参数
4. 重复测试直到满意

---

## 📝 代码修改总结

### 修改文件
1. `tiltrotor.h`：添加速度控制参数和状态变量
2. `tiltrotor.cpp`：
   - 添加参数定义（40-45号参数）
   - 构造函数初始化状态变量
   - `vtol_pid_get_rate`函数添加速度控制逻辑

### 新增成员变量
```cpp
// 参数
AP_Float bicopter_vel_p;
AP_Float bicopter_vel_i;
AP_Float bicopter_vel_d;
AP_Float bicopter_vel_imax;
AP_Float bicopter_max_vel_mps;
AP_Float bicopter_max_vel_pitch_deg;

// 状态变量
float bicopter_vel_integral;
float bicopter_last_vel_error;

// 日志变量
float log_velocity_error;
float log_current_velocity;
float log_desired_velocity;
float log_velocity_pitch_cmd;
```

---

## 🚀 后续优化建议

1. **速度滤波**：对EKF速度进行低通滤波，减少噪声
2. **自适应增益**：根据速度大小动态调整PID增益
3. **前馈控制**：根据遥控器输入直接计算前馈俯仰角
4. **降级模式**：GPS失效时切换到加速度积分（需要定期重置）
5. **速度限制**：根据电池电量动态调整最大速度
6. **日志优化**：添加完整的速度控制日志消息

---

## 📞 问题排查

### 速度控制不工作
- 检查 `Q_TILT_VEL_P` 是否为0
- 检查EKF是否健康（`EKF_STATUS`）
- 检查GPS是否定位（`GPS_STATUS`）

### 飞机漂移
- 增加 `VEL_I` 消除稳态误差
- 检查风速是否过大
- 检查IMU校准是否正确

### 振荡
- 减小 `VEL_P` 和 `VEL_D`
- 检查角度环和角速度环参数是否合理
- 检查机架是否刚性足够

---

**版本**: 1.0  
**日期**: 2026-05-25  
**作者**: Cascade AI Assistant
