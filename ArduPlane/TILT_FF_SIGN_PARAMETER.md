# Q_TILT_FF_SIGN 参数说明

## 📋 新增参数

### **Q_TILT_FF_SIGN**

**参数信息**：
```
参数名：Q_TILT_FF_SIGN
索引：51
默认值：1.0
可选值：-1.0 (反向), 1.0 (正向)
用户级别：Advanced
```

**说明**：
控制倾转电机前馈输出的符号方向。用于调整角度前馈对俯仰差分输出的影响方向。

---

## 🎯 参数作用

### **控制流程**
```
角度前馈输出 (feedforward_output)
         ↓
    × feedforward_sign (±1.0)
         ↓
    × pitch_range
         ↓
feedforward_diff → 添加到 pitch_diff
         ↓
    左右电机差分输出
```

### **代码实现**
```cpp
// tiltrotor.cpp:1607
float feedforward_diff = feedforward_sign * feedforward_output * pitch_range;
pitch_diff += feedforward_diff;
```

---

## 🔧 参数值说明

### **1.0 (正向 - 默认)**
```
feedforward_output = 0.5
feedforward_sign = 1.0
feedforward_diff = 1.0 × 0.5 × pitch_range = 0.5 × pitch_range

效果：前馈输出按原始方向作用
```

### **-1.0 (反向)**
```
feedforward_output = 0.5
feedforward_sign = -1.0
feedforward_diff = -1.0 × 0.5 × pitch_range = -0.5 × pitch_range

效果：前馈输出反向作用
```

---

## 💡 使用场景

### **场景1：默认配置（正向）**
```
Q_TILT_FF_SIGN = 1.0

适用：
- 标准倾转配置
- 前馈方向正确
- 控制响应符合预期
```

### **场景2：反向配置**
```
Q_TILT_FF_SIGN = -1.0

适用：
- 电机安装方向相反
- 前馈作用方向需要反转
- 调试发现前馈方向错误
```

---

## 🧪 如何判断需要反向

### **测试方法**

#### **1. 观察前馈响应**
```
操作：快速改变倾转角度目标值
观察：飞机俯仰响应

正确方向：
- 向前倾转 → 机头下压
- 向后倾转 → 机头上抬

错误方向：
- 向前倾转 → 机头上抬 ❌
- 向后倾转 → 机头下压 ❌

解决：设置 Q_TILT_FF_SIGN = -1.0
```

#### **2. 检查GCS日志**
```
查看：feedforward_output 和 pitch_diff

正常：
- feedforward_output > 0 → pitch_diff 增加
- feedforward_output < 0 → pitch_diff 减小

异常：
- feedforward_output > 0 → pitch_diff 减小 ❌
- feedforward_output < 0 → pitch_diff 增加 ❌

解决：设置 Q_TILT_FF_SIGN = -1.0
```

---

## 📊 参数影响

### **对系统的影响**

| 参数值 | 前馈方向 | 俯仰响应 | 适用场景 |
|--------|---------|---------|---------|
| 1.0 | 正向 | 正常 | 标准配置 |
| -1.0 | 反向 | 反转 | 特殊配置 |

---

### **与其他参数的关系**

```
前馈输出计算链：
1. 角度误差 → PID控制器 → feedforward_output
2. feedforward_output × feedforward_sign → 带符号的前馈
3. 带符号的前馈 × pitch_range → feedforward_diff
4. feedforward_diff + pitch_diff → 最终差分输出
```

---

## ⚙️ 地面站配置

### **Mission Planner**
```
1. Config/Tuning → Full Parameter List
2. 搜索：Q_TILT_FF_SIGN
3. 选择值：
   - 1.0 (Normal)
   - -1.0 (Reversed)
4. Write Params
```

### **QGroundControl**
```
1. Vehicle Setup → Parameters
2. 搜索：Q_TILT_FF_SIGN
3. 修改并保存
```

---

## 🔍 调试建议

### **步骤1：默认测试**
```
1. 保持默认值 Q_TILT_FF_SIGN = 1.0
2. 悬停测试
3. 观察倾转角度变化时的俯仰响应
```

### **步骤2：判断方向**
```
如果发现：
- 倾转前馈导致俯仰反向
- 控制不稳定
- 响应与预期相反

则：设置 Q_TILT_FF_SIGN = -1.0
```

### **步骤3：验证**
```
1. 修改参数后重新测试
2. 确认俯仰响应正确
3. 检查控制稳定性
```

---

## 📝 代码修改记录

### **tiltrotor.h**
```cpp
// 添加成员变量
AP_Float feedforward_sign;  // 前馈输出符号控制 (1.0=正向, -1.0=反向)
```

### **tiltrotor.cpp - 参数注册**
```cpp
// @Param: FF_SIGN
// @DisplayName: Feedforward sign control
// @Description: Controls the sign of feedforward output for pitch differential
// @Values: -1.0:Reversed, 1.0:Normal
// @User: Advanced
AP_GROUPINFO("FF_SIGN", 51, Tiltrotor, feedforward_sign, 1.0),
```

### **tiltrotor.cpp - 使用**
```cpp
// 添加角度前馈输出到俯仰差分
float feedforward_diff = feedforward_sign * feedforward_output * pitch_range;
pitch_diff += feedforward_diff;
```

---

## ⚠️ 注意事项

### **1. 仅影响前馈**
```
此参数只影响角度前馈输出
不影响：
- PID控制输出
- 偏航控制
- 速度前馈
```

### **2. 默认值安全**
```
默认值 1.0 适用于大多数配置
只在确认需要时才修改为 -1.0
```

### **3. 测试建议**
```
修改此参数后：
1. 先在地面测试
2. 低空悬停验证
3. 确认稳定后再进行其他飞行
```

---

## 🔄 故障排除

### **问题1：倾转时俯仰振荡**
```
可能原因：前馈方向错误
解决：尝试反转 Q_TILT_FF_SIGN
```

### **问题2：倾转响应迟钝**
```
可能原因：前馈被抵消
检查：Q_TILT_FF_SIGN 是否正确
```

### **问题3：修改参数无效**
```
检查：
1. 参数是否成功写入
2. 飞控是否重启
3. 是否在正确的飞行模式
```

---

## 📈 典型配置

### **标准双倾转配置**
```
Q_TILT_FF_SIGN = 1.0

电机配置：
- 左电机：正向安装
- 右电机：正向安装
- 前馈：正向作用
```

### **特殊配置**
```
Q_TILT_FF_SIGN = -1.0

可能原因：
- 电机安装方向特殊
- 控制逻辑定制
- 机架设计特殊
```

---

## 🎯 快速参考

| 现象 | 原因 | 解决方案 |
|------|------|---------|
| 倾转前倾，机头上抬 | 前馈反向 | Q_TILT_FF_SIGN = -1.0 |
| 倾转后倾，机头下压 | 前馈反向 | Q_TILT_FF_SIGN = -1.0 |
| 倾转时俯仰振荡 | 前馈方向错误 | 反转符号 |
| 响应正常 | 配置正确 | 保持 1.0 |

---

## 🔧 相关参数

### **倾转控制参数组**
```
Q_TILT_FF_SIGN      - 前馈符号控制 ⭐
Q_TILT_TS_ANGLE_P   - 角度P增益
Q_TILT_TS_ANGLE_I   - 角度I增益
Q_TILT_TS_ANGLE_D   - 角度D增益
Q_TILT_TS_ANGLE_IMAX - 积分限幅
Q_TILT_TS_CORR_MAX  - PID修正最大值
Q_TILT_RT           - 角度变化率
```

---

## 📊 测试检查表

### **参数修改前**
- [ ] 记录当前配置
- [ ] 备份参数文件
- [ ] 了解预期效果

### **参数修改后**
- [ ] 验证参数已写入
- [ ] 地面测试电机响应
- [ ] 低空悬停验证
- [ ] 记录测试结果

### **飞行验证**
- [ ] 悬停稳定性
- [ ] 倾转角度变化响应
- [ ] 俯仰控制正常
- [ ] 无异常振荡

---

**版本**: 1.0  
**日期**: 2026-05-31  
**参数索引**: 51  
**默认值**: 1.0
