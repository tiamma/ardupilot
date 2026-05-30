# 目标倾转角度持久化说明

## 📋 问题描述

**修改前**：
```cpp
void Tiltrotor::bicopter_update() {
    float target_tilt_angle = 0.0f;  // ❌ 每次调用都重置为0
    
    if (ch12_high) {
        // 动态调整角度
        manual_target_tilt_angle += angle_rate * dt_s;
        target_tilt_angle = manual_target_tilt_angle;
    } else {
        // 六段开关
        target_tilt_angle = 根据CH9选择;
    }
}
```

**问题**：
- `target_tilt_angle` 是局部变量
- 每次函数调用都重置为0
- 无法保持上次的角度值

---

## ✅ 解决方案

将 `target_tilt_angle` 改为**成员变量**，在构造函数中初始化为0，之后保持其值。

---

## 🔧 实现细节

### **1. 头文件修改（tiltrotor.h:180）**

```cpp
// 通道12控制目标角度状态变量
float manual_target_tilt_angle;      // 手动控制的目标倾转角度
uint32_t last_tilt_angle_update_ms;  // 上次角度更新时间
float target_tilt_angle;             // 当前目标倾转角度（度）⭐ 新增
```

**说明**：
- 作为私有成员变量
- 在多次函数调用之间保持值
- 单位：度（degrees）

---

### **2. 构造函数初始化（tiltrotor.cpp:404）**

```cpp
Tiltrotor::Tiltrotor(...) {
    // 初始化目标角度控制变量
    manual_target_tilt_angle = 0.0f;
    last_tilt_angle_update_ms = 0;
    target_tilt_angle = 0.0f;  // ⭐ 只在初始化时设置为0
}
```

**说明**：
- 只在对象创建时初始化一次
- 之后不会自动重置

---

### **3. 使用方式（tiltrotor.cpp:1559）**

```cpp
void Tiltrotor::bicopter_update() {
    // 获取通道12状态
    int ch12_value = RC_Channels::get_radio_in(11);
    bool ch12_high = (ch12_value > 1500);
    
    // target_tilt_angle 现在是成员变量，不需要每次重置为0 ⭐
    
    if (ch12_high) {
        // 动态调整：累加变化
        manual_target_tilt_angle += angle_rate * dt_s;
        target_tilt_angle = manual_target_tilt_angle;
    } else {
        // 六段开关：直接设置
        target_tilt_angle = 根据CH9选择的角度;
    }
    
    // 保存到全局变量
    plane.tilt_angle_cd = target_tilt_angle * 100.0f;
}
```

---

## 📊 行为对比

### **修改前（局部变量）**

| 调用次数 | CH12状态 | CH9位置 | manual_target | target_tilt | 说明 |
|---------|---------|---------|--------------|-------------|------|
| 1 | 低位 | 位置3 | 0 | 36° | 从0开始 |
| 2 | 低位 | 位置4 | 0 | 54° | 从0开始 |
| 3 | 高位 | - | 0→0.5 | 0.5° | ❌ 从0开始 |
| 4 | 高位 | - | 0.5→1.0 | 1.0° | ❌ 重新累加 |

**问题**：切换到CH12高位时，角度从0开始累加，而不是从当前角度。

---

### **修改后（成员变量）**

| 调用次数 | CH12状态 | CH9位置 | manual_target | target_tilt | 说明 |
|---------|---------|---------|--------------|-------------|------|
| 1 | 低位 | 位置3 | 0 | 36° | 初始化 |
| 2 | 低位 | 位置4 | 36 | 54° | 同步到54° |
| 3 | 高位 | - | 54→54.5 | 54.5° | ✅ 从54°开始 |
| 4 | 高位 | - | 54.5→55.0 | 55.0° | ✅ 平滑累加 |

**优势**：切换到CH12高位时，从当前角度开始调整，平滑过渡。

---

## 🎯 关键优势

### **1. 角度连续性**
```
六段开关（54°）→ 切换CH12高位 → 从54°开始调整
而不是从0°重新开始
```

### **2. 平滑过渡**
```
CH12低位 → 高位：角度保持
CH12高位 → 低位：同步到六段开关位置
```

### **3. 状态保持**
```
即使函数多次调用，角度值始终保持
不会意外重置
```

---

## 🔍 代码流程

### **初始化阶段**
```cpp
// 构造函数（只执行一次）
Tiltrotor::Tiltrotor() {
    target_tilt_angle = 0.0f;  // 初始化为0
}
```

### **运行阶段（每次循环）**
```cpp
void bicopter_update() {
    // 1. 读取通道12状态
    bool ch12_high = ...;
    
    // 2. 根据模式更新角度
    if (ch12_high) {
        // 动态模式：累加变化
        target_tilt_angle = manual_target_tilt_angle;
    } else {
        // 六段开关：直接设置
        target_tilt_angle = ch9_angle;
        manual_target_tilt_angle = target_tilt_angle;  // 同步
    }
    
    // 3. 保存到全局变量
    plane.tilt_angle_cd = target_tilt_angle * 100.0f;
}
```

---

## 📝 变量关系

### **三个角度变量**

| 变量名 | 类型 | 作用域 | 用途 |
|--------|------|--------|------|
| `manual_target_tilt_angle` | 成员变量 | Tiltrotor类 | CH12高位时的手动调整角度 |
| `target_tilt_angle` | 成员变量 | Tiltrotor类 | 当前目标角度（统一） |
| `plane.tilt_angle_cd` | 全局变量 | Plane类 | 供其他模块使用（centidegrees） |

### **数据流**

```
CH12高位：
  manual_target_tilt_angle (累加) 
    → target_tilt_angle 
      → plane.tilt_angle_cd

CH12低位：
  CH9六段开关 
    → target_tilt_angle 
      → manual_target_tilt_angle (同步)
        → plane.tilt_angle_cd
```

---

## ⚠️ 注意事项

### **1. 初始化时机**
- 只在构造函数中初始化为0
- 不要在其他地方重置

### **2. 模式切换同步**
```cpp
// CH12低位时，同步manual_target_tilt_angle
manual_target_tilt_angle = target_tilt_angle;
```
确保切换到CH12高位时，从当前角度开始。

### **3. 单位一致性**
- `target_tilt_angle`：度（degrees）
- `plane.tilt_angle_cd`：centidegrees（度×100）
- 转换：`plane.tilt_angle_cd = target_tilt_angle * 100.0f`

---

## 🧪 测试验证

### **测试1：角度保持**
```
1. CH12低位，CH9位置4（54°）
2. 观察 target_tilt_angle = 54°
3. 多次循环后，仍然是54°
4. ✅ 角度保持不变
```

### **测试2：平滑切换**
```
1. CH12低位，CH9位置4（54°）
2. 切换CH12高位
3. 观察 manual_target_tilt_angle = 54°
4. 前推CH2，角度从54°开始增加
5. ✅ 平滑过渡，无跳变
```

### **测试3：双向同步**
```
1. CH12高位，调整到60°
2. 切换CH12低位，CH9位置5（72°）
3. 观察 target_tilt_angle = 72°
4. manual_target_tilt_angle = 72°（同步）
5. 再切换CH12高位
6. 角度从72°开始调整
7. ✅ 双向同步正常
```

---

## 📈 性能影响

### **内存**
- 增加：4字节（1个float）
- 影响：可忽略

### **CPU**
- 无额外计算
- 影响：无

### **可靠性**
- 提升：避免意外重置
- 提升：平滑过渡

---

## 🔧 故障排查

### **问题1：角度意外重置为0**
**原因**：可能在其他地方重置了 `target_tilt_angle`

**检查**：
```bash
grep -n "target_tilt_angle = 0" tiltrotor.cpp
```
应该只在构造函数中出现。

### **问题2：切换模式时角度跳变**
**原因**：`manual_target_tilt_angle` 未同步

**检查**：
```cpp
// CH12低位时，必须同步
manual_target_tilt_angle = target_tilt_angle;
```

### **问题3：角度不更新**
**原因**：可能误用了局部变量

**检查**：
```cpp
// ❌ 错误：声明了同名局部变量
float target_tilt_angle = 0.0f;

// ✅ 正确：直接使用成员变量
// target_tilt_angle（无声明）
```

---

## 📚 相关文件

1. **tiltrotor.h:180**
   - 成员变量声明

2. **tiltrotor.cpp:404**
   - 构造函数初始化

3. **tiltrotor.cpp:1559**
   - 使用成员变量（移除局部声明）

4. **tiltrotor.cpp:1600**
   - 保存到全局变量

---

## 💡 最佳实践

### **成员变量 vs 局部变量**

**使用成员变量**：
- 需要在多次调用之间保持状态
- 需要在不同函数之间共享
- 例如：累加器、状态标志、历史值

**使用局部变量**：
- 只在单次调用内使用
- 不需要保持状态
- 例如：临时计算、循环变量

### **初始化原则**

1. **成员变量**：在构造函数中初始化
2. **局部变量**：在声明时初始化
3. **全局变量**：避免使用（或在启动时初始化）

---

**版本**: 1.0  
**日期**: 2026-05-25  
**修改**: 将 target_tilt_angle 从局部变量改为成员变量
