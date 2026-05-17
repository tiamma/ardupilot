# BMI088通过遥控通道5触发初始化

## 功能说明

如果BMI088传感器在启动时未成功初始化，可以通过遥控器通道5的高位信号手动触发初始化。

## 工作原理

### 初始化逻辑

```cpp
// BMI088初始化逻辑：如果未初始化成功，通过遥控通道5高位触发初始化（仅执行一次）
static bool bmi088_init_attempted = false;
if (!bmi088_initialized && !bmi088_init_attempted) {
    // 读取遥控通道5 (索引4，从0开始)
    RC_Channel *ch5 = RC_Channels::rc_channel(4);
    if (ch5 != nullptr && ch5->get_radio_in() > 1700) {
        // 通道5高位信号，触发初始化
        if (bmi088_init()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BMI088 initialized via CH5");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "BMI088 init failed via CH5");
        }
        bmi088_init_attempted = true;  // 标记已尝试初始化，不再重复
    }
}
```

### 触发条件

需要同时满足以下条件：

1. **BMI088未初始化**: `!bmi088_initialized`
2. **未尝试过初始化**: `!bmi088_init_attempted`
3. **通道5高位**: `ch5->get_radio_in() > 1700`

### 执行次数

**仅执行一次**: 使用静态变量 `bmi088_init_attempted` 确保初始化只尝试一次，避免重复初始化。

## 使用方法

### 步骤1: 检查初始化状态

启动飞控后，查看QGC消息：

```
情况1: 自动初始化成功
  "BMI088 initialized on I2C2"
  "BMI088: Init OK (Accel:0x1E Gyro:0x0F)"
  → 无需手动触发

情况2: 自动初始化失败
  "BMI088 init failed"
  "BMI088: Failed to get accel I2C device"
  → 需要手动触发
```

### 步骤2: 手动触发初始化

**操作**:
1. 将遥控器通道5拨到高位 (PWM > 1700)
2. 保持1-2秒
3. 观察QGC消息

**成功消息**:
```
"BMI088 initialized via CH5"
```

**失败消息**:
```
"BMI088 init failed via CH5"
```

### 步骤3: 验证初始化

查看调试输出中的 `bmi` 参数：

```
Bi bmi:1 am:45.00 fd:0.12 ta:90.00 PE:2.3 PD:0.15 L:512 R:488
    ↑
   初始化成功 (1)
```

## 遥控器设置

### 通道5 PWM范围

| 位置 | PWM值 | 说明 |
|------|-------|------|
| **低位** | < 1700 | 不触发初始化 |
| **高位** | > 1700 | 触发初始化 |

### 典型PWM值

```
低位:  1000 - 1500 PWM
中位:  1500 PWM
高位:  1700 - 2000 PWM  ← 触发初始化
```

### 推荐设置

**三段开关**:
```
下位: 1000 PWM (低位)
中位: 1500 PWM (中位)
上位: 2000 PWM (高位) ← 用于触发初始化
```

**两段开关**:
```
下位: 1000 PWM (低位)
上位: 2000 PWM (高位) ← 用于触发初始化
```

## 工作流程

```
飞控启动
    ↓
setup()中尝试初始化BMI088
    ↓
    ├─ 成功 → bmi088_initialized = true
    │         不再需要手动触发
    │
    └─ 失败 → bmi088_initialized = false
              bmi088_init_attempted = false
              ↓
              进入update()循环
              ↓
              检测通道5
              ↓
              ├─ CH5 < 1700 → 继续等待
              │
              └─ CH5 > 1700 → 触发初始化
                              ↓
                              ├─ 成功 → bmi088_initialized = true
                              │         bmi088_init_attempted = true
                              │         显示 "BMI088 initialized via CH5"
                              │
                              └─ 失败 → bmi088_initialized = false
                                        bmi088_init_attempted = true
                                        显示 "BMI088 init failed via CH5"
                                        不再重试
```

## 状态变量

### bmi088_initialized

**类型**: `bool`  
**作用**: 标记BMI088是否成功初始化  
**初始值**: `false`

```cpp
true  - 初始化成功，可以读取数据
false - 初始化失败，无法读取数据
```

### bmi088_init_attempted

**类型**: `static bool`  
**作用**: 标记是否已尝试通过CH5初始化  
**初始值**: `false`

```cpp
true  - 已尝试初始化（无论成功或失败），不再重试
false - 未尝试初始化，等待CH5触发
```

## 代码位置

### update()函数

**文件**: `tiltrotor.cpp`  
**函数**: `void Tiltrotor::update(void)`  
**位置**: 在函数开始处，检查 `enabled()` 之后

```cpp
void Tiltrotor::update(void)
{
    if (!enabled() || tilt_mask == 0) {
        return;
    }

    // BMI088初始化逻辑
    static bool bmi088_init_attempted = false;
    if (!bmi088_initialized && !bmi088_init_attempted) {
        RC_Channel *ch5 = RC_Channels::rc_channel(4);
        if (ch5 != nullptr && ch5->get_radio_in() > 1700) {
            if (bmi088_init()) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BMI088 initialized via CH5");
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "BMI088 init failed via CH5");
            }
            bmi088_init_attempted = true;
        }
    }
    
    // ... 其他代码
}
```

## 调试方法

### 1. 检查通道5输入

```cpp
// 临时添加调试代码
RC_Channel *ch5 = RC_Channels::rc_channel(4);
if (ch5 != nullptr) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CH5 PWM: %d", ch5->get_radio_in());
}
```

### 2. 检查初始化状态

```cpp
GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BMI init:%d attempted:%d", 
              (int)bmi088_initialized, 
              (int)bmi088_init_attempted);
```

### 3. 查看调试输出

每秒输出一次的调试信息中包含 `bmi` 参数：

```
Bi bmi:0 am:45.00 fd:0.12 ta:90.00 PE:2.3 PD:0.15 L:512 R:488
    ↑
   0 = 未初始化
   1 = 已初始化
```

## 常见问题

### Q1: 通道5拨到高位但没有触发初始化

**可能原因**:
1. PWM值不够高 (< 1700)
2. 已经尝试过初始化 (`bmi088_init_attempted = true`)
3. 通道5映射错误

**解决方法**:
```
1. 检查通道5 PWM值是否 > 1700
2. 重启飞控重置 bmi088_init_attempted
3. 确认遥控器通道5正确映射
```

### Q2: 初始化失败后无法重试

**原因**: `bmi088_init_attempted = true` 后不再重试

**解决方法**:
```
1. 重启飞控
2. 或修改代码允许多次重试（不推荐）
```

### Q3: 如何允许多次重试？

**不推荐**，但如果需要：

```cpp
// 修改前（仅一次）
static bool bmi088_init_attempted = false;
if (!bmi088_initialized && !bmi088_init_attempted) {
    // ...
    bmi088_init_attempted = true;
}

// 修改后（每次CH5高位都重试）
if (!bmi088_initialized) {
    RC_Channel *ch5 = RC_Channels::rc_channel(4);
    static bool last_ch5_high = false;
    bool ch5_high = (ch5 != nullptr && ch5->get_radio_in() > 1700);
    
    // 检测上升沿
    if (ch5_high && !last_ch5_high) {
        if (bmi088_init()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "BMI088 initialized via CH5");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "BMI088 init failed via CH5");
        }
    }
    last_ch5_high = ch5_high;
}
```

### Q4: 通道5被其他功能占用怎么办？

**更改通道**:

```cpp
// 使用通道6
RC_Channel *ch6 = RC_Channels::rc_channel(5);  // 索引5 = 通道6
if (ch6 != nullptr && ch6->get_radio_in() > 1700) {
    // ...
}

// 使用通道7
RC_Channel *ch7 = RC_Channels::rc_channel(6);  // 索引6 = 通道7
if (ch7 != nullptr && ch7->get_radio_in() > 1700) {
    // ...
}
```

## 使用场景

### 场景1: 启动时I2C总线未就绪

```
问题: 飞控启动时I2C2总线还未完全初始化
结果: BMI088初始化失败

解决:
1. 等待飞控完全启动
2. 通道5拨高位
3. 手动触发初始化
```

### 场景2: 传感器接触不良

```
问题: BMI088接线松动，启动时未检测到
结果: 初始化失败

解决:
1. 检查并重新插拔传感器
2. 通道5拨高位
3. 重新初始化
```

### 场景3: 调试测试

```
场景: 开发调试时需要多次测试初始化
方法:
1. 每次重启飞控
2. 通道5拨高位触发
3. 观察初始化结果
```

## 优势

✅ **灵活性**: 无需重启飞控即可重新初始化  
✅ **安全性**: 仅执行一次，避免重复初始化干扰  
✅ **便捷性**: 通过遥控器即可操作，无需连接电脑  
✅ **可靠性**: 有明确的成功/失败反馈  

## 总结

### 关键点

| 项目 | 说明 |
|------|------|
| **触发通道** | 通道5 (索引4) |
| **触发条件** | PWM > 1700 |
| **执行次数** | 仅一次 |
| **成功消息** | "BMI088 initialized via CH5" |
| **失败消息** | "BMI088 init failed via CH5" |

### 操作流程

```
1. 检查初始化状态 (查看QGC消息)
2. 如果失败，将CH5拨到高位 (> 1700)
3. 观察初始化结果
4. 验证 bmi 参数 (应为1)
```

现在你可以通过遥控器通道5手动触发BMI088初始化了！🎯
