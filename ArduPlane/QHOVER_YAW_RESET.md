# QHOVER模式偏航角重置功能

## 功能说明

在QHOVER模式下，通道7可以用来重置偏航角基准 (`yaw_angle_offset_deg`)，实现快速重新定向功能。

## 工作原理

### 代码实现

```cpp
// 在 mode_qhover.cpp 的 run() 函数中
RC_Channel *ch7 = RC_Channels::rc_channel(6);  // 通道7 (索引从0开始)
if (ch7 != nullptr) {
    int16_t ch7_value = ch7->get_radio_in();
    // 如果通道7 > 1700 (高位)，重置偏航角基准
    if (ch7_value > 1700) {
        yaw_angle_offset_deg = plane.ahrs.yaw_sensor * 0.01f;
    }
}
```

### 触发条件

| 通道7 PWM值 | 动作 |
|------------|------|
| **< 1700** | 无动作 |
| **≥ 1700** | 重置偏航角基准为当前航向 |

## 使用方法

### 1. 遥控器设置

**配置通道7为三段开关**:
```
低位:  PWM ~1000  (不重置)
中位:  PWM ~1500  (不重置)
高位:  PWM ~2000  (重置偏航角)
```

**或配置为两段开关**:
```
低位:  PWM ~1000  (不重置)
高位:  PWM ~2000  (重置偏航角)
```

### 2. 飞行操作

**场景1: 悬停中重新定向**
```
1. 飞机在QHOVER模式悬停
2. 手动旋转飞机到期望方向
3. 将通道7拨到高位
4. 当前航向被设置为新的基准
5. 将通道7拨回低位
6. 飞机将保持新的航向
```

**场景2: 起飞后设置航向**
```
1. 起飞进入QHOVER模式
2. 飞机可能偏离期望航向
3. 将通道7拨到高位
4. 当前航向被锁定为基准
5. 将通道7拨回低位
6. 飞机保持当前航向
```

**场景3: 快速归零**
```
1. 飞机在QHOVER模式
2. 偏航角误差累积较大
3. 将通道7拨到高位
4. 偏航角误差归零
5. 将通道7拨回低位
6. 重新开始偏航控制
```

## 工作流程

### 正常偏航控制

```
起飞时航向 → yaw_angle_offset_deg (基准)
             ↓
当前航向 → yaw_angle
             ↓
angle_error = yaw_angle - yaw_angle_offset_deg
             ↓
偏航控制 → 保持基准航向
```

### 通道7重置后

```
通道7 > 1700 → 触发重置
                ↓
yaw_angle_offset_deg = 当前航向
                ↓
angle_error = 0 (误差归零)
                ↓
新的基准航向 → 保持新航向
```

## 应用场景

### 1. 风向变化

**问题**: 风向改变，需要调整机头方向
```
操作:
1. 手动旋转到迎风方向
2. 通道7高位重置
3. 锁定新航向
```

### 2. 任务调整

**问题**: 需要改变观察方向
```
操作:
1. 用偏航杆旋转到目标方向
2. 通道7高位锁定
3. 保持新方向观察
```

### 3. 误差累积

**问题**: 长时间飞行后偏航角漂移
```
操作:
1. 通道7高位
2. 清零累积误差
3. 重新开始控制
```

### 4. 快速对准

**问题**: 需要快速对准某个方向
```
操作:
1. 手动对准目标
2. 通道7高位锁定
3. 自动保持对准
```

## 技术细节

### PWM阈值选择

**为什么选择1700?**
```
典型PWM范围: 1000 - 2000
中点: 1500
高位阈值: 1700

优点:
- 避免中位误触发
- 给予200μs的安全裕度
- 兼容大多数遥控器
```

### 触发频率

```
运行频率: 50Hz (每20ms检测一次)
响应延迟: < 20ms
重置时间: 瞬时
```

### 安全性

**防抖动**:
```
当前实现: 每次循环检测
建议: 如需防抖，可添加延迟
```

**空指针保护**:
```cpp
if (ch7 != nullptr) {
    // 确保通道7存在
}
```

## 调试方法

### 1. 检查通道7输入

**通过Mission Planner**:
```
1. 进入 SETUP → Radio Calibration
2. 观察通道7的PWM值
3. 确认高位 > 1700
```

**通过MAVLink消息**:
```
RC_CHANNELS.chan7_raw
应该显示通道7的PWM值
```

### 2. 验证重置功能

**测试步骤**:
```
1. 进入QHOVER模式
2. 记录当前 yaw_angle_offset_deg
3. 旋转飞机
4. 通道7拨高位
5. 检查 yaw_angle_offset_deg 是否更新
6. 检查 angle_error 是否归零
```

### 3. 日志分析

**关键变量**:
```
yaw_angle_offset_deg  - 偏航角基准
yaw_angle            - 当前偏航角
angle_error          - 偏航角误差
ch7_value            - 通道7 PWM值
```

## 改进建议

### 1. 添加防抖动

```cpp
// 添加状态变量
static bool last_ch7_high = false;
static uint32_t ch7_high_start_ms = 0;

// 防抖动逻辑
bool ch7_high = (ch7_value > 1700);
if (ch7_high && !last_ch7_high) {
    ch7_high_start_ms = now;
}
if (ch7_high && (now - ch7_high_start_ms > 100)) {  // 100ms防抖
    yaw_angle_offset_deg = plane.ahrs.yaw_sensor * 0.01f;
}
last_ch7_high = ch7_high;
```

### 2. 添加边沿触发

```cpp
// 只在上升沿触发
static bool last_ch7_high = false;
bool ch7_high = (ch7_value > 1700);

if (ch7_high && !last_ch7_high) {
    // 上升沿，触发重置
    yaw_angle_offset_deg = plane.ahrs.yaw_sensor * 0.01f;
}
last_ch7_high = ch7_high;
```

### 3. 添加GCS通知

```cpp
if (ch7_value > 1700) {
    yaw_angle_offset_deg = plane.ahrs.yaw_sensor * 0.01f;
    gcs().send_text(MAV_SEVERITY_INFO, "Yaw offset reset to %.1f deg", yaw_angle_offset_deg);
}
```

### 4. 可配置阈值

```cpp
// 在Parameters中添加
AP_Int16 qhover_yaw_reset_pwm;  // 默认1700

// 使用
if (ch7_value > g.qhover_yaw_reset_pwm) {
    yaw_angle_offset_deg = plane.ahrs.yaw_sensor * 0.01f;
}
```

## 常见问题

### Q1: 通道7无响应

**检查**:
```
1. 确认遥控器通道7已配置
2. 检查PWM值是否 > 1700
3. 确认在QHOVER模式
4. 检查通道映射是否正确
```

### Q2: 重置后仍有偏航

**原因**:
```
1. PID参数不合适
2. 偏航增益太小
3. 电机响应不足
```

**解决**:
```
1. 调整偏航PID
2. 增加偏航增益 (当前为10)
3. 检查电机输出
```

### Q3: 频繁触发

**原因**:
```
1. 通道7在1700附近抖动
2. 遥控器信号不稳定
```

**解决**:
```
1. 增加阈值到1750
2. 添加防抖动逻辑
3. 使用边沿触发
```

### Q4: 重置后立即漂移

**原因**:
```
1. 陀螺仪漂移
2. 磁罗盘干扰
3. 控制增益过小
```

**解决**:
```
1. 校准陀螺仪
2. 校准磁罗盘
3. 增加控制增益
```

## 总结

### 关键点

✅ **通道**: 通道7 (RC_Channels索引6)  
✅ **阈值**: PWM > 1700  
✅ **功能**: 重置偏航角基准  
✅ **响应**: 瞬时 (< 20ms)  
✅ **模式**: 仅QHOVER模式有效  

### 使用建议

1. **首次使用**: 地面测试确认功能
2. **飞行中**: 需要时拨动通道7
3. **重置后**: 通道7回到低位
4. **观察**: 检查飞机是否保持新航向

### 安全提示

⚠️ **重要**:
- 重置会改变航向基准
- 重置瞬间可能有小幅晃动
- 建议在稳定悬停时使用
- 避免在机动过程中重置

现在你可以使用通道7来快速重置偏航角基准了！🎯
