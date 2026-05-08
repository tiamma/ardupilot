# 双旋翼控制系统日志说明

## 概述

双旋翼控制系统会将详细的控制过程数据记录到`TILT`日志消息中，用于分析和调试。

## 日志字段说明

### TILT 日志消息

| 字段 | 全称 | 单位 | 范围 | 说明 |
|------|------|------|------|------|
| TimeUS | Time (microseconds) | μs | - | 系统启动后的微秒时间戳 |
| PErr | Pitch Error | deg | ±180 | 俯仰角度误差（目标-当前） |
| PDes | Pitch Desired Rate | deg/s | ±200 | 期望俯仰角速度（外环输出） |
| PDif | Pitch Differential | - | ±1.0 | 俯仰差分输出（内环输出） |
| YErr | Yaw Error | deg | ±180 | 偏航角度误差（目标-当前） |
| YDes | Yaw Desired Rate | deg/s | ±90 | 期望偏航角速度（外环输出） |
| YDif | Yaw Differential | - | ±1.0 | 偏航差分输出（内环输出） |
| LOut | Left Motor Output | - | 0-1000 | 左电机输出（舵机信号） |
| ROut | Right Motor Output | - | 0-1000 | 右电机输出（舵机信号） |

## 日志频率

- **记录频率**: 50Hz（每20ms一次）
- **文件格式**: BIN格式（ArduPilot标准日志）
- **存储位置**: SD卡或板载闪存

## 数据分析

### 1. 俯仰控制分析

#### 外环性能
```
PErr → PDes
```
- **PErr = 0**: 俯仰角度完全跟踪目标
- **PErr > 0**: 实际角度小于目标（需要抬头）
- **PErr < 0**: 实际角度大于目标（需要低头）
- **PDes**: 外环PID输出，反映期望的旋转速度

**分析要点**：
- PErr应该快速收敛到0附近
- PDes应该与PErr成比例关系
- 如果PErr振荡，说明外环P增益过大

#### 内环性能
```
PDes → PDif
```
- **PDif**: 俯仰电机差分输出
- **范围**: -1.0到+1.0
- **正值**: 左电机增加，右电机减少（抬头）
- **负值**: 左电机减少，右电机增加（低头）

**分析要点**：
- PDif应该快速响应PDes的变化
- 如果PDif振荡，说明内环P增益过大
- PDif饱和（±1.0）说明控制量不足

### 2. 偏航控制分析

#### 外环性能
```
YErr → YDes
```
- **YErr**: 偏航角度误差（自动处理360度环绕）
- **YDes**: 期望偏航角速度

**分析要点**：
- YErr应该在±180度范围内
- 大的YErr应该产生相应的YDes
- 偏航响应通常比俯仰慢

#### 内环性能
```
YDes → YDif
```
- **YDif**: 偏航电机差分输出
- **正值**: 两个电机都增加倾转角（左转）
- **负值**: 两个电机都减少倾转角（右转）

### 3. 电机输出分析

#### 组合公式
```
LOut = 500 + PDif*1000 + YDif*1000
ROut = 500 - PDif*1000 + YDif*1000
```

#### 输出范围
- **正常范围**: 0-1000
- **中位**: 500（垂直）
- **最大前倾**: 1000
- **最大后倾**: 0

**分析要点**：
- LOut和ROut应该在0-1000范围内
- 如果经常触及边界，说明控制幅度不足
- 两个输出的差值反映俯仰控制
- 两个输出的平均值反映偏航控制

## 使用Mission Planner分析日志

### 1. 加载日志
1. 打开Mission Planner
2. 点击 `Flight Data` → `DataFlash Logs`
3. 选择日志文件并点击 `Log Browser`

### 2. 查看TILT消息
1. 在左侧消息列表中找到 `TILT`
2. 勾选要查看的字段

### 3. 推荐的图表组合

#### 俯仰控制分析
```
图表1: PErr vs Time
图表2: PDes vs Time
图表3: PDif vs Time
```
观察三者的关系和响应速度

#### 偏航控制分析
```
图表1: YErr vs Time
图表2: YDes vs Time
图表3: YDif vs Time
```

#### 电机输出分析
```
图表1: LOut vs Time
图表2: ROut vs Time
图表3: (LOut+ROut)/2 vs Time  # 平均值
图表4: LOut-ROut vs Time      # 差值
```

#### 组合分析
```
图表1: PErr, YErr
图表2: PDif, YDif
图表3: LOut, ROut
```

## 典型问题诊断

### 问题1: 俯仰振荡
**症状**：
- PErr快速振荡
- PDif也跟着振荡
- LOut和ROut剧烈变化

**原因**：
- 外环P增益过大
- 内环P增益过大
- D增益不足

**解决**：
1. 减小 `Q_TILT_P_ANG_P`
2. 减小 `Q_TILT_P_RAT_P`
3. 增加 `Q_TILT_P_ANG_D`

### 问题2: 俯仰响应慢
**症状**：
- PErr长时间不为0
- PDes很小
- 飞行器姿态调整缓慢

**原因**：
- 外环P增益过小
- 内环P增益过小

**解决**：
1. 增大 `Q_TILT_P_ANG_P`
2. 增大 `Q_TILT_P_RAT_P`

### 问题3: 偏航漂移
**症状**：
- YErr持续不为0
- YDes有值但YErr不减小
- 飞行器持续偏航

**原因**：
- 外环I增益不足
- 内环I增益不足
- 磁罗盘干扰

**解决**：
1. 增大 `Q_TILT_Y_ANG_I`
2. 增大 `Q_TILT_Y_RAT_I`
3. 检查磁罗盘校准

### 问题4: 控制饱和
**症状**：
- PDif经常达到±1.0
- LOut或ROut经常达到0或1000
- 控制效果不佳

**原因**：
- 期望角速度过大
- 电机推力不足
- 重心偏移

**解决**：
1. 减小 `Q_TILT_P_MAX_R`
2. 检查电机和螺旋桨
3. 调整重心位置

### 问题5: 俯仰偏航耦合
**症状**：
- 俯仰动作时YErr变化
- 偏航动作时PErr变化
- 控制不独立

**原因**：
- 电机安装不对称
- 重心偏移
- 参数设置不当

**解决**：
1. 检查机械结构
2. 调整重心到两电机连线中点
3. 分别调整俯仰和偏航参数

## 数据导出

### 导出为CSV
1. 在Mission Planner中打开日志
2. 点击 `Export to CSV`
3. 选择 `TILT` 消息
4. 保存CSV文件

### 使用Python分析
```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV
df = pd.read_csv('TILT.csv')

# 绘制俯仰控制
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(df['TimeUS']/1e6, df['PErr'])
plt.ylabel('Pitch Error (deg)')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(df['TimeUS']/1e6, df['PDes'])
plt.ylabel('Desired Pitch Rate (deg/s)')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(df['TimeUS']/1e6, df['PDif'])
plt.ylabel('Pitch Differential')
plt.xlabel('Time (s)')
plt.grid(True)

plt.tight_layout()
plt.show()
```

## 实时监控

### 使用MAVProxy
```bash
# 连接到飞行器
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600

# 监控TILT消息
> watch TILT
```

### 使用Mission Planner
1. 连接飞行器
2. 打开 `Flight Data` 页面
3. 右键点击HUD → `User Items`
4. 添加自定义项：
   - `TILT.PErr` - 俯仰误差
   - `TILT.YErr` - 偏航误差
   - `TILT.LOut` - 左电机输出
   - `TILT.ROut` - 右电机输出

## 日志文件管理

### 自动日志
- 默认启用
- 每次解锁自动开始记录
- 上锁后自动停止

### 手动控制
```
# 开始记录
LOG_DISARMED = 1

# 停止记录
LOG_DISARMED = 0
```

### 日志大小
- 每条TILT消息: ~40字节
- 50Hz频率: ~2KB/秒
- 10分钟飞行: ~1.2MB

## 最佳实践

1. **每次飞行后下载日志**
   - 及时分析问题
   - 建立飞行数据库

2. **关注关键指标**
   - PErr和YErr的收敛速度
   - PDif和YDif的振荡情况
   - LOut和ROut的饱和情况

3. **对比调参前后**
   - 保存调参前的日志
   - 对比参数变化的效果
   - 记录最佳参数组合

4. **建立基准**
   - 记录良好飞行的日志特征
   - 作为后续调参的参考
   - 快速识别异常情况

## 故障排查流程

1. **下载日志**
2. **检查TILT消息**
3. **分析关键字段**
   - 是否有异常振荡？
   - 是否有饱和现象？
   - 响应速度是否合适？
4. **对比GCS调试输出**
5. **调整参数**
6. **重新测试**
7. **验证改进**

## 技术支持

提交问题时请提供：
1. 完整的BIN日志文件
2. 参数列表（.param文件）
3. 问题描述和时间点
4. TILT消息的截图或CSV导出
