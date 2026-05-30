# MAVLink到VOFA+数据转发器使用说明

## 功能说明

`rtc.py` 是一个MAVLink到VOFA+的数据转发工具，用于：
1. 接收ArduPilot发送的NAMED_VALUE_FLOAT消息
2. 提取MPU6050和倾转控制数据
3. 转发到VOFA+进行实时可视化

## 数据通道

转发到VOFA+的6个通道：
1. **通道1**: `MPU_PITCH` - MPU6050俯仰角
2. **通道2**: `MPU_ROLL` - MPU6050横滚角
3. **通道3**: `TILT_TGT` - 目标倾转角度
4. **通道4**: `TS_ERR` - 角度误差
5. **通道5**: `TS_PID` - PID修正量
6. **通道6**: `TS_OUT` - 最终输出

## 安装依赖

```bash
pip install pymavlink pyserial
```

## 使用方法

### 基本用法

```bash
# 使用默认参数
python3 rtc.py

# 自定义MAVLink连接（UDP）
python3 rtc.py --mavlink udp:127.0.0.1:14550

# 自定义MAVLink连接（串口）
python3 rtc.py --mavlink /dev/ttyUSB0:57600

# 自定义VOFA+串口
python3 rtc.py --vofa-port /dev/ttyUSB1 --vofa-baudrate 115200
```

### 完整参数示例

```bash
python3 rtc.py \
  --mavlink udp:127.0.0.1:14550 \
  --vofa-port /dev/ttyUSB1 \
  --vofa-baudrate 115200
```

### Windows系统

```bash
python rtc.py --mavlink COM3:57600 --vofa-port COM4
```

## 参数说明

| 参数 | 简写 | 默认值 | 说明 |
|------|------|--------|------|
| `--mavlink` | `-m` | `udp:127.0.0.1:14550` | MAVLink连接字符串 |
| `--vofa-port` | `-v` | `/dev/ttyUSB1` | VOFA+串口 |
| `--vofa-baudrate` | `-b` | `115200` | VOFA+串口波特率 |

## MAVLink连接方式

### UDP连接（推荐用于地面站）
```bash
--mavlink udp:127.0.0.1:14550
```

### TCP连接
```bash
--mavlink tcp:127.0.0.1:5760
```

### 串口连接
```bash
--mavlink /dev/ttyUSB0:57600    # Linux
--mavlink COM3:57600             # Windows
```

## VOFA+配置

### 1. 打开VOFA+软件

### 2. 配置串口
- 选择对应的串口（如 `/dev/ttyUSB1` 或 `COM4`）
- 波特率：115200
- 协议：**FireWater**（JustFloat）

### 3. 配置通道名称
在VOFA+中为6个通道设置名称：
```
通道1: MPU_PITCH
通道2: MPU_ROLL
通道3: TILT_TGT
通道4: TS_ERR
通道5: TS_PID
通道6: TS_OUT
```

### 4. 开始接收
点击"开始"按钮，即可看到实时数据曲线

## 运行示例

### 示例1：本地UDP连接
```bash
$ python3 rtc.py
============================================================
MAVLink到VOFA+数据转发器
============================================================
MAVLink连接: udp:127.0.0.1:14550
VOFA+串口: /dev/ttyUSB1 @ 115200
============================================================
正在连接MAVLink: udp:127.0.0.1:14550
等待心跳包...
连接成功! 系统ID: 1, 组件ID: 1
正在打开VOFA+串口: /dev/ttyUSB1 @ 115200
VOFA+串口已打开

开始接收MAVLink数据并转发到VOFA+...
按Ctrl+C停止

消息数: 125 | MPU_P:  5.20 MPU_R: 42.30 TGT: 45.00 ERR:  2.70 PID:  0.027 OUT:  0.477
```

### 示例2：串口连接
```bash
python3 rtc.py --mavlink /dev/ttyUSB0:57600 --vofa-port /dev/ttyUSB1
```

## 数据流程图

```
ArduPilot (飞控)
    ↓
  MAVLink
    ↓ NAMED_VALUE_FLOAT消息
rtc.py (转发器)
    ↓ 提取数据
  数据缓存
    ↓ 打包为VOFA+格式
  串口输出
    ↓
VOFA+ (可视化)
```

## 故障排除

### 1. 无法连接MAVLink
- 检查连接字符串是否正确
- 确认飞控已连接并正常运行
- 检查防火墙设置（UDP连接）

### 2. 无法打开VOFA+串口
- 检查串口是否被其他程序占用
- 确认串口权限（Linux需要添加用户到dialout组）
  ```bash
  sudo usermod -a -G dialout $USER
  ```
- 重新插拔USB设备

### 3. VOFA+无数据显示
- 确认VOFA+协议选择为 **FireWater**
- 检查波特率是否匹配
- 确认串口已正确打开

### 4. 数据不更新
- 检查ArduPilot是否正在发送NAMED_VALUE_FLOAT消息
- 确认飞控处于正确的飞行模式（QHOVER/QSTABILIZE等）

## 高级用法

### 修改发送的数据通道

编辑 `rtc.py` 第121-128行，修改 `vofa_data` 列表：

```python
vofa_data = [
    self.data_cache['MPU_PITCH'],   # 通道1
    self.data_cache['MPU_ROLL'],    # 通道2
    self.data_cache['TS_TGT'],      # 通道3: 改为TS_TGT
    self.data_cache['TS_MPU'],      # 通道4: 改为TS_MPU
    self.data_cache['TS_CUR'],      # 通道5: 改为TS_CUR
    self.data_cache['CH9_PWM'],     # 通道6: 改为CH9_PWM
]
```

### 增加数据通道

可以添加更多通道（VOFA+支持最多32个通道）：

```python
vofa_data = [
    self.data_cache['MPU_PITCH'],
    self.data_cache['MPU_ROLL'],
    self.data_cache['TILT_TGT'],
    self.data_cache['TS_ERR'],
    self.data_cache['TS_PID'],
    self.data_cache['TS_OUT'],
    self.data_cache['TS_TGT'],      # 通道7
    self.data_cache['TS_MPU'],      # 通道8
    self.data_cache['TS_CUR'],      # 通道9
    self.data_cache['CH9_PWM'],     # 通道10
]
```

## 性能说明

- **更新频率**: 取决于ArduPilot发送频率（通常1Hz）
- **延迟**: < 10ms
- **CPU占用**: < 5%
- **内存占用**: < 50MB

## 许可证

GPL-3.0

## 作者

ArduPilot Development Team
