import time
import math
import smbus2
import serial
import struct

# ====================== 全局配置区 ======================
I2C_BUS      = 1
MPU6050_ADDR = 0x68

# 采样参数
SAMPLE_RATE  = 50          # 采样率 Hz
DT           = 1.0 / SAMPLE_RATE  # 采样周期

# 传感器量程配置
GYRO_RANGE   = 500         # 陀螺仪量程 ±2000°/s
ACCEL_RANGE  = 2            # 加速度计量程 ±4g

# 量程对应的灵敏度系数
GYRO_SCALE   = None         # 将在初始化时计算
ACCEL_SCALE  = None         # 将在初始化时计算

# 陀螺仪零偏校准值
gyro_zero_x = 0
gyro_zero_y = 0
gyro_zero_z = 0

# 角度校准偏移
angle_roll_offset  = 0.0
angle_pitch_offset = 0.0
angle_yaw_offset   = 0.0

# I2C总线
bus = smbus2.SMBus(I2C_BUS)

# 串口VOFA+配置
SERIAL_PORT = '/dev/ttyS0'     # Pin8=TX, Pin10=RX (树莓派串口)
SERIAL_BAUD = 115200           # 波特率
VOFA_TAIL = bytes([0x00, 0x00, 0x80, 0x7f])  # VOFA+帧尾
# ========================================================

# ====================== MPU6050 寄存器定义 ======================
MPU6050_SMPLRT_DIV   = 0x19
MPU6050_CONFIG       = 0x1A
MPU6050_GYRO_CONFIG  = 0x1B
MPU6050_ACCEL_CONFIG = 0x1C
MPU6050_INT_ENABLE   = 0x38
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_GYRO_XOUT_H  = 0x43
MPU6050_PWR_MGMT_1   = 0x6B
MPU6050_WHO_AM_I     = 0x75

# ====================== 快速平方根倒数 ======================
def inv_sqrt(x):
    """快速平方根倒数算法（Python简化版）"""
    return 1.0 / math.sqrt(x) if x > 0 else 0.0

# ====================== MPU6050 初始化 ======================
def mpu6050_init():
    global GYRO_SCALE, ACCEL_SCALE
    
    print("【1/5】MPU6050 寄存器初始化...")
    
    # 计算灵敏度系数
    if GYRO_RANGE == 250:
        gyro_lsb = 131.0
    elif GYRO_RANGE == 500:
        gyro_lsb = 65.5
    elif GYRO_RANGE == 1000:
        gyro_lsb = 32.8
    else:  # 2000
        gyro_lsb = 16.4
    
    if ACCEL_RANGE == 2:
        accel_lsb = 16384.0
    elif ACCEL_RANGE == 4:
        accel_lsb = 8192.0
    elif ACCEL_RANGE == 8:
        accel_lsb = 4096.0
    else:  # 16
        accel_lsb = 2048.0
    
    ACCEL_SCALE = 1.0 / accel_lsb
    GYRO_SCALE = math.radians(1.0) / gyro_lsb  # 转换为弧度/s
    
    # 复位设备
    bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x80)
    time.sleep(0.1)
    
    # 唤醒设备
    bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00)
    time.sleep(0.01)
    
    # 配置采样率分频
    smplrt_div = int(1000.0 / SAMPLE_RATE - 1)
    bus.write_byte_data(MPU6050_ADDR, MPU6050_SMPLRT_DIV, smplrt_div)
    
    # 配置中断
    bus.write_byte_data(MPU6050_ADDR, 0x37, 0x00)
    bus.write_byte_data(MPU6050_ADDR, MPU6050_INT_ENABLE, 0x01)  # 数据就绪中断
    
    # 配置低通滤波器 (21Hz)
    bus.write_byte_data(MPU6050_ADDR, MPU6050_CONFIG, 0x04)
    
    # 配置陀螺仪量程
    gyro_config = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}[GYRO_RANGE]
    bus.write_byte_data(MPU6050_ADDR, MPU6050_GYRO_CONFIG, gyro_config)
    
    # 配置加速度计量程
    accel_config = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}[ACCEL_RANGE]
    bus.write_byte_data(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, accel_config)
    
    # 设置时钟源为X轴陀螺仪
    bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01)
    
    time.sleep(0.1)
    
    # 读取WHO_AM_I验证
    chip_id = bus.read_byte_data(MPU6050_ADDR, MPU6050_WHO_AM_I)
    print(f"  芯片ID: 0x{chip_id:02X} (期望: 0x68)")
    print(f"  采样率: {SAMPLE_RATE}Hz")
    print(f"  陀螺仪: ±{GYRO_RANGE}°/s")
    print(f"  加速度: ±{ACCEL_RANGE}g")
    print("MPU6050 初始化完成\n")

# ====================== 读取原始数据 ======================
def read_raw_data():
    """连续读取14字节：加速度(6) + 温度(2) + 陀螺仪(6)"""
    data = bus.read_i2c_block_data(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 14)
    
    # 加速度原始值
    acc_x = (data[0] << 8) | data[1]
    acc_y = (data[2] << 8) | data[3]
    acc_z = (data[4] << 8) | data[5]
    
    # 陀螺仪原始值
    gyro_x = (data[8] << 8) | data[9]
    gyro_y = (data[10] << 8) | data[11]
    gyro_z = (data[12] << 8) | data[13]
    
    # 转换为有符号数
    if acc_x > 32767: acc_x -= 65536
    if acc_y > 32767: acc_y -= 65536
    if acc_z > 32767: acc_z -= 65536
    if gyro_x > 32767: gyro_x -= 65536
    if gyro_y > 32767: gyro_y -= 65536
    if gyro_z > 32767: gyro_z -= 65536
    
    # 应用陀螺仪零偏校准
    gyro_x -= gyro_zero_x
    gyro_y -= gyro_zero_y
    gyro_z -= gyro_zero_z
    
    return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z

# ====================== 陀螺仪零偏校准 ======================
def gyro_calibrate():
    global gyro_zero_x, gyro_zero_y, gyro_zero_z
    
    print("【2/5】陀螺仪零偏校准...")
    print("  请保持传感器静止3秒")
    time.sleep(2)
    
    sum_x = sum_y = sum_z = 0
    samples = 200
    
    for i in range(samples):
        data = bus.read_i2c_block_data(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 6)
        gx = (data[0] << 8) | data[1]
        gy = (data[2] << 8) | data[3]
        gz = (data[4] << 8) | data[5]
        
        if gx > 32767: gx -= 65536
        if gy > 32767: gy -= 65536
        if gz > 32767: gz -= 65536
        
        sum_x += gx
        sum_y += gy
        sum_z += gz
        time.sleep(0.005)
    
    gyro_zero_x = int(sum_x / samples)
    gyro_zero_y = int(sum_y / samples)
    gyro_zero_z = int(sum_z / samples)
    
    print(f"  零偏: X={gyro_zero_x} Y={gyro_zero_y} Z={gyro_zero_z}")
    
    # 验证校准效果
    sum_x = sum_y = sum_z = 0
    for _ in range(100):
        data = bus.read_i2c_block_data(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, 6)
        gx = ((data[0] << 8) | data[1]) - (65536 if (data[0] << 8) | data[1] > 32767 else 0)
        gy = ((data[2] << 8) | data[3]) - (65536 if (data[2] << 8) | data[3] > 32767 else 0)
        gz = ((data[4] << 8) | data[5]) - (65536 if (data[4] << 8) | data[5] > 32767 else 0)
        sum_x += abs(gx - gyro_zero_x)
        sum_y += abs(gy - gyro_zero_y)
        sum_z += abs(gz - gyro_zero_z)
        time.sleep(0.005)
    
    avg_err = (sum_x + sum_y + sum_z) / 300
    print(f"  校准误差: {avg_err:.1f} (期望<5)")
    print("陀螺仪校准完成\n")

# ====================== 简单互补滤波姿态解算 ======================
class ComplementaryFilter:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gyro_roll = 0.0
        self.gyro_pitch = 0.0
    
    def update(self, ax, ay, az, gx, gy, gz):
        """互补滤波更新姿态"""
        # 加速度计计算角度
        acc_mag = math.sqrt(ax*ax + ay*ay + az*az)
        
        if acc_mag > 0.01:
            # 动态权重：运动剧烈时降低加速度计权重
            if acc_mag > 1.2:
                weight = 0.80
            else:
                weight = 0.98
            
            # 陀螺仪积分
            self.gyro_roll += gy * DT
            self.gyro_pitch += gx * DT
            
            # 加速度计角度
            acc_roll = math.atan2(ay, az)
            acc_pitch = -math.atan2(ax, az)
            
            # 互补滤波融合
            self.roll = weight * acc_roll + (1 - weight) * self.gyro_roll
            self.pitch = weight * acc_pitch + (1 - weight) * self.gyro_pitch
            self.yaw += gz * DT
        
        return math.degrees(self.roll), math.degrees(self.pitch), math.degrees(self.yaw)

# ====================== 四元数姿态解算 ======================
class QuaternionFilter:
    def __init__(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.times = 0
        self.unwrapped_yaw = 0.0
        self.first_run = True
    
    def update(self, ax, ay, az, gx, gy, gz):
        """四元数更新 + 动态互补滤波"""
        # 加速度幅值
        acc_mag = ax*ax + ay*ay + az*az
        
        # 动态增益调整
        if self.times < 400:
            self.times += 1
            kp = 8.0
            ki = 0.002
        else:
            if acc_mag > 1.44 or acc_mag < 0.64:
                kp = 3.6
                ki = 0.001
            else:
                kp = 4.8
                ki = 0.0015
        
        # 加速度计校正
        if acc_mag > 0.01:
            recip_norm = inv_sqrt(acc_mag)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm
            
            # 重力方向误差
            vx = 2.0 * (self.q1 * self.q3 - self.q0 * self.q2)
            vy = 2.0 * (self.q0 * self.q1 + self.q2 * self.q3)
            vz = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3
            
            ex = ay * vz - az * vy
            ey = az * vx - ax * vz
            ez = ax * vy - ay * vx
            
            # 积分校正
            if ki > 0:
                self.integral_x += ex * DT
                self.integral_y += ey * DT
                self.integral_z += ez * DT
                gx += ki * self.integral_x
                gy += ki * self.integral_y
                gz += ki * self.integral_z
            
            # 比例校正
            gx += kp * ex
            gy += kp * ey
            gz += kp * ez
        
        # 四元数微分
        q_dot0 = 0.5 * (-self.q1*gx - self.q2*gy - self.q3*gz)
        q_dot1 = 0.5 * (self.q0*gx + self.q2*gz - self.q3*gy)
        q_dot2 = 0.5 * (self.q0*gy - self.q1*gz + self.q3*gx)
        q_dot3 = 0.5 * (self.q0*gz + self.q1*gy - self.q2*gx)
        
        # 四元数积分
        self.q0 += q_dot0 * DT
        self.q1 += q_dot1 * DT
        self.q2 += q_dot2 * DT
        self.q3 += q_dot3 * DT
        
        # 归一化
        norm = inv_sqrt(self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3)
        self.q0 *= norm
        self.q1 *= norm
        self.q2 *= norm
        self.q3 *= norm
        
        # 四元数转欧拉角
        roll = math.atan2(2.0*(self.q0*self.q1 + self.q2*self.q3), 
                          1.0 - 2.0*(self.q1*self.q1 + self.q2*self.q2))
        pitch = math.asin(2.0*(self.q0*self.q2 - self.q3*self.q1))
        current_yaw = math.atan2(2.0*(self.q0*self.q3 + self.q1*self.q2),
                                 1.0 - 2.0*(self.q2*self.q2 + self.q3*self.q3))
        
        # Yaw角解包裹
        current_yaw_deg = math.degrees(current_yaw)
        if self.first_run:
            self.unwrapped_yaw = current_yaw_deg
            self.first_run = False
        else:
            diff = current_yaw_deg - self.unwrapped_yaw
            if diff > 180.0:
                self.unwrapped_yaw += diff - 360.0
            elif diff < -180.0:
                self.unwrapped_yaw += diff + 360.0
            else:
                self.unwrapped_yaw = current_yaw_deg
        
        return math.degrees(roll), math.degrees(pitch), self.unwrapped_yaw

# ====================== VOFA+数据发送 ======================
def send_vofa_data(ser, roll, pitch, yaw, acc_mag):
    """
    发送VOFA+格式数据
    格式: 4个float + 帧尾 {0x00, 0x00, 0x80, 0x7f}
    """
    # 打包4个float数据为字节流（小端序）
    data = struct.pack('<ffff', roll, pitch, yaw, acc_mag)
    
    # 发送数据
    ser.write(data)
    
    # 发送帧尾
    ser.write(VOFA_TAIL)

# ====================== 角度校准 ======================
def calibrate_angles(filter_obj, samples=1000):
    """记录当前姿态作为零点"""
    global angle_roll_offset, angle_pitch_offset, angle_yaw_offset
    
    print("【3/5】姿态角度校准...")
    print("  请将传感器放置在期望的零点位置")
    time.sleep(2)
    
    roll_sum = pitch_sum = yaw_sum = 0
    
    for _ in range(samples):
        ax, ay, az, gx, gy, gz = read_raw_data()
        ax *= ACCEL_SCALE
        ay *= ACCEL_SCALE
        az *= ACCEL_SCALE
        gx *= GYRO_SCALE
        gy *= GYRO_SCALE
        gz *= GYRO_SCALE
        
        roll, pitch, yaw = filter_obj.update(ax, ay, az, gx, gy, gz)
        roll_sum += roll
        pitch_sum += pitch
        yaw_sum += yaw
        time.sleep(DT)
    
    angle_roll_offset = roll_sum / samples
    angle_pitch_offset = pitch_sum / samples
    angle_yaw_offset = yaw_sum / samples
    
    print(f"  Roll偏移: {angle_roll_offset:.2f}°")
    print(f"  Pitch偏移: {angle_pitch_offset:.2f}°")
    print(f"  Yaw偏移: {angle_yaw_offset:.2f}°")
    print("角度校准完成\n")

# ====================== 主程序 ======================
if __name__ == "__main__":
    print("\n========== MPU6050 姿态解算系统 ==========\n")
    
    # 1. 初始化MPU6050
    mpu6050_init()
    time.sleep(0.5)
    
    # 2. 陀螺仪零偏校准
    gyro_calibrate()
    time.sleep(0.5)
    
    # 3. 选择滤波算法
    print("【4/5】选择姿态解算算法")
    print("  1 - 互补滤波（快速，适合简单应用）")
    print("  2 - 四元数法（精确，推荐）")
    choice = input("请选择 [1/2]: ").strip()
    
    if choice == '1':
        attitude_filter = ComplementaryFilter()
        print("使用互补滤波算法\n")
    else:
        attitude_filter = QuaternionFilter()
        print("使用四元数算法\n")
    
    # 4. 角度校准
    calibrate_angles(attitude_filter, samples=500)
    
    # 5. 初始化串口连接（VOFA+）
    print("【5/5】初始化串口连接（VOFA+格式）...")
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        print(f"  串口: {SERIAL_PORT}")
        print(f"  波特率: {SERIAL_BAUD}")
        print(f"  数据格式: 4 x float32 + 帧尾")
        print("串口连接成功\n")
    except Exception as e:
        print(f"  ⚠️ 串口连接失败: {e}")
        print("  将继续运行但不发送数据\n")
        ser = None
    
    # 6. 主循环
    print("【6/6】开始姿态解算...")
    print("按 Ctrl+C 退出\n")
    print("时间(s)  | Roll(°)  | Pitch(°) | Yaw(°)   | 加速度")
    print("-" * 60)
    
    start_time = time.time()
    
    try:
        while True:
            # 读取原始数据
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = read_raw_data()
            
            # 转换为物理单位
            ax = acc_x * ACCEL_SCALE
            ay = acc_y * ACCEL_SCALE
            az = acc_z * ACCEL_SCALE
            gx = gyro_x * GYRO_SCALE
            gy = gyro_y * GYRO_SCALE
            gz = gyro_z * GYRO_SCALE
            
            # 姿态解算
            roll, pitch, yaw = attitude_filter.update(ax, ay, az, gx, gy, gz)
            
            # 应用校准偏移
            roll -= angle_roll_offset
            pitch -= angle_pitch_offset
            yaw -= angle_yaw_offset
            
            # 计算加速度幅值
            acc_mag = math.sqrt(ax*ax + ay*ay + az*az)
            
            # 发送VOFA+数据
            if ser:
                send_vofa_data(ser, roll, pitch, yaw, acc_mag)
            
            # 显示结果
            elapsed = time.time() - start_time
            print(f"{elapsed:7.2f}  | {roll:7.2f}° | {pitch:7.2f}° | {yaw:7.2f}° | {acc_mag:5.2f}g")
            
            time.sleep(DT)
    
    except KeyboardInterrupt:
        print("\n\n程序正常退出")
        # 关闭串口
        if ser:
            ser.close()
            print("已关闭串口")
        print("========================================\n")