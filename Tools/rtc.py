#!/usr/bin/env python3
"""
MAVLink to VOFA+ Data Forwarder
接收ArduPilot的NAMED_VALUE_FLOAT消息并转发到VOFA+可视化工具
"""

import sys
import struct
import serial
import socket
import time
from pymavlink import mavutil

# VOFA+帧尾标识
VOFA_TAIL = bytes([0x00, 0x00, 0x80, 0x7f])

class MAVLinkToVOFA:
    def __init__(self, mavlink_connection, vofa_host='127.0.0.1', vofa_port=1347):
        """
        初始化MAVLink到VOFA+转发器
        
        Args:
            mavlink_connection: MAVLink连接字符串 (例如: '/dev/ttyUSB0:57600')
            vofa_host: VOFA+ TCP服务器地址
            vofa_port: VOFA+ TCP端口
        """
        self.mavlink_connection = mavlink_connection
        self.vofa_host = vofa_host
        self.vofa_port = vofa_port
        
        # 数据缓存字典
        self.data_cache = {
            'MPU_PITCH': 0.0,
            'MPU_ROLL': 0.0,
            'TILT_TGT': 0.0,
            'TS_ERR': 0.0,
            'TS_PID': 0.0,
            'TS_OUT': 0.0,
            'TS_TGT': 0.0,
            'TS_MPU': 0.0,
            'TS_CUR': 0.0,
            'CH9_PWM': 0.0,
            'CH10_MODE': 0.0,
        }
        
        # MAVLink连接
        self.master = None
        # VOFA+ TCP socket
        self.vofa_socket = None
        
        # 统计信息
        self.msg_count = 0
        self.last_print_time = time.time()
        
    def connect(self):
        """建立MAVLink和VOFA+连接"""
        try:
            # 连接MAVLink串口
            print(f"正在连接MAVLink串口: {self.mavlink_connection}")
            self.master = mavutil.mavlink_connection(self.mavlink_connection)
            print("等待心跳包...")
            self.master.wait_heartbeat()
            print(f"连接成功! 系统ID: {self.master.target_system}, 组件ID: {self.master.target_component}")
            
            # 连接VOFA+ TCP服务器
            print(f"正在连接VOFA+ TCP服务器: {self.vofa_host}:{self.vofa_port}")
            self.vofa_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.vofa_socket.connect((self.vofa_host, self.vofa_port))
            print("VOFA+ TCP连接已建立")
            
            return True
            
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def send_vofa_data(self, data_list):
        """
        发送VOFA+格式数据
        格式: N个float + 帧尾 {0x00, 0x00, 0x80, 0x7f}
        
        Args:
            data_list: float数据列表
        """
        if not self.vofa_socket:
            return
        
        try:
            # 打包float数据为字节流（小端序）
            format_str = '<' + 'f' * len(data_list)
            data = struct.pack(format_str, *data_list)
            
            # 发送数据
            self.vofa_socket.sendall(data)
            
            # 发送帧尾
            self.vofa_socket.sendall(VOFA_TAIL)
            
        except Exception as e:
            print(f"发送VOFA+数据失败: {e}")
    
    def process_named_value_float(self, msg):
        """
        处理NAMED_VALUE_FLOAT消息
        
        Args:
            msg: MAVLink NAMED_VALUE_FLOAT消息
        """
        name = msg.name
        value = msg.value
        
        # 更新数据缓存
        if name in self.data_cache:
            self.data_cache[name] = value
            self.msg_count += 1
            
            # 准备发送到VOFA+的数据
            # 方案1: 发送主要的6个参数
            vofa_data = [
                self.data_cache['MPU_PITCH'],   # 通道1: MPU俯仰角
                self.data_cache['MPU_ROLL'],    # 通道2: MPU横滚角
                self.data_cache['TILT_TGT'],    # 通道3: 目标倾转角度
                self.data_cache['TS_ERR'],      # 通道4: 角度误差
                self.data_cache['TS_PID'],      # 通道5: PID修正量
                self.data_cache['TS_OUT'],      # 通道6: 最终输出
            ]
            
            # 发送到VOFA+
            self.send_vofa_data(vofa_data)
            
            # 每秒打印一次统计信息
            current_time = time.time()
            if current_time - self.last_print_time >= 1.0:
                print(f"\r消息数: {self.msg_count} | "
                      f"MPU_P:{self.data_cache['MPU_PITCH']:6.2f} "
                      f"MPU_R:{self.data_cache['MPU_ROLL']:6.2f} "
                      f"TGT:{self.data_cache['TILT_TGT']:6.2f} "
                      f"ERR:{self.data_cache['TS_ERR']:6.2f} "
                      f"PID:{self.data_cache['TS_PID']:7.3f} "
                      f"OUT:{self.data_cache['TS_OUT']:6.3f}", end='')
                self.last_print_time = current_time
    
    def run(self):
        """主循环：接收MAVLink消息并转发到VOFA+"""
        if not self.connect():
            return
        
        print("\n开始接收MAVLink数据并转发到VOFA+...")
        print("按Ctrl+C停止\n")
        
        try:
            while True:
                # 接收MAVLink消息
                msg = self.master.recv_match(type='NAMED_VALUE_FLOAT', blocking=True, timeout=1)
                
                if msg:
                    self.process_named_value_float(msg)
                    
        except KeyboardInterrupt:
            print("\n\n用户中断，正在关闭...")
        except Exception as e:
            print(f"\n错误: {e}")
        finally:
            self.close()
    
    def close(self):
        """关闭连接"""
        if self.vofa_socket:
            self.vofa_socket.close()
            print("VOFA+ TCP连接已关闭")
        
        if self.master:
            self.master.close()
            print("MAVLink连接已关闭")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='MAVLink到VOFA+数据转发器 (串口→TCP)')
    parser.add_argument('--mavlink', '-m', 
                        default='COM5:115200',
                        help='MAVLink串口连接 (默认: COM5:115200)')
    parser.add_argument('--vofa-host', '-H',
                        default='127.0.0.1',
                        help='VOFA+ TCP服务器地址 (默认: 127.0.0.1)')
    parser.add_argument('--vofa-port', '-p',
                        type=int,
                        default=1347,
                        help='VOFA+ TCP端口 (默认: 1347)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("MAVLink到VOFA+数据转发器 (串口→TCP)")
    print("=" * 60)
    print(f"MAVLink串口: {args.mavlink}")
    print(f"VOFA+ TCP: {args.vofa_host}:{args.vofa_port}")
    print("=" * 60)
    
    # 创建转发器并运行
    forwarder = MAVLinkToVOFA(
        mavlink_connection=args.mavlink,
        vofa_host=args.vofa_host,
        vofa_port=args.vofa_port
    )
    
    forwarder.run()


if __name__ == '__main__':
    main()
