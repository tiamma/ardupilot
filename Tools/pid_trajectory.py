#!/usr/bin/env python3
"""
PID串级控制 + 最速曲线轨迹规划

将最速曲线（Brachistochrone）算法应用到双旋翼的角度控制中，
生成最优的角度变化轨迹，使得从当前角度到目标角度的时间最短。

应用场景：
- 双旋翼俯仰角控制
- 双旋翼偏航角控制
- 快速姿态调整
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import argparse

# 配置中文字体
def setup_chinese_font():
    """配置matplotlib中文字体"""
    import platform
    import matplotlib.font_manager as fm
    
    system = platform.system()
    
    if system == 'Windows':
        fonts = ['SimHei', 'Microsoft YaHei', 'SimSun', 'KaiTi']
    elif system == 'Darwin':
        fonts = ['Arial Unicode MS', 'PingFang SC', 'Heiti SC', 'STHeiti']
    else:
        fonts = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Droid Sans Fallback', 'Noto Sans CJK SC']
    
    available_fonts = [f.name for f in fm.fontManager.ttflist]
    
    for font in fonts:
        if font in available_fonts:
            plt.rcParams['font.sans-serif'] = [font]
            print(f"使用中文字体: {font}")
            break
    else:
        print("警告: 未找到中文字体")
    
    plt.rcParams['axes.unicode_minus'] = False

setup_chinese_font()


class BrachistochroneTrajectory:
    """基于最速曲线的轨迹规划器"""
    
    def __init__(self, angle_start=0, angle_end=45, max_rate=90, max_accel=180):
        """
        初始化轨迹规划器
        
        参数:
            angle_start: 起始角度 (度)
            angle_end: 目标角度 (度)
            max_rate: 最大角速度 (度/秒)
            max_accel: 最大角加速度 (度/秒²)
        """
        self.angle_start = angle_start
        self.angle_end = angle_end
        self.angle_delta = angle_end - angle_start
        self.max_rate = max_rate
        self.max_accel = max_accel
        
        # 计算轨迹参数
        self._calculate_trajectory()
    
    def _calculate_trajectory(self):
        """计算最优轨迹"""
        # 使用S曲线（摆线变形）生成平滑轨迹
        # 分为三个阶段：加速、匀速、减速
        
        # 计算加速和减速时间
        t_accel = self.max_rate / self.max_accel
        
        # 加速和减速阶段的角度变化
        angle_accel = 0.5 * self.max_accel * t_accel**2
        
        # 检查是否需要匀速阶段
        if 2 * angle_accel < abs(self.angle_delta):
            # 需要匀速阶段
            self.has_constant_phase = True
            angle_constant = abs(self.angle_delta) - 2 * angle_accel
            t_constant = angle_constant / self.max_rate
            self.total_time = 2 * t_accel + t_constant
        else:
            # 不需要匀速阶段（三角形速度曲线）
            self.has_constant_phase = False
            # 重新计算加速时间
            t_accel = np.sqrt(abs(self.angle_delta) / self.max_accel)
            self.max_rate = self.max_accel * t_accel
            self.total_time = 2 * t_accel
        
        self.t_accel = t_accel
        self.t_constant = t_constant if self.has_constant_phase else 0
    
    def get_angle(self, t):
        """
        获取指定时间的角度
        
        参数:
            t: 时间 (秒)
            
        返回:
            angle: 角度 (度)
        """
        if t <= 0:
            return self.angle_start
        elif t >= self.total_time:
            return self.angle_end
        
        sign = 1 if self.angle_delta > 0 else -1
        
        if t < self.t_accel:
            # 加速阶段
            angle = self.angle_start + sign * 0.5 * self.max_accel * t**2
        elif self.has_constant_phase and t < (self.t_accel + self.t_constant):
            # 匀速阶段
            t_const = t - self.t_accel
            angle_accel = sign * 0.5 * self.max_accel * self.t_accel**2
            angle = self.angle_start + angle_accel + sign * self.max_rate * t_const
        else:
            # 减速阶段
            if self.has_constant_phase:
                t_decel = t - self.t_accel - self.t_constant
            else:
                t_decel = t - self.t_accel
            
            # 从终点反推
            remaining_time = self.total_time - t
            angle = self.angle_end - sign * 0.5 * self.max_accel * remaining_time**2
        
        return angle
    
    def get_rate(self, t):
        """
        获取指定时间的角速度
        
        参数:
            t: 时间 (秒)
            
        返回:
            rate: 角速度 (度/秒)
        """
        if t <= 0 or t >= self.total_time:
            return 0
        
        sign = 1 if self.angle_delta > 0 else -1
        
        if t < self.t_accel:
            # 加速阶段
            rate = sign * self.max_accel * t
        elif self.has_constant_phase and t < (self.t_accel + self.t_constant):
            # 匀速阶段
            rate = sign * self.max_rate
        else:
            # 减速阶段
            remaining_time = self.total_time - t
            rate = sign * self.max_accel * remaining_time
        
        return rate
    
    def get_accel(self, t):
        """
        获取指定时间的角加速度
        
        参数:
            t: 时间 (秒)
            
        返回:
            accel: 角加速度 (度/秒²)
        """
        if t <= 0 or t >= self.total_time:
            return 0
        
        sign = 1 if self.angle_delta > 0 else -1
        
        if t < self.t_accel:
            # 加速阶段
            return sign * self.max_accel
        elif self.has_constant_phase and t < (self.t_accel + self.t_constant):
            # 匀速阶段
            return 0
        else:
            # 减速阶段
            return -sign * self.max_accel
    
    def get_trajectory(self, dt=0.02):
        """
        获取完整轨迹
        
        参数:
            dt: 时间步长 (秒)
            
        返回:
            t, angle, rate, accel: 时间、角度、角速度、角加速度数组
        """
        t = np.arange(0, self.total_time + dt, dt)
        angle = np.array([self.get_angle(ti) for ti in t])
        rate = np.array([self.get_rate(ti) for ti in t])
        accel = np.array([self.get_accel(ti) for ti in t])
        
        return t, angle, rate, accel


class CascadePIDController:
    """串级PID控制器"""
    
    def __init__(self, angle_p=4.5, angle_i=0.5, angle_d=0.1,
                 rate_p=0.15, rate_i=0.05, rate_d=0.01):
        """
        初始化PID控制器
        
        参数:
            angle_p, angle_i, angle_d: 角度环PID参数
            rate_p, rate_i, rate_d: 角速度环PID参数
        """
        # 角度环PID参数
        self.angle_p = angle_p
        self.angle_i = angle_i
        self.angle_d = angle_d
        
        # 角速度环PID参数
        self.rate_p = rate_p
        self.rate_i = rate_i
        self.rate_d = rate_d
        
        # 积分项
        self.angle_integral = 0
        self.rate_integral = 0
        
        # 上次误差（用于微分）
        self.last_angle_error = 0
        self.last_rate_error = 0
    
    def update(self, target_angle, current_angle, current_rate, dt):
        """
        更新控制器
        
        参数:
            target_angle: 目标角度
            current_angle: 当前角度
            current_rate: 当前角速度
            dt: 时间步长
            
        返回:
            control_output: 控制输出
        """
        # 外环：角度控制
        angle_error = target_angle - current_angle
        
        self.angle_integral += angle_error * dt
        angle_d_term = (angle_error - self.last_angle_error) / dt
        self.last_angle_error = angle_error
        
        # 期望角速度（外环输出）
        desired_rate = (self.angle_p * angle_error + 
                       self.angle_i * self.angle_integral +
                       self.angle_d * angle_d_term)
        
        # 内环：角速度控制
        rate_error = desired_rate - current_rate
        
        # 积分清零策略
        if abs(angle_error) < 1.0:
            self.rate_integral = 0
        else:
            self.rate_integral += rate_error * dt
        
        rate_d_term = (rate_error - self.last_rate_error) / dt
        self.last_rate_error = rate_error
        
        # 控制输出（内环输出）
        control_output = (self.rate_p * rate_error +
                         self.rate_i * self.rate_integral +
                         self.rate_d * rate_d_term)
        
        return control_output, desired_rate
    
    def reset(self):
        """重置控制器状态"""
        self.angle_integral = 0
        self.rate_integral = 0
        self.last_angle_error = 0
        self.last_rate_error = 0


def simulate_control(trajectory, controller, dt=0.02, noise_level=0.1):
    """
    模拟PID控制过程
    
    参数:
        trajectory: 轨迹规划器
        controller: PID控制器
        dt: 时间步长
        noise_level: 噪声水平
        
    返回:
        仿真结果
    """
    # 获取参考轨迹
    t_ref, angle_ref, rate_ref, accel_ref = trajectory.get_trajectory(dt)
    
    # 初始化状态
    current_angle = trajectory.angle_start
    current_rate = 0
    
    # 记录数据
    t_sim = []
    angle_sim = []
    rate_sim = []
    control_sim = []
    desired_rate_sim = []
    
    controller.reset()
    
    for i, t in enumerate(t_ref):
        # 获取目标角度
        target_angle = angle_ref[i]
        
        # PID控制
        control_output, desired_rate = controller.update(
            target_angle, current_angle, current_rate, dt)
        
        # 简化的动力学模型（一阶系统）
        # 角加速度 = 控制输出 + 噪声
        accel = control_output + np.random.normal(0, noise_level)
        
        # 更新状态
        current_rate += accel * dt
        current_angle += current_rate * dt
        
        # 记录数据
        t_sim.append(t)
        angle_sim.append(current_angle)
        rate_sim.append(current_rate)
        control_sim.append(control_output)
        desired_rate_sim.append(desired_rate)
    
    return {
        't': np.array(t_sim),
        'angle': np.array(angle_sim),
        'rate': np.array(rate_sim),
        'control': np.array(control_sim),
        'desired_rate': np.array(desired_rate_sim),
        'angle_ref': angle_ref,
        'rate_ref': rate_ref
    }


def plot_trajectory_planning(angle_start=0, angle_end=45, max_rate=90, max_accel=180):
    """绘制轨迹规划结果"""
    
    # 创建轨迹规划器
    traj = BrachistochroneTrajectory(angle_start, angle_end, max_rate, max_accel)
    
    # 获取轨迹
    t, angle, rate, accel = traj.get_trajectory()
    
    # 创建图形
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # 角度曲线
    axes[0].plot(t, angle, 'b-', linewidth=2, label='角度轨迹')
    axes[0].axhline(y=angle_start, color='g', linestyle='--', alpha=0.5, label='起始角度')
    axes[0].axhline(y=angle_end, color='r', linestyle='--', alpha=0.5, label='目标角度')
    axes[0].set_ylabel('角度 (度)', fontsize=12)
    axes[0].set_title(f'最优轨迹规划: {angle_start}° → {angle_end}° (总时间: {traj.total_time:.3f}秒)', 
                     fontsize=14, fontweight='bold')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # 角速度曲线
    axes[1].plot(t, rate, 'g-', linewidth=2, label='角速度')
    axes[1].axhline(y=max_rate if angle_end > angle_start else -max_rate, 
                   color='r', linestyle='--', alpha=0.5, label='最大角速度')
    axes[1].set_ylabel('角速度 (度/秒)', fontsize=12)
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # 角加速度曲线
    axes[2].plot(t, accel, 'r-', linewidth=2, label='角加速度')
    axes[2].axhline(y=max_accel if angle_end > angle_start else -max_accel, 
                   color='r', linestyle='--', alpha=0.5, label='最大角加速度')
    axes[2].axhline(y=-max_accel if angle_end > angle_start else max_accel, 
                   color='r', linestyle='--', alpha=0.5)
    axes[2].set_xlabel('时间 (秒)', fontsize=12)
    axes[2].set_ylabel('角加速度 (度/秒²)', fontsize=12)
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    # 标注阶段
    if traj.has_constant_phase:
        axes[0].axvline(x=traj.t_accel, color='k', linestyle=':', alpha=0.5)
        axes[0].axvline(x=traj.t_accel + traj.t_constant, color='k', linestyle=':', alpha=0.5)
        axes[0].text(traj.t_accel/2, angle_start + 0.1*traj.angle_delta, 
                    '加速', ha='center', fontsize=10)
        axes[0].text(traj.t_accel + traj.t_constant/2, 
                    angle_start + 0.5*traj.angle_delta, 
                    '匀速', ha='center', fontsize=10)
        axes[0].text(traj.t_accel + traj.t_constant + traj.t_accel/2, 
                    angle_start + 0.9*traj.angle_delta, 
                    '减速', ha='center', fontsize=10)
    
    plt.tight_layout()
    plt.show()


def plot_pid_control(angle_start=0, angle_end=45, max_rate=90, max_accel=180):
    """绘制PID控制仿真结果"""
    
    # 创建轨迹和控制器
    traj = BrachistochroneTrajectory(angle_start, angle_end, max_rate, max_accel)
    controller = CascadePIDController()
    
    # 仿真
    result = simulate_control(traj, controller)
    
    # 创建图形
    fig, axes = plt.subplots(4, 1, figsize=(14, 12))
    
    # 角度跟踪
    axes[0].plot(result['t'], result['angle_ref'], 'b--', linewidth=2, 
                label='参考轨迹', alpha=0.7)
    axes[0].plot(result['t'], result['angle'], 'r-', linewidth=2, 
                label='实际角度')
    axes[0].set_ylabel('角度 (度)', fontsize=12)
    axes[0].set_title('PID串级控制 + 最速曲线轨迹跟踪', fontsize=14, fontweight='bold')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # 角度误差
    angle_error = result['angle_ref'] - result['angle']
    axes[1].plot(result['t'], angle_error, 'r-', linewidth=2)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('角度误差 (度)', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    
    # 角速度跟踪
    axes[2].plot(result['t'], result['rate_ref'], 'b--', linewidth=2, 
                label='参考角速度', alpha=0.7)
    axes[2].plot(result['t'], result['desired_rate'], 'g:', linewidth=2, 
                label='期望角速度(外环输出)')
    axes[2].plot(result['t'], result['rate'], 'r-', linewidth=2, 
                label='实际角速度')
    axes[2].set_ylabel('角速度 (度/秒)', fontsize=12)
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    # 控制输出
    axes[3].plot(result['t'], result['control'], 'purple', linewidth=2, 
                label='控制输出(内环输出)')
    axes[3].set_xlabel('时间 (秒)', fontsize=12)
    axes[3].set_ylabel('控制量', fontsize=12)
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)
    
    # 计算性能指标
    rmse = np.sqrt(np.mean(angle_error**2))
    max_error = np.max(np.abs(angle_error))
    settling_time = result['t'][-1]
    
    info_text = f'性能指标:\n'
    info_text += f'RMSE: {rmse:.3f}°\n'
    info_text += f'最大误差: {max_error:.3f}°\n'
    info_text += f'调节时间: {settling_time:.3f}s'
    
    axes[0].text(0.02, 0.98, info_text, transform=axes[0].transAxes,
                verticalalignment='top', bbox=dict(boxstyle='round', 
                facecolor='wheat', alpha=0.5), fontsize=10, family='monospace')
    
    plt.tight_layout()
    plt.show()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='PID串级控制 + 最速曲线轨迹规划')
    parser.add_argument('--start', type=float, default=0, help='起始角度 (默认: 0)')
    parser.add_argument('--end', type=float, default=45, help='目标角度 (默认: 45)')
    parser.add_argument('--max-rate', type=float, default=90, help='最大角速度 (默认: 90 deg/s)')
    parser.add_argument('--max-accel', type=float, default=180, help='最大角加速度 (默认: 180 deg/s²)')
    parser.add_argument('--mode', type=str, default='control',
                       choices=['trajectory', 'control'],
                       help='模式: trajectory(轨迹规划), control(PID控制)')
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("PID串级控制 + 最速曲线轨迹规划")
    print("=" * 70)
    print(f"起始角度: {args.start}°")
    print(f"目标角度: {args.end}°")
    print(f"最大角速度: {args.max_rate} deg/s")
    print(f"最大角加速度: {args.max_accel} deg/s²")
    print(f"模式: {args.mode}")
    print("=" * 70)
    
    if args.mode == 'trajectory':
        plot_trajectory_planning(args.start, args.end, args.max_rate, args.max_accel)
    else:
        plot_pid_control(args.start, args.end, args.max_rate, args.max_accel)


if __name__ == '__main__':
    main()
