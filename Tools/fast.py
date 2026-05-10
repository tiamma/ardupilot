#!/usr/bin/env python3
"""
最速曲线（Brachistochrone Curve）绘制工具

最速曲线是连接两点之间，在重力作用下质点下滑时间最短的曲线。
这条曲线是一条摆线（Cycloid）。

数学原理：
- 摆线参数方程：
  x(θ) = a(θ - sin(θ))
  y(θ) = a(1 - cos(θ))
- 其中 a 是摆线的半径参数，θ 是参数角度
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import argparse

# 配置中文字体支持
def setup_chinese_font():
    """配置matplotlib中文字体"""
    import platform
    import matplotlib.font_manager as fm
    
    # 根据操作系统选择字体
    system = platform.system()
    
    if system == 'Windows':
        # Windows系统常用中文字体
        fonts = ['SimHei', 'Microsoft YaHei', 'SimSun', 'KaiTi']
    elif system == 'Darwin':  # macOS
        # macOS系统常用中文字体
        fonts = ['Arial Unicode MS', 'PingFang SC', 'Heiti SC', 'STHeiti']
    else:  # Linux
        # Linux系统常用中文字体
        fonts = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Droid Sans Fallback', 'Noto Sans CJK SC']
    
    # 获取系统所有可用字体
    available_fonts = [f.name for f in fm.fontManager.ttflist]
    
    # 查找第一个可用的中文字体
    for font in fonts:
        if font in available_fonts:
            plt.rcParams['font.sans-serif'] = [font]
            print(f"使用中文字体: {font}")
            break
    else:
        # 如果没有找到中文字体，使用默认字体并警告
        print("警告: 未找到中文字体，中文可能显示为方框")
        print("可用字体列表（部分）:")
        for font in available_fonts[:10]:
            print(f"  - {font}")
    
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 初始化中文字体
setup_chinese_font()


class BrachistochroneCurve:
    """最速曲线类"""
    
    def __init__(self, x_end=10, y_end=5):
        """
        初始化最速曲线
        
        参数:
            x_end: 终点x坐标
            y_end: 终点y坐标（正值表示向下）
        """
        self.x_end = x_end
        self.y_end = y_end
        self.a = None  # 摆线半径参数
        self.theta_max = None  # 最大角度
        self._calculate_parameters()
    
    def _calculate_parameters(self):
        """计算摆线参数"""
        # 通过数值方法求解参数 a 和 theta_max
        # 使得曲线通过 (x_end, y_end)
        
        # 使用迭代方法寻找合适的 theta_max
        for theta in np.linspace(0.1, 2*np.pi, 1000):
            # 对于给定的 theta，计算对应的 a
            a_candidate = self.x_end / (theta - np.sin(theta))
            y_candidate = a_candidate * (1 - np.cos(theta))
            
            # 检查是否接近目标 y 值
            if abs(y_candidate - self.y_end) < 0.01:
                self.a = a_candidate
                self.theta_max = theta
                break
        
        if self.a is None:
            # 如果没找到精确解，使用近似值
            self.theta_max = np.pi
            self.a = self.x_end / (self.theta_max - np.sin(self.theta_max))
    
    def get_curve(self, num_points=1000):
        """
        获取曲线上的点
        
        参数:
            num_points: 曲线点数
            
        返回:
            x, y: 曲线坐标数组
        """
        theta = np.linspace(0, self.theta_max, num_points)
        x = self.a * (theta - np.sin(theta))
        y = self.a * (1 - np.cos(theta))
        return x, y
    
    def get_time(self, g=9.81):
        """
        计算下滑时间
        
        参数:
            g: 重力加速度 (m/s²)
            
        返回:
            下滑总时间 (秒)
        """
        # 最速曲线的下滑时间公式
        time = self.theta_max * np.sqrt(self.a / g)
        return time
    
    def get_velocity_at_point(self, theta, g=9.81):
        """
        计算质点在某点的速度
        
        参数:
            theta: 参数角度
            g: 重力加速度
            
        返回:
            速度大小 (m/s)
        """
        y = self.a * (1 - np.cos(theta))
        v = np.sqrt(2 * g * y)
        return v


def plot_brachistochrone(x_end=10, y_end=5, show_comparison=True, animate=False):
    """
    绘制最速曲线
    
    参数:
        x_end: 终点x坐标
        y_end: 终点y坐标
        show_comparison: 是否显示与直线的对比
        animate: 是否显示动画
    """
    # 创建最速曲线
    curve = BrachistochroneCurve(x_end, y_end)
    x, y = curve.get_curve()
    
    # 创建图形
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # 绘制最速曲线
    ax.plot(x, y, 'b-', linewidth=2, label='最速曲线 (Brachistochrone)')
    
    if show_comparison:
        # 绘制直线对比
        x_line = np.array([0, x_end])
        y_line = np.array([0, y_end])
        ax.plot(x_line, y_line, 'r--', linewidth=2, label='直线')
        
        # 绘制圆弧对比（四分之一圆）
        radius = np.sqrt(x_end**2 + y_end**2)
        theta_circle = np.linspace(0, np.arctan2(y_end, x_end), 100)
        x_circle = radius * np.cos(theta_circle)
        y_circle = radius * np.sin(theta_circle)
        ax.plot(x_circle, y_circle, 'g-.', linewidth=2, label='圆弧')
    
    # 标记起点和终点
    ax.plot(0, 0, 'ko', markersize=10, label='起点')
    ax.plot(x_end, y_end, 'ro', markersize=10, label='终点')
    
    # 计算并显示下滑时间
    time_brach = curve.get_time()
    time_line = np.sqrt(2 * np.sqrt(x_end**2 + y_end**2) / 9.81)  # 直线下滑时间（近似）
    
    # 添加文本信息
    info_text = f'最速曲线下滑时间: {time_brach:.3f}秒\n'
    info_text += f'直线下滑时间（近似）: {time_line:.3f}秒\n'
    info_text += f'时间节省: {(time_line - time_brach):.3f}秒 ({(time_line - time_brach)/time_line*100:.1f}%)'
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
            fontsize=10, family='monospace')
    
    # 设置图形属性
    ax.set_xlabel('水平距离 (m)', fontsize=12)
    ax.set_ylabel('垂直距离 (m)', fontsize=12)
    ax.set_title('最速曲线 (Brachistochrone Curve)', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.invert_yaxis()  # 反转y轴，使向下为正
    
    plt.tight_layout()
    
    if animate:
        # 创建动画显示质点运动
        create_animation(curve, ax, fig)
    else:
        plt.show()


def create_animation(curve, ax, fig):
    """创建质点下滑动画"""
    x, y = curve.get_curve(num_points=200)
    
    # 创建质点
    particle, = ax.plot([], [], 'ro', markersize=15, label='质点')
    trail, = ax.plot([], [], 'r-', linewidth=1, alpha=0.5)
    
    # 速度文本
    velocity_text = ax.text(0.02, 0.85, '', transform=ax.transAxes,
                           bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7),
                           fontsize=10, family='monospace')
    
    trail_x, trail_y = [], []
    
    def init():
        particle.set_data([], [])
        trail.set_data([], [])
        velocity_text.set_text('')
        return particle, trail, velocity_text
    
    def animate(frame):
        # 更新质点位置
        particle.set_data([x[frame]], [y[frame]])
        
        # 更新轨迹
        trail_x.append(x[frame])
        trail_y.append(y[frame])
        trail.set_data(trail_x, trail_y)
        
        # 计算并显示速度
        theta = curve.theta_max * frame / len(x)
        v = curve.get_velocity_at_point(theta)
        velocity_text.set_text(f'速度: {v:.2f} m/s')
        
        return particle, trail, velocity_text
    
    anim = FuncAnimation(fig, animate, init_func=init,
                        frames=len(x), interval=20, blit=True, repeat=True)
    
    plt.show()


def plot_multiple_curves():
    """绘制多条不同终点的最速曲线"""
    fig, ax = plt.subplots(figsize=(14, 10))
    
    # 不同的终点
    endpoints = [
        (5, 3, 'blue'),
        (8, 5, 'green'),
        (10, 6, 'red'),
        (12, 8, 'purple'),
        (15, 10, 'orange')
    ]
    
    for x_end, y_end, color in endpoints:
        curve = BrachistochroneCurve(x_end, y_end)
        x, y = curve.get_curve()
        time = curve.get_time()
        ax.plot(x, y, color=color, linewidth=2, 
               label=f'终点({x_end}, {y_end}) - 时间: {time:.3f}s')
    
    # 标记所有起点
    ax.plot(0, 0, 'ko', markersize=12, label='起点', zorder=10)
    
    # 设置图形属性
    ax.set_xlabel('水平距离 (m)', fontsize=12)
    ax.set_ylabel('垂直距离 (m)', fontsize=12)
    ax.set_title('多条最速曲线对比', fontsize=14, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.invert_yaxis()
    
    plt.tight_layout()
    plt.show()


def plot_velocity_profile(x_end=10, y_end=5):
    """绘制速度变化曲线"""
    curve = BrachistochroneCurve(x_end, y_end)
    
    # 生成曲线点
    theta = np.linspace(0, curve.theta_max, 1000)
    x = curve.a * (theta - np.sin(theta))
    y = curve.a * (1 - np.cos(theta))
    v = curve.get_velocity_at_point(theta)
    
    # 创建子图
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # 上图：曲线形状
    ax1.plot(x, y, 'b-', linewidth=2)
    ax1.plot(0, 0, 'go', markersize=10, label='起点')
    ax1.plot(x_end, y_end, 'ro', markersize=10, label='终点')
    ax1.set_xlabel('水平距离 (m)', fontsize=12)
    ax1.set_ylabel('垂直距离 (m)', fontsize=12)
    ax1.set_title('最速曲线形状', fontsize=12, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.invert_yaxis()
    ax1.set_aspect('equal')
    
    # 下图：速度变化
    distance = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    cumulative_distance = np.concatenate([[0], np.cumsum(distance)])
    
    ax2.plot(cumulative_distance, v, 'r-', linewidth=2)
    ax2.fill_between(cumulative_distance, 0, v, alpha=0.3, color='red')
    ax2.set_xlabel('沿曲线的距离 (m)', fontsize=12)
    ax2.set_ylabel('速度 (m/s)', fontsize=12)
    ax2.set_title('速度变化曲线', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # 添加平均速度线
    avg_velocity = cumulative_distance[-1] / curve.get_time()
    ax2.axhline(y=avg_velocity, color='g', linestyle='--', 
               label=f'平均速度: {avg_velocity:.2f} m/s')
    ax2.legend()
    
    plt.tight_layout()
    plt.show()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='最速曲线绘制工具')
    parser.add_argument('--x', type=float, default=10, help='终点x坐标 (默认: 10)')
    parser.add_argument('--y', type=float, default=5, help='终点y坐标 (默认: 5)')
    parser.add_argument('--mode', type=str, default='single',
                       choices=['single', 'multiple', 'velocity', 'animate'],
                       help='绘制模式: single(单条), multiple(多条), velocity(速度), animate(动画)')
    parser.add_argument('--no-comparison', action='store_true',
                       help='不显示与直线的对比')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("最速曲线 (Brachistochrone Curve) 绘制工具")
    print("=" * 60)
    print(f"终点坐标: ({args.x}, {args.y})")
    print(f"绘制模式: {args.mode}")
    print("=" * 60)
    
    if args.mode == 'single':
        plot_brachistochrone(args.x, args.y, 
                           show_comparison=not args.no_comparison,
                           animate=False)
    elif args.mode == 'animate':
        plot_brachistochrone(args.x, args.y,
                           show_comparison=not args.no_comparison,
                           animate=True)
    elif args.mode == 'multiple':
        plot_multiple_curves()
    elif args.mode == 'velocity':
        plot_velocity_profile(args.x, args.y)


if __name__ == '__main__':
    main()
