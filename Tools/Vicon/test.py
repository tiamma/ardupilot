import numpy as np
import matplotlib.pyplot as plt

# ---------------- 全局参数 ----------------
M_PI = np.pi
CTRL_DT = 0.01
MAX_ERR_ANG = 45.0
MAX_OMEGA = 70.0
SIM_TOTAL = 3.0

# ---------------- PID 类 ----------------
class PID:
    def __init__(self):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.out_limit = 0
        self.integral_max = 0
        self.integral = 0.0
        self.last_err = 0.0

    def set_pid(self, p, i, d, out_lim, int_ratio=0.5):
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.out_limit = out_lim
        self.integral_max = out_lim * int_ratio
        self.reset()

    def calc(self, err, dt):
        P = self.Kp * err
        if abs(self.integral) < self.integral_max:
            self.integral += err * dt
        I = self.Ki * self.integral
        D = self.Kd * (err - self.last_err) / dt
        self.last_err = err
        out = P + I + D
        return np.clip(out, -self.out_limit, self.out_limit)

    def reset(self):
        self.integral = 0.0
        self.last_err = 0.0

# ---------------- 最速曲线：角度误差 → 目标角速度 ----------------
def brach_err2omega(ang_err):
    e_abs = abs(ang_err)
    tau = np.clip(e_abs / MAX_ERR_ANG, 0.0, 1.0)
    phi = 2 * M_PI * tau
    v_norm = (1.0 - np.cos(phi)) / 2.0
    sign = 1.0 if ang_err > 0 else -1.0
    return MAX_OMEGA * sign * v_norm

# ---------------- 仿真初始化 ----------------
ang_pid = PID()
omega_pid = PID()
ang_pid.set_pid(0.6, 0.08, 0.04, MAX_OMEGA)
omega_pid.set_pid(2.5, 0.15, 0.08, 100.0)

tar_angle = 30.0
now_angle = 0.0
now_omega = 0.0
inertia = 0.08

t_list, ang_list, omega_list, tar_omega_list = [], [], [], []

# ---------------- 仿真循环 ----------------
sim_steps = int(SIM_TOTAL / CTRL_DT)
for _ in range(sim_steps):
    t = _ * CTRL_DT
    ang_err = tar_angle - now_angle
    tar_omega = brach_err2omega(ang_err)
    omega_err = tar_omega - now_omega
    ctrl_out = omega_pid.calc(omega_err, CTRL_DT)

    # 被控对象惯性
    now_omega += ctrl_out * inertia * CTRL_DT
    now_angle += now_omega * CTRL_DT

    t_list.append(t)
    ang_list.append(now_angle)
    omega_list.append(now_omega)
    tar_omega_list.append(tar_omega)

# ---------------- 绘图 ----------------
plt.figure(figsize=(12, 7))

# 上：角度曲线
plt.subplot(2,1,1)
plt.plot(t_list, ang_list, label='实际角度', linewidth=2)
plt.axhline(tar_angle, color='r', linestyle='--', label='目标角度 30°')
plt.title('最速曲线 + 串级PID 角度响应')
plt.ylabel('角度 (°)')
plt.grid(True)
plt.legend()

# 下：角速度曲线
plt.subplot(2,1,2)
plt.plot(t_list, tar_omega_list, 'r--', label='最速期望角速度')
plt.plot(t_list, omega_list, 'g-', label='实际角速度')
plt.title('角速度跟随对比')
plt.ylabel('角速度 (°/s)')
plt.xlabel('时间 (s)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()