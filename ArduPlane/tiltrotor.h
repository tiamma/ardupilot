/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include "transition.h"
#include <AP_Logger/LogStructure.h>

class QuadPlane;
class AP_MotorsMulticopter;
class Tiltrotor_Transition;
class Tiltrotor
{
friend class QuadPlane;
friend class Plane;
friend class Tiltrotor_Transition;
public:

    Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors);

    bool enabled() const { return (enable > 0) && setup_complete;}

    void setup();

    void slew(float tilt);
    void binary_slew(bool forward);
    void update();
    void continuous_update();
    void binary_update();
    void vectoring();
    void bicopter_output();
    void tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul);
    void tilt_compensate(float *thrust, uint8_t num_motors);
    bool tilt_over_max_angle(void) const;

    bool is_motor_tilting(uint8_t motor) const {
        return tilt_mask.get() & (1U<<motor);
    }

    bool fully_fwd() const;
    bool fully_up() const;
    float tilt_max_change(bool up, bool in_flap_range = false) const;
    float get_fully_forward_tilt() const;
    float get_forward_flight_tilt() const;
    
    // update yaw target for tiltrotor transition
    void update_yaw_target();

    bool is_vectored() const { return enabled() && _is_vectored; }

    bool has_fw_motor() const { return _have_fw_motor; }

    bool has_vtol_motor() const { return _have_vtol_motor; }

    bool motors_active() const { return enabled() && _motors_active; }

    // true if the tilts have completed slewing
    // always return true if not enabled or not a continuous type
    bool tilt_angle_achieved() const { return !enabled() || (type != TILT_TYPE_CONTINUOUS) || angle_achieved; }

    // Write tiltrotor specific log
    void write_log();

    AP_Int8 enable;
    AP_Int16 tilt_mask;
    AP_Int16 max_rate_up_dps;
    AP_Int16 max_rate_down_dps;
    AP_Int8  max_angle_deg;
    AP_Int8  type;
    AP_Float tilt_yaw_angle;
    AP_Float fixed_angle;
    AP_Float fixed_gain;
    AP_Float flap_angle_deg;

    float angle_revise = 0.0f;
    float angle_k = 0.0f;
    float current_tilt;
    float current_throttle;
    bool _motors_active:1;
    float transition_yaw_cd;
    uint32_t transition_yaw_set_ms;
    bool _is_vectored;

    // types of tilt mechanisms
    enum {TILT_TYPE_CONTINUOUS    =0,
          TILT_TYPE_BINARY        =1,
          TILT_TYPE_VECTORED_YAW  =2,
          TILT_TYPE_BICOPTER      =3
    };

    static const struct AP_Param::GroupInfo var_info[];

    // 双旋翼串级PID控制参数
    // 外环：角度控制 (Pitch Angle → Desired Pitch Rate)
    AP_Float bicopter_pitch_angle_p;     // 角度P增益
    AP_Float bicopter_pitch_angle_i;     // 角度I增益
    AP_Float bicopter_pitch_angle_d;     // 角度D增益
    AP_Float bicopter_pitch_angle_imax;  // 角度积分限幅
    
    // 内环：角速度控制 (Pitch Rate Error → Motor Differential)
    AP_Float bicopter_pitch_rate_p;      // 角速度P增益
    AP_Float bicopter_pitch_rate_i;      // 角速度I增益
    AP_Float bicopter_pitch_rate_d;      // 角速度D增益
    AP_Float bicopter_pitch_rate_imax;   // 角速度积分限幅
    
    // 控制输出限制
    AP_Float bicopter_max_rate_dps;      // 最大期望角速度 (deg/s)
    AP_Float bicopter_max_motor_diff;    // 最大电机差分输出 (0-1)
    
    // 偏航控制参数
    // 外环：偏航角度控制 (Yaw Angle → Desired Yaw Rate)
    AP_Float bicopter_yaw_angle_p;       // 偏航角度P增益
    AP_Float bicopter_yaw_angle_i;       // 偏航角度I增益
    AP_Float bicopter_yaw_angle_d;       // 偏航角度D增益
    AP_Float bicopter_yaw_angle_imax;    // 偏航角度积分限幅
    
    // 内环：偏航角速度控制 (Yaw Rate Error → Motor Differential)
    AP_Float bicopter_yaw_rate_p;        // 偏航角速度P增益
    AP_Float bicopter_yaw_rate_i;        // 偏航角速度I增益
    AP_Float bicopter_yaw_rate_d;        // 偏航角速度D增益
    AP_Float bicopter_yaw_rate_imax;     // 偏航角速度积分限幅
    
    AP_Float bicopter_max_yaw_rate_dps;  // 最大期望偏航角速度 (deg/s)
    
    // 速度控制参数 (Velocity → Pitch Angle)
    AP_Float bicopter_vel_p;             // 速度P增益
    AP_Float bicopter_vel_i;             // 速度I增益
    AP_Float bicopter_vel_d;             // 速度D增益
    AP_Float bicopter_vel_imax;          // 速度积分限幅
    AP_Float bicopter_max_vel_mps;       // 最大期望速度 (m/s)
    AP_Float bicopter_max_vel_pitch_deg; // 速度控制最大俯仰角 (度)
    AP_Int8 velocity_pitch_sign;         // 速度控制俯仰方向 (1=正向, -1=反向)
    
    // 电机方向控制参数 (1, 0, -1)
    AP_Int8 left_pitch_sign;             // 左电机俯仰方向 (1=正向, 0=禁用, -1=反向)
    AP_Int8 right_pitch_sign;            // 右电机俯仰方向 (1=正向, 0=禁用, -1=反向)
    AP_Int8 left_yaw_sign;               // 左电机偏航方向 (1=正向, 0=禁用, -1=反向)
    AP_Int8 right_yaw_sign;              // 右电机偏航方向 (1=正向, 0=禁用, -1=反向)
    
    // 轨迹规划参数
    AP_Int8 enable_trajectory;           // 启用轨迹规划 (0=禁用, 1=启用)
    AP_Float trajectory_max_rate;        // 最大角速度 (度/秒)
    AP_Float trajectory_max_accel;       // 最大角加速度 (度/秒²)
    
    // 倾转角度闭环控制PID参数
    AP_Float tilt_ts_angle_p;            // 倾转角度P增益
    AP_Float tilt_ts_angle_i;            // 倾转角度I增益
    AP_Float tilt_ts_angle_d;            // 倾转角度D增益
    AP_Float tilt_ts_angle_imax;         // 倾转角度积分限幅
    AP_Float tilt_ts_correction_max;     // PID修正量最大值
    AP_Int8 vofa_enable;                 // 启用VOFA日志输出 (0=禁用, 1=启用)
    AP_Float tilt_angle_rate_max;        // 通道2控制目标角度的最大变化率 (度/秒)
    AP_Float feedforward_sign;           // 前馈输出符号控制 (1.0=正向, -1.0=反向)
    AP_Float tilt_angle_scale;           // 通道16控制倾转角度的缩放系数 (度)

private:

    // 轨迹规划状态变量
    float traj_start_angle;              // 轨迹起始角度
    float traj_end_angle;                // 轨迹目标角度
    float traj_start_time;               // 轨迹开始时间 (毫秒)
    float traj_total_time;               // 轨迹总时间 (秒)
    float traj_accel_time;               // 加速时间 (秒)
    float traj_constant_time;            // 匀速时间 (秒)
    float traj_max_rate_actual;          // 实际最大角速度
    bool traj_has_constant_phase;        // 是否有匀速阶段
    bool traj_active;                    // 轨迹是否激活
    
    // 通道12控制目标角度状态变量
    float manual_target_tilt_angle;      // 手动控制的目标倾转角度
    uint32_t last_tilt_angle_update_ms;  // 上次角度更新时间
    float target_tilt_angle;             // 当前目标倾转角度（度）

    // Tiltrotor specific log message
    struct PACKED log_tiltrotor {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float pitch_angle_error;      // 俯仰角度误差 (度)
        float pitch_desired_rate;     // 期望俯仰角速度 (度/秒)
        float pitch_differential;     // 俯仰差分输出 (-1 to 1)
        float yaw_angle_error;        // 偏航角度误差 (度)
        float yaw_desired_rate;       // 期望偏航角速度 (度/秒)
        float yaw_differential;       // 偏航差分输出 (-1 to 1)
        float left_motor_output;      // 左电机输出 (0-1000)
        float right_motor_output;     // 右电机输出 (0-1000)
        float gyro_y_raw;             // 陀螺仪Y轴原始值 (rad/s)
    };

    bool setup_complete;

    // true if a fixed forward motor is setup
    bool _have_fw_motor;

    // true if all motors tilt with no fixed VTOL motor
    bool _have_vtol_motor;

    // true if the current tilt angle is equal to the desired
    // with slow tilt rates the tilt angle can lag
    bool angle_achieved;
    
    // refences for convenience
    QuadPlane& quadplane;
    AP_MotorsMulticopter*& motors;

    Tiltrotor_Transition* transition;
    
    // 双旋翼串级PID控制状态变量 - 俯仰
    float bicopter_angle_integral;        // 俯仰角度环积分项
    float bicopter_rate_integral;         // 俯仰角速度环积分项
    float bicopter_last_pitch_error;      // 上次俯仰角度误差（用于微分）
    float bicopter_last_rate_error;       // 上次俯仰角速度误差（用于微分）
    
    // 双旋翼串级PID控制状态变量 - 偏航
    float bicopter_yaw_angle_integral;    // 偏航角度环积分项
    float bicopter_yaw_rate_integral;     // 偏航角速度环积分项
    float bicopter_last_yaw_error;        // 上次偏航角度误差（用于微分）
    float bicopter_last_yaw_rate_error;   // 上次偏航角速度误差（用于微分）
    
    // 速度控制状态变量
    float bicopter_vel_integral;          // 速度环积分项
    float bicopter_last_vel_error;        // 上次速度误差（用于微分）
    float bicopter_vel_feedforward_filtered; // 速度前馈滤波后的值
    
    uint32_t bicopter_last_update_ms;     // 上次更新时间
    uint32_t bicopter_motor_update_counter; // 电机更新计数器（奇偶交替）
    
    // 双旋翼日志变量
    float log_pitch_angle_error;          // 俯仰角度误差
    float log_pitch_desired_rate;         // 期望俯仰角速度
    float log_pitch_differential;         // 俯仰差分输出
    float log_pitch_rate_error;           // 俯仰角速度误差
    float log_pitch_rate_integral;        // 俯仰角速度积分项
    float log_pitch_rate_p;               // 俯仰角速度P输出
    float log_yaw_angle_error;            // 偏航角度误差
    float log_yaw_desired_rate;           // 期望偏航角速度
    float log_yaw_differential;           // 偏航差分输出
    float log_left_motor_output;          // 左电机输出
    float log_right_motor_output;         // 右电机输出
    float log_velocity_error;             // 速度误差
    float log_current_velocity;           // 当前速度
    float log_desired_velocity;           // 期望速度
    float log_velocity_pitch_cmd;         // 速度控制输出的俯仰角指令
    float log_gyro_y_raw;                 // 陀螺仪Y轴原始值
    
    // 双旋翼控制函数
    void bicopter_update();
    
    // 垂直姿态飞行PID计算
    void vtol_pid_arcsin_rate(float base_output, float zero_out, 
                           float &pitch_differential, float &yaw_differential,
                           float &pitch_angle_error, float &desired_pitch_rate,
                           float &yaw_angle_error, float &desired_yaw_rate,
                           float &left_motor_output, float &right_motor_output);

    // 垂直姿态飞行PID计算
    void vtol_pid_get_rate(float base_output, float zero_out, 
                           float &pitch_differential, float &yaw_differential,
                           float &pitch_angle_error, float &desired_pitch_rate,
                           float &yaw_angle_error, float &desired_yaw_rate,
                           float &left_motor_output, float &right_motor_output);
    
    // 切换姿态倾转角度闭环控制 (基于MPU6050反馈)
    void transition_pid_get_rate();

    void transition_get_rate(float zero_out, float pitch_angle_error, float &pitch_differential, float &left_motor_output);

};

// Transition for separate left thrust quadplanes
class Tiltrotor_Transition : public SLT_Transition
{
friend class Tiltrotor;
public:

    Tiltrotor_Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors, Tiltrotor& _tiltrotor):SLT_Transition(_quadplane, _motors), tiltrotor(_tiltrotor) {};

    bool update_yaw_target(float& yaw_target_cd) override;

    bool show_vtol_view() const override;

    bool use_multirotor_control_in_fwd_transition() const override;

private:

    Tiltrotor& tiltrotor;

};
