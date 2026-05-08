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

private:

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
    
    uint32_t bicopter_last_update_ms;     // 上次更新时间
    
    // 双旋翼日志变量
    float log_pitch_angle_error;          // 俯仰角度误差
    float log_pitch_desired_rate;         // 期望俯仰角速度
    float log_pitch_differential;         // 俯仰差分输出
    float log_yaw_angle_error;            // 偏航角度误差
    float log_yaw_desired_rate;           // 期望偏航角速度
    float log_yaw_differential;           // 偏航差分输出
    float log_left_motor_output;          // 左电机输出
    float log_right_motor_output;         // 右电机输出
    
    // 双旋翼控制函数
    void bicopter_update();

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
