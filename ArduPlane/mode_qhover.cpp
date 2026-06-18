#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQHover::_enter()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_speed_z_max_up*100, quadplane.pilot_accel_z*100);
    quadplane.set_climb_rate_cms(0);
    quadplane.init_throttle_wait();
    return true;
}

void ModeQHover::update()
{
    float tilt_angle_deg = plane.tilt_angle_cd * 0.01f;
    if (tilt_angle_deg <= 30.0f) {
        plane.mode_qstabilize.update();
    }
    if (tilt_angle_deg > 30.0f) {
        plane.mode_fbwa.update();
    }
}

/*
  control QHOVER mode
 */
void ModeQHover::run()
{
    // 获取当前倾转角度（centidegrees → degrees）
    float tilt_angle_deg = plane.tilt_angle_cd * 0.01f;
    
    // 判断是否使用FBWA风格控制（倾转角度 > 30度）
    if (tilt_angle_deg > 30.0f) {
        // ========== FBWA风格控制 ==========
        // set nav_roll and nav_pitch using sticks
        plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
        plane.update_load_factor();
        
        // CH12低位：使用摇杆控制俯仰
        float pitch_input = plane.channel_pitch->norm_input();
        if (pitch_input > 0) {
            plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max * 100;
        } else {
            plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min * 100);
        }
        plane.adjust_nav_pitch_throttle();
        plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, 
                                                plane.pitch_limit_min * 100, 
                                                plane.aparm.pitch_limit_max.get() * 100);
        // Run base class function and then output throttle
        Mode::run();
        
        const float throttle = plane.get_throttle_input(true);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    } else {
        // ========== 原有VTOL悬停控制 ==========
        const float rudder_input = (float)plane.channel_rudder->get_control_in() / plane.channel_rudder->get_range();
        // 检测通道7，用于重置偏航角基准
        quadplane.assist.check_VTOL_recovery();

        if (quadplane.throttle_wait) {
            quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
            attitude_control->set_throttle_out(0, true, 0);
            quadplane.relax_attitude_control();
            pos_control->relax_z_controller(0);
        } else {
            plane.quadplane.assign_tilt_to_fwd_thr();
            quadplane.hold_hover(quadplane.get_pilot_desired_climb_rate_cms());
        }

        // Stabilize with fixed wing surfaces
        // plane.stabilize_roll();
        // plane.stabilize_pitch();
        
        const float desired_yaw_rate_dps = rudder_input * plane.g2.vtol_yaw_input_rate;
        // 使用纯角速度控制（无角度环）
        // 根据倾转角度计算缩放系数：0°时=1.0，30°时=0.0
        // 线性映射：scaling = 1.0 - (tilt_angle / 30.0)
        const float max_tilt_for_yaw = 30.0f;  // 最大有效倾转角度
        const float scaling = constrain_float(1.0f - (tilt_angle_deg / max_tilt_for_yaw), 0.0f, 1.0f);

        plane.stabilize_vtol_yaw_rate(desired_yaw_rate_dps, scaling);
        // Center rudder
        output_rudder_and_steering(0.0);
        // possibly apply spin recovery
        quadplane.assist.output_spin_recovery();
    }
}

#endif
