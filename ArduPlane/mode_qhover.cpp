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
    yaw_aileron_active = false;
    yaw_angle_offset_deg = plane.ahrs.yaw_sensor * 0.01f;
    return true;
}

void ModeQHover::update()
{
    float tilt_angle_deg = plane.tilt_angle_cd * 0.01f;
    if (tilt_angle_deg < 30.0f) {
        plane.mode_qstabilize.update();
    }
    if (tilt_angle_deg > 30.0f) {
        plane.mode_fbwa.update();
        // 检测通道12状态
        // int ch12_value = RC_Channels::get_radio_in(11); // Channel 12 (index 11)
        // bool ch12_high = (ch12_value > 1500);

        // // 检查CH12状态决定俯仰控制
        // if (ch12_high) {
        //     // CH12高位：锁定nav_pitch_cd = 5度
        //     plane.nav_pitch_cd = 800;  // 5度 × 100 = 500 centidegrees
        // }
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
        // 检测通道12状态
        // int ch12_value = RC_Channels::get_radio_in(11); // Channel 12 (index 11)
        // bool ch12_high = (ch12_value > 1500);

        // 检查CH12状态决定俯仰控制
        // if (ch12_high) {
        //     // CH12高位：锁定nav_pitch_cd = 5度
        //     plane.nav_pitch_cd = 800;  // 5度 × 100 = 500 centidegrees
        // }
        
        // Run base class function and then output throttle
        Mode::run();
        output_pilot_throttle();
        plane.quadplane.assign_tilt_to_fwd_thr();
        quadplane.hold_hover(quadplane.get_pilot_desired_climb_rate_cms());

    } else {
        // ========== 原有VTOL悬停控制 ==========
        const float rudder_input = (float)plane.channel_rudder->get_control_in() / plane.channel_rudder->get_range();
        
        // 检测通道7，用于重置偏航角基准
        int16_t ch7_value = 0;
        RC_Channel *ch7 = RC_Channels::rc_channel(6);  // 通道7 (索引从0开始，所以是6)
        if (ch7 != nullptr) {
            ch7_value = ch7->get_radio_in();
            // 如果通道7 > 1700 (高位)，重置偏航角基准
            if (ch7_value > 1700) {
                yaw_angle_offset_deg = plane.ahrs.yaw_sensor * 0.01f;
            }
        }

        quadplane.assist.check_VTOL_recovery();

        const uint32_t now = AP_HAL::millis();
        if (quadplane.tailsitter.in_vtol_transition(now)) {
            // Tailsitters in FW pull up phase of VTOL transition run FW controllers
            Mode::run();
            return;
        }

        if (quadplane.throttle_wait) {
            quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
            attitude_control->set_throttle_out(0, true, 0);
            quadplane.relax_attitude_control();
            pos_control->relax_z_controller(0);
        } else {
            plane.quadplane.assign_tilt_to_fwd_thr();
            quadplane.hold_hover(quadplane.get_pilot_desired_climb_rate_cms());
        }

        float yaw_angle = plane.ahrs.yaw_sensor * 0.01f;
        float angle_error = fmodf((float)(yaw_angle - yaw_angle_offset_deg + 180), 360.0f) - 180.0f;
        
        // 每隔1秒输出调试信息
        static uint32_t last_debug_ms = 0;
        if (now - last_debug_ms >= 1000) {
            last_debug_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "QHOVER: angle_error=%.2f ch7=%d offset=%.2f tilt=%.2f", 
                         (double)angle_error, (int)ch7_value, (double)yaw_angle_offset_deg, (double)tilt_angle_deg);
        }
        
        // 横滚摇杆 → 偏航速率（deg/s）
        // get_rate() 返回的就是 deg/s，不需要除以100
        const float yaw_rate_dps = rudder_input * quadplane.command_model_pilot.get_rate();
        // 累积到目标偏航角（度）
        float max_change = 0.0f;

        if (yaw_rate_dps * plane.G_Dt > 0) {
            max_change = (float)MIN(yaw_rate_dps * plane.G_Dt, 90);
        } else if (yaw_rate_dps * plane.G_Dt < 0) {
            max_change = (float)MAX(-90, yaw_rate_dps * plane.G_Dt);
        }

         
        
        plane.nav_yaw_cd = yaw_angle_offset_deg + max_change;
        plane.yaw_error_cd = angle_error;
        
        // Stabilize with fixed wing surfaces
        // plane.stabilize_roll();
        plane.stabilize_pitch();
        plane.stabilize_vtol_yaw(angle_error);
        // Center rudder
        output_rudder_and_steering(0.0);
        // possibly apply spin recovery
        quadplane.assist.output_spin_recovery();
    }
}

#endif
