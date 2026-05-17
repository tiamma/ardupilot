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
    plane.mode_qstabilize.update();
}

/*
  control QHOVER mode
 */
void ModeQHover::run()
{
    const float rudder_input = (float)plane.channel_rudder->get_control_in() / plane.channel_rudder->get_range();

    // 检测通道7，用于重置偏航角基准
    RC_Channel *ch7 = RC_Channels::rc_channel(6);  // 通道7 (索引从0开始，所以是6)
    if (ch7 != nullptr) {
        int16_t ch7_value = ch7->get_radio_in();
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
    // 横滚摇杆 → 偏航速率（deg/s）
    const float yaw_rate_dps = rudder_input * quadplane.command_model_pilot.get_rate() / 100.0f;
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
    // plane.stabilize_pitch();

    plane.stabilize_vtol_yaw(angle_error);

    // Center rudder
    output_rudder_and_steering(0.0);

    // possibly apply spin recovery
    quadplane.assist.output_spin_recovery();
}

#endif
