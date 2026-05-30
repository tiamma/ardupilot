#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQStabilize::_enter()
{
    quadplane.throttle_wait = false;
    yaw_aileron_active = false;
    yaw_angle_offset_deg = plane.ahrs.yaw_sensor * 0.01f;
    return true;
}

void ModeQStabilize::update()
{
    // set nav_roll and nav_pitch using sticks
    // Beware that QuadPlane::tailsitter_check_input (called from Plane::read_radio)
    // may alter the control_in values for roll and yaw, but not the corresponding
    // radio_in values. This means that the results for norm_input would not necessarily
    // be correct for tailsitters, so get_control_in() must be used instead.
    // normalize control_input to [-1,1]
    const float roll_input = (float)plane.channel_roll->get_control_in() / plane.channel_roll->get_range();
    const float pitch_input = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();

    // then scale to target angles in centidegrees
    if (plane.quadplane.tailsitter.active()) {
        // tailsitters are different
        set_tailsitter_roll_pitch(roll_input, pitch_input);
        return;
    }

    // if (!plane.quadplane.option_is_set(QuadPlane::OPTION::INGORE_FW_ANGLE_LIMITS_IN_Q_MODES)) {
    //     // by default angles are also constrained by forward flight limits
    //     set_limited_roll_pitch(roll_input, pitch_input);
    // } else {
        // use angle max for both roll and pitch
    plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
    plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
    // }
}

// quadplane stabilize mode
void ModeQStabilize::run()
{
    // ========== 原有VTOL悬停控制 ==========
    const float rudder_input = (float)plane.channel_rudder->get_control_in() / plane.channel_rudder->get_range();
        
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

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

    // float yaw_angle = plane.ahrs.yaw_sensor * 0.01f;
    // float angle_error = fmodf((float)(yaw_angle - yaw_angle_offset_deg + 180), 360.0f) - 180.0f;
        

    plane.quadplane.assign_tilt_to_fwd_thr();

    // special check for ESC calibration in QSTABILIZE
    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }

    // normal QSTABILIZE mode
    float pilot_throttle_scaled = quadplane.get_pilot_throttle();
    quadplane.hold_stabilize(pilot_throttle_scaled);

    // 方向舵摇杆 → 期望偏航速率（deg/s）
    // get_rate() 返回的就是 deg/s，不需要除以100
    const float desired_yaw_rate_dps = rudder_input * quadplane.command_model_pilot.get_rate();
    
    // 使用纯角速度控制（无角度环）
    plane.stabilize_vtol_yaw_rate(desired_yaw_rate_dps);
    // Stabilize with fixed wing surfaces
    // plane.stabilize_roll();
    plane.stabilize_pitch();
    // Center rudder
    output_rudder_and_steering(0.0);
}

// set the desired roll and pitch for a tailsitter
void ModeQStabilize::set_tailsitter_roll_pitch(const float roll_input, const float pitch_input)
{
    // separate limit for roll, if set
    if (plane.quadplane.tailsitter.max_roll_angle > 0) {
        // roll param is in degrees not centidegrees
        plane.nav_roll_cd = plane.quadplane.tailsitter.max_roll_angle * 100.0f * roll_input;
    } else {
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
    }

    // angle max for tailsitter pitch
    plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;

    plane.quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
}

// set the desired roll and pitch for normal quadplanes, also limited by forward flight limits
void ModeQStabilize::set_limited_roll_pitch(const float roll_input, const float pitch_input)
{
    plane.nav_roll_cd = roll_input * MIN(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);
    // pitch is further constrained by PTCH_LIM_MIN/MAX which may impose
    // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max*100, plane.quadplane.aparm.angle_max);
    } else {
        plane.nav_pitch_cd = pitch_input * MIN(-plane.pitch_limit_min*100, plane.quadplane.aparm.angle_max);
    }
}

#endif
