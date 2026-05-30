#include "tiltrotor.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED
const AP_Param::GroupInfo Tiltrotor::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Tiltrotor functionality
    // @Values: 0:Disable, 1:Enable
    // @Description: This enables Tiltrotor functionality
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    // @Bitmask: 0:Motor 1, 1:Motor 2, 2:Motor 3, 3:Motor 4, 4:Motor 5, 5:Motor 6, 6:Motor 7, 7:Motor 8, 8:Motor 9, 9:Motor 10, 10:Motor 11, 11:Motor 12
    AP_GROUPINFO("MASK", 2, Tiltrotor, tilt_mask, 0),

    // @Param: RATE_UP
    // @DisplayName: Tiltrotor upwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_UP", 3, Tiltrotor, max_rate_up_dps, 40),

    // @Param: MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: deg
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("MAX", 4, Tiltrotor, max_angle_deg, 45),

    // @Param: TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover, Bicopter tiltrotor must use the tailsitter frame class (10)
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw,3:Bicopter
    AP_GROUPINFO("TYPE", 5, Tiltrotor, type, TILT_TYPE_CONTINUOUS),

    // @Param: RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_DN", 6, Tiltrotor, max_rate_down_dps, 0),

    // @Param: YAW_ANGLE
    // @DisplayName: Tilt minimum angle for vectored yaw
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output (fully back). This needs to be set in addition to Q_TILT_TYPE=2, to enable vectored control for yaw in tilt quadplanes. This is also used to limit the forward travel of bicopter tilts(Q_TILT_TYPE=3) when in VTOL modes.
    // @Range: 0 30
    AP_GROUPINFO("YAW_ANGLE", 7, Tiltrotor, tilt_yaw_angle, 0),

    // @Param: FIX_ANGLE
    // @DisplayName: Fixed wing tiltrotor angle
    // @Description: This is the angle the motors tilt down when at maximum output for forward flight. Set this to a non-zero value to enable vectoring for roll/pitch in forward flight on tilt-vectored aircraft
    // @Units: deg
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("FIX_ANGLE", 8, Tiltrotor, fixed_angle, 0),

    // @Param: FIX_GAIN
    // @DisplayName: Fixed wing tiltrotor gain
    // @Description: This is the gain for use of tilting motors in fixed wing flight for tilt vectored quadplanes
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("FIX_GAIN", 9, Tiltrotor, fixed_gain, 0),

    // @Param: WING_FLAP
    // @DisplayName: Tiltrotor tilt angle that will be used as flap
    // @Description: For use on tilt wings, the wing will tilt up to this angle for flap, transition will be complete when the wing reaches this angle from the forward fight position, 0 disables
    // @Units: deg
    // @Increment: 1
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("WING_FLAP", 10, Tiltrotor, flap_angle_deg, 0),

    // 双旋翼串级PID控制参数
    // @Param: BCP_ANG_P
    // @DisplayName: Bicopter pitch angle P gain
    // @Description: Outer loop pitch angle P gain for bicopter control
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("P_ANG_P", 11, Tiltrotor, bicopter_pitch_angle_p, 4.5),

    // @Param: BCP_ANG_I
    // @DisplayName: Bicopter pitch angle I gain
    // @Description: Outer loop pitch angle I gain for bicopter control
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("P_ANG_I", 12, Tiltrotor, bicopter_pitch_angle_i, 0.5),

    // @Param: BCP_ANG_D
    // @DisplayName: Bicopter pitch angle D gain
    // @Description: Outer loop pitch angle D gain for bicopter control
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("P_ANG_D", 13, Tiltrotor, bicopter_pitch_angle_d, 0.1),

    // @Param: BCP_ANG_IMAX
    // @DisplayName: Bicopter pitch angle I max
    // @Description: Outer loop pitch angle integral maximum
    // @Units: deg/s
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("P_A_IM", 14, Tiltrotor, bicopter_pitch_angle_imax, 30),

    // @Param: BCP_RAT_P
    // @DisplayName: Bicopter pitch rate P gain
    // @Description: Inner loop pitch rate P gain for bicopter control
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("P_RAT_P", 15, Tiltrotor, bicopter_pitch_rate_p, 0.15),

    // @Param: BCP_RAT_I
    // @DisplayName: Bicopter pitch rate I gain
    // @Description: Inner loop pitch rate I gain for bicopter control
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("P_RAT_I", 16, Tiltrotor, bicopter_pitch_rate_i, 0.1),

    // @Param: BCP_RAT_D
    // @DisplayName: Bicopter pitch rate D gain
    // @Description: Inner loop pitch rate D gain for bicopter control
    // @Range: 0 0.1
    // @User: Standard
    AP_GROUPINFO("P_RAT_D", 17, Tiltrotor, bicopter_pitch_rate_d, 0.003),

    // @Param: BCP_RAT_IMAX
    // @DisplayName: Bicopter pitch rate I max
    // @Description: Inner loop pitch rate integral maximum
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("P_R_IM", 18, Tiltrotor, bicopter_pitch_rate_imax, 0.3),

    // @Param: BCP_MAX_RATE
    // @DisplayName: Bicopter maximum pitch rate
    // @Description: Maximum desired pitch rate from angle controller
    // @Units: deg/s
    // @Range: 50 500
    // @User: Standard
    AP_GROUPINFO("P_MAX_R", 19, Tiltrotor, bicopter_max_rate_dps, 200),

    // @Param: BCP_MAX_DIFF
    // @DisplayName: Bicopter maximum motor differential
    // @Description: Maximum motor differential output (0-1 range)
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("P_MAX_D", 20, Tiltrotor, bicopter_max_motor_diff, 0.5),

    // 偏航控制参数
    // @Param: Y_ANG_P
    // @DisplayName: Bicopter yaw angle P gain
    // @Description: Outer loop yaw angle P gain for bicopter control
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("Y_ANG_P", 21, Tiltrotor, bicopter_yaw_angle_p, 4.5),

    // @Param: Y_ANG_I
    // @DisplayName: Bicopter yaw angle I gain
    // @Description: Outer loop yaw angle I gain for bicopter control
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("Y_ANG_I", 22, Tiltrotor, bicopter_yaw_angle_i, 0.5),

    // @Param: Y_ANG_D
    // @DisplayName: Bicopter yaw angle D gain
    // @Description: Outer loop yaw angle D gain for bicopter control
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("Y_ANG_D", 23, Tiltrotor, bicopter_yaw_angle_d, 0.1),

    // @Param: Y_ANG_IM
    // @DisplayName: Bicopter yaw angle I max
    // @Description: Outer loop yaw angle integral maximum
    // @Units: deg/s
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("Y_A_IM", 24, Tiltrotor, bicopter_yaw_angle_imax, 30),

    // @Param: Y_RAT_P
    // @DisplayName: Bicopter yaw rate P gain
    // @Description: Inner loop yaw rate P gain for bicopter control
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("Y_RAT_P", 25, Tiltrotor, bicopter_yaw_rate_p, 0.15),

    // @Param: Y_RAT_I
    // @DisplayName: Bicopter yaw rate I gain
    // @Description: Inner loop yaw rate I gain for bicopter control
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("Y_RAT_I", 26, Tiltrotor, bicopter_yaw_rate_i, 0.1),

    // @Param: Y_RAT_D
    // @DisplayName: Bicopter yaw rate D gain
    // @Description: Inner loop yaw rate D gain for bicopter control
    // @Range: 0 0.1
    // @User: Standard
    AP_GROUPINFO("Y_RAT_D", 27, Tiltrotor, bicopter_yaw_rate_d, 0.003),

    // @Param: Y_RAT_IM
    // @DisplayName: Bicopter yaw rate I max
    // @Description: Inner loop yaw rate integral maximum
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("Y_R_IM", 28, Tiltrotor, bicopter_yaw_rate_imax, 0.3),

    // @Param: Y_MAX_R
    // @DisplayName: Bicopter maximum yaw rate
    // @Description: Maximum desired yaw rate from angle controller
    // @Units: deg/s
    // @Range: 50 500
    // @User: Standard
    AP_GROUPINFO("Y_MAX_R", 29, Tiltrotor, bicopter_max_yaw_rate_dps, 90),

    

    // 电机方向控制参数
    // @Param: L_P_SIGN
    // @DisplayName: Left motor pitch sign
    // @Description: Left motor pitch control direction (1=normal, 0=disabled, -1=reversed)
    // @Values: -1:Reversed, 0:Disabled, 1:Normal
    // @User: Standard
    AP_GROUPINFO("L_P_SI", 30, Tiltrotor, left_pitch_sign, 1),

    // @Param: R_P_SIGN
    // @DisplayName: Right motor pitch sign
    // @Description: Right motor pitch control direction (1=normal, 0=disabled, -1=reversed)
    // @Values: -1:Reversed, 0:Disabled, 1:Normal
    // @User: Standard
    AP_GROUPINFO("R_P_SI", 31, Tiltrotor, right_pitch_sign, -1),

    // @Param: L_Y_SIGN
    // @DisplayName: Left motor yaw sign
    // @Description: Left motor yaw control direction (1=normal, 0=disabled, -1=reversed)
    // @Values: -1:Reversed, 0:Disabled, 1:Normal
    // @User: Standard
    AP_GROUPINFO("L_Y_SI", 32, Tiltrotor, left_yaw_sign, -1),

    // @Param: R_Y_SIGN
    // @DisplayName: Right motor yaw sign
    // @Description: Right motor yaw control direction (1=normal, 0=disabled, -1=reversed)
    // @Values: -1:Reversed, 0:Disabled, 1:Normal
    // @User: Standard
    AP_GROUPINFO("R_Y_SI", 33, Tiltrotor, right_yaw_sign, 1),

    // 轨迹规划参数
    // @Param: TRAJ_EN
    // @DisplayName: Enable trajectory planning
    // @Description: Enable optimal trajectory planning for angle control
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("TR_EN", 34, Tiltrotor, enable_trajectory, 0),

    // @Param: TRAJ_RATE
    // @DisplayName: Trajectory max rate
    // @Description: Maximum angular rate for trajectory planning
    // @Units: deg/s
    // @Range: 30 180
    // @User: Advanced
    AP_GROUPINFO("TR_R", 35, Tiltrotor, trajectory_max_rate, 90),

    // @Param: TR_A
    // @DisplayName: Trajectory max acceleration
    // @Description: Maximum angular acceleration for trajectory planning
    // @Units: deg/s/s
    // @Range: 60 360
    // @User: Advanced
    AP_GROUPINFO("TR_A", 36, Tiltrotor, trajectory_max_accel, 180),

    // @Param: TS_A_P
    // @DisplayName: Tilt transition angle P gain
    // @Description: P gain for tilt angle closed-loop control based on MPU6050 feedback
    // @Range: 0 0.1
    // @User: Advanced
    AP_GROUPINFO("TS_A_P", 37, Tiltrotor, tilt_ts_angle_p, 0.01),

    // @Param: TS_A_I
    // @DisplayName: Tilt transition angle I gain
    // @Description: I gain for tilt angle closed-loop control based on MPU6050 feedback
    // @Range: 0 0.01
    // @User: Advanced
    AP_GROUPINFO("TS_A_I", 38, Tiltrotor, tilt_ts_angle_i, 0.002),

    // @Param: TS_A_D
    // @DisplayName: Tilt transition angle D gain
    // @Description: D gain for tilt angle closed-loop control based on MPU6050 feedback
    // @Range: 0 0.01
    // @User: Advanced
    AP_GROUPINFO("TS_A_D", 39, Tiltrotor, tilt_ts_angle_d, 0.003),

    // @Param: TS_A_IM
    // @DisplayName: Tilt transition angle I max
    // @Description: Integral maximum for tilt angle closed-loop control
    // @Range: 0 0.5
    // @User: Advanced
    AP_GROUPINFO("TS_A_IM", 40, Tiltrotor, tilt_ts_angle_imax, 0.15),

    // @Param: TS_COR_MAX
    // @DisplayName: Tilt transition correction max
    // @Description: Maximum PID correction value per update cycle for tilt angle control
    // @Range: 0 0.2
    // @User: Advanced
    AP_GROUPINFO("TS_C_MX", 41, Tiltrotor, tilt_ts_correction_max, 0.05),

    // @Param: VOFA_EN
    // @DisplayName: Enable VOFA debug output
    // @Description: Enable 50Hz NAMED_VALUE_FLOAT debug output for VOFA+ visualization
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("VOFA_EN", 42, Tiltrotor, vofa_enable, 0),


    // 速度控制参数
    // @Param: VEL_P
    // @DisplayName: Bicopter velocity P gain
    // @Description: Velocity control P gain (velocity error to pitch angle)
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("V_P", 43, Tiltrotor, bicopter_vel_p, 0.5),

    // @Param: VEL_I
    // @DisplayName: Bicopter velocity I gain
    // @Description: Velocity control I gain
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("V_I", 44, Tiltrotor, bicopter_vel_i, 0.1),

    // @Param: VEL_D
    // @DisplayName: Bicopter velocity D gain
    // @Description: Velocity control D gain
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("V_D", 45, Tiltrotor, bicopter_vel_d, 0.05),

    // @Param: VEL_IMAX
    // @DisplayName: Bicopter velocity I max
    // @Description: Velocity control integral maximum
    // @Units: m/s
    // @Range: 0 10
    // @User: Standard
    AP_GROUPINFO("V_IM", 46, Tiltrotor, bicopter_vel_imax, 5.0),

    // @Param: VEL_MAX
    // @DisplayName: Bicopter maximum velocity
    // @Description: Maximum desired velocity from RC input
    // @Units: m/s
    // @Range: 0.5 10
    // @User: Standard
    AP_GROUPINFO("V_MAX", 47, Tiltrotor, bicopter_max_vel_mps, 2.0),

    // @Param: VEL_P_MAX
    // @DisplayName: Bicopter velocity pitch max
    // @Description: Maximum pitch angle output from velocity control
    // @Units: deg
    // @Range: 5 30
    // @User: Standard
    AP_GROUPINFO("V_P_M", 48, Tiltrotor, bicopter_max_vel_pitch_deg, 15.0),

    // @Param: VEL_P_SIGN
    // @DisplayName: Velocity pitch direction sign
    // @Description: Velocity control pitch direction (1=forward is positive pitch, -1=forward is negative pitch)
    // @Values: -1:Reversed, 1:Normal
    // @User: Standard
    AP_GROUPINFO("V_P_S", 49, Tiltrotor, velocity_pitch_sign, 1),

    
    // @Param: TILT_RATE
    // @DisplayName: Tilt angle manual control rate
    // @Description: Maximum rate of change for target tilt angle when controlled by CH2 (CH12 high)
    // @Units: deg/s
    // @Range: 0.1 90
    // @User: Standard
    AP_GROUPINFO("TILT_RT", 50, Tiltrotor, tilt_angle_rate_max, 10.0),

    AP_GROUPEND
};

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */

Tiltrotor::Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):quadplane(_quadplane),motors(_motors)
{
    AP_Param::setup_object_defaults(this, var_info);
    bicopter_motor_update_counter = 0;
    
    // 初始化速度控制状态变量
    bicopter_vel_integral = 0.0f;
    bicopter_last_vel_error = 0.0f;
    bicopter_vel_feedforward_filtered = 0.0f;
    
    // 初始化目标角度控制变量
    manual_target_tilt_angle = 0.0f;
    last_tilt_angle_update_ms = 0;
    target_tilt_angle = 0.0f;
    
    // 初始化轨迹规划变量
    traj_active = false;
    traj_start_angle = 0;
    traj_end_angle = 0;
    traj_start_time = 0;
    traj_total_time = 0;
    traj_accel_time = 0;
    traj_constant_time = 0;
    traj_max_rate_actual = 0;
    traj_has_constant_phase = false;
}

void Tiltrotor::setup()
{

    SRV_Channels::set_range(SRV_Channel::k_scripting1, 1000);
    SRV_Channels::set_range(SRV_Channel::k_scripting2, 1000);
    SRV_Channels::set_range(SRV_Channel::k_motor_tilt, 1000);
    SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
    SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);

    if (!enable.configured() && ((tilt_mask != 0) || (type == TILT_TYPE_BICOPTER))) {
        enable.set_and_save(1);
    }

    if (enable <= 0) {
        return;
    }

    quadplane.thrust_type = QuadPlane::ThrustType::TILTROTOR;

    _is_vectored = tilt_mask != 0 && type == TILT_TYPE_VECTORED_YAW;

    // true if a fixed forward motor is configured, either throttle, throttle left  or throttle right.
    // bicopter tiltrotors use throttle left and right as tilting motors, so they don't count in that case.
    _have_fw_motor = SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
                    ((SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) || SRV_Channels::function_assigned(SRV_Channel::k_throttleRight))
                        && (type != TILT_TYPE_BICOPTER));


    
    // check if there are any permanent VTOL motors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (motors->is_motor_enabled(i) && !is_motor_tilting(i)) {
            // enabled motor not set in tilt mask
            _have_vtol_motor = true;
            break;
        }
    }

    if (_is_vectored) {
        // we will be using vectoring for yaw
        motors->disable_yaw_torque();
    }

    if (tilt_mask != 0) {
        // setup tilt compensation
        motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&Tiltrotor::tilt_compensate, void, float *, uint8_t));
        if (type == TILT_TYPE_VECTORED_YAW) {
            // setup tilt servos for vectored yaw
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRear,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearLeft, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearRight, 1000);
        }
    }
    
 

    transition = NEW_NOTHROW Tiltrotor_Transition(quadplane, motors, *this);
    if (!transition) {
        AP_BoardConfig::allocation_error("tiltrotor transition");
    }
    quadplane.transition = transition;

    setup_complete = true;
}

/*
  calculate maximum tilt change as a proportion from 0 to 1 of tilt
 */
float Tiltrotor::tilt_max_change(bool up, bool in_flap_range) const
{
    float rate;
    if (up || max_rate_down_dps <= 0) {
        rate = max_rate_up_dps;
    } else {
        rate = max_rate_down_dps;
    }
    if (type != TILT_TYPE_BINARY && !up && !in_flap_range) {
        bool fast_tilt = false;
        if (plane.control_mode == &plane.mode_manual) {
            fast_tilt = true;
        }
        if (plane.arming.is_armed_and_safety_off() && !quadplane.in_vtol_mode() && !quadplane.assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            // allow a minimum of 90 DPS in manual or if we are not
            // stabilising, to give fast control
            rate = MAX(rate, 90);
        }
    }
    return rate * plane.G_Dt * (1/90.0);
}

/*
  output a slew limited tiltrotor angle. tilt is from 0 to 1
 */
void Tiltrotor::slew(float newtilt)
{
    float max_change = tilt_max_change(newtilt<current_tilt, newtilt > get_fully_forward_tilt());
    current_tilt = constrain_float(newtilt, current_tilt-max_change, current_tilt+max_change);

    angle_achieved = is_equal(newtilt, current_tilt);

    // translate to 0..1000 range and output
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting1, 1000 * current_tilt);
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting2, 1000 * current_tilt);
}

// return the current tilt value that represents forward flight
// tilt wings can sustain forward flight with some amount of wing tilt
float Tiltrotor::get_fully_forward_tilt() const
{
    return 1.0 - (flap_angle_deg * (1/90.0));
}

// return the target tilt value for forward flight
float Tiltrotor::get_forward_flight_tilt() const
{
    return 1.0 - ((flap_angle_deg * (1/90.0)) * SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto) * 0.01);
}

/*
  update motor tilt for continuous tilt servos
 */
void Tiltrotor::continuous_update(void)
{
    // default to inactive
    _motors_active = false;

    // the maximum rate of throttle change
    float max_change;

    if (!quadplane.in_vtol_mode() && (!plane.arming.is_armed_and_safety_off() || !quadplane.assisted_flight)) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor

        // option set then if disarmed move to VTOL position to prevent ground strikes, allow tilt forward in manual mode for testing
        const bool disarmed_tilt_up = !plane.arming.is_armed_and_safety_off() && (plane.control_mode != &plane.mode_manual) && quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT_UP);
        slew(disarmed_tilt_up ? 0.0 : get_forward_flight_tilt());

        max_change = tilt_max_change(false);

        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
        if (current_tilt < get_fully_forward_tilt()) {
            current_throttle = constrain_float(new_throttle,
                                                    current_throttle-max_change,
                                                    current_throttle+max_change);
        } else {
            current_throttle = new_throttle;
        }
        if (!plane.arming.is_armed_and_safety_off()) {
            current_throttle = 0;
        } else {
            // prevent motor shutdown
            _motors_active = true;
        }
        if (!quadplane.motor_test.running) {
            // the motors are all the way forward, start using them for fwd thrust
            const uint16_t mask = is_zero(current_throttle)?0U:tilt_mask.get();
            motors->output_motor_mask(current_throttle, mask, plane.rudder_dt);
        }
        return;
    }

    // remember the throttle level we're using for VTOL flight
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<current_throttle);
    current_throttle = constrain_float(motors_throttle,
                                            current_throttle-max_change,
                                            current_throttle+max_change);

    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 5 strategies we will use:

      1) With use of a forward throttle controlled by Q_FWD_THR_GAIN in
         VTOL modes except Q_AUTOTUNE determined by Q_FWD_THR_USE. We set the angle based on a calculated
         forward throttle.

      2) With manual forward throttle control we set the angle based on the
         RC input demanded forward throttle for QACRO, QSTABILIZE and QHOVER.

      3) Without a RC input or calculated forward throttle value, the angle
         will be set to zero in QAUTOTUNE, QACRO, QSTABILIZE and QHOVER.
         This enables these modes to be used as a safe recovery mode.

      4) In fixed wing assisted flight or velocity controlled modes we will
         set the angle based on the demanded forward throttle, with a maximum
         tilt given by Q_TILT_MAX. This relies on Q_FWD_THR_GAIN or Q_VFWD_GAIN
         being set.

      5) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
    */

#if QAUTOTUNE_ENABLED
    if (plane.control_mode == &plane.mode_qautotune) {
        slew(0);
        return;
    }
#endif

    if (!quadplane.assisted_flight &&
        quadplane.get_vfwd_method() == QuadPlane::ActiveFwdThr::NEW &&
        quadplane.is_flying_vtol())
    {
        // We are using the rotor tilt functionality controlled by Q_FWD_THR_GAIN which can
        // operate in all VTOL modes except Q_AUTOTUNE. Forward rotor tilt is used to produce
        // forward thrust equivalent to what would have been produced by a forward thrust motor
        // set to quadplane.forward_throttle_pct()
        const float fwd_g_demand = 0.01 * quadplane.forward_throttle_pct();
        const float fwd_tilt_deg = MIN(degrees(atanf(fwd_g_demand)), (float)max_angle_deg);
        slew(MIN(fwd_tilt_deg * (1/90.0), get_forward_flight_tilt()));
        return;
    }
    else if (!quadplane.assisted_flight &&
               (plane.control_mode == &plane.mode_qacro ||
               plane.control_mode == &plane.mode_qstabilize ||
               plane.control_mode == &plane.mode_qhover))
    {
        if (quadplane.rc_fwd_thr_ch == nullptr) {
            // no manual throttle control, set angle to zero
            slew(0);

            // 
        } else {
            // manual control of forward throttle up to max VTOL angle
            float settilt = .01f * quadplane.forward_throttle_pct();
            slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
        }
        return;
    }

    if (quadplane.assisted_flight &&
        transition->transition_state >= Tiltrotor_Transition::TRANSITION_TIMER) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        slew(get_forward_flight_tilt());
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
       float settilt = constrain_float((SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)-MAX(plane.aparm.throttle_min.get(),0)) * 0.02, 0, 1);
       slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
    }
}


/*
  output a slew limited tiltrotor angle. tilt is 0 or 1
 */
void Tiltrotor::binary_slew(bool forward)
{
    // The servo output is binary, not slew rate limited
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    float max_change = tilt_max_change(!forward);
    if (forward) {
        current_tilt = constrain_float(current_tilt+max_change, 0, 1);
    } else {
        current_tilt = constrain_float(current_tilt-max_change, 0, 1);
    }
}

/*
  update motor tilt for binary tilt servos
 */
void Tiltrotor::binary_update(void)
{
    // motors always active
    _motors_active = true;

    if (!quadplane.in_vtol_mode()) {
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
        binary_slew(true);

        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (current_tilt >= 1) {
            const uint16_t mask = is_zero(new_throttle)?0U:tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
        }
    } else {
        binary_slew(false);
    }
}


/*
  update motor tilt
 */
void Tiltrotor::update(void)
{
    if (!enabled() || tilt_mask == 0) {
        // no motors to tilt
        return;
    }

    // if (type == TILT_TYPE_BINARY) {
    //     binary_update();
    // } else if (type == TILT_TYPE_BICOPTER) {
    //     // 双旋翼串级PID控制
    //     bicopter_update();
    // } else {
    //     continuous_update();
    // }

    if (type == TILT_TYPE_BICOPTER) {
        bicopter_update();
    }
    // if (type == TILT_TYPE_BICOPTER) {
        
    // }
    // if (type == TILT_TYPE_VECTORED_YAW) {
    //     vectoring();
    // }
}

#if HAL_LOGGING_ENABLED
// Write tiltrotor specific log
void Tiltrotor::write_log()
{
    // Only valid on a tiltrotor
    if (!enabled()) {
        return;
    }

    // 双旋翼模式使用新的日志格式
    if (type == TILT_TYPE_BICOPTER) {
        struct log_tiltrotor pkt {
            LOG_PACKET_HEADER_INIT(LOG_TILT_MSG),
            time_us              : AP_HAL::micros64(),
            pitch_angle_error    : log_pitch_angle_error,
            pitch_desired_rate   : log_pitch_desired_rate,
            pitch_differential   : log_pitch_differential,
            yaw_angle_error      : log_yaw_angle_error,
            yaw_desired_rate     : log_yaw_desired_rate,
            yaw_differential     : log_yaw_differential,
            left_motor_output    : log_left_motor_output,
            right_motor_output   : log_right_motor_output,
        };
        plane.logger.WriteBlock(&pkt, sizeof(pkt));
        return;
    }

    // 原有的倾转翼日志格式（保持兼容性）
    struct log_tiltrotor pkt {
        LOG_PACKET_HEADER_INIT(LOG_TILT_MSG),
        time_us              : AP_HAL::micros64(),
        pitch_angle_error    : 0,
        pitch_desired_rate   : 0,
        pitch_differential   : 0,
        yaw_angle_error      : 0,
        yaw_desired_rate     : 0,
        yaw_differential     : 0,
        left_motor_output    : 0,
        right_motor_output   : 0,
    };

    plane.logger.WriteBlock(&pkt, sizeof(pkt));
}
#endif

/*
  tilt compensation for angle of tilt. When the rotors are tilted the
  roll effect of differential thrust on the tilted rotors is decreased
  and the yaw effect increased
  We have two factors we apply.

  1) when we are transitioning to fwd flight we scale the tilted rotors by 1/cos(angle). This pushes us towards more flight speed

  2) when we are transitioning to hover we scale the non-tilted rotors by cos(angle). This pushes us towards lower fwd thrust

  We also apply an equalisation to the tilted motors in proportion to
  how much tilt we have. This smoothly reduces the impact of the roll
  gains as we tilt further forward.

  For yaw, we apply differential thrust in proportion to the demanded
  yaw control and sin of the tilt angle

  Finally we ensure no requested thrust is over 1 by scaling back all
  motors so the largest thrust is at most 1.0
 */
void Tiltrotor::tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul)
{
    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // apply tilt_factors first
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= non_tilted_mul;
        } else {
            thrust[i] *= tilted_mul;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;
    const float sin_tilt = sinf(radians(current_tilt*90));
    // yaw_gain relates the amount of differential thrust we get from
    // tilt, so that the scaling of the yaw control is the same at any
    // tilt angle
    const float yaw_gain = sinf(radians(tilt_yaw_angle));
    const float avg_tilt_thrust = tilt_total / tilt_count;

    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            // as we tilt we need to reduce the impact of the roll
            // controller. This simple method keeps the same average,
            // but moves us to no roll control as the angle increases
            thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1-current_tilt);
            // add in differential thrust for yaw control, scaled by tilt angle
            const float diff_thrust = motors->get_roll_factor(i) * (motors->get_yaw()+motors->get_yaw_ff()) * sin_tilt * yaw_gain;
            thrust[i] += diff_thrust;
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // if we are saturating one of the motors then reduce all motors
    // to keep them in proportion to the original thrust. This helps
    // maintain stability when tilted at a large angle
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}

/*
  choose up or down tilt compensation based on flight mode When going
  to a fixed wing mode we use tilt_compensate_down, when going to a
  VTOL mode we use tilt_compensate_up
 */
void Tiltrotor::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
        return;
    }
    if (quadplane.in_vtol_mode()) {
        // we are transitioning to VTOL flight
        const float tilt_factor = cosf(radians(current_tilt*90));
        tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
    } else {
        float inv_tilt_factor;
        if (current_tilt > 0.98f) {
            inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
        } else {
            inv_tilt_factor = 1.0 / cosf(radians(current_tilt*90));
        }
        tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
    }
}

/*
  return true if the rotors are fully tilted forward
 */
bool Tiltrotor::fully_fwd(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt >= get_fully_forward_tilt());
}

/*
  return true if the rotors are fully tilted up
 */
bool Tiltrotor::fully_up(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt <= 0);
}

/*
  control vectoring for tilt multicopters
 */
void Tiltrotor::vectoring(void)
{
    // // total angle the tilt can go through
    // const float total_angle = 90 + tilt_yaw_angle + fixed_angle;
    // // output value (0 to 1) to get motors pointed straight up
    // const float zero_out = tilt_yaw_angle / total_angle;
    // const float fixed_tilt_limit = fixed_angle / total_angle;
    // const float level_out = 1.0 - fixed_tilt_limit;

    // // calculate the basic tilt amount from current_tilt
    // float base_output = zero_out + (current_tilt * (level_out - zero_out));
    // // for testing when disarmed, apply vectored yaw in proportion to rudder stick
    // // Wait TILT_DELAY_MS after disarming to allow props to spin down first.
    // constexpr uint32_t TILT_DELAY_MS = 3000;
    // uint32_t now = AP_HAL::millis();
    // if (!plane.arming.is_armed_and_safety_off() && plane.quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT)) {
    //     // this test is subject to wrapping at ~49 days, but the consequences are insignificant
    //     if ((now - hal.util->get_last_armed_change()) > TILT_DELAY_MS) {
    //         if (quadplane.in_vtol_mode()) {
    //             float yaw_out = plane.channel_rudder->get_control_in();
    //             yaw_out /= plane.channel_rudder->get_range();
    //             float yaw_range = zero_out;

    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output,0,1));
    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
    //         } else {
    //             // fixed wing tilt
    //             const float gain = fixed_gain * fixed_tilt_limit;
    //             // base the tilt on elevon mixing, which means it
    //             // takes account of the MIXING_GAIN. The rear tilt is
    //             // based on elevator
    //             const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
    //             const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
    //             const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
    //             // front tilt is effective canards, so need to swap and use negative. Rear motors are treated live elevons.
    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
    //             SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
    //         }
    //     }
    //     return;
    // }

    // const bool no_yaw = tilt_over_max_angle();
    // if (no_yaw) {
    //     // fixed wing  We need to apply inverse scaling with throttle, and remove the surface speed scaling as
    //     // we don't want tilt impacted by airspeed
    //     const float scaler = plane.control_mode == &plane.mode_manual?1:(quadplane.FW_vector_throttle_scaling() / plane.get_speed_scaler());
    //     const float gain = fixed_gain * fixed_tilt_limit * scaler;
    //     const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
    //     const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
    //     const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
    // } else {
    //     const float yaw_out = motors->get_yaw()+motors->get_yaw_ff();
    //     const float roll_out = motors->get_roll()+motors->get_roll_ff();
    //     const float yaw_range = zero_out;

    //     // Scaling yaw with throttle
    //     const float throttle = motors->get_throttle_out();
    //     const float scale_min = 0.5;
    //     const float scale_max = 2.0;
    //     float throttle_scaler = scale_max;
    //     if (is_positive(throttle)) {
    //         throttle_scaler = constrain_float(motors->get_throttle_hover() / throttle, scale_min, scale_max);
    //     }

    //     // now apply vectored thrust for yaw and roll.
    //     const float tilt_rad = radians(current_tilt*90);
    //     const float sin_tilt = sinf(tilt_rad);
    //     const float cos_tilt = cosf(tilt_rad);
    //     // the MotorsMatrix library normalises roll factor to 0.5, so
    //     // we need to use the same factor here to keep the same roll
    //     // gains when tilted as we have when not tilted
    //     const float avg_roll_factor = 0.5;
    //     float tilt_scale = throttle_scaler * yaw_out * cos_tilt + avg_roll_factor * roll_out * sin_tilt;

    //     if (fabsf(tilt_scale) > 1.0) {
    //         tilt_scale = constrain_float(tilt_scale, -1.0, 1.0);
    //         motors->limit.yaw = true;
    //     }

    //     const float tilt_offset = tilt_scale * yaw_range;

    //     float left_tilt = base_output + tilt_offset;
    //     float right_tilt = base_output - tilt_offset;

    //     // if output saturation of both left and right then set yaw limit flag
    //     if (((left_tilt > 1.0) || (left_tilt < 0.0)) &&
    //         ((right_tilt > 1.0) || (right_tilt < 0.0))) {
    //         motors->limit.yaw = true;
    //     }

    //     // constrain and scale to ouput range
    //     left_tilt = constrain_float(left_tilt,0.0,1.0) * 1000.0;
    //     right_tilt = constrain_float(right_tilt,0.0,1.0) * 1000.0;

    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, left_tilt);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_tilt);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 1000.0 * constrain_float(base_output,0.0,1.0));
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft, left_tilt);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, right_tilt);
    // }
}

/*
  control bicopter tiltrotors
 */
void Tiltrotor::bicopter_output(void)
{
    if (type != TILT_TYPE_BICOPTER || quadplane.motor_test.running) {
        // don't override motor test with motors_output
        return;
    }

    // if (!quadplane.in_vtol_mode() && fully_fwd()) {
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  -SERVO_MAX);
    //     SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
    //     return;
    // }

    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (quadplane.assisted_flight) {
        quadplane.hold_stabilize(throttle * 0.01f);
        quadplane.motors_output(true);
    } else {
        quadplane.motors_output(false);
    }

    // bicopter assumes that trim is up so we scale down so match
    // float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    // float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    // if (is_negative(tilt_left)) {
    //     tilt_left *= tilt_yaw_angle * (1/90.0);
    // }
    // if (is_negative(tilt_right)) {
    //     tilt_right *= tilt_yaw_angle * (1/90.0);
    // }

    // // reduce authority of bicopter as motors are tilted forwards
    // const float scaling = cosf(current_tilt * M_PI_2);
    // tilt_left  *= scaling;
    // tilt_right *= scaling;

    // // add current tilt and constrain
    // tilt_left  = constrain_float(-(current_tilt * SERVO_MAX) + tilt_left,  -SERVO_MAX, SERVO_MAX);
    // tilt_right = constrain_float(-(current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

    // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  tilt_left);
    // SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
    
    // 输出QGC 看看是否运行
    static uint32_t last_output_debug_ms = 0;
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_output_debug_ms > 1000) {
        last_output_debug_ms = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                     "bicopter_output running, throttle:%.0f",
                     (double)throttle);
    }
}

/*
  when doing a forward transition of a tilt-vectored quadplane we use
  euler angle control to maintain good yaw. This updates the yaw
  target based on pilot input and target roll
 */
void Tiltrotor::update_yaw_target(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - transition_yaw_set_ms > 100 ||
        !is_zero(quadplane.get_pilot_input_yaw_rate_cds())) {
        // lock initial yaw when transition is started or when
        // pilot commands a yaw change. This allows us to track
        // straight in transitions for tilt-vectored planes, but
        // allows for turns when level transition is not wanted
        transition_yaw_cd = quadplane.ahrs.yaw_sensor;
    }

    /*
      now calculate the equivalent yaw rate for a coordinated turn for
      the desired bank angle given the airspeed
     */
    float aspeed;
    bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
    if (have_airspeed && labs(plane.nav_roll_cd)>1000) {
        float dt = (now - transition_yaw_set_ms) * 0.001;
        // calculate the yaw rate to achieve the desired turn rate
        const float airspeed_min = MAX(plane.aparm.airspeed_min,5);
        const float yaw_rate_cds = fixedwing_turn_rate(plane.nav_roll_cd*0.01, MAX(aspeed,airspeed_min))*100;
        transition_yaw_cd += yaw_rate_cds * dt;
    }
    transition_yaw_set_ms = now;
}


/*
  control use of multirotor rate control in forward transition
 */
bool Tiltrotor_Transition::use_multirotor_control_in_fwd_transition() const
{
    return tiltrotor.is_vectored() && transition_state <= TRANSITION_TIMER;
}

bool Tiltrotor_Transition::update_yaw_target(float& yaw_target_cd)
{
    if (!use_multirotor_control_in_fwd_transition()) {
        return false;
    }
    tiltrotor.update_yaw_target();
    yaw_target_cd = tiltrotor.transition_yaw_cd;
    return true;
}

// return true if we should show VTOL view
bool Tiltrotor_Transition::show_vtol_view() const
{
    bool show_vtol = quadplane.in_vtol_mode();

    if (!show_vtol && tiltrotor.is_vectored() && transition_state <= TRANSITION_TIMER) {
        // we use multirotor controls during fwd transition for
        // vectored yaw vehicles
        return true;
    }

    return show_vtol;
}

// return true if we are tilted over the max angle threshold
bool Tiltrotor::tilt_over_max_angle(void) const
{
    const float tilt_threshold = (max_angle_deg/90.0f);
    return (current_tilt > MIN(tilt_threshold, get_forward_flight_tilt()));
}

void Tiltrotor::transition_get_rate(float zero_out, float pitch_angle_error, float &pitch_differential, float &left_motor_output) {
    const uint32_t now_ms = AP_HAL::millis();
    
    // 计算时间间隔
    float dt_s = 0.02f;  // 默认50Hz
    if (bicopter_last_update_ms != 0) {
        dt_s = (now_ms - bicopter_last_update_ms) * 0.001f;
        if (dt_s > 1.0f || dt_s <= 0.0f) {
            dt_s = 0.02f;
        }
    }
    bicopter_last_update_ms = now_ms;
    float current_tilt_angle = target_tilt_angle;
    // total angle the tilt can go through
    const float total_angle = 90 + tilt_yaw_angle;
    float current_pitch_rate = plane.ahrs.get_gyro().y * RAD_TO_DEG;
    float current_pitch_deg = plane.ahrs.pitch_sensor * 0.01f;

    // output value (0 to 1) to get motors pointed straight up
    const float pwm_out = (tilt_yaw_angle + current_tilt_angle + current_pitch_deg) / total_angle;

    float pitch_rate_error = 0 - current_pitch_rate;

    float pitch_rate_p = bicopter_pitch_rate_p * pitch_rate_error;
    
    bicopter_rate_integral += pitch_rate_error * dt_s;
    bicopter_rate_integral = constrain_float(bicopter_rate_integral, 
                                            -bicopter_pitch_rate_imax, 
                                            bicopter_pitch_rate_imax);
    
                                               float target_pitch_deg_raw = plane.nav_pitch_cd * 0.01f;    // 使用原始导航指令（保持悬停）
    
    // 获取最大角度限制
    float angle_max = plane.quadplane.aparm.angle_max * 0.01;  // centidegrees → degrees
    
    // 限制目标角度在 ±angle_max 范围内
    float target_pitch_change = target_pitch_deg_raw;
    float feedforward_output = 0.0f;
    
    // 如果目标改变超过阈值，计算前馈
    if (fabsf(target_pitch_change) > 0.1f) {
        // 前馈增益：目标变化直接映射到舵机输出
        // 使用 angle_max 作为归一化基准
        feedforward_output = target_pitch_change / angle_max;
        feedforward_output = constrain_float(feedforward_output, -1.0f, 1.0f);
    }
    
    // 2. 计算俯仰角度误差
    pitch_angle_error = target_pitch_deg_raw - current_pitch_deg;
    // 根据角度误差大小调整积分项
    if (fabsf(pitch_angle_error) < 0.5f) {
        bicopter_rate_integral = 0.0f;  // 极小误差：清零
    } else if (fabsf(pitch_angle_error) < 1.0f) {
        bicopter_rate_integral *= 0.8f;  // 小误差：快速衰减
    }
    
    log_pitch_rate_integral = bicopter_rate_integral;

    float pitch_rate_i = bicopter_pitch_rate_i * bicopter_rate_integral;
    
    float pitch_rate_d_input = (pitch_rate_error - bicopter_last_rate_error) / dt_s;
    bicopter_last_rate_error = pitch_rate_error;
    
    float pitch_rate_d = bicopter_pitch_rate_d * pitch_rate_d_input;
    
    // 6. 计算俯仰电机差分输出（内环输出）
    pitch_differential = pitch_rate_p + pitch_rate_i + pitch_rate_d;
    pitch_differential = constrain_float(pitch_differential, -1.0f, 1.0f);

    float pitch_range = zero_out;
    float pitch_diff = pitch_differential * pitch_range;
    
    // 添加速度控制前馈到俯仰差分
    if (pitch_diff > bicopter_max_motor_diff) {
        pitch_diff = bicopter_max_motor_diff;
    } else if (pitch_diff < -bicopter_max_motor_diff) {
        pitch_diff = -bicopter_max_motor_diff;
    }
    
    float left_tilt = pwm_out + left_pitch_sign * pitch_diff;
    // 限制输出范围并转换为舵机信号 (0-1000)
    left_motor_output = 1000 * constrain_float(left_tilt, 0.0, 1.0);
    // 输出到倾转舵机
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting1, left_motor_output);
}


// 切换姿态飞行计算 - 基于MPU6050反馈的倾转角度闭环控制
void Tiltrotor::transition_pid_get_rate() {
    // 读取MPU6050传感器反馈的倾转角度
    // mpu6050_angle_roll: 0°表示水平, 90°表示垂直

    // total angle the tilt can go through
    // const float total_angle = 90 + tilt_yaw_angle;
    // output value (0 to 1) to get motors pointed straight up
    float mpu6050_angle_roll = -(quadplane.mpu6050_angle_roll);
    // float current_pitch_deg = plane.ahrs.pitch_sensor * 0.01f;  // centidegrees → degrees
    // 将MPU6050的roll角度转换为倾转角度
    // MPU6050: 0°(水平) -> 倾转角度: 90°(水平前飞)
    // MPU6050: 90°(垂直) -> 倾转角度: 0°(垂直悬停)
    
    float current_tilt_angle = target_tilt_angle;
    // const float zero_out = (tilt_yaw_angle + target_tilt_angle - current_pitch_deg) / total_angle;
    // 计算角度误差
    float angle_error = mpu6050_angle_roll - current_tilt_angle;

    // 读取当前电机位置并转换为0-1范围
    int16_t current_output = SRV_Channels::get_output_scaled(SRV_Channel::k_scripting1);
    float norm_current_output = current_output / 1000.0f;
    
    // 静态变量保存状态
    static float tilt_integral = 0.0f;
    static float last_angle_error = 0.0f;
    static uint32_t last_update_ms = 0;
    
    // 计算时间间隔
    uint32_t now_ms = AP_HAL::millis();
    float dt_s = 0.02f; // 默认50Hz
    if (last_update_ms != 0) {
        dt_s = (now_ms - last_update_ms) * 0.001f;
        if (dt_s > 1.0f || dt_s <= 0.0f) {
            dt_s = 0.02f;
        }
    }
    last_update_ms = now_ms;
    
    // P项 - 角度误差转换为位置修正
    float p_term = tilt_ts_angle_p.get() * angle_error;
    
    // I项 (带积分限幅)
    tilt_integral += angle_error * dt_s;
    tilt_integral = constrain_float(tilt_integral, -tilt_ts_angle_imax.get(), tilt_ts_angle_imax.get());
    
    // 当误差小于2度时清空积分，防止小误差积分累积
    if (fabsf(angle_error) < 2.0f) {
        tilt_integral = 0.0f;
    }
    float i_term = tilt_ts_angle_i.get() * tilt_integral;
    
    // D项
    float d_input = (angle_error - last_angle_error) / dt_s;
    last_angle_error = angle_error;
    float d_term = tilt_ts_angle_d.get() * d_input;
    
    // PID输出 (位置修正量)
    float pid_correction = p_term + i_term + d_term;
    
    // 限制修正量范围
    float correction_limit = tilt_ts_correction_max.get();
    pid_correction = constrain_float(pid_correction, -correction_limit, correction_limit);
    
    // 将PID修正应用到当前电机位置
    float final_output = norm_current_output - pid_correction;
    
    // 限制最终输出在0-1范围内
    final_output = constrain_float(final_output, 0.0f, 1.0f);
    
    // 每秒输出一次调试信息
    static uint32_t last_debug_ms = 0;
    if (now_ms - last_debug_ms > 1000) {
        last_debug_ms = now_ms;
        
        // 发送倾转PID控制数据到上位机（MAVLink NAMED_VALUE_FLOAT）
        gcs().send_named_float("TS_TGT", current_tilt_angle);
        gcs().send_named_float("TS_MPU", mpu6050_angle_roll);
        gcs().send_named_float("TS_ERR", angle_error);
        gcs().send_named_float("TS_CUR", norm_current_output);
        gcs().send_named_float("TS_PID1", pid_correction);
        gcs().send_named_float("TS_OUT", final_output);
        
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                    "Tilt Tgt:%.1f MPU:%.1f Err:%.1f",
                    (double)current_tilt_angle,
                    (double)mpu6050_angle_roll,
                    (double)angle_error);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                    "Tilt CurOut:%d Norm:%.3f PID:%.3f Out:%.3f",
                    current_output,
                    (double)norm_current_output,
                    (double)pid_correction,
                    (double)final_output);
    }
    
    // 将倾转输出转换为舵机信号 (0-1000)
    // final_output范围: 0-1 (0=垂直, 1=水平)
    float left_servo_output = 1000 * final_output;
    float right_servo_output = 1000 * final_output;
    
    // 输出到倾转舵机
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting1, left_servo_output);
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting2, right_servo_output);
}

// 垂直姿态飞行计算
void Tiltrotor::vtol_pid_get_rate(float base_output, float zero_out, 
                                   float &pitch_differential, float &yaw_differential,
                                   float &pitch_angle_error, float &desired_pitch_rate,
                                   float &yaw_angle_error, float &desired_yaw_rate,
                                   float &left_motor_output, float &right_motor_output)
{
    const uint32_t now_ms = AP_HAL::millis();
    
    // 计算时间间隔
    float dt_s = 0.02f;  // 默认50Hz
    if (bicopter_last_update_ms != 0) {
        dt_s = (now_ms - bicopter_last_update_ms) * 0.001f;
        if (dt_s > 1.0f || dt_s <= 0.0f) {
            dt_s = 0.02f;
        }
    }
    bicopter_last_update_ms = now_ms;
    
    // ========== 速度控制（新增） ==========
    // 外环：速度控制 (Velocity Error → Pitch Angle Command)
    
    // 1. 获取遥控器通道2输入（俯仰摇杆）
    float rc_pitch_input = 0.0f;
    // 获取通道2的归一化输入 [-1, 1]
    rc_pitch_input = plane.channel_pitch->norm_input();
    
    // 添加死区，避免摇杆中位抖动
    if (fabsf(rc_pitch_input) < 0.05f) {
        rc_pitch_input = 0.0f;
    }

    // 2. 计算期望速度 (m/s)
    // 遥控器中位时期望速度为0，前推/后拉时按比例增加期望速度
    float desired_velocity_mps = rc_pitch_input * bicopter_max_vel_mps;
    log_desired_velocity = desired_velocity_mps;
    
    // 3. 获取当前前后速度
    Vector3f velocity_ned;
    float current_velocity_mps = 0.0f;
    bool velocity_valid = false;
    
    // 使用EKF融合速度（推荐方案）
    if (plane.ahrs.get_velocity_NED(velocity_ned)) {
        // 将NED速度转换到机体坐标系
        Vector3f velocity_body = plane.ahrs.get_rotation_body_to_ned().transposed() * velocity_ned;
        current_velocity_mps = velocity_body.x;  // 机体X轴为前向
        velocity_valid = true;
    }
    
    log_current_velocity = current_velocity_mps;
    
    // 4. 速度环PID计算
    float velocity_pitch_cmd = 0.0f;  // 速度控制输出的俯仰角指令
    
    if (velocity_valid) {
        float velocity_error = current_velocity_mps - desired_velocity_mps;
        log_velocity_error = velocity_error;
        // P项
        float vel_p = bicopter_vel_p * velocity_error;
        // I项（带抗饱和）
        bicopter_vel_integral += velocity_error * dt_s;
        bicopter_vel_integral = constrain_float(bicopter_vel_integral, 
                                               -bicopter_vel_imax, 
                                               bicopter_vel_imax);
        
        // 小速度误差时衰减积分
        if (fabsf(velocity_error) < 0.1f) {
            bicopter_vel_integral *= 0.95f;
        } else if (fabsf(velocity_error) < 0.2f) {
            bicopter_vel_integral *= 0.98f;
        }
        
        float vel_i = bicopter_vel_i * bicopter_vel_integral;
        
        // D项
        float vel_d_input = (velocity_error - bicopter_last_vel_error) / dt_s;
        bicopter_last_vel_error = velocity_error;
        float vel_d = bicopter_vel_d * vel_d_input;
        
        // 5. 速度PID输出 → 期望俯仰角（度）
        velocity_pitch_cmd = vel_p + vel_i + vel_d;
        velocity_pitch_cmd = constrain_float(velocity_pitch_cmd, 
                                            -bicopter_max_vel_pitch_deg, 
                                            bicopter_max_vel_pitch_deg);
    } else {
        // 速度无效时，清空积分项
        bicopter_vel_integral = 0.0f;
        bicopter_last_vel_error = 0.0f;
        log_velocity_error = 0.0f;
    }
    
    log_velocity_pitch_cmd = velocity_pitch_cmd;
    
    // 6. 将速度控制输出归一化为前馈量 [-1, 1]
    // 速度控制输出的俯仰角 → 归一化前馈
    float velocity_feedforward = velocity_pitch_cmd / bicopter_max_vel_pitch_deg;
    velocity_feedforward = constrain_float(velocity_feedforward, -1.0f, 1.0f);
    
    // 7. 低通滤波平滑前馈输出，降低灵敏度
    // 滤波系数：0.1 = 较慢响应，0.5 = 中等响应，0.9 = 快速响应
    const float filter_alpha = 0.3f;  // 可调整：0.1-0.5之间
    bicopter_vel_feedforward_filtered = bicopter_vel_feedforward_filtered * (1.0f - filter_alpha) + 
                                        velocity_feedforward * filter_alpha;
    velocity_feedforward = bicopter_vel_feedforward_filtered;
    
    // ========== 俯仰控制 ==========
    // 外环：俯仰角度控制 (Pitch Angle → Desired Pitch Rate)
    
    // 1. 获取当前俯仰角度和目标角度
    float current_pitch_deg = plane.ahrs.pitch_sensor * 0.01f;  // centidegrees → degrees
    float target_pitch_deg_raw = plane.nav_pitch_cd * 0.01f;    // 使用原始导航指令（保持悬停）
    
    // 获取最大角度限制
    float angle_max = plane.quadplane.aparm.angle_max * 0.01;  // centidegrees → degrees
    
    // 限制目标角度在 ±angle_max 范围内
    float target_pitch_change = target_pitch_deg_raw;
    float feedforward_output = 0.0f;
    
    // 如果目标改变超过阈值，计算前馈
    if (fabsf(target_pitch_change) > 0.1f) {
        // 前馈增益：目标变化直接映射到舵机输出
        // 使用 angle_max 作为归一化基准
        feedforward_output = target_pitch_change / angle_max;
        feedforward_output = constrain_float(feedforward_output, -1.0f, 1.0f);
    }
    
    // 2. 计算俯仰角度误差
    pitch_angle_error = target_pitch_deg_raw - current_pitch_deg;
    
    // 3. 俯仰角度环PID计算
    float pitch_angle_p = bicopter_pitch_angle_p * pitch_angle_error;
    
    bicopter_angle_integral += pitch_angle_error * dt_s;
    bicopter_angle_integral = constrain_float(bicopter_angle_integral, 
                                            -bicopter_pitch_angle_imax, 
                                            bicopter_pitch_angle_imax);
    
    float pitch_angle_i = bicopter_pitch_angle_i * bicopter_angle_integral;
    
    float pitch_angle_d_input = (pitch_angle_error - bicopter_last_pitch_error) / dt_s;
    bicopter_last_pitch_error = pitch_angle_error;
    float pitch_angle_d = bicopter_pitch_angle_d * pitch_angle_d_input;
    
    // 4. 计算期望俯仰角速度（外环输出）
    desired_pitch_rate = pitch_angle_p + pitch_angle_i + pitch_angle_d;
    desired_pitch_rate = constrain_float(desired_pitch_rate, 
                                        -bicopter_max_rate_dps, 
                                        bicopter_max_rate_dps);
    
    float extra_elevator = desired_pitch_rate;
    
    // 内环：俯仰角速度控制 (Pitch Rate Error → Motor Differential)
    float current_pitch_rate = plane.ahrs.get_gyro().y * RAD_TO_DEG;  // rad/s → deg/s
    float pitch_rate_error = extra_elevator - current_pitch_rate;
    
    // 5. 俯仰角速度环PID计算
    float pitch_rate_p = bicopter_pitch_rate_p * pitch_rate_error;
    
    bicopter_rate_integral += pitch_rate_error * dt_s;
    bicopter_rate_integral = constrain_float(bicopter_rate_integral, 
                                            -bicopter_pitch_rate_imax, 
                                            bicopter_pitch_rate_imax);
    
    // 根据角度误差大小调整积分项
    if (fabsf(pitch_angle_error) < 0.5f) {
        bicopter_rate_integral = 0.0f;  // 极小误差：清零
    } else if (fabsf(pitch_angle_error) < 1.0f) {
        bicopter_rate_integral *= 0.8f;  // 小误差：快速衰减
    }
    
    log_pitch_rate_integral = bicopter_rate_integral;

    float pitch_rate_i = bicopter_pitch_rate_i * bicopter_rate_integral;
    
    float pitch_rate_d_input = (pitch_rate_error - bicopter_last_rate_error) / dt_s;
    bicopter_last_rate_error = pitch_rate_error;
    
    float pitch_rate_d = bicopter_pitch_rate_d * pitch_rate_d_input;
    
    // 6. 计算俯仰电机差分输出（内环输出）
    pitch_differential = pitch_rate_p + pitch_rate_i + pitch_rate_d;
    pitch_differential = constrain_float(pitch_differential, -1.0f, 1.0f);
    log_pitch_rate_error = pitch_rate_error / MAX((float)bicopter_max_rate_dps, 1.0f); // [-1, 1]
    log_pitch_rate_p     = pitch_rate_p;                                                // 原始P分量

    // ========== 偏航控制 ==========
    // 外环：偏航角度控制 (Yaw Angle → Desired Yaw Rate)
    
    // 1. 获取当前偏航角度和目标角度
    float current_yaw_deg = plane.ahrs.yaw_sensor * 0.01f;  // centidegrees → degrees
    float target_yaw_deg = 0;        // 目标偏航角度
    
    // 2. 计算偏航角度误差（处理360度环绕）
    yaw_angle_error = target_yaw_deg - current_yaw_deg;
    if (yaw_angle_error > 180.0f) {
        yaw_angle_error -= 360.0f;
    } else if (yaw_angle_error < -180.0f) {
        yaw_angle_error += 360.0f;
    }
    
    // 3. 偏航角度环PID计算
    float yaw_angle_p = bicopter_yaw_angle_p * yaw_angle_error;
    
    bicopter_yaw_angle_integral += yaw_angle_error * dt_s;
    bicopter_yaw_angle_integral = constrain_float(bicopter_yaw_angle_integral, 
                                                -bicopter_yaw_angle_imax, 
                                                bicopter_yaw_angle_imax);
    float yaw_angle_i = bicopter_yaw_angle_i * bicopter_yaw_angle_integral;
    
    float yaw_angle_d_input = (yaw_angle_error - bicopter_last_yaw_error) / dt_s;
    bicopter_last_yaw_error = yaw_angle_error;
    float yaw_angle_d = bicopter_yaw_angle_d * yaw_angle_d_input;
    
    // 4. 计算期望偏航角速度（外环输出）
    desired_yaw_rate = yaw_angle_p + yaw_angle_i + yaw_angle_d;
    desired_yaw_rate = constrain_float(desired_yaw_rate, 
                                    -bicopter_max_yaw_rate_dps, 
                                    bicopter_max_yaw_rate_dps);
    
    // 内环：偏航角速度控制 (Yaw Rate Error → Motor Differential)
    float current_yaw_rate = plane.ahrs.get_gyro().z * RAD_TO_DEG;  // rad/s → deg/s
    float yaw_rate_error = desired_yaw_rate - current_yaw_rate;
    
    // 5. 偏航角速度环PID计算
    float yaw_rate_p = bicopter_yaw_rate_p * yaw_rate_error;
    
    bicopter_yaw_rate_integral += yaw_rate_error * dt_s;
    bicopter_yaw_rate_integral = constrain_float(bicopter_yaw_rate_integral, 
                                                -bicopter_yaw_rate_imax, 
                                                bicopter_yaw_rate_imax);
    
    // 根据角度误差大小调整积分项
    if (fabsf(yaw_angle_error) < 0.5f) {
        bicopter_yaw_rate_integral = 0.0f;  // 极小误差：清零
    } else if (fabsf(yaw_angle_error) < 1.0f) {
        bicopter_yaw_rate_integral *= 0.8f;  // 小误差：快速衰减
    }
    
    float yaw_rate_i = bicopter_yaw_rate_i * bicopter_yaw_rate_integral;
    
    float yaw_rate_d_input = (yaw_rate_error - bicopter_last_yaw_rate_error) / dt_s;
    bicopter_last_yaw_rate_error = yaw_rate_error;
    float yaw_rate_d = bicopter_yaw_rate_d * yaw_rate_d_input;
    
    // 6. 计算偏航电机差分输出（内环输出）
    yaw_differential = yaw_rate_p + yaw_rate_i + yaw_rate_d;
    yaw_differential = constrain_float(yaw_differential, -1.0f, 1.0f);
    
    // ========== 组合输出到左右倾转电机 ==========
    float pitch_range = zero_out;
    float pitch_diff = pitch_differential * pitch_range;
    float yaw_diff = (yaw_differential / 2.0f) * pitch_range;
    
    // 添加角度前馈输出到俯仰差分
    float feedforward_diff = feedforward_output * pitch_range;
    pitch_diff += feedforward_diff;
    
    // 添加速度控制前馈到俯仰差分
    // float velocity_feedforward_diff = velocity_pitch_sign * velocity_feedforward * pitch_range;
    // pitch_diff += velocity_feedforward_diff;
    
    if (pitch_diff > bicopter_max_motor_diff) {
        pitch_diff = bicopter_max_motor_diff;
    } else if (pitch_diff < -bicopter_max_motor_diff) {
        pitch_diff = -bicopter_max_motor_diff;
    }
    
    float left_tilt = base_output + left_pitch_sign * pitch_diff - left_yaw_sign * yaw_diff;
    float right_tilt = base_output + right_pitch_sign * pitch_diff + right_yaw_sign * yaw_diff;
    
    // 限制输出范围并转换为舵机信号 (0-1000)
    left_motor_output = 1000 * constrain_float(left_tilt, 0.0, 1.0);
    right_motor_output = 1000 * constrain_float(right_tilt, 0.0, 1.0);
    
    // 使用计数器交替更新电机：奇数更新左电机，偶数更新右电机
    // 奇数：更新左电机
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting1, left_motor_output);
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting2, right_motor_output);
}


/*
  双旋翼串级PID控制
  俯仰控制：角度误差 → 期望角速度 → 电机差分输出
  偏航控制：角度误差 → 期望角速度 → 电机差分输出
  最终输出：俯仰差分 + 偏航差分 → 左右电机
*/
void Tiltrotor::bicopter_update()
{
    // 使用 遥控器通道8 重新校准MPU6050
    static bool calibration_done = false;  // 校准完成标识
    bool ch8_high = (RC_Channels::get_radio_in(7) > 1800);  // Channel 8 (index 7)
    
    if (!ch8_high) {
        // CH8低位：重置校准标识，允许下次校准
        calibration_done = false;
    } else if (ch8_high && !calibration_done) {
        // CH8高位且未校准：执行校准（只执行一次）
        plane.mpu6050_recalibrate();
        calibration_done = true;  // 标记已校准
    }

    // 获取通道12状态
    int ch12_value = RC_Channels::get_radio_in(11); // Channel 12 (index 11)
    bool ch12_high = (ch12_value > 1500);
    
    // target_tilt_angle 现在是成员变量，不需要每次重置为0
    
    if (ch12_high) {
        // 通道12高位：使用通道2动态控制目标角度
        const uint32_t now_ms = AP_HAL::millis();
        
        // 计算时间间隔
        float dt_s = 0.02f;  // 默认50Hz
        if (last_tilt_angle_update_ms != 0) {
            dt_s = (now_ms - last_tilt_angle_update_ms) * 0.001f;
            if (dt_s > 1.0f || dt_s <= 0.0f) {
                dt_s = 0.02f;
            }
        }
        last_tilt_angle_update_ms = now_ms;
        
        // 获取通道16输入
        float ch16_input = 0.0f;
        RC_Channel *ch16 = RC_Channels::rc_channel(14);  // Channel 16 (index 15)
        if (ch16 != nullptr) {
            ch16_input = ch16->norm_input();  // [-1, 1]
            // 映射到 [0, 1] 范围
            ch16_input = (ch16_input + 1.0f) * 0.5f;  // [-1,1] → [0,1]
        }
        
        // 直接映射目标角度
        // 通道16 = 0 → 0°
        // 通道16 = 0.5 → 2.5°
        // 通道16 = 1 → 5°
        manual_target_tilt_angle = ch16_input * 5.0f;  // [0,1] → [0°,5°]
        
        // 限制在0-5度范围
        manual_target_tilt_angle = constrain_float(manual_target_tilt_angle, 0.0f, 5.0f);
        
        target_tilt_angle = manual_target_tilt_angle;
    }
    
    // 保存目标角度到全局变量（度 → centidegrees）
    plane.tilt_angle_cd = target_tilt_angle * 100.0f;
    
    
    // total angle the tilt can go through
    const float total_angle = 90 + tilt_yaw_angle;
    // output value (0 to 1) to get motors pointed straight up
    const float zero_out = tilt_yaw_angle / total_angle;

    // calculate the basic tilt amount from current_tilt
    float base_output = zero_out;

    if (!quadplane.in_vtol_mode() && (!plane.arming.is_armed_and_safety_off() || !quadplane.assisted_flight)) {
        // option set then if disarmed move to VTOL position to prevent ground strikes, allow tilt forward in manual mode for testing
        // const bool disarmed_tilt_up = !plane.arming.is_armed_and_safety_off() && (plane.control_mode != &plane.mode_manual) && quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT_UP);
        // slew(disarmed_tilt_up ? 0.0 : get_forward_flight_tilt());
        return;
    }

    if (!quadplane.assisted_flight &&
               (plane.control_mode == &plane.mode_qacro ||
               plane.control_mode == &plane.mode_qstabilize ||
               plane.control_mode == &plane.mode_qhover))
    {
        // 使用通道10切换控制模式
        int ch10_value = RC_Channels::get_radio_in(9); // Channel 10 (index 9)
        bool ch10_high = (ch10_value > 1500); // 高位：使用倾转角度闭环控制
        
        float adjusted_base_output = base_output;
        
        // 声明变量在外部，确保整个作用域都能访问
        float pitch_differential = 0.0f, yaw_differential = 0.0f, pitch_angle_error = 0.0f;
        float desired_pitch_rate = 0.0f, yaw_angle_error = 0.0f, desired_yaw_rate = 0.0f;
        float left_motor_output = 0.0f, right_motor_output = 0.0f;
        
        if (ch10_high) {
            // 高位：使用通道9的闭环倾转角度控制
            // 根据MPU6050反馈调整倾转角度到目标值
            // transition_pid_get_rate();
            transition_get_rate(zero_out, pitch_angle_error, pitch_differential, left_motor_output);
        } else {
            target_tilt_angle = 0.0f;
            manual_target_tilt_angle = 0.0f;
            // 低位：使用默认的VTOL PID控制
            vtol_pid_get_rate(adjusted_base_output, zero_out, 
                            pitch_differential, yaw_differential,
                            pitch_angle_error, desired_pitch_rate,
                            yaw_angle_error, desired_yaw_rate,
                            left_motor_output, right_motor_output);
        }
        
        // ========== 保存日志数据（归一化到 [-1,1] 或 [0,1]） ==========
        const float angle_max_deg = MAX(plane.quadplane.aparm.angle_max * 0.01f, 1.0f);
        const float max_pitch_rate = MAX((float)bicopter_max_rate_dps, 1.0f);
        const float max_yaw_rate   = MAX((float)bicopter_max_yaw_rate_dps, 1.0f);

        log_pitch_angle_error  = pitch_angle_error  / angle_max_deg;   // [-1, 1]
        log_pitch_desired_rate = desired_pitch_rate  / max_pitch_rate;  // [-1, 1]
        log_pitch_differential = pitch_differential;                    // [-1, 1]
        log_yaw_angle_error    = yaw_angle_error     / 180.0f;         // [-1, 1]
        log_yaw_desired_rate   = desired_yaw_rate    / max_yaw_rate;   // [-1, 1]
        log_yaw_differential   = yaw_differential;                     // [-1, 1]
        log_left_motor_output  = left_motor_output   / 1000.0f;        // [0, 1]
        log_right_motor_output = right_motor_output  / 1000.0f;        // [0, 1]
        log_pitch_rate_integral = log_pitch_rate_integral / MAX((float)bicopter_pitch_rate_imax, 1e-3f); // [-1, 1]

        const uint32_t now_ms = AP_HAL::millis();
                
        // ========== 调试输出（50Hz，TILT_VOFA_EN=1时启用） ==========
        if (vofa_enable) {
        static uint32_t last_debug_ms = 0;
        if (now_ms - last_debug_ms > 20) {
            last_debug_ms = now_ms;
            const struct { const char *name; float value; } log_fields[] = {
                {"LOG_VEL_E",  log_velocity_error},        // 速度误差 (m/s)
                {"LOG_VEL_C",  log_current_velocity},      // 当前速度 (m/s)
                {"LOG_VEL_D",  log_desired_velocity},      // 期望速度 (m/s)
                {"LOG_VEL_P",  log_velocity_pitch_cmd},    // 速度输出俯仰角 (deg)
                {"LOG_PAE",  log_pitch_angle_error},       // 俯仰角度误差
                {"LOG_PDR",  log_pitch_desired_rate},      // 期望俯仰角速度
                {"LOG_PRE",  log_pitch_rate_error},        // 俯仰角速度误差
                {"LOG_PRP",  log_pitch_rate_p},            // 俯仰角速度P项
                {"LOG_PRI", log_pitch_rate_integral},      // 俯仰角速度积分
                {"LOG_PD",  log_pitch_differential},       // 俯仰差分输出
                {"LOG_YAE",  log_yaw_angle_error},         // 偏航角度误差
                {"LOG_YDR",  log_yaw_desired_rate},        // 期望偏航角速度
                {"LOG_YD",   log_yaw_differential},        // 偏航差分输出
                {"LOG_LM",   log_left_motor_output},       // 左电机输出
                {"LOG_RM",   log_right_motor_output},      // 右电机输出
            };
            for (const auto &f : log_fields) {
                gcs().send_named_float(f.name, f.value);
            }
        }

        static uint32_t last_text_ms = 0;
        if (now_ms - last_text_ms > 1000) {
            last_text_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                        "Bi b:%.2f adj:%.2f PE:%.1f PD:%.1f L:%.0f R:%.0f",
                        (double)base_output,
                        (double)adjusted_base_output,
                        (double)pitch_angle_error,
                        (double)pitch_differential,
                        (double)left_motor_output,
                        (double)right_motor_output);
        }
        } // vofa_enable
        return;
    }
    
    if
    (
        quadplane.assisted_flight &&
        transition->transition_state >= Tiltrotor_Transition::TRANSITION_TIMER
    ) 
    {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        // slew(get_forward_flight_tilt());
    } else {
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
    //    float settilt = constrain_float((SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)-MAX(plane.aparm.throttle_min.get(),0)) * 0.02, 0, 1);
    //    slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
    }
}

#endif  // HAL_QUADPLANE_ENABLED
