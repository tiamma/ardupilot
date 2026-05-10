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

    // @Param: TRAJ_ACCEL
    // @DisplayName: Trajectory max acceleration
    // @Description: Maximum angular acceleration for trajectory planning
    // @Units: deg/s/s
    // @Range: 60 360
    // @User: Advanced
    AP_GROUPINFO("TR_A", 36, Tiltrotor, trajectory_max_accel, 180),

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

    // if (type == TILT_TYPE_BICOPTER) {
        bicopter_update();
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

/*
  双旋翼串级PID控制
  俯仰控制：角度误差 → 期望角速度 → 电机差分输出
  偏航控制：角度误差 → 期望角速度 → 电机差分输出
  最终输出：俯仰差分 + 偏航差分 → 左右电机
*/
void Tiltrotor::bicopter_update()
{
    // total angle the tilt can go through
    const float total_angle = 90 + tilt_yaw_angle;
    // output value (0 to 1) to get motors pointed straight up
    const float zero_out = tilt_yaw_angle / total_angle;

    // calculate the basic tilt amount from current_tilt
    float base_output = zero_out;

    if (!quadplane.in_vtol_mode() && (!plane.arming.is_armed_and_safety_off() || !quadplane.assisted_flight)) {
        // option set then if disarmed move to VTOL position to prevent ground strikes, allow tilt forward in manual mode for testing
        const bool disarmed_tilt_up = !plane.arming.is_armed_and_safety_off() && (plane.control_mode != &plane.mode_manual) && quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT_UP);
        slew(disarmed_tilt_up ? 0.0 : get_forward_flight_tilt());
        return;
    }

    if (!quadplane.assisted_flight &&
               (plane.control_mode == &plane.mode_qacro ||
               plane.control_mode == &plane.mode_qstabilize ||
               plane.control_mode == &plane.mode_qhover))
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
        
        // ========== 俯仰控制 ==========
        // 外环：俯仰角度控制 (Pitch Angle → Desired Pitch Rate)
        
        // 1. 获取当前俯仰角度和目标角度
        float current_pitch_deg = plane.ahrs.pitch_sensor * 0.01f;  // centidegrees → degrees
        float target_pitch_deg_raw = plane.nav_pitch_cd * 0.01f;    // 原始目标俯仰角度
        
        // 1.5 轨迹规划（如果启用）
        float target_pitch_deg = target_pitch_deg_raw;
        if (enable_trajectory == 1) {
            // 检查目标是否改变，如果改变则重新初始化轨迹
            static float last_target = 0;
            if (fabsf(target_pitch_deg_raw - last_target) > 0.5f) {
                // 目标改变，初始化新轨迹
                trajectory_init(current_pitch_deg, target_pitch_deg_raw);
                last_target = target_pitch_deg_raw;
            }
            
            // 如果轨迹激活，使用轨迹规划的目标
            if (traj_active) {
                float t = (now_ms - traj_start_time) * 0.001f;  // 转换为秒
                target_pitch_deg = trajectory_get_angle(t);
            }
        }
        
        // 2. 计算俯仰角度误差
        float pitch_angle_error = target_pitch_deg - current_pitch_deg;
        
        // 3. 俯仰角度环PID计算
        float pitch_angle_p = bicopter_pitch_angle_p * pitch_angle_error;
        
       

        bicopter_angle_integral += pitch_angle_error * dt_s;
        bicopter_angle_integral = constrain_float(bicopter_angle_integral, 
                                                -bicopter_pitch_angle_imax, 
                                                bicopter_pitch_angle_imax);

        // if (fabsf(target_pitch_deg) > 0) {
        //     bicopter_angle_integral = 0;
        // }
        float pitch_angle_i = bicopter_pitch_angle_i * bicopter_angle_integral;
        
        float pitch_angle_d_input = (pitch_angle_error - bicopter_last_pitch_error) / dt_s;
        bicopter_last_pitch_error = pitch_angle_error;
        float pitch_angle_d = bicopter_pitch_angle_d * pitch_angle_d_input;
        
        // 4. 计算期望俯仰角速度（外环输出）
        float desired_pitch_rate = pitch_angle_p + pitch_angle_i + pitch_angle_d;
        desired_pitch_rate = constrain_float(desired_pitch_rate, 
                                            -bicopter_max_rate_dps, 
                                            bicopter_max_rate_dps);
        
        float extra_elevator = desired_pitch_rate;
        // float extra_sign = desired_pitch_rate > 0?1:-1;
        // if (!is_zero(desired_pitch_rate) ) {
        //     extra_elevator = extra_sign * powf(fabsf(desired_pitch_rate), 2.5) * 0.5;
        // }

        // 内环：俯仰角速度控制 (Pitch Rate Error → Motor Differential)
        float current_pitch_rate = plane.ahrs.get_gyro().y * RAD_TO_DEG;  // rad/s → deg/s
        float pitch_rate_error = extra_elevator - current_pitch_rate;
        
        // 5. 俯仰角速度环PID计算
        float pitch_rate_p = bicopter_pitch_rate_p * pitch_rate_error;
        
        // 当角度误差小于1度时，清空积分项，防止小误差积分累积
    
        bicopter_rate_integral += pitch_rate_error * dt_s;
        bicopter_rate_integral = constrain_float(bicopter_rate_integral, 
                                                -bicopter_pitch_rate_imax, 
                                                bicopter_pitch_rate_imax);
    

        if (fabsf(pitch_angle_error) < 1.0f) {
            bicopter_rate_integral = 0.0f;
        }

        float pitch_rate_i = bicopter_pitch_rate_i * bicopter_rate_integral;
        
        float pitch_rate_d_input = (pitch_rate_error - bicopter_last_rate_error) / dt_s;
        bicopter_last_rate_error = pitch_rate_error;

        float pitch_rate_d = bicopter_pitch_rate_d * pitch_rate_d_input;
        
        // 6. 计算俯仰电机差分输出（内环输出）
        float pitch_differential = pitch_rate_p + pitch_rate_i + pitch_rate_d;
        pitch_differential = constrain_float(pitch_differential, -1.0f, 1.0f);

        // ========== 偏航控制 ==========
        // 外环：偏航角度控制 (Yaw Angle → Desired Yaw Rate)
        
        // 1. 获取当前偏航角度和目标角度
        float current_yaw_deg = plane.ahrs.yaw_sensor * 0.01f;  // centidegrees → degrees
        float target_yaw_deg = 0;        // 目标偏航角度
        
        // 2. 计算偏航角度误差（处理360度环绕）
        float yaw_angle_error = target_yaw_deg - current_yaw_deg;
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
        float desired_yaw_rate = yaw_angle_p + yaw_angle_i + yaw_angle_d;
        desired_yaw_rate = constrain_float(desired_yaw_rate, 
                                        -bicopter_max_yaw_rate_dps, 
                                        bicopter_max_yaw_rate_dps);
        
        // 内环：偏航角速度控制 (Yaw Rate Error → Motor Differential)
        float current_yaw_rate = plane.ahrs.get_gyro().z * RAD_TO_DEG;  // rad/s → deg/s
        float yaw_rate_error = desired_yaw_rate - current_yaw_rate;
        
        // 5. 偏航角速度环PID计算
        float yaw_rate_p = bicopter_yaw_rate_p * yaw_rate_error;
        
        // 当角度误差小于1度时，清空积分项，防止小误差积分累积
        if (fabsf(yaw_angle_error) < 1.0f) {
            bicopter_yaw_rate_integral = 0.0f;
        } else {
            bicopter_yaw_rate_integral += yaw_rate_error * dt_s;
            bicopter_yaw_rate_integral = constrain_float(bicopter_yaw_rate_integral, 
                                                        -bicopter_yaw_rate_imax, 
                                                        bicopter_yaw_rate_imax);
        }
        float yaw_rate_i = bicopter_yaw_rate_i * bicopter_yaw_rate_integral;
        
        float yaw_rate_d_input = (yaw_rate_error - bicopter_last_yaw_rate_error) / dt_s;
        bicopter_last_yaw_rate_error = yaw_rate_error;
        float yaw_rate_d = bicopter_yaw_rate_d * yaw_rate_d_input;
        
        // 6. 计算偏航电机差分输出（内环输出）
        float yaw_differential = yaw_rate_p + yaw_rate_i + yaw_rate_d;
        yaw_differential = constrain_float(yaw_differential, -1.0f, 1.0f);
        
        // ========== 组合输出到左右倾转电机 ==========
        
        // 基础位置（中位，0.5对应舵机信号500）
        // float base_position = 0.5f;
        
        // 组合俯仰和偏航控制
        // 俯仰：左右电机反向（左+，右-）
        // 偏航：左右电机同向（左+，右+）用于转向
        // float left_tilt = base_output + pitch_differential + yaw_differential;
        // float right_tilt = base_output - pitch_differential + yaw_differential;

        float pitch_range = zero_out;
        float pitch_diff = pitch_differential * pitch_range;
        float yaw_diff = (yaw_differential / 2.0f) * pitch_range;

        if (pitch_diff > bicopter_max_motor_diff) {
            pitch_diff = bicopter_max_motor_diff;
        } else if (pitch_diff < -bicopter_max_motor_diff) {
            pitch_diff = -bicopter_max_motor_diff;
        }

        float left_tilt = base_output + left_pitch_sign * pitch_diff - left_yaw_sign * yaw_diff;
        float right_tilt = base_output + right_pitch_sign * pitch_diff + right_yaw_sign * yaw_diff;
        
        // 限制输出范围并转换为舵机信号 (0-1000)
        float left_motor_output = 1000 * constrain_float(left_tilt, 0.0, 1.0);
        float right_motor_output = 1000 * constrain_float(right_tilt, 0.0, 1.0);
        
        // 使用计数器交替更新电机：奇数更新左电机，偶数更新右电机
        bicopter_motor_update_counter++;
        if (bicopter_motor_update_counter >= 10000) {
            bicopter_motor_update_counter = 0;
        }
        
        if (bicopter_motor_update_counter % 2 == 1) {
            // 奇数：更新左电机
            SRV_Channels::set_output_scaled(SRV_Channel::k_scripting1, left_motor_output);
        } else {
            // 偶数：更新右电机
            SRV_Channels::set_output_scaled(SRV_Channel::k_scripting2, right_motor_output);
        }
        
        // ========== 保存日志数据 ==========
        log_pitch_angle_error = pitch_angle_error;
        log_pitch_desired_rate = desired_pitch_rate;
        log_pitch_differential = pitch_differential;
        log_yaw_angle_error = yaw_angle_error;
        log_yaw_desired_rate = desired_yaw_rate;
        log_yaw_differential = yaw_differential;
        log_left_motor_output = left_motor_output;
        log_right_motor_output = right_motor_output;
        
                
        // ========== 调试输出（每秒一次） ==========
        static uint32_t last_debug_ms = 0;
        if (now_ms - last_debug_ms > 1000) {
            last_debug_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, 
                        "Bi b:%.2f zt=%.2f,ta=%.2f PE:%.1f PD:%.1f L:%.0f R:%.0f",
                        (double)base_output,
                        (double)zero_out,
                        (double)total_angle,
                        (double)pitch_angle_error,
                        (double)pitch_differential,
                        (double)left_motor_output,
                        (double)right_motor_output);
        }
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
  轨迹规划辅助函数实现
  基于S曲线（摆线变形）生成最优轨迹
*/

// 初始化轨迹规划
void Tiltrotor::trajectory_init(float start_angle, float end_angle)
{
    traj_start_angle = start_angle;
    traj_end_angle = end_angle;
    traj_start_time = AP_HAL::millis();
    
    float angle_delta = fabsf(end_angle - start_angle);
    float max_rate = trajectory_max_rate;
    float max_accel = trajectory_max_accel;
    
    // 计算加速时间
    float t_accel = max_rate / max_accel;
    
    // 加速和减速阶段的角度变化
    float angle_accel = 0.5f * max_accel * t_accel * t_accel;
    
    // 检查是否需要匀速阶段
    if (2.0f * angle_accel < angle_delta) {
        // 需要匀速阶段（梯形速度曲线）
        traj_has_constant_phase = true;
        float angle_constant = angle_delta - 2.0f * angle_accel;
        traj_constant_time = angle_constant / max_rate;
        traj_total_time = 2.0f * t_accel + traj_constant_time;
        traj_max_rate_actual = max_rate;
    } else {
        // 不需要匀速阶段（三角形速度曲线）
        traj_has_constant_phase = false;
        traj_constant_time = 0;
        // 重新计算加速时间和最大速度
        t_accel = sqrtf(angle_delta / max_accel);
        traj_max_rate_actual = max_accel * t_accel;
        traj_total_time = 2.0f * t_accel;
    }
    
    traj_accel_time = t_accel;
    traj_active = true;
}

// 获取当前时刻的目标角度
float Tiltrotor::trajectory_get_angle(float t)
{
    if (t <= 0) {
        return traj_start_angle;
    }
    if (t >= traj_total_time) {
        traj_active = false;
        return traj_end_angle;
    }
    
    float angle_delta = traj_end_angle - traj_start_angle;
    float sign = (angle_delta > 0) ? 1.0f : -1.0f;
    float max_accel = trajectory_max_accel;
    
    float angle;
    
    if (t < traj_accel_time) {
        // 加速阶段
        angle = traj_start_angle + sign * 0.5f * max_accel * t * t;
    } else if (traj_has_constant_phase && t < (traj_accel_time + traj_constant_time)) {
        // 匀速阶段
        float t_const = t - traj_accel_time;
        float angle_accel = sign * 0.5f * max_accel * traj_accel_time * traj_accel_time;
        angle = traj_start_angle + angle_accel + sign * traj_max_rate_actual * t_const;
    } else {
        // 减速阶段
        float remaining_time = traj_total_time - t;
        angle = traj_end_angle - sign * 0.5f * max_accel * remaining_time * remaining_time;
    }
    
    return angle;
}

// 获取当前时刻的目标角速度
float Tiltrotor::trajectory_get_rate(float t)
{
    if (t <= 0 || t >= traj_total_time) {
        return 0;
    }
    
    float angle_delta = traj_end_angle - traj_start_angle;
    float sign = (angle_delta > 0) ? 1.0f : -1.0f;
    float max_accel = trajectory_max_accel;
    
    float rate;
    
    if (t < traj_accel_time) {
        // 加速阶段
        rate = sign * max_accel * t;
    } else if (traj_has_constant_phase && t < (traj_accel_time + traj_constant_time)) {
        // 匀速阶段
        rate = sign * traj_max_rate_actual;
    } else {
        // 减速阶段
        float remaining_time = traj_total_time - t;
        rate = sign * max_accel * remaining_time;
    }
    
    return rate;
}

#endif  // HAL_QUADPLANE_ENABLED
