#include "Plane.h"
#include "AC_AttitudeControl/AC_AttitudeControl_TS.h"

const AP_Param::GroupInfo QuadPlane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable QuadPlane
    // @Description: This enables QuadPlane functionality, assuming multicopter motors start on output 5. If this is set to 2 then when starting AUTO mode it will initially be in VTOL AUTO mode.
    // @Values: 0:Disable,1:Enable,2:Enable VTOL AUTO
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: M_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    AP_SUBGROUPVARPTR(motors, "M_", 2, QuadPlane, plane.quadplane.motors_var_info),

    // 3 ~ 8 were used by quadplane attitude control PIDs

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all VTOL flight modes
    // @Units: cdeg
    // @Increment: 10
    // @Range: 1000 8000
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 10, QuadPlane, aparm.angle_max, 3000),

    // @Param: TRANSITION_MS
    // @DisplayName: Transition time
    // @Description: Transition time in milliseconds after minimum airspeed is reached
    // @Units: ms
    // @Range: 0 30000
    // @User: Advanced
    AP_GROUPINFO("TRANSITION_MS", 11, QuadPlane, transition_time_ms, 5000),

    // 12 ~ 16 were used by position, velocity and acceleration PIDs

    // @Group: P
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    AP_SUBGROUPPTR(pos_control, "P", 17, QuadPlane, AC_PosControl),

    // @Param: VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed up
    // @Description: The maximum ascending vertical velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("VELZ_MAX", 18, QuadPlane, pilot_velocity_z_max_up, 250),
   
   // @Param: VELZ_MAX_DN
    // @DisplayName: Pilot maximum vertical speed down
    // @Description: The maximum vertical velocity the pilot may request in cm/s going down. If 0, uses Q_VELZ_MAX value.
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("VELZ_MAX_DN", 60, QuadPlane, pilot_velocity_z_max_dn, 0),

     // @Param: ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",  19, QuadPlane, pilot_accel_z,  250),

    // @Group: WP_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    AP_SUBGROUPPTR(wp_nav, "WP_",  20, QuadPlane, AC_WPNav),

    // @Param: RC_SPEED
    // @DisplayName: RC output speed in Hz
    // @Description: This is the PWM refresh rate in Hz for QuadPlane quad motors
    // @Units: Hz
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RC_SPEED", 21, QuadPlane, rc_speed, 490),

    // @Param: THR_MIN_PWM
    // @DisplayName: Minimum PWM output
    // @Description: This is the minimum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MIN_PWM", 22, QuadPlane, thr_min_pwm, 1000),

    // @Param: THR_MAX_PWM
    // @DisplayName: Maximum PWM output
    // @Description: This is the maximum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX_PWM", 23, QuadPlane, thr_max_pwm, 2000),

    // @Param: ASSIST_SPEED
    // @DisplayName: Quadplane assistance speed
    // @Description: This is the speed below which the quad motors will provide stability and lift assistance in fixed wing modes. Zero means no assistance except during transition
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ASSIST_SPEED", 24, QuadPlane, assist_speed, 0),

    // @Param: YAW_RATE_MAX
    // @DisplayName: Maximum yaw rate
    // @Description: This is the maximum yaw rate for pilot input on rudder stick in degrees/second
    // @Units: deg/s
    // @Range: 50 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_RATE_MAX", 25, QuadPlane, yaw_rate_max, 100),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("LAND_SPEED", 26, QuadPlane, land_speed_cms, 50),

    // @Param: LAND_FINAL_ALT
    // @DisplayName: Land final altitude
    // @Description: The altitude at which we should switch to Q_LAND_SPEED descent rate
    // @Units: m
    // @Range: 0.5 50
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LAND_FINAL_ALT", 27, QuadPlane, land_final_alt, 6),

    // 28 was used by THR_MID

    // @Param: TRAN_PIT_MAX
    // @DisplayName: Transition max pitch
    // @Description: Maximum pitch during transition to auto fixed wing flight
    // @User: Standard
    // @Range: 0 30
    // @Units: deg
    // @Increment: 1
    AP_GROUPINFO("TRAN_PIT_MAX", 29, QuadPlane, transition_pitch_max, 3),

    // frame class was moved from 30 when consolidating AP_Motors classes
#define FRAME_CLASS_OLD_IDX 30
    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 7:Tri, 10: TailSitter, 12:DodecaHexa, 14:Deca, 15:Scripting Matrix
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 46, QuadPlane, frame_class, 1),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X or V)
    // @Description: Controls motor mixing for multicopter component
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F, 12:BetaFlightX, 13:DJIX, 14:ClockwiseX, 15:I, 16:MOTOR_FRAME_TYPE_NYT_PLUS, 17:MOTOR_FRAME_TYPE_NYT_X, 18: BetaFlightXReversed
    // @User: Standard
    AP_GROUPINFO("FRAME_TYPE", 31, QuadPlane, frame_type, 1),

    // @Param: VFWD_GAIN
    // @DisplayName: Forward velocity hold gain
    // @Description: Controls use of forward motor in vtol modes. If this is zero then the forward motor will not be used for position control in VTOL modes. A value of 0.05 is a good place to start if you want to use the forward motor for position control. No forward motor will be used in QSTABILIZE or QHOVER modes. Use QLOITER for position hold with the forward motor.
    // @Range: 0 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("VFWD_GAIN", 32, QuadPlane, vel_forward.gain, 0),

    // @Param: WVANE_GAIN
    // @DisplayName: Weathervaning gain
    // @Description: This controls the tendency to yaw to face into the wind. A value of 0.1 is to start with and will give a slow turn into the wind. Use a value of 0.4 for more rapid response. The weathervaning works by turning into the direction of roll.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WVANE_GAIN", 33, QuadPlane, weathervane.gain, 0),

    // @Param: WVANE_MINROLL
    // @DisplayName: Weathervaning min roll
    // @Description: This set the minimum roll in degrees before active weathervaning will start. This may need to be larger if your aircraft has bad roll trim.
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("WVANE_MINROLL", 34, QuadPlane, weathervane.min_roll, 1),

    // @Param: RTL_ALT
    // @DisplayName: QRTL return altitude
    // @Description: The altitude which QRTL mode heads to initially
    // @Units: m
    // @Range: 1 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_ALT", 35, QuadPlane, qrtl_alt, 15),

    // @Param: RTL_MODE
    // @DisplayName: VTOL RTL mode
    // @Description: If this is set to 1 then an RTL will change to QRTL when within RTL_RADIUS meters of the RTL destination, VTOL approach: vehicle will RTL at RTL alt and circle with a radius of Q_FW_LND_APR_RAD down to Q_RLT_ALT and then transission into the wind and QRTL, see 'AUTO VTOL Landing', QRTL Always: do a QRTL instead of RTL
    // @Values: 0:Disabled,1:Enabled,2:VTOL approach,3:QRTL Always
    // @User: Standard
    AP_GROUPINFO("RTL_MODE", 36, QuadPlane, rtl_mode, 0),

    // @Param: TILT_MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    AP_GROUPINFO("TILT_MASK", 37, QuadPlane, tilt.tilt_mask, 0),

    // @Param: TILT_RATE_UP
    // @DisplayName: Tiltrotor upwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("TILT_RATE_UP", 38, QuadPlane, tilt.max_rate_up_dps, 40),

    // @Param: TILT_MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: deg
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("TILT_MAX", 39, QuadPlane, tilt.max_angle_deg, 45),

    // @Param: GUIDED_MODE
    // @DisplayName: Enable VTOL in GUIDED mode
    // @Description: This enables use of VTOL in guided mode. When enabled the aircraft will switch to VTOL flight when the guided destination is reached and hover at the destination.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GUIDED_MODE", 40, QuadPlane, guided_mode, 0),

    // 41 was used by THR_MIN

    // @Param: ESC_CAL
    // @DisplayName: ESC Calibration
    // @Description: This is used to calibrate the throttle range of the VTOL motors. Please read https://ardupilot.org/plane/docs/quadplane-esc-calibration.html before using. This parameter is automatically set back to 0 on every boot. This parameter only takes effect in QSTABILIZE mode. When set to 1 the output of all motors will come directly from the throttle stick when armed, and will be zero when disarmed. When set to 2 the output of all motors will be maximum when armed and zero when disarmed. Make sure you remove all properllers before using.
    // @Values: 0:Disabled,1:ThrottleInput,2:FullInput
    // @User: Standard
    AP_GROUPINFO("ESC_CAL", 42, QuadPlane, esc_calibration,  0),

    // @Param: VFWD_ALT
    // @DisplayName: Forward velocity alt cutoff
    // @Description: Controls altitude to disable forward velocity assist when below this relative altitude. This is useful to keep the forward velocity propeller from hitting the ground. Rangefinder height data is incorporated when available.
    // @Units: m
    // @Range: 0 10
    // @Increment: 0.25
    // @User: Standard
    AP_GROUPINFO("VFWD_ALT", 43, QuadPlane, vel_forward_alt_cutoff,  0),

    // @Param: LAND_ICE_CUT
    // @DisplayName: Cut IC engine on landing
    // @Description: This controls stopping an internal combustion engine in the final landing stage of a VTOL. This is important for aircraft where the forward thrust engine may experience prop-strike if left running during landing. This requires the engine controls are enabled using the ICE_* parameters.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LAND_ICE_CUT", 44, QuadPlane, land_icengine_cut,  1),
    
    // @Param: ASSIST_ANGLE
    // @DisplayName: Quadplane assistance angle
    // @Description: This is the angular error in attitude beyond which the quadplane VTOL motors will provide stability assistance. This will only be used if Q_ASSIST_SPEED is also non-zero. Assistance will be given if the attitude is outside the normal attitude limits by at least 5 degrees and the angular error in roll or pitch is greater than this angle for at least Q_ASSIST_DELAY seconds. Set to zero to disable angle assistance.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ANGLE", 45, QuadPlane, assist_angle, 30),

    // @Param: TILT_TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover, Bicopter tiltrottor must use the tailsitter frame class (10)
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw,3:Bicopter
    AP_GROUPINFO("TILT_TYPE", 47, QuadPlane, tilt.tilt_type, TILT_TYPE_CONTINUOUS),

    // @Param: TAILSIT_ANGLE
    // @DisplayName: Tailsitter fixed wing transition angle
    // @Description: This is the pitch angle at which tailsitter aircraft will change from VTOL control to fixed wing control.
    // @Units: deg
    // @Range: 5 80
    AP_GROUPINFO("TAILSIT_ANGLE", 48, QuadPlane, tailsitter.transition_angle_fw, 45),

    // @Param: TAILSIT_ANG_VT
    // @DisplayName: Tailsitter VTOL transition angle
    // @Description: This is the pitch angle at which tailsitter aircraft will change from fixed wing control to VTOL control, if zero Q_TAILSIT_ANGLE will be used
    // @Units: deg
    // @Range: 5 80
    AP_GROUPINFO("TAILSIT_ANG_VT", 61, QuadPlane, tailsitter.transition_angle_vtol, 0),

    // @Param: TILT_RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("TILT_RATE_DN", 49, QuadPlane, tilt.max_rate_down_dps, 0),
        
    // @Param: TAILSIT_INPUT
    // @DisplayName: Tailsitter input type bitmask
    // @Description: This controls whether stick input when hovering as a tailsitter follows the conventions for fixed wing hovering or multicopter hovering. When PlaneMode is not enabled (bit0 = 0) the roll stick will roll the aircraft in earth frame and yaw stick will yaw in earth frame. When PlaneMode input is enabled, the roll and yaw sticks are swapped so that the roll stick controls earth-frame yaw and rudder controls earth-frame roll. When body-frame roll is enabled (bit1 = 1), the yaw stick controls earth-frame yaw rate and the roll stick controls roll in the tailsitter's body frame when flying level.
    // @Bitmask: 0:PlaneMode,1:BodyFrameRoll
    AP_GROUPINFO("TAILSIT_INPUT", 50, QuadPlane, tailsitter.input_type, 0),

    // @Param: TAILSIT_MASK
    // @DisplayName: Tailsitter input mask
    // @Description: This controls what channels have full manual control when hovering as a tailsitter and the Q_TAILSIT_MASKCH channel in high. This can be used to teach yourself to prop-hang a 3D plane by learning one or more channels at a time.
    // @Bitmask: 0:Aileron,1:Elevator,2:Throttle,3:Rudder
    AP_GROUPINFO("TAILSIT_MASK", 51, QuadPlane, tailsitter.input_mask, 0),

    // @Param: TAILSIT_MASKCH
    // @DisplayName: Tailsitter input mask channel
    // @Description: This controls what input channel will activate the Q_TAILSIT_MASK mask. When this channel goes above 1700 then the pilot will have direct manual control of the output channels specified in Q_TAILSIT_MASK. Set to zero to disable.
    // @Values: 0:Disabled,1:Channel1,2:Channel2,3:Channel3,4:Channel4,5:Channel5,6:Channel6,7:Channel7,8:Channel8
    AP_GROUPINFO("TAILSIT_MASKCH", 52, QuadPlane, tailsitter.input_mask_chan, 0),

    // @Param: TAILSIT_VFGAIN
    // @DisplayName: Tailsitter vector thrust gain in forward flight
    // @Description: This controls the amount of vectored thrust control used in forward flight for a vectored tailsitter
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("TAILSIT_VFGAIN", 53, QuadPlane, tailsitter.vectored_forward_gain, 0),

    // @Param: TAILSIT_VHGAIN
    // @DisplayName: Tailsitter vector thrust gain in hover
    // @Description: This controls the amount of vectored thrust control used in hover for a vectored tailsitter
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("TAILSIT_VHGAIN", 54, QuadPlane, tailsitter.vectored_hover_gain, 0.5),

    // @Param: TILT_YAW_ANGLE
    // @DisplayName: Tilt minimum angle for vectored yaw
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output. This needs to be set for Q_TILT_TYPE=3 to enable vectored control for yaw of tricopter tilt quadplanes. This is also used to limit the forwards travel of bicopter tilts when in VTOL modes
    // @Range: 0 30
    AP_GROUPINFO("TILT_YAW_ANGLE", 55, QuadPlane, tilt.tilt_yaw_angle, 0),

    // @Param: TAILSIT_VHPOW
    // @DisplayName: Tailsitter vector thrust gain power
    // @Description: This controls the amount of extra pitch given to the vectored control when at high pitch errors
    // @Range: 0 4
    // @Increment: 0.1
    AP_GROUPINFO("TAILSIT_VHPOW", 56, QuadPlane, tailsitter.vectored_hover_power, 2.5),

    // @Param: MAV_TYPE
    // @DisplayName: MAVLink type identifier
    // @Description: This controls the mavlink type given in HEARTBEAT messages. For some GCS types a particular setting will be needed for correct operation.
    // @Values: 0:AUTO,1:FIXED_WING,2:QUADROTOR,3:COAXIAL,4:HELICOPTER,7:AIRSHIP,8:FREE_BALLOON,9:ROCKET,10:GROUND_ROVER,11:SURFACE_BOAT,12:SUBMARINE,16:FLAPPING_WING,17:KITE,19:VTOL_DUOROTOR,20:VTOL_QUADROTOR,21:VTOL_TILTROTOR
    AP_GROUPINFO("MAV_TYPE", 57, QuadPlane, mav_type, 0),

    // @Param: OPTIONS
    // @DisplayName: quadplane options
    // @Description: Level Transition:Keep wings within LEVEL_ROLL_LIMIT and only use forward motor(s) for climb during transition, Allow FW Takeoff: If bit is not set then NAV_TAKEOFF command on quadplanes will instead perform a NAV_VTOL takeoff, Allow FW Land:If bit is not set then NAV_LAND command on quadplanes will instead perform a NAV_VTOL_LAND, Vtol Takeoff Frame: command NAV_VTOL_TAKEOFF altitude is as set by the command's reference frame rather than a delta above current location, Use FW Approach:Use a fixed wing approach for VTOL landings, USE QRTL:instead of QLAND for rc failsafe when in VTOL modes, Use Governor:Use ICE Idle Governor in MANUAL for forward motor, Force Qassist: on always,Mtrs_Only_Qassist: in tailsitters only, uses VTOL motors and not flying surfaces for QASSIST, Airmode_On_Arm:Airmode enabled when arming by aux switch, Disarmed Yaw Tilt:Enable motor tilt for yaw when disarmed, Delay Spoolup:Delay VTOL spoolup for 2 seconds after arming, ThrLandControl: enable throttle stick control of landing rate, DisableApproach: Disable use of approach and airbrake stages in VTOL landing, EnableLandResponsition: enable pilot controlled repositioning in AUTO land. Descent will pause while repositioning.
    // @Bitmask: 0:Level Transition,1:Allow FW Takeoff,2:Allow FW Land,3:Vtol Takeoff Frame,4:Use FW Approach,5:Use QRTL,6:Use Governor,7:Force Qassist,8:Mtrs_Only_Qassist,9:Airmode_On_Arm,10:Disarmed Yaw Tilt,11:Delay Spoolup,12:disable Qassist based on synthetic airspeed,13:Disable Ground Effect Compensation,14:Ignore forward flight angle limits in Qmodes,15:ThrLandControl,16:DisableApproach,17:EnableLandResponsition
    AP_GROUPINFO("OPTIONS", 58, QuadPlane, options, 0),

    AP_SUBGROUPEXTENSION("",59, QuadPlane, var_info2),

    // 60 is used above for VELZ_MAX_DN
    // 61 is used above for TS_ANGLE_VTOL

    AP_GROUPEND
};

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo QuadPlane::var_info2[] = {
    // @Param: TRANS_DECEL
    // @DisplayName: Transition deceleration
    // @Description: This is deceleration rate that will be used in calculating the stopping distance when transitioning from fixed wing flight to multicopter flight.
    // @Units: m/s/s
    // @Increment: 0.1
    // @Range: 0.2 5
    // @User: Standard
    AP_GROUPINFO("TRANS_DECEL", 1, QuadPlane, transition_decel, 2.0),

    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    AP_SUBGROUPPTR(loiter_nav, "LOIT_",  2, QuadPlane, AC_Loiter),

    // @Param: TAILSIT_GSCMAX
    // @DisplayName: Maximum tailsitter gain scaling
    // @Description: Maximum gain scaling for tailsitter Q_TAILSIT_GSCMSK options
    // @Range: 1 5
    // @User: Standard
    AP_GROUPINFO("TAILSIT_GSCMAX", 3, QuadPlane, tailsitter.throttle_scale_max, 2),

    // @Param: TRIM_PITCH
    // @DisplayName: Quadplane AHRS trim pitch
    // @Description: This sets the compensation for the pitch angle trim difference between forward and vertical flight pitch, NOTE! this is relative to forward flight trim not mounting locaiton. For tailsitters this is relative to a baseline of 90 degrees.
    // @Units: deg
    // @Range: -10 +10
    // @Increment: 0.1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TRIM_PITCH", 4, QuadPlane, ahrs_trim_pitch, 0),

    // @Param: TAILSIT_RLL_MX
    // @DisplayName: Maximum Roll angle
    // @Description: Maximum Allowed roll angle for tailsitters. If this is zero then Q_ANGLE_MAX is used.
    // @Units: deg
    // @Range: 0 80
    // @User: Standard
    AP_GROUPINFO("TAILSIT_RLL_MX", 5, QuadPlane, tailsitter.max_roll_angle, 0),

#if QAUTOTUNE_ENABLED
    // @Group: AUTOTUNE_
    // @Path: ../libraries/AC_AutoTune/AC_AutoTune.cpp
    AP_SUBGROUPINFO(qautotune, "AUTOTUNE_",  6, QuadPlane, QAutoTune),
#endif

    // @Param: FW_LND_APR_RAD
    // @DisplayName: Quadplane fixed wing landing approach radius
    // @Description: This provides the radius used, when using a fixed wing landing approach. If set to 0 then the WP_LOITER_RAD will be selected.
    // @Units: m
    // @Range: 0 200
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("FW_LND_APR_RAD", 7, QuadPlane, fw_land_approach_radius, 0),

    // @Param: TRANS_FAIL
    // @DisplayName: Quadplane transition failure time
    // @Description: Maximum time allowed for forward transitions, exceeding this time will cancel the transition and the aircraft will immediately change to QLAND. 0 for no limit.
    // @Units: s
    // @Range: 0 20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRANS_FAIL", 8, QuadPlane, transition_failure, 0),

    // @Param: TAILSIT_MOTMX
    // @DisplayName: Tailsitter motor mask
    // @Description: Bitmask of motors to remain active in forward flight for a 'Copter' tailsitter. Non-zero indicates airframe is a Copter tailsitter and uses copter style motor layouts determined by Q_FRAME_CLASS and Q_FRAME_TYPE. This should be zero for non-Copter tailsitters.
    // @User: Standard
    // @Bitmask: 0:Motor 1,1:Motor 2,2:Motor 3,3:Motor 4, 4:Motor 5,5:Motor 6,6:Motor 7,7:Motor 8
    AP_GROUPINFO("TAILSIT_MOTMX", 9, QuadPlane, tailsitter.motor_mask, 0),

    // @Param: THROTTLE_EXPO
    // @DisplayName: Throttle expo strength
    // @Description: Amount of curvature in throttle curve: 0 is linear, 1 is cubic
    // @Range: 0 1
    // @Increment: .1
    // @User: Advanced
    AP_GROUPINFO("THROTTLE_EXPO", 10, QuadPlane, throttle_expo, 0.2),

    // @Param: ACRO_RLL_RATE
    // @DisplayName: QACRO mode roll rate
    // @Description: The maximum roll rate at full stick deflection in QACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_RLL_RATE", 11, QuadPlane, acro_roll_rate, 360),

    // @Param: ACRO_PIT_RATE
    // @DisplayName: QACRO mode pitch rate
    // @Description: The maximum pitch rate at full stick deflection in QACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_PIT_RATE", 12, QuadPlane, acro_pitch_rate, 180),

    // @Param: ACRO_YAW_RATE
    // @DisplayName: QACRO mode yaw rate
    // @Description: The maximum yaw rate at full stick deflection in QACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ACRO_YAW_RATE", 13, QuadPlane, acro_yaw_rate, 90),

    // @Param: TKOFF_FAIL_SCL
    // @DisplayName: Takeoff time failure scalar
    // @Description: Scalar for how long past the expected takeoff time a takeoff should be considered as failed and the vehicle will switch to QLAND. If set to 0 there is no limit on takeoff time.
    // @Range: 1.1 5.0
    // @Increment: 5.1
    // @User: Advanced
    AP_GROUPINFO("TKOFF_FAIL_SCL", 14, QuadPlane, takeoff_failure_scalar, 0),

    // @Param: TKOFF_ARSP_LIM
    // @DisplayName: Takeoff airspeed limit
    // @Description: Airspeed limit during takeoff. If the airspeed exceeds this level the vehicle will switch to QLAND. This is useful for ensuring that you don't takeoff into excessively strong wind. If set to 0 there is no limit on airspeed during takeoff.
    // @Units: m/s
    // @Range: 0 20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TKOFF_ARSP_LIM", 15, QuadPlane, maximum_takeoff_airspeed, 0),

    // @Param: ASSIST_ALT
    // @DisplayName: Quadplane assistance altitude
    // @Description: This is the altitude below which quadplane assistance will be triggered. This acts the same way as Q_ASSIST_ANGLE and Q_ASSIST_SPEED, but triggers if the aircraft drops below the given altitude while the VTOL motors are not running. A value of zero disables this feature. The altutude is calculated as being above ground level. The height above ground is given from a Lidar used if available and RNGFND_LANDING=1. Otherwise it comes from terrain data if TERRAIN_FOLLOW=1 and comes from height above home otherwise.
    // @Units: m
    // @Range: 0 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ALT", 16, QuadPlane, assist_alt, 0),

    // @Param: TAILSIT_GSCMSK
    // @DisplayName: Tailsitter gain scaling mask
    // @Description: Bitmask of gain scaling methods to be applied: Throttle: scale gains with throttle, ATT_THR: reduce gain at high throttle/tilt, 2:Disk theory velocity calculation, requires Q_TAILSIT_DSKLD to be set, ATT_THR must not be set, 3:Altitude correction, scale with air density
    // @User: Standard
    // @Bitmask: 0:Throttle,1:ATT_THR,2:Disk Theory,3:Altitude correction
    AP_GROUPINFO("TAILSIT_GSCMSK", 17, QuadPlane, tailsitter.gain_scaling_mask, TAILSITTER_GSCL_THROTTLE),

    // @Param: TAILSIT_GSCMIN
    // @DisplayName: Minimum tailsitter gain scaling
    // @Description: Minimum gain scaling for tailsitter Q_TAILSIT_GSCMSK options
    // @Range: 0.1 1
    // @User: Standard
    AP_GROUPINFO("TAILSIT_GSCMIN", 18, QuadPlane, tailsitter.gain_scaling_min, 0.4),

    // @Param: ASSIST_DELAY
    // @DisplayName: Quadplane assistance delay
    // @Description: This is delay between the assistance thresholds being met and the assistance starting.
    // @Units: s
    // @Range: 0 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ASSIST_DELAY", 19, QuadPlane, assist_delay, 0.5),
    
    // @Param: FWD_MANTHR_MAX
    // @DisplayName: VTOL manual forward throttle max percent
    // @Description: Maximum value for manual forward throttle; used with RC option FWD_THR (209)
    // @Range: 0 100
    // @RebootRequired: False
    AP_GROUPINFO("FWD_MANTHR_MAX", 20, QuadPlane, fwd_thr_max, 0),

    // @Param: TAILSIT_DSKLD
    // @DisplayName: Tailsitter disk loading
    // @Description: This is the vehicle weight in kg divided by the total disk area of all propellers in m^2. Only used with Q_TAILSIT_GSCMSK = 4
    // @Units: kg/m/m
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("TAILSIT_DSKLD", 21, QuadPlane, tailsitter.disk_loading, 0),

    // @Param: TILT_FIX_ANGLE
    // @DisplayName: Fixed wing tiltrotor angle
    // @Description: This is the angle the motors tilt down when at maximum output for forward flight. Set this to a non-zero value to enable vectoring for roll/pitch in forward flight on tilt-vectored aircraft
    // @Units: deg
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("TILT_FIX_ANGLE", 22, QuadPlane, tilt.fixed_angle, 0),

    // @Param: TILT_FIX_GAIN
    // @DisplayName: Fixed wing tiltrotor gain
    // @Description: This is the gain for use of tilting motors in fixed wing flight for tilt vectored quadplanes
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TILT_FIX_GAIN", 23, QuadPlane, tilt.fixed_gain, 0),

    // @Param: TAILSIT_RAT_FW
    // @DisplayName: Tailsitter VTOL to forward flight transition rate
    // @Description: The pitch rate at which tailsitter aircraft will pitch down in the transition from VTOL to forward flight
    // @Units: deg/s
    // @Range: 10 500
    AP_GROUPINFO("TAILSIT_RAT_FW", 24, QuadPlane, tailsitter.transition_rate_fw, 50),

    // @Param: TAILSIT_RAT_VT
    // @DisplayName: Tailsitter forward flight to VTOL transition rate
    // @Description: The pitch rate at which tailsitter aircraft will pitch up in the transition from forward flight to VTOL
    // @Units: deg/s
    // @Range: 10 500
    AP_GROUPINFO("TAILSIT_RAT_VT", 25, QuadPlane, tailsitter.transition_rate_vtol, 50),

    AP_GROUPEND
};

/*
  defaults for all quadplanes
 */
static const struct AP_Param::defaults_table_struct defaults_table[] = {
    { "Q_A_RAT_RLL_P",    0.25 },
    { "Q_A_RAT_RLL_I",    0.25 },
    { "Q_A_RAT_RLL_FLTD", 10.0 },
    { "Q_A_RAT_RLL_SMAX", 50.0 },
    { "Q_A_RAT_PIT_P",    0.25 },
    { "Q_A_RAT_PIT_I",    0.25 },
    { "Q_A_RAT_PIT_FLTD", 10.0 },
    { "Q_A_RAT_PIT_SMAX", 50.0 },
    { "Q_A_RAT_YAW_SMAX", 50.0 },
    { "Q_M_SPOOL_TIME",   0.25 },
    { "Q_LOIT_ANG_MAX",   15.0 },
    { "Q_LOIT_ACC_MAX",   250.0 },
    { "Q_LOIT_BRK_ACCEL", 50.0 },
    { "Q_LOIT_BRK_JERK",  250 },
    { "Q_LOIT_SPEED",     500 },
    { "Q_WP_SPEED",       500 },
    { "Q_WP_ACCEL",       100 },
};

/*
  extra defaults for tailsitters
 */
static const struct AP_Param::defaults_table_struct defaults_table_tailsitter[] = {
    { "KFF_RDDRMIX",       0.02 },
    { "Q_A_RAT_PIT_FF",    0.2 },
    { "Q_A_RAT_YAW_FF",    0.2 },
    { "Q_A_RAT_YAW_I",     0.18 },
    { "Q_A_ANGLE_BOOST",   0 },
    { "LIM_PITCH_MAX",    3000 },
    { "LIM_PITCH_MIN",    -3000 },
    { "MIXING_GAIN",      1.0 },
    { "RUDD_DT_GAIN",      10 },
    { "Q_TRANSITION_MS",   2000 },
    { "Q_TRANS_DECEL",    6 },
};

/*
  conversion table for quadplane parameters
 */
const AP_Param::ConversionInfo q_conversion_table[] = {
    { Parameters::k_param_quadplane, 4044, AP_PARAM_FLOAT, "Q_P_POSZ_P" },     //  Q_PZ_P
    { Parameters::k_param_quadplane, 4045, AP_PARAM_FLOAT, "Q_P_POSXY_P"},     //  Q_PXY_P
    { Parameters::k_param_quadplane, 4046, AP_PARAM_FLOAT, "Q_P_VELXY_P"},     //  Q_VXY_P
    { Parameters::k_param_quadplane, 78,   AP_PARAM_FLOAT, "Q_P_VELXY_I"},     //  Q_VXY_I
    { Parameters::k_param_quadplane, 142,  AP_PARAM_FLOAT, "Q_P_VELXY_IMAX"},  //  Q_VXY_IMAX
    { Parameters::k_param_quadplane, 206,  AP_PARAM_FLOAT, "Q_P_VELXY_FILT"},  //  Q_VXY_FILT_HZ
    { Parameters::k_param_quadplane, 4047, AP_PARAM_FLOAT, "Q_P_VELZ_P"},      //  Q_VZ_P
    { Parameters::k_param_quadplane, 4048, AP_PARAM_FLOAT, "Q_P_ACCZ_P"},      //  Q_AZ_P
    { Parameters::k_param_quadplane, 80,   AP_PARAM_FLOAT, "Q_P_ACCZ_I"},      //  Q_AZ_I
    { Parameters::k_param_quadplane, 144,  AP_PARAM_FLOAT, "Q_P_ACCZ_D"},      //  Q_AZ_D
    { Parameters::k_param_quadplane, 336,  AP_PARAM_FLOAT, "Q_P_ACCZ_IMAX"},   //  Q_AZ_IMAX
    { Parameters::k_param_quadplane, 400,  AP_PARAM_FLOAT, "Q_P_ACCZ_FLTD"},   //  Q_AZ_FILT
    { Parameters::k_param_quadplane, 464,  AP_PARAM_FLOAT, "Q_P_ACCZ_FF"},     //  Q_AZ_FF
    { Parameters::k_param_quadplane, 276,  AP_PARAM_FLOAT, "Q_LOIT_SPEED"},    //  Q_WP_LOIT_SPEED
    { Parameters::k_param_quadplane, 468,  AP_PARAM_FLOAT, "Q_LOIT_BRK_JERK" },//  Q_WP_LOIT_JERK
    { Parameters::k_param_quadplane, 532,  AP_PARAM_FLOAT, "Q_LOIT_ACC_MAX" }, //  Q_WP_LOIT_MAXA
    { Parameters::k_param_quadplane, 596,  AP_PARAM_FLOAT, "Q_LOIT_BRK_ACCEL" },// Q_WP_LOIT_MINA
    { Parameters::k_param_q_attitude_control, 385,  AP_PARAM_FLOAT, "Q_A_RAT_RLL_FLTD" },// Q_A_RAT_RLL_FILT
    { Parameters::k_param_q_attitude_control, 386,  AP_PARAM_FLOAT, "Q_A_RAT_PIT_FLTD" },// Q_A_RAT_PIT_FILT
    { Parameters::k_param_q_attitude_control, 387,  AP_PARAM_FLOAT, "Q_A_RAT_YAW_FLTE" },// Q_A_RAT_YAW_FILT
    { Parameters::k_param_q_attitude_control, 449,  AP_PARAM_FLOAT, "Q_A_RAT_RLL_FF" },  // Q_A_RAT_RLL_FF
    { Parameters::k_param_q_attitude_control, 450,  AP_PARAM_FLOAT, "Q_A_RAT_PIT_FF" },  // Q_A_RAT_PIT_FF
    { Parameters::k_param_q_attitude_control, 451,  AP_PARAM_FLOAT, "Q_A_RAT_YAW_FF" },  // Q_A_RAT_YAW_FILT
};


QuadPlane::QuadPlane(AP_AHRS_NavEKF &_ahrs) :
    ahrs(_ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);

    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one Quadplane");
    }
    _singleton = this;
}


// setup default motors for the frame class
void QuadPlane::setup_default_channels(uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        SRV_Channels::set_aux_channel_default(SRV_Channels::get_motor_function(i), CH_5+i);
    }
}
    

bool QuadPlane::setup(void)
{
    if (initialised) {
        return true;
    }
    if (!enable || hal.util->get_soft_armed()) {
        return false;
    }
    float loop_delta_t = 1.0 / plane.scheduler.get_loop_rate_hz();

    enum Rotation rotation = ROTATION_NONE;

    /*
      cope with upgrade from old AP_Motors values for frame_class
     */
    AP_Int8 old_class;
    const AP_Param::ConversionInfo cinfo { Parameters::k_param_quadplane, FRAME_CLASS_OLD_IDX, AP_PARAM_INT8, nullptr };
    if (AP_Param::find_old_parameter(&cinfo, &old_class) && !frame_class.load()) {
        uint8_t new_value = 0;
        // map from old values to new values
        switch (old_class.get()) {
        case 0:
            new_value = AP_Motors::MOTOR_FRAME_QUAD;
            break;
        case 1:
            new_value = AP_Motors::MOTOR_FRAME_HEXA;
            break;
        case 2:
            new_value = AP_Motors::MOTOR_FRAME_OCTA;
            break;
        case 3:
            new_value = AP_Motors::MOTOR_FRAME_OCTAQUAD;
            break;
        case 4:
            new_value = AP_Motors::MOTOR_FRAME_Y6;
            break;
        }
        frame_class.set_and_save(new_value);
    }
    
    if (hal.util->available_memory() <
        4096 + sizeof(*motors) + sizeof(*attitude_control) + sizeof(*pos_control) + sizeof(*wp_nav) + sizeof(*ahrs_view) + sizeof(*loiter_nav)) {
        AP_BoardConfig::config_error("Not enough memory for quadplane");
    }

    /*
      dynamically allocate the key objects for quadplane. This ensures
      that the objects don't affect the vehicle unless enabled and
      also saves memory when not in use
     */
    switch ((AP_Motors::motor_frame_class)frame_class) {
    case AP_Motors::MOTOR_FRAME_QUAD:
        setup_default_channels(4);
        break;
    case AP_Motors::MOTOR_FRAME_HEXA:
        setup_default_channels(6);
        break;
    case AP_Motors::MOTOR_FRAME_OCTA:
    case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        setup_default_channels(8);
        break;
    case AP_Motors::MOTOR_FRAME_Y6:
        setup_default_channels(7);
        break;
    case AP_Motors::MOTOR_FRAME_TRI:
        SRV_Channels::set_default_function(CH_5, SRV_Channel::k_motor1);
        SRV_Channels::set_default_function(CH_6, SRV_Channel::k_motor2);
        SRV_Channels::set_default_function(CH_8, SRV_Channel::k_motor4);
        SRV_Channels::set_default_function(CH_11, SRV_Channel::k_motor7);
        AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
        break;
    case AP_Motors::MOTOR_FRAME_TAILSITTER:
        break;
    default:
        AP_BoardConfig::config_error("Unsupported Q_FRAME_CLASS %u", frame_class);
    }

    if (tailsitter.motor_mask == 0) {
        // this is a normal quadplane
        switch ((AP_Motors::motor_frame_class)frame_class) {
        case AP_Motors::MOTOR_FRAME_TRI:
            motors = new AP_MotorsTri(plane.scheduler.get_loop_rate_hz(), rc_speed);
            motors_var_info = AP_MotorsTri::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            // this is a duo-motor tailsitter (vectored thrust if tilt.tilt_mask != 0)
            motors = new AP_MotorsTailsitter(plane.scheduler.get_loop_rate_hz(), rc_speed);
            motors_var_info = AP_MotorsTailsitter::var_info;
            if (tilt.tilt_type != TILT_TYPE_BICOPTER) {
                rotation = ROTATION_PITCH_90;
            }
            break;
        default:
            motors = new AP_MotorsMatrix(plane.scheduler.get_loop_rate_hz(), rc_speed);
            motors_var_info = AP_MotorsMatrix::var_info;
            break;
        }
    } else {
        // this is a copter tailsitter with motor layout specified by frame_class and frame_type
        // tilting motors are not supported (tiltrotor control variables are ignored)
        if (tilt.tilt_mask != 0) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Warning: Motor tilt not supported\n");
        }
        rotation = ROTATION_PITCH_90;
        motors = new AP_MotorsMatrix(plane.scheduler.get_loop_rate_hz(), rc_speed);
        motors_var_info = AP_MotorsMatrix::var_info;
    }

    if (!motors) {
        AP_BoardConfig::config_error("Unable to allocate %s", "motors");
    }

    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    // create the attitude view used by the VTOL code
    ahrs_view = ahrs.create_view(rotation, ahrs_trim_pitch);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::config_error("Unable to allocate %s", "ahrs_view");
    }

    attitude_control = new AC_AttitudeControl_TS(*ahrs_view, aparm, *motors, loop_delta_t);
    if (!attitude_control) {
        AP_BoardConfig::config_error("Unable to allocate %s", "attitude_control");
    }

    AP_Param::load_object_from_eeprom(attitude_control, attitude_control->var_info);
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control, loop_delta_t);
    if (!pos_control) {
        AP_BoardConfig::config_error("Unable to allocate %s", "pos_control");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);
    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (!wp_nav) {
        AP_BoardConfig::config_error("Unable to allocate %s", "wp_nav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);
#if AP_TERRAIN_AVAILABLE
    wp_nav->set_terrain(&plane.terrain);
#endif

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (!loiter_nav) {
        AP_BoardConfig::config_error("Unable to allocate %s", "loiter_nav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

    motors->init(frame_class, frame_type);

    tilt.is_vectored = tilt.tilt_mask != 0 && tilt.tilt_type == TILT_TYPE_VECTORED_YAW;

    if (motors_var_info == AP_MotorsMatrix::var_info && tilt.is_vectored) {
        // we will be using vectoring for yaw
        motors->disable_yaw_torque();
    }

    motors->set_throttle_range(thr_min_pwm, thr_max_pwm);
    motors->set_update_rate(rc_speed);
    motors->set_interlock(true);
    attitude_control->parameter_sanity_check();
    wp_nav->wp_and_spline_init();

    // TODO: update this if servo function assignments change
    // used by relax_attitude_control() to control special behavior for vectored tailsitters
    _is_vectored = (frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) &&
                   (!is_zero(tailsitter.vectored_hover_gain) &&
                    (SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft) ||
                     SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight)));

    // setup the trim of any motors used by AP_Motors so I/O board
    // failsafe will disable motors
    for (uint8_t i=0; i<8; i++) {
        SRV_Channel::Aux_servo_function_t func = SRV_Channels::get_motor_function(i);
        SRV_Channels::set_failsafe_pwm(func, thr_min_pwm);
    }

    transition_state = TRANSITION_DONE;

    if (tilt.tilt_mask != 0) {
        // setup tilt compensation
        motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&QuadPlane::tilt_compensate, void, float *, uint8_t));
        if (tilt.tilt_type == TILT_TYPE_VECTORED_YAW) {
            // setup tilt servos for vectored yaw
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRear,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearLeft, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearRight, 1000);
        }
    }

    // default QAssist state as set with Q_OPTIONS
    if ((options & OPTION_Q_ASSIST_FORCE_ENABLE) != 0) {
        q_assist_state = Q_ASSIST_STATE_ENUM::Q_ASSIST_FORCE;
    }

    setup_defaults();

    AP_Param::convert_old_parameters(&q_conversion_table[0], ARRAY_SIZE(q_conversion_table));

    // Set tailsitter transition rate to match old caculation
    if (!tailsitter.transition_rate_fw.configured()) {
        tailsitter.transition_rate_fw.set_and_save(tailsitter.transition_angle_fw / (transition_time_ms/2000.0f));
    }

    // param count will have changed
    AP_Param::invalidate_count();

    gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane initialised, class: %s, type: %s", motors->get_frame_string(), motors->get_type_string());
    initialised = true;
    return true;
}

/*
  setup default parameters from defaults_table
 */
void QuadPlane::setup_defaults(void)
{
    AP_Param::set_defaults_from_table(defaults_table, ARRAY_SIZE(defaults_table));

    if (frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) {
        AP_Param::set_defaults_from_table(defaults_table_tailsitter, ARRAY_SIZE(defaults_table_tailsitter));
    }
    
    // reset ESC calibration
    if (esc_calibration != 0) {
        esc_calibration.set_and_save(0);
    }
    // Quadplanes need the same level of GPS error checking as Copters do, Plane is more relaxed
    AP_Param::set_default_by_name("EK2_CHECK_SCALE",100);
    AP_Param::set_default_by_name("EK3_CHECK_SCALE",100);

}

// run ESC calibration
void QuadPlane::run_esc_calibration(void)
{
    if (!motors->armed()) {
        motors->set_throttle_passthrough_for_esc_calibration(0);
        AP_Notify::flags.esc_calibration = false;
        return;
    }
    if (!AP_Notify::flags.esc_calibration) {
        gcs().send_text(MAV_SEVERITY_INFO, "Starting ESC calibration");
    }
    AP_Notify::flags.esc_calibration = true;
    switch (esc_calibration) {
    case 1:
        // throttle based calibration
        motors->set_throttle_passthrough_for_esc_calibration(plane.get_throttle_input() * 0.01f);
        break;
    case 2:
        // full range calibration
        motors->set_throttle_passthrough_for_esc_calibration(1);
        break;
    }
}


// init quadplane stabilize mode 
void QuadPlane::init_stabilize(void)
{
    throttle_wait = false;
}

/*
  when doing a forward transition of a tilt-vectored quadplane we use
  euler angle control to maintain good yaw. This updates the yaw
  target based on pilot input and target roll
 */
void QuadPlane::update_yaw_target(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - tilt.transition_yaw_set_ms > 100 ||
        !is_zero(get_pilot_input_yaw_rate_cds())) {
        // lock initial yaw when transition is started or when
        // pilot commands a yaw change. This allows us to track
        // straight in transitions for tilt-vectored planes, but
        // allows for turns when level transition is not wanted
        tilt.transition_yaw_cd = ahrs.yaw_sensor;
    }

    /*
      now calculate the equivalent yaw rate for a coordinated turn for
      the desired bank angle given the airspeed
     */
    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(aspeed);
    if (have_airspeed && labs(plane.nav_roll_cd)>1000) {
        float dt = (now - tilt.transition_yaw_set_ms) * 0.001;
        // calculate the yaw rate to achieve the desired turn rate
        const float airspeed_min = MAX(plane.aparm.airspeed_min,5);
        const float yaw_rate_cds = fixedwing_turn_rate(plane.nav_roll_cd*0.01, MAX(aspeed,airspeed_min))*100;
        tilt.transition_yaw_cd += yaw_rate_cds * dt;
    }
    tilt.transition_yaw_set_ms = now;
}

/*
  ask the multicopter attitude control to match the roll and pitch rates being demanded by the
  fixed wing controller if not in a pure VTOL mode
 */
void QuadPlane::multicopter_attitude_rate_update(float yaw_rate_cds)
{
    check_attitude_relax();

    bool use_multicopter_control = in_vtol_mode() && !in_tailsitter_vtol_transition();
    bool use_multicopter_eulers = false;

    if (!use_multicopter_control &&
        tilt.is_vectored &&
        transition_state <= TRANSITION_TIMER) {
        update_yaw_target();
        use_multicopter_control = true;
        use_multicopter_eulers = true;
    }

    // normal control modes for VTOL and FW flight
    // tailsitter in transition to VTOL flight is not really in a VTOL mode yet
    if (use_multicopter_control) {

        // tailsitter-only body-frame roll control options
        // Angle mode attitude control for pitch and body-frame roll, rate control for euler yaw.
        if (is_tailsitter() &&
            (tailsitter.input_type & TAILSITTER_INPUT_BF_ROLL)) {

            if (!(tailsitter.input_type & TAILSITTER_INPUT_PLANE)) {
                // In multicopter input mode, the roll and yaw stick axes are independent of pitch
                attitude_control->input_euler_rate_yaw_euler_angle_pitch_bf_roll(false,
                                                                                plane.nav_roll_cd,
                                                                                plane.nav_pitch_cd,
                                                                                yaw_rate_cds);
                return;
            } else {
                // In plane input mode, the roll and yaw sticks are swapped
                // and their effective axes rotate from yaw to roll and vice versa
                // as pitch goes from zero to 90.
                // So it is necessary to also rotate their scaling.

                // Get the roll angle and yaw rate limits
                int16_t roll_limit = aparm.angle_max;
                // separate limit for tailsitter roll, if set
                if (tailsitter.max_roll_angle > 0) {
                    roll_limit = tailsitter.max_roll_angle * 100.0f;
                }
                // Prevent a divide by zero
                float yaw_rate_limit = ((yaw_rate_max < 1.0f) ? 1 : yaw_rate_max) * 100.0f;
                float yaw2roll_scale = roll_limit / yaw_rate_limit;

                // Rotate as a function of Euler pitch and swap roll/yaw
                float euler_pitch = radians(.01f * plane.nav_pitch_cd);
                float spitch = fabsf(sinf(euler_pitch));
                float y2r_scale = linear_interpolate(1, yaw2roll_scale, spitch, 0, 1);

                float p_yaw_rate = plane.nav_roll_cd / y2r_scale;
                float p_roll_angle = -y2r_scale * yaw_rate_cds;

                attitude_control->input_euler_rate_yaw_euler_angle_pitch_bf_roll(true,
                                                                                p_roll_angle,
                                                                                plane.nav_pitch_cd,
                                                                                p_yaw_rate);
                return;
            }
        }

        if (use_multicopter_eulers) {
            attitude_control->input_euler_angle_roll_pitch_yaw(plane.nav_roll_cd,
                                                               plane.nav_pitch_cd,
                                                               tilt.transition_yaw_cd,
                                                               true);
        } else {
            // use euler angle attitude control
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                          plane.nav_pitch_cd,
                                                                          yaw_rate_cds);
        }
    } else {
        // use the fixed wing desired rates
        float roll_rate = plane.rollController.get_pid_info().target;
        float pitch_rate = plane.pitchController.get_pid_info().target;
        if (is_tailsitter()) {
            // tailsitter roll and yaw swapped due to change in reference frame
            attitude_control->input_rate_bf_roll_pitch_yaw_2(yaw_rate_cds,pitch_rate*100.0f, -roll_rate*100.0f);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw_2(roll_rate*100.0f, pitch_rate*100.0f, yaw_rate_cds);
        }
    }
}

// hold in stabilize with given throttle
void QuadPlane::hold_stabilize(float throttle_in)
{    
    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds());

    if ((throttle_in <= 0) && (air_mode == AirMode::OFF)) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        relax_attitude_control();
    } else {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        bool should_boost = true;
        if (is_tailsitter() && assisted_flight) {
            // tailsitters in forward flight should not use angle boost
            should_boost = false;
        }
        attitude_control->set_throttle_out(throttle_in, should_boost, 0);
    }
}

// quadplane stabilize mode
void QuadPlane::control_stabilize(void)
{
    // special check for ESC calibration in QSTABILIZE
    if (esc_calibration != 0) {
        run_esc_calibration();
        return;
    }

    // normal QSTABILIZE mode
    float pilot_throttle_scaled = get_pilot_throttle();
    hold_stabilize(pilot_throttle_scaled);
}

// run the multicopter Z controller
void QuadPlane::run_z_controller(void)
{
    const uint32_t now = AP_HAL::millis();
    if (!pos_control->is_active_z()) {
        // set vertical speed and acceleration limits
        pos_control->set_max_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);

        // initialise the vertical position controller
        if (!is_tailsitter()) {
            pos_control->init_z_controller();
        } else {
            // initialise the vertical position controller with no descent
            pos_control->init_z_controller_no_descent();
        }
    }
    last_pidz_active_ms = now;
    pos_control->update_z_controller();
}

void QuadPlane::relax_attitude_control()
{
    // disable roll and yaw control for vectored tailsitters
    // if not a vectored tailsitter completely disable attitude control
    attitude_control->relax_attitude_controllers(_is_vectored);
}

/*
  check if we should relax the attitude controllers

  We relax them whenever we will be using them after a period of
  inactivity
 */
void QuadPlane::check_attitude_relax(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_att_control_ms > 100) {
        relax_attitude_control();
    }
    last_att_control_ms = now;
}

/*
  init QACRO mode
 */
void QuadPlane::init_qacro(void)
{
    throttle_wait = false;
    transition_state = TRANSITION_DONE;
    attitude_control->relax_attitude_controllers();
}

// init quadplane hover mode 
void QuadPlane::init_hover(void)
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);
    set_climb_rate_cms(0, false);

    init_throttle_wait();
}

/*
  check for an EKF yaw reset
 */
void QuadPlane::check_yaw_reset(void)
{
    if (!initialised) {
        return;
    }
    float yaw_angle_change_rad = 0.0f;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control->inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "EKF yaw reset %.2f", (double)degrees(yaw_angle_change_rad));
    }
}

void QuadPlane::set_climb_rate_cms(float target_climb_rate_cms, bool force_descend)
{
    pos_control->input_vel_accel_z(target_climb_rate_cms, 0, force_descend);
}

/*
  hold hover with target climb rate
 */
void QuadPlane::hold_hover(float target_climb_rate_cms)
{
    // motors use full range
    set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);

    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds());

    // call position controller
    set_climb_rate_cms(target_climb_rate_cms, false);

    run_z_controller();
}

float QuadPlane::get_pilot_throttle()
{
    // get scaled throttle input
    float throttle_in = plane.channel_throttle->get_control_in();

    // normalize to [0,1]
    throttle_in /= plane.channel_throttle->get_range();

    if (is_positive(throttle_expo)) {
        // get hover throttle level [0,1]
        float thr_mid = motors->get_throttle_hover();
        float thrust_curve_expo = constrain_float(throttle_expo, 0.0f, 1.0f);

        // this puts mid stick at hover throttle
        return throttle_curve(thr_mid, thrust_curve_expo, throttle_in);;
    } else {
        return throttle_in;
    }
}

/*
  get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle.
  The angle_max_cd and angle_limit_cd are mode dependent
*/
void QuadPlane::get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const
{
    // failsafe check
    if (plane.failsafe.rc_failsafe || plane.failsafe.throttle_counter > 0) {
        roll_out_cd = 0;
        pitch_out_cd = 0;
        return;
    }

    // fetch roll and pitch inputs
    roll_out_cd = plane.channel_roll->get_control_in();
    pitch_out_cd = plane.channel_pitch->get_control_in();

    // limit max lean angle, always allow for 10 degrees
    angle_limit_cd = constrain_float(angle_limit_cd, 1000.0f, angle_max_cd);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max_cd/4500.0;
    roll_out_cd *= scaler;
    pitch_out_cd *= scaler;

    // apply circular limit
    float total_in = norm(pitch_out_cd, roll_out_cd);
    if (total_in > angle_limit_cd) {
        float ratio = angle_limit_cd / total_in;
        roll_out_cd *= ratio;
        pitch_out_cd *= ratio;
    }

    // apply lateral tilt to euler roll conversion
    roll_out_cd = 100 * degrees(atanf(cosf(radians(pitch_out_cd*0.01))*tanf(radians(roll_out_cd*0.01))));
}

/*
  get pilot throttle in for landing code. Return value on scale of 0 to 1
*/
float QuadPlane::get_pilot_land_throttle(void) const
{
    if (plane.rc_failsafe_active()) {
        // assume zero throttle if lost RC
        return 0;
    }
    // get scaled throttle input
    float throttle_in = plane.channel_throttle->get_control_in();

    // normalize to [0,1]
    throttle_in /= plane.channel_throttle->get_range();

    return constrain_float(throttle_in, 0, 1);
}

/*
  control QACRO mode
 */
void QuadPlane::control_qacro(void)
{
    if (throttle_wait) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        relax_attitude_control();
    } else {
        check_attitude_relax();

        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // convert the input to the desired body frame rate
        float target_roll = 0;
        float target_pitch = plane.channel_pitch->norm_input() * acro_pitch_rate * 100.0f;
        float target_yaw = 0;
        if (is_tailsitter()) {
            // Note that the 90 degree Y rotation for copter mode swaps body-frame roll and yaw
            target_roll =  plane.channel_rudder->norm_input() * acro_yaw_rate * 100.0f;
            target_yaw  = -plane.channel_roll->norm_input() * acro_roll_rate * 100.0f;
        } else {
            target_roll = plane.channel_roll->norm_input() * acro_roll_rate * 100.0f;
            target_yaw  = plane.channel_rudder->norm_input() * acro_yaw_rate * 100.0;
        }

        float throttle_out = get_pilot_throttle();

        // run attitude controller
        if (plane.g.acro_locking) {
            attitude_control->input_rate_bf_roll_pitch_yaw_3(target_roll, target_pitch, target_yaw);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
        }

        // output pilot's throttle without angle boost
        attitude_control->set_throttle_out(throttle_out, false, 10.0f);
    }
}

/*
  control QHOVER mode
 */
void QuadPlane::control_hover(void)
{
    if (throttle_wait) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        relax_attitude_control();
        pos_control->relax_z_controller(0);
    } else {
        hold_hover(get_pilot_desired_climb_rate_cms());
    }
}

void QuadPlane::init_loiter(void)
{
    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);

    init_throttle_wait();

    // remember initial pitch
    loiter_initial_pitch_cd = MAX(plane.ahrs.pitch_sensor, 0);

    // prevent re-init of target position
    last_loiter_ms = AP_HAL::millis();
}

void QuadPlane::init_qland(void)
{
    init_loiter();
    throttle_wait = false;
    setup_target_position();
    poscontrol.set_state(QPOS_LAND_DESCEND);
    poscontrol.pilot_correction_done = false;
    last_land_final_agl = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    landing_detect.lower_limit_start_ms = 0;
    landing_detect.land_start_ms = 0;
#if LANDING_GEAR_ENABLED == ENABLED
    plane.g2.landing_gear.deploy_for_landing();
#endif
#if AC_FENCE == ENABLED
    plane.fence.auto_disable_fence_for_landing();
#endif
}


// helper for is_flying()
bool QuadPlane::is_flying(void)
{
    if (!available()) {
        return false;
    }
    if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        return true;
    }
    if (motors->get_throttle() > 0.01f && !motors->limit.throttle_lower) {
        return true;
    }
    if (in_tailsitter_vtol_transition()) {
        return true;
    }
    return false;
}

// crude landing detector to prevent tipover
bool QuadPlane::should_relax(void)
{
    const uint32_t tnow = millis();

    bool motor_at_lower_limit = motors->limit.throttle_lower && attitude_control->is_throttle_mix_min();
    if (motors->get_throttle() < 0.01f) {
        motor_at_lower_limit = true;
    }

    if (!motor_at_lower_limit) {
        landing_detect.lower_limit_start_ms = 0;
        landing_detect.land_start_ms = 0;
        return false;
    } else if (landing_detect.lower_limit_start_ms == 0) {
        landing_detect.lower_limit_start_ms = tnow;
    }

    return (tnow - landing_detect.lower_limit_start_ms) > 1000;
}

// see if we are flying in vtol
bool QuadPlane::is_flying_vtol(void) const
{
    if (!available()) {
        return false;
    }
    if (motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN) {
        // assume that with no motor outputs we're not flying in VTOL mode
        return false;
    }
    if (motors->get_throttle() > 0.01f) {
        // if we are demanding more than 1% throttle then don't consider aircraft landed
        return true;
    }
    if (plane.control_mode->is_vtol_man_throttle() && air_mode == AirMode::ON) {
        // in manual throttle modes with airmode on, don't consider aircraft landed
        return true;
    }
    if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        return true;
    }
    if (plane.control_mode->is_vtol_man_mode()) {
        // in manual flight modes only consider aircraft landed when pilot demanded throttle is zero
        return plane.get_throttle_input() > 0;
    }
    if (in_vtol_mode() && millis() - landing_detect.lower_limit_start_ms > 5000) {
        // use landing detector
        return true;
    }
    return false;
}

/*
  smooth out descent rate for landing to prevent a jerk as we get to
  land_final_alt. 
 */
float QuadPlane::landing_descent_rate_cms(float height_above_ground) const
{
    if (poscontrol.get_state() == QPOS_LAND_FINAL) {
        // when in final use descent rate for final even if alt has climbed again
        height_above_ground = MIN(height_above_ground, land_final_alt);
    }
    const float max_climb_speed = wp_nav->get_default_speed_up();
    float ret = linear_interpolate(land_speed_cms, wp_nav->get_default_speed_down(),
                                   height_above_ground,
                                   land_final_alt, land_final_alt+6);

    if ((options & OPTION_THR_LANDING_CONTROL) != 0) {
        // allow throttle control for landing speed
        const float thr_in = get_pilot_land_throttle();
        const float dz = 0.1;
        const float thresh1 = 0.5+dz;
        const float thresh2 = 0.5-dz;
        const float scaling = 1.0 / (0.5 - dz);
        if (thr_in > thresh1) {
            // start climbing
            ret = -(thr_in - thresh1)*scaling*max_climb_speed;
        } else if (thr_in > thresh2) {
            // hold height
            ret = 0;
        } else {
            ret *= (thresh2 - thr_in)*scaling;
        }
    }

    if (poscontrol.pilot_correction_active) {
        // stop descent when repositioning
        ret = MIN(0, ret);
    }

    return ret;
}


// run quadplane loiter controller
void QuadPlane::control_loiter()
{
    if (throttle_wait) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0, true, 0);
        relax_attitude_control();
        pos_control->relax_z_controller(0);
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        return;
    }
    if (!motors->armed()) {
        init_loiter();
    }

    check_attitude_relax();

    if (should_relax()) {
        loiter_nav->soften_for_landing();
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_loiter_ms > 500) {
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
    }
    last_loiter_ms = now;

    // motors use full range
    set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);

    // process pilot's roll and pitch input
    float target_roll_cd, target_pitch_cd;
    get_pilot_desired_lean_angles(target_roll_cd, target_pitch_cd, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());
    loiter_nav->set_pilot_desired_acceleration(target_roll_cd, target_pitch_cd);
    
    // run loiter controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }
    loiter_nav->update();

    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = loiter_nav->get_roll();
    plane.nav_pitch_cd = loiter_nav->get_pitch();

    if (now - last_pidz_init_ms < (uint32_t)transition_time_ms*2 && !is_tailsitter()) {
        // we limit pitch during initial transition
        float pitch_limit_cd = linear_interpolate(loiter_initial_pitch_cd, aparm.angle_max,
                                                  now,
                                                  last_pidz_init_ms, last_pidz_init_ms+transition_time_ms*2);
        if (plane.nav_pitch_cd > pitch_limit_cd) {
            plane.nav_pitch_cd = pitch_limit_cd;
            pos_control->set_externally_limited_xy();
        }
    }
    
    
    // call attitude controller with conservative smoothing gain of 4.0f
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  get_desired_yaw_rate_cds());

    if (plane.control_mode == &plane.mode_qland) {
        if (poscontrol.get_state() < QPOS_LAND_FINAL && check_land_final()) {
            poscontrol.set_state(QPOS_LAND_FINAL);
            setup_target_position();
            // cut IC engine if enabled
            if (land_icengine_cut != 0) {
                plane.g2.ice_control.engine_control(0, 0, 0);
            }
        }
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        float descent_rate_cms = landing_descent_rate_cms(height_above_ground);

        if (poscontrol.get_state() == QPOS_LAND_FINAL && (options & OPTION_DISABLE_GROUND_EFFECT_COMP) == 0) {
            ahrs.set_touchdown_expected(true);
        }

        set_climb_rate_cms(-descent_rate_cms, descent_rate_cms>0);
        check_land_complete();
    } else if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        set_climb_rate_cms(0, false);
    } else {
        // update altitude target and call position controller
        set_climb_rate_cms(get_pilot_desired_climb_rate_cms(), false);
    }
    run_z_controller();
}

/*
  get pilot input yaw rate in cd/s
 */
float QuadPlane::get_pilot_input_yaw_rate_cds(void) const
{
    bool manual_air_mode = plane.control_mode->is_vtol_man_throttle() && (air_mode == AirMode::ON);
    if (!manual_air_mode &&
        plane.get_throttle_input() <= 0 && !plane.control_mode->does_auto_throttle() &&
        plane.arming.get_rudder_arming_type() != AP_Arming::RudderArming::IS_DISABLED) {
        // the user may be trying to disarm
        return 0;
    }

    if ((plane.g.stick_mixing == STICK_MIXING_DISABLED) &&
        (plane.control_mode == &plane.mode_qrtl ||
         plane.control_mode->is_guided_mode() ||
         in_vtol_auto())) {
        return 0;
    }

    // add in rudder input
    float max_rate = yaw_rate_max;
    if (is_tailsitter() &&
        tailsitter.input_type & TAILSITTER_INPUT_BF_ROLL) {
        // must have a non-zero max yaw rate for scaling to work
        max_rate = (yaw_rate_max < 1.0f) ? 1 : yaw_rate_max;
    }
    return plane.channel_rudder->get_control_in() * max_rate / 45;
}

/*
  get overall desired yaw rate in cd/s
 */
float QuadPlane::get_desired_yaw_rate_cds(void)
{
    float yaw_cds = 0;
    if (assisted_flight) {
        // use bank angle to get desired yaw rate
        yaw_cds += desired_auto_yaw_rate_cds();
    }
    bool manual_air_mode = plane.control_mode->is_vtol_man_throttle() && (air_mode == AirMode::ON);
    if (plane.get_throttle_input() <= 0 && !plane.control_mode->does_auto_throttle() && !manual_air_mode) {
        // the user may be trying to disarm
        return 0;
    }
    // add in pilot input
    yaw_cds += get_pilot_input_yaw_rate_cds();

    // add in weathervaning
    yaw_cds += get_weathervane_yaw_rate_cds();
    
    return yaw_cds;
}

// get pilot desired climb rate in cm/s
float QuadPlane::get_pilot_desired_climb_rate_cms(void) const
{
    if (plane.failsafe.rc_failsafe || plane.failsafe.throttle_counter > 0) {
        // descend at 0.5m/s for now
        return -50;
    }
    uint16_t dead_zone = plane.channel_throttle->get_dead_zone();
    uint16_t trim = (plane.channel_throttle->get_radio_max() + plane.channel_throttle->get_radio_min())/2;
    const float throttle_request = plane.channel_throttle->pwm_to_angle_dz_trim(dead_zone, trim) *0.01f;
    return throttle_request * (throttle_request > 0.0f ? pilot_velocity_z_max_up : get_pilot_velocity_z_max_dn());
}


/*
  initialise throttle_wait based on throttle and is_flying()
 */
void QuadPlane::init_throttle_wait(void)
{
    if (plane.get_throttle_input() >= 10 ||
        plane.is_flying()) {
        throttle_wait = false;
    } else {
        throttle_wait = true;        
    }
}
    
// set motor arming
void QuadPlane::set_armed(bool armed)
{
    if (!initialised) {
        return;
    }
    motors->armed(armed);
}


/*
  estimate desired climb rate for assistance (in cm/s)
 */
float QuadPlane::assist_climb_rate_cms(void) const
{
    float climb_rate;
    if (plane.control_mode->does_auto_throttle()) {
        // use altitude_error_cm, spread over 10s interval
        climb_rate = plane.altitude_error_cm * 0.1f;
    } else {
        // otherwise estimate from pilot input
        climb_rate = plane.g.flybywire_climb_rate * (plane.nav_pitch_cd/(float)plane.aparm.pitch_limit_max_cd);
        climb_rate *= plane.get_throttle_input();
    }
    climb_rate = constrain_float(climb_rate, -wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up());

    // bring in the demanded climb rate over 2 seconds
    const uint32_t ramp_up_time_ms = 2000;
    const uint32_t dt_since_start = last_pidz_active_ms - last_pidz_init_ms;
    if (dt_since_start < ramp_up_time_ms) {
        climb_rate = linear_interpolate(0, climb_rate, dt_since_start, 0, ramp_up_time_ms);
    }
    
    return climb_rate;
}

/*
  calculate desired yaw rate for assistance
 */
float QuadPlane::desired_auto_yaw_rate_cds(void) const
{
    float aspeed;
    if (!ahrs.airspeed_estimate(aspeed) || aspeed < plane.aparm.airspeed_min) {
        aspeed = plane.aparm.airspeed_min;
    }
    if (aspeed < 1) {
        aspeed = 1;
    }
    float yaw_rate = degrees(GRAVITY_MSS * tanf(radians(plane.nav_roll_cd*0.01f))/aspeed) * 100;
    return yaw_rate;
}

/*
  return true if the quadplane should provide stability assistance
 */
bool QuadPlane::assistance_needed(float aspeed, bool have_airspeed)
{
    if (assist_speed <= 0 || is_control_surface_tailsitter()) {
        // assistance disabled
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }

    // assistance due to Q_ASSIST_SPEED
    // if option bit is enabled only allow assist with real airspeed sensor
    if ((have_airspeed && aspeed < assist_speed) && 
       (((options & OPTION_DISABLE_SYNTHETIC_AIRSPEED_ASSIST) == 0) || ahrs.airspeed_sensor_enabled())) {
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return true;
    }

    const uint32_t now = AP_HAL::millis();

    /*
      optional assistance when altitude is too close to the ground
     */
    if (assist_alt > 0) {
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        if (height_above_ground < assist_alt) {
            if (alt_error_start_ms == 0) {
                alt_error_start_ms = now;
            }
            if (now - alt_error_start_ms > assist_delay*1000) {
                // we've been below assistant alt for Q_ASSIST_DELAY seconds
                if (!in_alt_assist) {
                    in_alt_assist = true;
                    gcs().send_text(MAV_SEVERITY_INFO, "Alt assist %.1fm", height_above_ground);
                }
                return true;
            }
        } else {
            in_alt_assist = false;
            alt_error_start_ms = 0;
        }
    }

    if (assist_angle <= 0) {
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }

    /*
      now check if we should provide assistance due to attitude error
     */

    const uint16_t allowed_envelope_error_cd = 500U;
    if (labs(ahrs.roll_sensor) <= plane.aparm.roll_limit_cd+allowed_envelope_error_cd &&
        ahrs.pitch_sensor < plane.aparm.pitch_limit_max_cd+allowed_envelope_error_cd &&
        ahrs.pitch_sensor > -(allowed_envelope_error_cd-plane.aparm.pitch_limit_min_cd)) {
        // we are inside allowed attitude envelope
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }
    
    int32_t max_angle_cd = 100U*assist_angle;
    if ((labs(ahrs.roll_sensor - plane.nav_roll_cd) < max_angle_cd &&
         labs(ahrs.pitch_sensor - plane.nav_pitch_cd) < max_angle_cd)) {
        // not beyond angle error
        angle_error_start_ms = 0;
        in_angle_assist = false;
        return false;
    }

    if (angle_error_start_ms == 0) {
        angle_error_start_ms = now;
    }
    bool ret = (now - angle_error_start_ms) >= assist_delay*1000;
    if (ret && !in_angle_assist) {
        in_angle_assist = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Angle assist r=%d p=%d",
                                         (int)(ahrs.roll_sensor/100),
                                         (int)(ahrs.pitch_sensor/100));
    }
    return ret;
}

// return true if it is safe to provide assistance
bool QuadPlane::assistance_safe()
{
    return hal.util->get_soft_armed() && ( (plane.control_mode->does_auto_throttle() && !plane.throttle_suppressed)
                                                                      || plane.get_throttle_input()>0 
                                                                      || plane.is_flying() );
}

/*
  update for transition from quadplane to fixed wing mode
 */
void QuadPlane::update_transition(void)
{
    if (plane.control_mode == &plane.mode_manual ||
        plane.control_mode == &plane.mode_acro ||
        plane.control_mode == &plane.mode_training) {
        // in manual modes quad motors are always off
        if (!tilt.motors_active && !is_tailsitter()) {
            set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
        }
        transition_state = TRANSITION_DONE;
        transition_start_ms = 0;
        transition_low_airspeed_ms = 0;
        assisted_flight = false;
        return;
    }

    const uint32_t now = millis();

    if (!hal.util->get_soft_armed()) {
        // reset the failure timer if we haven't started transitioning
        transition_start_ms = now;
    } else if ((transition_state != TRANSITION_DONE) &&
        (transition_start_ms != 0) &&
        (transition_failure > 0) &&
        ((now - transition_start_ms) > ((uint32_t)transition_failure * 1000))) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Transition failed, exceeded time limit");
        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TRANSITION);
    }

    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(aspeed);

    // tailsitters use angle wait, not airspeed wait
    if (is_tailsitter() && transition_state == TRANSITION_AIRSPEED_WAIT) {
        transition_state = TRANSITION_ANGLE_WAIT_FW;
    }

    /*
      see if we should provide some assistance
     */
    if (assistance_safe() && (q_assist_state == Q_ASSIST_STATE_ENUM::Q_ASSIST_FORCE ||
        (q_assist_state == Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED && assistance_needed(aspeed, have_airspeed)))) {
        // the quad should provide some assistance to the plane
        assisted_flight = true;
        if (!is_tailsitter()) {
            // update transition state for vehicles using airspeed wait
            if (transition_state != TRANSITION_AIRSPEED_WAIT) {
                gcs().send_text(MAV_SEVERITY_INFO, "Transition started airspeed %.1f", (double)aspeed);
            }
            transition_state = TRANSITION_AIRSPEED_WAIT;
            if (transition_start_ms == 0) {
                transition_start_ms = now;
            }
        }
    } else {
        assisted_flight = false;
    }

    if (is_tailsitter()) {
        if (transition_state == TRANSITION_ANGLE_WAIT_FW &&
            tailsitter_transition_fw_complete()) {
            transition_state = TRANSITION_DONE;
            transition_start_ms = 0;
            transition_low_airspeed_ms = 0;
        }
    }
    
    // if rotors are fully forward then we are not transitioning,
    // unless we are waiting for airspeed to increase (in which case
    // the tilt will decrease rapidly)
    if (tiltrotor_fully_fwd() && transition_state != TRANSITION_AIRSPEED_WAIT) {
        if (transition_state == TRANSITION_TIMER) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
        }
        transition_state = TRANSITION_DONE;
        transition_start_ms = 0;
        transition_low_airspeed_ms = 0;
    }
    
    if (transition_state < TRANSITION_TIMER) {
        // set a single loop pitch limit in TECS
        if (plane.ahrs.groundspeed() < 3) {
            // until we have some ground speed limit to zero pitch
            plane.TECS_controller.set_pitch_max_limit(0);
        } else {
            plane.TECS_controller.set_pitch_max_limit(transition_pitch_max);
        }
    } else if (transition_state < TRANSITION_DONE) {
        plane.TECS_controller.set_pitch_max_limit((transition_pitch_max+1)*2);
    }
    if (transition_state < TRANSITION_DONE) {
        // during transition we ask TECS to use a synthetic
        // airspeed. Otherwise the pitch limits will throw off the
        // throttle calculation which is driven by pitch
        plane.TECS_controller.use_synthetic_airspeed();
    }
    
    switch (transition_state) {
    case TRANSITION_AIRSPEED_WAIT: {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        // we hold in hover until the required airspeed is reached
        if (transition_start_ms == 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed wait");
            transition_start_ms = now;
        }

        transition_low_airspeed_ms = now;
        if (have_airspeed && aspeed > plane.aparm.airspeed_min && !assisted_flight) {
            transition_state = TRANSITION_TIMER;
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed reached %.1f", (double)aspeed);
        }
        assisted_flight = true;

        // do not allow a climb on the quad motors during transition a
        // climb would add load to the airframe, and prolongs the
        // transition. We don't limit the climb rate on tilt rotors as
        // otherwise the plane can end up in high-alpha flight with
        // low VTOL thrust and may not complete a transition
        float climb_rate_cms = assist_climb_rate_cms();
        if ((options & OPTION_LEVEL_TRANSITION) && tilt.tilt_mask == 0) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
        }
        hold_hover(climb_rate_cms);

        if (!tilt.is_vectored) {
            // set desired yaw to current yaw in both desired angle
            // and rate request. This reduces wing twist in transition
            // due to multicopter yaw demands. This is disabled when
            // using vectored yaw for tilt-rotors as the yaw control
            // is needed to maintain good control in forward
            // transitions
            attitude_control->reset_yaw_target_and_rate();
            attitude_control->rate_bf_yaw_target(ahrs.get_gyro().z);
        }

        last_throttle = motors->get_throttle();

        // reset integrators while we are below target airspeed as we
        // may build up too much while still primarily under
        // multicopter control
        plane.pitchController.reset_I();
        plane.rollController.reset_I();

        // give full authority to attitude control
        attitude_control->set_throttle_mix_max(1.0f);
        break;
    }
        
    case TRANSITION_TIMER: {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        // after airspeed is reached we degrade throttle over the
        // transition time, but continue to stabilize
        const uint32_t transition_timer_ms = now - transition_low_airspeed_ms;
        if (transition_timer_ms > (unsigned)transition_time_ms) {
            transition_state = TRANSITION_DONE;
            transition_start_ms = 0;
            transition_low_airspeed_ms = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "Transition done");
        }
        float trans_time_ms = (float)transition_time_ms.get();
        float transition_scale = (trans_time_ms - transition_timer_ms) / trans_time_ms;
        float throttle_scaled = last_throttle * transition_scale;

        // set zero throttle mix, to give full authority to
        // throttle. This ensures that the fixed wing controllers get
        // a chance to learn the right integrators during the transition
        attitude_control->set_throttle_mix_value(0.5*transition_scale);

        if (throttle_scaled < 0.01) {
            // ensure we don't drop all the way to zero or the motors
            // will stop stabilizing
            throttle_scaled = 0.01;
        }
        assisted_flight = true;
        hold_stabilize(throttle_scaled);

        // set desired yaw to current yaw in both desired angle and
        // rate request while waiting for transition to
        // complete. Navigation should be controlled by fixed wing
        // control surfaces at this stage.
        // We disable this for vectored yaw tilt rotors as they do need active
        // yaw control throughout the transition
        if (!tilt.is_vectored) {
            attitude_control->reset_yaw_target_and_rate();
            attitude_control->rate_bf_yaw_target(ahrs.get_gyro().z);
        }
        break;
    }

    case TRANSITION_ANGLE_WAIT_FW: {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        assisted_flight = true;
        uint32_t dt = now - transition_start_ms;
        // multiply by 0.1 to convert (degrees/second * milliseconds) to centi degrees
        plane.nav_pitch_cd = constrain_float(transition_initial_pitch - (tailsitter.transition_rate_fw * dt) * 0.1f * (plane.fly_inverted()?-1.0f:1.0f), -8500, 8500);
        plane.nav_roll_cd = 0;
        check_attitude_relax();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      0);
        // set throttle at either hover throttle or current throttle, whichever is higher, through the transition
        attitude_control->set_throttle_out(MAX(motors->get_throttle_hover(),attitude_control->get_throttle_in()), true, 0);
        break;
    }

    case TRANSITION_ANGLE_WAIT_VTOL:
        // nothing to do, this is handled in the fixed wing attitude controller
        return;

    case TRANSITION_DONE:
        if (!tilt.motors_active && !is_tailsitter()) {
            set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
        }
        return;
    }

    motors_output();
}

/*
  update motor output for quadplane
 */
void QuadPlane::update(void)
{
    if (!setup()) {
        return;
    }

    if ((ahrs_view != NULL) && !is_equal(_last_ahrs_trim_pitch, ahrs_trim_pitch.get())) {
        _last_ahrs_trim_pitch = ahrs_trim_pitch.get();
        ahrs_view->set_pitch_trim(_last_ahrs_trim_pitch);
    }

#if ADVANCED_FAILSAFE == ENABLED
    if (plane.afs.should_crash_vehicle() && !plane.afs.terminating_vehicle_via_landing()) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        return;
    }
#endif
    
    if (motor_test.running) {
        motor_test_output();
        return;
    }

    if (SRV_Channels::get_emergency_stop()) {
        attitude_control->reset_rate_controller_I_terms();
    }

    if (!hal.util->get_soft_armed()) {
        /*
          make sure we don't have any residual control from previous flight stages
         */
        if (is_tailsitter()) {
            // tailsitters only relax I terms, to make ground testing easier
            attitude_control->reset_rate_controller_I_terms();
        } else {
            // otherwise full relax
            attitude_control->relax_attitude_controllers();
        }
        // todo: do you want to set the throttle at this point?
        pos_control->relax_z_controller(0);
    }

    const uint32_t now = AP_HAL::millis();
    if (!in_vtol_mode()) {
        // we're in a fixed wing mode, cope with transitions and check
        // for assistance needed
        update_transition();
    } else {

        assisted_flight = false;

        // output to motors
        motors_output();

        if (now - last_vtol_mode_ms > 1000 && is_tailsitter()) {
            /*
              we are just entering a VTOL mode as a tailsitter, set
              our transition state so the fixed wing controller brings
              the nose up before we start trying to fly as a
              multicopter
             */
            transition_state = TRANSITION_ANGLE_WAIT_VTOL;
            transition_start_ms = now;
            transition_initial_pitch = constrain_float(ahrs.pitch_sensor,-8500,8500);
        } else if (is_tailsitter() &&
                   transition_state == TRANSITION_ANGLE_WAIT_VTOL) {
            float aspeed;
            bool have_airspeed = ahrs.airspeed_estimate(aspeed);
            // provide asistance in forward flight portion of tailsitter transision
            if (assistance_safe() && (q_assist_state == Q_ASSIST_STATE_ENUM::Q_ASSIST_FORCE ||
                (q_assist_state == Q_ASSIST_STATE_ENUM::Q_ASSIST_ENABLED && assistance_needed(aspeed, have_airspeed)))) {
                assisted_flight = true;
            }
            if (tailsitter_transition_vtol_complete()) {
                /*
                  we have completed transition to VTOL as a tailsitter,
                  setup for the back transition when needed
                */
                transition_state = TRANSITION_ANGLE_WAIT_FW;
                transition_start_ms = now;
            }
        } else {
            /*
              setup the transition state appropriately for next time we go into a non-VTOL mode
            */
            transition_start_ms = 0;
            transition_low_airspeed_ms = 0;
            if (throttle_wait && !plane.is_flying()) {
                transition_state = TRANSITION_DONE;
            } else if (is_tailsitter()) {
                /*
                  setup for the transition back to fixed wing for later
                 */
                transition_state = TRANSITION_ANGLE_WAIT_FW;
                transition_start_ms = now;
                transition_initial_pitch = constrain_float(ahrs_view->pitch_sensor,-8500,8500);
            } else {
                /*
                  setup for airspeed wait for later
                 */
                transition_state = TRANSITION_AIRSPEED_WAIT;
            }
            last_throttle = motors->get_throttle();
        }
            
        last_vtol_mode_ms = now;        
    }

    // disable throttle_wait when throttle rises above 10%
    if (throttle_wait &&
        (plane.get_throttle_input() > 10 ||
         plane.failsafe.rc_failsafe ||
         plane.failsafe.throttle_counter>0)) {
        throttle_wait = false;
    }

    tiltrotor_update();

    // motors logging
    if (motors->armed()) {
        const bool motors_active = in_vtol_mode() || assisted_flight;
        if (motors_active && (motors->get_spool_state() != AP_Motors::SpoolState::SHUT_DOWN)) {
            // log RATE at main loop rate
            ahrs_view->Write_Rate(*motors, *attitude_control, *pos_control);

            // log CTRL at 10 Hz
            if (now - last_ctrl_log_ms > 100) {
                last_ctrl_log_ms = now;
                attitude_control->control_monitor_log();
            }
        }
        // log QTUN at 25 Hz if motors are active, or have been active in the last quarter second
        if ((motors_active || (now - last_motors_active_ms < 250)) && (now - last_qtun_log_ms > 40)) {
            last_qtun_log_ms = now;
            Log_Write_QControl_Tuning();
        }
    }

}

/*
  see if motors should be shutdown. If they should be then change AP_Motors state to 
  AP_Motors::DesiredSpoolState::SHUT_DOWN

  This is a safety check to prevent accidental motor runs on the
  ground, such as if RC fails and QRTL is started
 */
void QuadPlane::update_throttle_suppression(void)
{
    // if the motors have been running in the last 2 seconds then
    // allow them to run now
    if (AP_HAL::millis() - last_motors_active_ms < 2000) {
        return;
    }

    // see if motors are already disabled
    if (motors->get_desired_spool_state() < AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        return;
    }

    // if the users throttle is above zero then allow motors to run
    if (plane.get_throttle_input() != 0) {
        return;
    }

    // if in a VTOL manual throttle mode and air_mode is on then allow motors to run
    if (plane.control_mode->is_vtol_man_throttle() && air_mode == AirMode::ON) {
        return;
    }

    // if we are in a fixed wing auto throttle mode and we have
    // unsuppressed the throttle then allow motors to run
    if (plane.control_mode->does_auto_throttle() && !plane.throttle_suppressed) {
        return;
    }

    // if our vertical velocity is greater than 1m/s then allow motors to run
    if (fabsf(inertial_nav.get_velocity_z()) > 100) {
        return;
    }

    // if we are more than 5m from home altitude then allow motors to run
    if (plane.relative_ground_altitude(plane.g.rangefinder_landing) > 5) {
        return;
    }

    // allow for takeoff
    if (plane.control_mode == &plane.mode_auto && is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
        return;
    }
    
    // motors should be in the spin when armed state to warn user they could become active
    set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    motors->set_throttle(0);
    last_motors_active_ms = 0;
}

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void QuadPlane::update_throttle_hover()
{
    if (!available()) {
        return;
    }
    
    // if not armed or landed exit
    if (!motors->armed() || !is_flying_vtol()) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_vel_desired_cms().z)) {
        return;
    }

    // do not update if quadplane forward motor is running (wing may be generating lift)
    // we use the THR_MIN value to account for petrol motors idling at THR_MIN
    if (!is_tailsitter() && (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > MAX(0,plane.aparm.throttle_min+10))) {
        return;
    }

    // don't update if Z controller not running
    const uint32_t now = AP_HAL::millis();
    if (now - last_pidz_active_ms > 20) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    float aspeed;
    // calc average throttle if we are in a level hover and low airspeed
    if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z()) < 60 &&
        labs(ahrs_view->roll_sensor) < 500 && labs(ahrs_view->pitch_sensor) < 500 &&
        ahrs.airspeed_estimate(aspeed) && aspeed < plane.aparm.airspeed_min*0.3) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
    }
}
/*
  output motors and do any copter needed
 */
void QuadPlane::motors_output(bool run_rate_controller)
{
    if (run_rate_controller) {
        attitude_control->rate_controller_run();
    }

    /* Delay for ARMING_DELAY_MS after arming before allowing props to spin:
       1) for safety (OPTION_DELAY_ARMING)
       2) to allow motors to return to vertical (OPTION_DISARMED_TILT)
     */
    if ((options & OPTION_DISARMED_TILT) || (options & OPTION_DELAY_ARMING)) {
        if (plane.arming.get_delay_arming()) {
            // delay motor start after arming
            set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
            return;
        }
    }

#if ADVANCED_FAILSAFE == ENABLED
    if (!hal.util->get_soft_armed() ||
        (plane.afs.should_crash_vehicle() && !plane.afs.terminating_vehicle_via_landing()) ||
         SRV_Channels::get_emergency_stop()) {
#else
    if (!hal.util->get_soft_armed() || SRV_Channels::get_emergency_stop()) {
#endif
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
        return;
    }
    if (esc_calibration && AP_Notify::flags.esc_calibration && plane.control_mode == &plane.mode_qstabilize) {
        // output is direct from run_esc_calibration()
        return;
    }

    if (in_tailsitter_vtol_transition() && !assisted_flight) {
        /*
          don't run the motor outputs while in tailsitter->vtol
          transition. That is taken care of by the fixed wing
          stabilisation code
         */
        return;
    }

    // see if motors should be shut down
    update_throttle_suppression();
    
    motors->output();

    // remember when motors were last active for throttle suppression
    if (motors->get_throttle() > 0.01f || tilt.motors_active) {
        last_motors_active_ms = AP_HAL::millis();
    }

}

/*
  update control mode for quadplane modes
 */
void QuadPlane::control_run(void)
{
    if (!initialised) {
        return;
    }

    switch (plane.control_mode->mode_number()) {
    case Mode::Number::QACRO:
        control_qacro();
        break;
    case Mode::Number::QSTABILIZE:
        control_stabilize();
        break;
    case Mode::Number::QHOVER:
        control_hover();
        break;
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
        control_loiter();
        break;
    case Mode::Number::QRTL:
        control_qrtl();
        break;
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
        qautotune.run();
        break;
#endif
    default:
        break;
    }

    // we also stabilize using fixed wing surfaces
    float speed_scaler = plane.get_speed_scaler();
    if (plane.control_mode->mode_number() == Mode::Number::QACRO) {
        plane.stabilize_acro(speed_scaler);
    } else {
        plane.stabilize_roll(speed_scaler);
        plane.stabilize_pitch(speed_scaler);
    }
}

/*
  enter a quadplane mode
 */
bool QuadPlane::init_mode(void)
{
    if (!setup()) {
        return false;
    }
    if (!initialised) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "QuadPlane mode refused");
        return false;
    }

    AP_Notify::flags.esc_calibration = false;

    switch (plane.control_mode->mode_number()) {
    case Mode::Number::QSTABILIZE:
        init_stabilize();
        break;
    case Mode::Number::QHOVER:
        init_hover();
        break;
    case Mode::Number::QLOITER:
        init_loiter();
        break;
    case Mode::Number::QLAND:
        init_qland();
        break;
    case Mode::Number::QRTL:
        init_qrtl();
        break;
    case Mode::Number::GUIDED:
        guided_takeoff = false;
        break;
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
        return qautotune.init();
#endif
    case Mode::Number::QACRO:
        init_qacro();
        break;
    default:
        break;
    }
    return true;
}

/*
  handle a MAVLink DO_VTOL_TRANSITION
 */
bool QuadPlane::handle_do_vtol_transition(enum MAV_VTOL_STATE state) const
{
    if (!available()) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "VTOL not available");
        return false;
    }
    if (plane.control_mode != &plane.mode_auto) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "VTOL transition only in AUTO");
        return false;
    }
    switch (state) {
    case MAV_VTOL_STATE_MC:
        if (!plane.auto_state.vtol_mode) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Entered VTOL mode");
        }
        plane.auto_state.vtol_mode = true;
        return true;
        
    case MAV_VTOL_STATE_FW:
        if (plane.auto_state.vtol_mode) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Exited VTOL mode");
        }
        plane.auto_state.vtol_mode = false;

        return true;

    default:
        break;
    }

    gcs().send_text(MAV_SEVERITY_NOTICE, "Invalid VTOL mode");
    return false;
}

/*
  are we in a VTOL auto state?
 */
bool QuadPlane::in_vtol_auto(void) const
{
    if (!available()) {
        return false;
    }
    if (plane.control_mode != &plane.mode_auto) {
        return false;
    }
    if (plane.auto_state.vtol_mode) {
        return true;
    }
    uint16_t id = plane.mission.get_current_nav_cmd().id;
    switch (id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return true;
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TO_ALT:
        return plane.auto_state.vtol_loiter;
    case MAV_CMD_NAV_TAKEOFF:
        return is_vtol_takeoff(id);
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        return is_vtol_land(id);
    default:
        return false;
    }
}

/*
  are we in a VTOL mode? This is used to decide if we run the
  transition handling code or not
 */
bool QuadPlane::in_vtol_mode(void) const
{
    if (!available()) {
        return false;
    }
    if (plane.control_mode == &plane.mode_qrtl &&
        (poscontrol.get_state() == QPOS_APPROACH ||
         poscontrol.get_state() == QPOS_AIRBRAKE)) {
        return false;
    }
    if (in_vtol_land_approach() &&
        poscontrol.get_state() == QPOS_APPROACH) {
        return false;
    }
    if (plane.control_mode->is_vtol_mode()) {
        return true;
    }
    if (plane.control_mode->is_guided_mode()
        && plane.auto_state.vtol_loiter &&
        poscontrol.get_state() > QPOS_APPROACH) {
        return true;
    }
    if (in_vtol_auto()) {
        if (!plane.auto_state.vtol_loiter || poscontrol.get_state() > QPOS_APPROACH) {
            return true;
        }
    }
    return false;
}

/*
  are we in a VTOL mode that needs position and velocity estimates?
 */
bool QuadPlane::in_vtol_posvel_mode(void) const
{
    if (!available()) {
        return false;
    }
    return (plane.control_mode == &plane.mode_qloiter ||
            plane.control_mode == &plane.mode_qland ||
            plane.control_mode == &plane.mode_qrtl ||
            plane.control_mode == &plane.mode_qautotune ||
            (plane.control_mode->is_guided_mode() &&
            plane.auto_state.vtol_loiter &&
             poscontrol.get_state() > QPOS_APPROACH) ||
            in_vtol_auto());
}

/*
  update landing positioning offset
 */
void QuadPlane::update_land_positioning(void)
{
    if ((options & OPTION_REPOSITION_LANDING) == 0) {
        // not enabled
        poscontrol.pilot_correction_active = false;
        poscontrol.target_vel_cms.zero();
        return;
    }
    const float scale = 1.0 / 4500;
    float roll_in = plane.channel_roll->get_control_in() * scale;
    float pitch_in = plane.channel_pitch->get_control_in() * scale;

    // limit correction speed to accel with stopping time constant of 0.5s
    const float speed_max_cms = wp_nav->get_wp_acceleration() * 0.5;
    const float dt = plane.scheduler.get_loop_period_s();

    poscontrol.target_vel_cms = Vector3f(-pitch_in, roll_in, 0) * speed_max_cms;
    poscontrol.target_vel_cms.rotate_xy(ahrs_view->yaw);

    poscontrol.target_cm += (poscontrol.target_vel_cms * dt).topostype();

    poscontrol.pilot_correction_active = (!is_zero(roll_in) || !is_zero(pitch_in));
    if (poscontrol.pilot_correction_active) {
        poscontrol.pilot_correction_done = true;
    }
}

/*
  run (and possibly init) xy controller
 */
void QuadPlane::run_xy_controller(void)
{
    if (!pos_control->is_active_xy()) {
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->init_xy_controller();
    }
    pos_control->update_xy_controller();
}

/*
  initialise QPOS_APPROACH
 */
void QuadPlane::poscontrol_init_approach(void)
{
    if ((options & OPTION_DISABLE_APPROACH) != 0) {
        // go straight to QPOS_POSITION1
        poscontrol.set_state(QPOS_POSITION1);
    } else if (poscontrol.get_state() != QPOS_APPROACH) {
        const float dist = plane.current_loc.get_distance(plane.next_WP_loc);
        gcs().send_text(MAV_SEVERITY_INFO,"VTOL approach d=%.1f", dist);
        poscontrol.set_state(QPOS_APPROACH);
        poscontrol.thrust_loss_start_ms = 0;
    }
}

/*
  change position control state
 */
void QuadPlane::PosControlState::set_state(enum position_control_state s)
{
    if (state != s) {
        auto &qp = plane.quadplane;
        pilot_correction_done = false;
        // handle resets needed for when the state changes
        if (s == QPOS_POSITION1) {
            reached_wp_speed = false;
            qp.attitude_control->reset_yaw_target_and_rate();
        } else if (s == QPOS_POSITION2) {
            // POSITION2 changes target speed, so we need to change it
            // back to normal
            qp.pos_control->set_max_speed_accel_xy(qp.wp_nav->get_default_speed_xy(),
                                                   qp.wp_nav->get_wp_acceleration());
            qp.pos_control->set_correction_speed_accel_xy(qp.wp_nav->get_default_speed_xy(),
                                                   qp.wp_nav->get_wp_acceleration());
        } else if (s == QPOS_AIRBRAKE) {
            // start with zero integrator on vertical throttle
            qp.pos_control->get_accel_z_pid().set_integrator(0);
        }
    }
    state = s;
    last_state_change_ms = AP_HAL::millis();
}

/*
  main landing controller. Used for landing and RTL.
 */
void QuadPlane::vtol_position_controller(void)
{
    if (!setup()) {
        return;
    }

    const Location &loc = plane.next_WP_loc;
    uint32_t now_ms = AP_HAL::millis();

    // distance that we switch to QPOS_POSITION2
    const float position2_dist_threshold = 5.0;

    // target speed when we reach position2 threshold
    const float position2_target_speed = 2.0;

    check_attitude_relax();

    // horizontal position control
    switch (poscontrol.get_state()) {

    case QPOS_NONE:
        poscontrol.set_state(QPOS_POSITION1);
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;

    case QPOS_APPROACH:
        if (in_vtol_mode()) {
            // this means we're not running update_transition() and
            // thus not doing qassist checking, force POSITION1 mode
            // now. We don't expect this to trigger, it is a failsafe
            // for a logic error
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 nvtol");
            poscontrol.set_state(QPOS_POSITION1);
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
        FALLTHROUGH;

    case QPOS_AIRBRAKE: {
        float aspeed;
        const Vector2f closing_vel = landing_closing_velocity();
        const Vector2f desired_closing_vel = landing_desired_closing_velocity();
        const float groundspeed = plane.ahrs.groundspeed();
        const float distance = plane.auto_state.wp_distance;
        const float closing_speed = closing_vel.length();
        const float desired_closing_speed = desired_closing_vel.length();
        if (!plane.ahrs.airspeed_estimate(aspeed)) {
            aspeed = groundspeed;
        }

        // speed for crossover to POSITION1 controller
        const float aspeed_threshold = MAX(plane.aparm.airspeed_min-2, assist_speed);

        // run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.current_loc, loc);

        // use TECS for throttle
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.SpdHgt_Controller->get_throttle_demand());

        // use TECS for pitch
        int32_t commanded_pitch = plane.SpdHgt_Controller->get_pitch_demand();
        plane.nav_pitch_cd = constrain_int32(commanded_pitch, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
        if (poscontrol.get_state() == QPOS_AIRBRAKE) {
            // don't allow down pitch in airbrake
            plane.nav_pitch_cd = MAX(plane.nav_pitch_cd, 0);
        }

        // use nav controller roll
        plane.calc_nav_roll();

        const float stop_distance = stopping_distance();

        if (poscontrol.get_state() == QPOS_AIRBRAKE) {
            hold_hover(0);
        }

        /*
          see if we should start airbraking stage. For non-tailsitters
          we can use the VTOL motors as airbrakes by firing them up
          before we transition. This gives a smoother transition and
          gives us a nice lot of deceleration
         */
        if (poscontrol.get_state() == QPOS_APPROACH && distance < stop_distance) {
            if (is_tailsitter() || motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
                // tailsitters don't use airbrake stage for landing
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 v=%.1f d=%.0f sd=%.0f h=%.1f",
                                groundspeed,
                                plane.auto_state.wp_distance,
                                stop_distance,
                                plane.relative_ground_altitude(plane.g.rangefinder_landing));
                poscontrol.set_state(QPOS_POSITION1);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL airbrake v=%.1f d=%.0f sd=%.0f h=%.1f",
                                groundspeed,
                                distance,
                                stop_distance,
                                plane.relative_ground_altitude(plane.g.rangefinder_landing));
                poscontrol.set_state(QPOS_AIRBRAKE);
            }
        }

        /*
          we must switch to POSITION1 if our airspeed drops below the
          assist speed. We additionally switch to POSITION1 if we are
          too far above our desired velocity profile, or our attitude
          has deviated too much
         */
        const int32_t attitude_error_threshold_cd = 1000;

        // use at least 1s of airbrake time to ensure motors have a chance to
        // properly spin up
        const uint32_t min_airbrake_ms = 1000;
        if (poscontrol.get_state() == QPOS_AIRBRAKE &&
            poscontrol.time_since_state_start_ms() > min_airbrake_ms &&
            (aspeed < aspeed_threshold ||
             closing_speed > MAX(desired_closing_speed*1.2, desired_closing_speed+2) ||
             labs(plane.ahrs.roll_sensor - plane.nav_roll_cd) > attitude_error_threshold_cd ||
             labs(plane.ahrs.pitch_sensor - plane.nav_pitch_cd) > attitude_error_threshold_cd)) {
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 v=%.1f d=%.1f h=%.1f dc=%.1f",
                            (double)groundspeed,
                            (double)plane.auto_state.wp_distance,
                            plane.relative_ground_altitude(plane.g.rangefinder_landing),
                            desired_closing_speed);
            poscontrol.set_state(QPOS_POSITION1);

            // switch to vfwd for throttle control
            vel_forward.integrator = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
            vel_forward.last_ms = now_ms;
        }

        if (tilt.tilt_mask == 0 && !is_tailsitter()) {
            /*
              cope with fwd motor thrust loss during approach. We detect
              this by looking for the fwd throttle saturating. This only
              applies to separate lift-thrust vehicles
            */
            bool throttle_saturated = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) >= plane.aparm.throttle_max;
            if (throttle_saturated &&
                motors->get_desired_spool_state() < AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED &&
                plane.auto_state.sink_rate > 0.2 && aspeed < aspeed_threshold+4) {
                if (poscontrol.thrust_loss_start_ms == 0) {
                    poscontrol.thrust_loss_start_ms = now_ms;
                }
                if (now_ms - poscontrol.thrust_loss_start_ms > 5000) {
                    gcs().send_text(MAV_SEVERITY_INFO,"VTOL pos1 thrust loss as=%.1f at=%.1f",
                                    aspeed, aspeed_threshold);
                    poscontrol.set_state(QPOS_POSITION1);
                }
            } else {
                poscontrol.thrust_loss_start_ms = 0;
            }

            // handle loss of forward thrust in approach based on low airspeed detection
            if (poscontrol.get_state() == QPOS_APPROACH && aspeed < aspeed_threshold &&
                motors->get_desired_spool_state() < AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
                gcs().send_text(MAV_SEVERITY_INFO,"VTOL pos1 low speed as=%.1f at=%.1f",
                                aspeed, aspeed_threshold);
                poscontrol.set_state(QPOS_POSITION1);
            }
        }

        
        if (poscontrol.get_state() == QPOS_APPROACH) {
            poscontrol_init_approach();
        }
        break;
    }

    case QPOS_POSITION1: {
        setup_target_position();

        if (is_tailsitter()) {
            if (in_tailsitter_vtol_transition()) {
                break;
            }
            poscontrol.set_state(QPOS_POSITION2);
            poscontrol.pilot_correction_done = false;
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position2 started v=%.1f d=%.1f",
                                    (double)ahrs.groundspeed(), (double)plane.auto_state.wp_distance);
            break;
        }


        const Vector2f diff_wp = plane.current_loc.get_distance_NE(loc);
        const float distance = diff_wp.length();

        // calculate speed we should be at to reach the position2
        // target speed at the position2 distance threshold, assuming
        // Q_TRANS_DECEL is correct
        const float stopping_speed = safe_sqrt(MAX(0, distance-position2_dist_threshold) * 2 * transition_decel) + position2_target_speed;

        float target_speed = stopping_speed;

        // maximum configured VTOL speed
        const float wp_speed = pos_control->get_max_speed_xy_cms() * 0.01;
        const float current_speed_sq = plane.ahrs.groundspeed_vector().length_squared();
        const float scaled_wp_speed = get_scaled_wp_speed(degrees(diff_wp.angle()));

        if (poscontrol.reached_wp_speed ||
            current_speed_sq < sq(wp_speed) ||
            wp_speed > 1.35*scaled_wp_speed) {
            // once we get below the Q_WP_SPEED then we don't want to
            // speed up again. At that point we should fly within the
            // limits of the configured VTOL controller we also apply
            // this limit when we are more than 45 degrees off the
            // target in yaw, which is when we start to become
            // unstable
            target_speed = MIN(target_speed, scaled_wp_speed);
            poscontrol.reached_wp_speed = true;
        }

        // run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.current_loc, loc);

        Vector2f target_speed_xy;
        if (distance > 0.1) {
            target_speed_xy = diff_wp.normalized() * target_speed;
        }
        pos_control->set_vel_desired_xy_cms(target_speed_xy * 100);

        // reset position controller xy target to current position
        // because we only want velocity control (no position control)
        const Vector3f& curr_pos = inertial_nav.get_position();
        pos_control->set_pos_target_xy_cm(curr_pos.x, curr_pos.y);
        pos_control->set_accel_desired_xy_cmss(Vector2f());

        // run horizontal velocity controller
        run_xy_controller();

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        /*
          limit the pitch down with an expanding envelope. This
          prevents the velocity controller demanding nose down during
          the initial slowdown if the target velocity curve is higher
          than the actual velocity curve (for a high drag
          aircraft). Nose down will cause a lot of downforce on the
          wings which will draw a lot of current and also cause the
          aircraft to lose altitude rapidly.pitch limit varies also with speed
          to prevent inability to progress to position if moving from a loiter
          to landing
         */
        float minlimit_cd = linear_interpolate(-300, MAX(-aparm.angle_max,plane.aparm.pitch_limit_min_cd),
                                               poscontrol.time_since_state_start_ms(),
                                               0, 5000);
        if (plane.nav_pitch_cd < minlimit_cd) {
            plane.nav_pitch_cd = minlimit_cd;
            // tell the pos controller we have limited the pitch to
            // stop integrator buildup
            pos_control->set_externally_limited_xy();
        }

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      desired_auto_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        if (plane.auto_state.wp_distance < position2_dist_threshold) {
            poscontrol.set_state(QPOS_POSITION2);
            poscontrol.pilot_correction_done = false;
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position2 started v=%.1f d=%.1f",
                                    (double)ahrs.groundspeed(), (double)plane.auto_state.wp_distance);
        }
        break;
    }

    case QPOS_POSITION2:
    case QPOS_LAND_DESCEND: {
        /*
          for final land repositioning and descent we run the position controller
         */
        if (poscontrol.pilot_correction_done) {
            // if the pilot has repositioned the vehicle then we switch to velocity control.  This prevents the vehicle
            // shifting position in the event of GPS glitches.
            Vector2f zero;
            pos_control->input_vel_accel_xy(poscontrol.target_vel_cms.xy(), zero);
        } else {
            Vector2f zero;
            pos_control->input_pos_vel_accel_xy(poscontrol.target_cm.xy(), zero, zero);
        }

        // also run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.current_loc, loc);

        update_land_positioning();

        /*
          apply the same asymmetric speed limits from POSITION1, so we
          don't suddenly speed up when we change to POSITION2 and
          LAND_DESCEND
         */
        const Vector2f diff_wp = plane.current_loc.get_distance_NE(loc);
        const float scaled_wp_speed = get_scaled_wp_speed(degrees(diff_wp.angle()));

        pos_control->set_max_speed_accel_xy(scaled_wp_speed*100, wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(scaled_wp_speed*100, wp_nav->get_wp_acceleration());

        run_xy_controller();

        // nav roll and pitch are controlled by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        break;
    }

    case QPOS_LAND_FINAL:
        update_land_positioning();

        // relax when close to the ground
        if (should_relax()) {
            pos_control->relax_velocity_controller_xy();
        } else {
            // we use velocity control in QPOS_LAND_FINAL to allow for GPS glitch handling
            Vector2f zero;
            pos_control->input_vel_accel_xy(poscontrol.target_vel_cms.xy(), zero);
        }

        run_xy_controller();

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll_cd();
        plane.nav_pitch_cd = pos_control->get_pitch_cd();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        break;

    case QPOS_LAND_COMPLETE:
        // nothing to do
        break;
    }

    // now height control
    switch (poscontrol.get_state()) {
    case QPOS_NONE:
        poscontrol.set_state(QPOS_POSITION1);
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;

    case QPOS_APPROACH:
    case QPOS_AIRBRAKE:
        // we just want stability from the VTOL controller in these
        // phases of landing, so relax the Z controller, unless we are
        // providing assistance
        if (transition_state == TRANSITION_DONE) {
            pos_control->relax_z_controller(0);
        }
        break;
    case QPOS_POSITION1:
        if (in_tailsitter_vtol_transition()) {
            pos_control->relax_z_controller(0);
            break;
        }
        FALLTHROUGH;
    case QPOS_POSITION2: {
        bool vtol_loiter_auto = false;
        if (plane.control_mode == &plane.mode_auto) {
            switch (plane.mission.get_current_nav_cmd().id) {
            case MAV_CMD_NAV_LOITER_UNLIM:
            case MAV_CMD_NAV_LOITER_TIME:
            case MAV_CMD_NAV_LOITER_TURNS:
            case MAV_CMD_NAV_LOITER_TO_ALT:
                vtol_loiter_auto = true;
                break;
            }
        }
        if (plane.control_mode == &plane.mode_guided || vtol_loiter_auto) {
            plane.ahrs.get_position(plane.current_loc);
            int32_t target_altitude_cm;
            if (!plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME,target_altitude_cm)) {
                break;
            }
            if (poscontrol.slow_descent &&
                plane.prev_WP_loc.get_distance(plane.next_WP_loc) > 50) {
                // gradually descend as we approach target
                plane.auto_state.wp_proportion = plane.current_loc.line_path_proportion(plane.prev_WP_loc, plane.next_WP_loc);
                int32_t prev_alt;
                if (plane.prev_WP_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME,prev_alt)) {
                    target_altitude_cm = linear_interpolate(prev_alt,
                                                         target_altitude_cm,
                                                         plane.auto_state.wp_proportion,
                                                         0, 1);
                }
            }
#if AP_TERRAIN_AVAILABLE
            float terrain_altitude_offset_cm;
            if (plane.next_WP_loc.terrain_alt && plane.terrain.height_terrain_difference_home(terrain_altitude_offset_cm, true)) {
                // Climb if current terrain is above home, target_altitude_cm is reltive to home
                target_altitude_cm += MAX(terrain_altitude_offset_cm*100,0);
            }
#endif
            float zero = 0;
            float target_z = target_altitude_cm;
            pos_control->input_pos_vel_accel_z(target_z, zero, 0);
        } else {
            set_climb_rate_cms(0, false);
        }
        break;
    }

    case QPOS_LAND_DESCEND:
    case QPOS_LAND_FINAL: {
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        if (poscontrol.get_state() == QPOS_LAND_FINAL) {
            if ((options & OPTION_DISABLE_GROUND_EFFECT_COMP) == 0) {
                ahrs.set_touchdown_expected(true);
            }
        }
        const float descent_rate_cms = landing_descent_rate_cms(height_above_ground);
        set_climb_rate_cms(-descent_rate_cms, descent_rate_cms>0);
        break;
    }

    case QPOS_LAND_COMPLETE:
        break;
    }
    
    run_z_controller();

    if (now_ms - poscontrol.last_log_ms >= 40) {
        // log poscontrol at 25Hz
        poscontrol.last_log_ms = now_ms;
        AP::logger().Write("QPOS", "TimeUS,State,Dist", "QBf",
                           AP_HAL::micros64(),
                           poscontrol.get_state(),
                           plane.auto_state.wp_distance);
    }
}


/*
  we want to limit WP speed to a lower speed when more than 20 degrees
  off pointing at the destination. quadplanes are often
  unstable when flying sideways or backwards
*/
float QuadPlane::get_scaled_wp_speed(float target_bearing_deg) const
{
    const float yaw_difference = fabsf(wrap_180(degrees(plane.ahrs.yaw) - target_bearing_deg));
    const float wp_speed = wp_nav->get_default_speed_xy() * 0.01;
    if (yaw_difference > 20) {
        // this gives a factor of 2x reduction in max speed when
        // off by 90 degrees, and 3x when off by 180 degrees
        const float speed_reduction = linear_interpolate(1, 3,
                                                         yaw_difference,
                                                         20, 160);
        return wp_speed / speed_reduction;
    }
    return wp_speed;
}

/*
  setup the target position based on plane.next_WP_loc
 */
void QuadPlane::setup_target_position(void)
{
    const Location &loc = plane.next_WP_loc;
    Location origin;
    if (!ahrs.get_origin(origin)) {
        origin.zero();
    }
    if (!in_vtol_land_approach() || poscontrol.get_state() > QPOS_APPROACH) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    const Vector2f diff2d = origin.get_distance_NE(loc);
    poscontrol.target_cm.x = diff2d.x * 100;
    poscontrol.target_cm.y = diff2d.y * 100;
    poscontrol.target_cm.z = plane.next_WP_loc.alt - origin.alt;

    const uint32_t now = AP_HAL::millis();
    if (!loc.same_latlon_as(last_auto_target) ||
        plane.next_WP_loc.alt != last_auto_target.alt ||
        now - last_loiter_ms > 500) {
        wp_nav->set_wp_destination(poscontrol.target_cm.tofloat());
        last_auto_target = loc;
    }
    last_loiter_ms = now;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);
}

/*
  run takeoff controller to climb vertically
 */
void QuadPlane::takeoff_controller(void)
{
    /*
      for takeoff we use the position controller
    */
    check_attitude_relax();

    setup_target_position();

    // set position controller desired velocity and acceleration to zero
    pos_control->set_vel_desired_xy_cms(Vector2f());
    pos_control->set_accel_desired_xy_cmss(Vector2f());

    // set position control target and update
    Vector2f zero;
    pos_control->input_vel_accel_xy(zero, zero);

    run_xy_controller();

    // nav roll and pitch are controller by position controller
    plane.nav_roll_cd = pos_control->get_roll_cd();
    plane.nav_pitch_cd = pos_control->get_pitch_cd();

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());

    set_climb_rate_cms(wp_nav->get_default_speed_up(), false);
    run_z_controller();
}

/*
  run waypoint controller between prev_WP_loc and next_WP_loc
 */
void QuadPlane::waypoint_controller(void)
{
    setup_target_position();

    check_attitude_relax();

    /*
      this is full copter control of auto flight
    */
    // run wpnav controller
    wp_nav->update_wpnav();
    
    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(),
                                                       wp_nav->get_pitch(),
                                                       wp_nav->get_yaw(),
                                                       true);
    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = wp_nav->get_roll();
    plane.nav_pitch_cd = wp_nav->get_pitch();
    
    // climb based on altitude error
    set_climb_rate_cms(assist_climb_rate_cms(), false);
    run_z_controller();
}


/*
  handle auto-mode when auto_state.vtol_mode is true
 */
void QuadPlane::control_auto(void)
{
    if (!setup()) {
        return;
    }

    if (poscontrol.get_state() > QPOS_APPROACH) {
        if (!plane.arming.get_delay_arming()) {
            set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }
    }

    uint16_t id = plane.mission.get_current_nav_cmd().id;
    switch (id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        if (is_vtol_takeoff(id)) {
            takeoff_controller();
        }
        break;
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        if (is_vtol_land(id)) {
            vtol_position_controller();
        }
        break;
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TO_ALT:
        vtol_position_controller();
        break;
    default:
        waypoint_controller();
        break;
    }
}

/*
  handle QRTL mode
 */
void QuadPlane::control_qrtl(void)
{
    vtol_position_controller();
    if (poscontrol.get_state() >= QPOS_POSITION2) {
        // change target altitude to home alt
        plane.next_WP_loc.alt = plane.home.alt;
        verify_vtol_land();
    }
}

/*
  handle QRTL mode
 */
void QuadPlane::init_qrtl(void)
{
    // use do_RTL() to setup next_WP_loc
    plane.do_RTL(plane.home.alt + qrtl_alt*100UL);
    plane.prev_WP_loc = plane.current_loc;
    pos_control->set_accel_desired_xy_cmss(Vector2f());
    pos_control->init_xy_controller();
    poscontrol_init_approach();
    float dist = plane.next_WP_loc.get_distance(plane.current_loc);
    const float radius = MAX(fabsf(plane.aparm.loiter_radius), fabsf(plane.g.rtl_radius));
    if (dist < 1.5*radius &&
        motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        // we're close to destination and already running VTOL motors, don't transition
        gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 d=%.1f r=%.1f", dist, radius);
        poscontrol.set_state(QPOS_POSITION1);
    }
    int32_t from_alt;
    int32_t to_alt;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,from_alt) && plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,to_alt)) {
        poscontrol.slow_descent = from_alt > to_alt;
        return;
    }
    // defualt back to old method
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
}

/*
  start a VTOL takeoff
 */
bool QuadPlane::do_vtol_takeoff(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }

    // we always use the current location in XY for takeoff. The altitude defaults
    // to relative to current height, but if Q_OPTIONS is set to respect takeoff frame
    // then it will use normal frame handling for height
    Location loc = cmd.content.location;
    loc.lat = 0;
    loc.lng = 0;
    plane.set_next_WP(loc);
    if (options & OPTION_RESPECT_TAKEOFF_FRAME) {
        if (plane.current_loc.alt >= plane.next_WP_loc.alt) {
            // we are above the takeoff already, no need to do anything
            return false;
        }
    } else {
        plane.next_WP_loc.alt = plane.current_loc.alt + cmd.content.location.alt;
    }
    throttle_wait = false;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_velocity_z_max_dn(), pilot_velocity_z_max_up, pilot_accel_z);

    // initialise the vertical position controller
    pos_control->init_z_controller();

    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.current_loc, plane.next_WP_loc);

    // calculate the time required to complete a takeoff
    // this may be conservative and accept extra time due to clamping
    // derived from the following latex equations if you want a nicely formatted view
    // t_{accel} = \frac{V_max - V_z}{a}
    // d_{accel} = V_z*t_{accel} + \frac{1}{2}*a*t_{accel}^2
    // d_{remaining} = d_{total} - d_{accel}
    // t_{constant} = \frac{d_{remaining}}{V_z}
    // t = max(t_{accel}, 0) + max(t_{constant}, 0)
    const float d_total = (plane.next_WP_loc.alt - plane.current_loc.alt) * 0.01f;
    const float accel_m_s_s = MAX(10, pilot_accel_z) * 0.01f;
    const float vel_max = MAX(10, pilot_velocity_z_max_up) * 0.01f;
    const float vel_z = inertial_nav.get_velocity_z() * 0.01f;
    const float t_accel = (vel_max - vel_z) / accel_m_s_s;
    const float d_accel = vel_z * t_accel + 0.5f * accel_m_s_s * sq(t_accel);
    const float d_remaining = d_total - d_accel;
    const float t_constant = d_remaining / vel_max;
    const float travel_time = MAX(t_accel, 0) + MAX(t_constant, 0);

    // setup the takeoff failure handling code
    takeoff_start_time_ms = millis();
    takeoff_time_limit_ms = MAX(travel_time * takeoff_failure_scalar * 1000, 5000); // minimum time 5 seconds

    return true;
}


/*
  start a VTOL landing
 */
bool QuadPlane::do_vtol_land(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }
    attitude_control->reset_rate_controller_I_terms();
    
    plane.set_next_WP(cmd.content.location);
    // initially aim for current altitude
    plane.next_WP_loc.alt = plane.current_loc.alt;
    poscontrol.set_state(QPOS_POSITION1);

    // initialise the position controller
    pos_control->init_xy_controller();
    pos_control->init_z_controller();

    throttle_wait = false;
    landing_detect.lower_limit_start_ms = 0;
    landing_detect.land_start_ms = 0;

    plane.crash_state.is_crashed = false;
    
    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.current_loc, plane.next_WP_loc);

    poscontrol_init_approach();
    return true;
}

/*
  check if a VTOL takeoff has completed
 */
bool QuadPlane::verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd)
{
    if (!available()) {
        return true;
    }

    const uint32_t now = millis();

    // reset takeoff start time if we aren't armed, as we won't have made any progress
    if (!hal.util->get_soft_armed()) {
        takeoff_start_time_ms = now;
    }

    if (now - takeoff_start_time_ms < 3000 &&
        (options & OPTION_DISABLE_GROUND_EFFECT_COMP) == 0) {
        ahrs.set_takeoff_expected(true);
    }
    
    // check for failure conditions
    if (is_positive(takeoff_failure_scalar) && ((now - takeoff_start_time_ms) > takeoff_time_limit_ms)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to complete takeoff within time limit");
        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TAKEOFF);
        return false;
    }

    if (is_positive(maximum_takeoff_airspeed) && (plane.airspeed.get_airspeed() > maximum_takeoff_airspeed)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to complete takeoff, excessive wind");
        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TAKEOFF);
        return false;
    }

    if (plane.current_loc.alt < plane.next_WP_loc.alt) {
        return false;
    }
    transition_state = is_tailsitter() ? TRANSITION_ANGLE_WAIT_FW : TRANSITION_AIRSPEED_WAIT;
    plane.TECS_controller.set_pitch_max_limit(transition_pitch_max);

    // todo: why are you doing this, I want to delete it.
    set_alt_target_current();

#if AC_FENCE == ENABLED
    plane.fence.auto_enable_fence_after_takeoff();
#endif

    if (plane.control_mode == &plane.mode_auto) {
        // we reset TECS so that the target height filter is not
        // constrained by the climb and sink rates from the initial
        // takeoff height.
        plane.SpdHgt_Controller->reset();
    }

    // don't crosstrack on next WP
    plane.auto_state.next_wp_crosstrack = false;

    return true;
}

/*
  a landing detector based on change in altitude over a timeout
 */
bool QuadPlane::land_detector(uint32_t timeout_ms)
{
    const uint32_t now = AP_HAL::millis();
    bool might_be_landed =  (landing_detect.lower_limit_start_ms != 0 &&
                             now - landing_detect.lower_limit_start_ms > 1000);
    if (!might_be_landed) {
        landing_detect.land_start_ms = 0;
        return false;
    }
    float height = inertial_nav.get_altitude()*0.01f;
    if (landing_detect.land_start_ms == 0) {
        landing_detect.land_start_ms = now;
        landing_detect.vpos_start_m = height;
    }

    // we only consider the vehicle landed when the motors have been
    // at minimum for timeout_ms+1000 and the vertical position estimate has not
    // changed by more than 20cm for timeout_ms
    if (fabsf(height - landing_detect.vpos_start_m) > 0.2) {
        // height has changed, call off landing detection
        landing_detect.land_start_ms = 0;
        return false;
    }
           
    if ((now - landing_detect.land_start_ms) < timeout_ms ||
        (now - landing_detect.lower_limit_start_ms) < (timeout_ms+1000)) {
        // not landed yet
        return false;
    }

    return true;
}

/*
  check if a landing is complete
 */
bool QuadPlane::check_land_complete(void)
{
    if (poscontrol.get_state() != QPOS_LAND_FINAL) {
        // only apply to final landing phase
        return false;
    }
    if (land_detector(4000)) {
        poscontrol.set_state(QPOS_LAND_COMPLETE);
        gcs().send_text(MAV_SEVERITY_INFO,"Land complete");
        if (plane.control_mode != &plane.mode_auto ||
            !plane.mission.continue_after_land()) {
            // disarm on land unless we have MIS_OPTIONS setup to
            // continue after land in AUTO
            plane.arming.disarm(AP_Arming::Method::LANDED);
        }
        return true;
    }
    return false;
}


/*
  check if we should switch from QPOS_LAND_DESCEND to QPOS_LAND_FINAL
 */
bool QuadPlane::check_land_final(void)
{
    float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    // we require 2 readings at 10Hz to be within 5m of each other to
    // trigger the switch to land final. This prevents a short term
    // glitch at high altitude from triggering land final
    const float max_change = 5;
    if (height_above_ground < land_final_alt &&
        fabsf(height_above_ground - last_land_final_agl) < max_change) {
        return true;
    }
    last_land_final_agl = height_above_ground;

    /*
      also apply landing detector, in case we have landed in descent
      phase. Use a longer threshold
     */
    return land_detector(6000);
}

/*
  check if a VTOL landing has completed
 */
bool QuadPlane::verify_vtol_land(void)
{
    if (!available()) {
        return true;
    }

    if (poscontrol.get_state() == QPOS_POSITION2) {
        // see if we should move onto the descend stage of landing
        const float descend_dist_threshold = 2.0;
        const float descend_speed_threshold = 3.0;
        bool reached_position = false;
        if (poscontrol.pilot_correction_done) {
            reached_position = !poscontrol.pilot_correction_active;
        } else {
            const float dist = (inertial_nav.get_position().topostype() - poscontrol.target_cm).xy().length() * 0.01;
            reached_position = dist < descend_dist_threshold;
        }
        if (reached_position &&
            plane.ahrs.groundspeed() < descend_speed_threshold) {
            poscontrol.set_state(QPOS_LAND_DESCEND);
            poscontrol.pilot_correction_done = false;
#if AC_FENCE == ENABLED
            plane.fence.auto_disable_fence_for_landing();
#endif
#if LANDING_GEAR_ENABLED == ENABLED
            plane.g2.landing_gear.deploy_for_landing();
#endif
            last_land_final_agl = plane.relative_ground_altitude(plane.g.rangefinder_landing);
            gcs().send_text(MAV_SEVERITY_INFO,"Land descend started");
            if (plane.control_mode == &plane.mode_auto) {
                // set height to mission height, so we can use the mission
                // WP height for triggering land final if no rangefinder
                // available
                plane.set_next_WP(plane.mission.get_current_nav_cmd().content.location);
            } else {
                plane.set_next_WP(plane.next_WP_loc);
                plane.next_WP_loc.alt = ahrs.get_home().alt;
            }
        }
    }

    // at land_final_alt begin final landing
    if (poscontrol.get_state() == QPOS_LAND_DESCEND && check_land_final()) {
        poscontrol.set_state(QPOS_LAND_FINAL);

        // cut IC engine if enabled
        if (land_icengine_cut != 0) {
            plane.g2.ice_control.engine_control(0, 0, 0);
        }
        gcs().send_text(MAV_SEVERITY_INFO,"Land final started");
    }

    if (check_land_complete() && plane.mission.continue_after_land()) {
        gcs().send_text(MAV_SEVERITY_INFO,"Mission continue");
        return true;
    }
    return false;
}

// Write a control tuning packet
void QuadPlane::Log_Write_QControl_Tuning()
{
    float des_alt_m = 0.0f;
    int16_t target_climb_rate_cms = 0;
    if (plane.control_mode != &plane.mode_qstabilize) {
        des_alt_m = pos_control->get_pos_target_z_cm() / 100.0f;
        target_climb_rate_cms = pos_control->get_vel_target_z_cms();
    }

    struct log_QControl_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_QTUN_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : des_alt_m,
        inav_alt            : inertial_nav.get_altitude() / 100.0f,
        baro_alt            : int32_t(plane.barometer.get_altitude() * 100),
        target_climb_rate   : target_climb_rate_cms,
        climb_rate          : int16_t(inertial_nav.get_velocity_z()),
        throttle_mix        : attitude_control->get_throttle_mix(),
        speed_scaler        : log_spd_scaler,
        transition_state    : static_cast<uint8_t>(transition_state),
        assist              : assisted_flight,
    };
    plane.logger.WriteBlock(&pkt, sizeof(pkt));

    // write multicopter position control message
    pos_control->write_log();
}


/*
  calculate the forward throttle percentage. The forward throttle can
  be used to assist with position hold and with landing approach. It
  reduces the need for down pitch which reduces load on the vertical
  lift motors.
 */
int8_t QuadPlane::forward_throttle_pct()
{
    /*
      Unless an RC channel is assigned for manual forward throttle control,
      we don't use forward throttle in QHOVER or QSTABILIZE as they are the primary
      recovery modes for a quadplane and need to be as simple as
      possible. They will drift with the wind.
    */
    if (plane.control_mode == &plane.mode_qacro ||
        plane.control_mode == &plane.mode_qstabilize ||
        plane.control_mode == &plane.mode_qhover) {

        if (rc_fwd_thr_ch == nullptr) {
            return 0;
        } else {
            // calculate fwd throttle demand from manual input
            float fwd_thr = rc_fwd_thr_ch->percent_input();

            // set forward throttle to fwd_thr_max * (manual input + mix): range [0,100]
            fwd_thr *= .01f * constrain_float(fwd_thr_max, 0, 100);
            return fwd_thr;
        }
    }

    /*
      in qautotune mode or modes without a velocity controller
    */
    if (vel_forward.gain <= 0 ||
        plane.control_mode == &plane.mode_qautotune) {
        return 0;
    }

    /*
      in modes with a velocity controller
    */
    float deltat = (AP_HAL::millis() - vel_forward.last_ms) * 0.001f;
    if (deltat > 1 || deltat < 0) {
        vel_forward.integrator = 0;
        deltat = 0.1;
    }
    if (deltat < 0.1) {
        // run at 10Hz
        return vel_forward.last_pct;
    }
    vel_forward.last_ms = AP_HAL::millis();
    
    // work out the desired speed in forward direction
    Vector3f desired_velocity_cms = pos_control->get_vel_desired_cms();

    // convert to NED m/s
    desired_velocity_cms.z *= -1;

    Vector3f vel_ned;
    if (!plane.ahrs.get_velocity_NED(vel_ned)) {
        // we don't know our velocity? EKF must be pretty sick
        vel_forward.last_pct = 0;
        vel_forward.integrator = 0;
        return 0;
    }
    // get component of velocity error in fwd body frame direction
    Vector3f vel_error_body = ahrs.get_rotation_body_to_ned().transposed() * ((desired_velocity_cms*0.01f) - vel_ned);

    float fwd_vel_error = vel_error_body.x;

    // scale forward velocity error by maximum airspeed
    fwd_vel_error /= MAX(plane.aparm.airspeed_max, 5);

    // add in a component from our current pitch demand. This tends to
    // move us to zero pitch. Assume that LIM_PITCH would give us the
    // WP nav speed.
    fwd_vel_error -= (wp_nav->get_default_speed_xy() * 0.01f) * plane.nav_pitch_cd / (float)plane.aparm.pitch_limit_max_cd;

    if (should_relax() && vel_ned.length() < 1) {
        // we may be landed
        fwd_vel_error = 0;
        vel_forward.integrator *= 0.95f;
    }
    
    // integrator as throttle percentage (-100 to 100)
    vel_forward.integrator += fwd_vel_error * deltat * vel_forward.gain * 100;

    // inhibit reverse throttle and allow petrol engines with min > 0
    int8_t fwd_throttle_min = plane.have_reverse_thrust() ? 0 : plane.aparm.throttle_min;
    vel_forward.integrator = constrain_float(vel_forward.integrator, fwd_throttle_min, plane.aparm.throttle_max);

    if (in_vtol_land_approach()) {
        // when we are doing horizontal positioning in a VTOL land
        // we always allow the fwd motor to run. Otherwise a bad
        // lidar could cause the aircraft not to be able to
        // approach the landing point when landing below the takeoff point
        vel_forward.last_pct = vel_forward.integrator;
    } else if ((in_vtol_land_final() && motors->limit.throttle_lower) ||
              (plane.g.rangefinder_landing && (plane.rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::OutOfRangeLow))) {
        // we're in the settling phase of landing or using a rangefinder that is out of range low, disable fwd motor
        vel_forward.last_pct = 0;
        vel_forward.integrator = 0;
    } else {
        // If we are below alt_cutoff then scale down the effect until
        // it turns off at alt_cutoff and decay the integrator
        float alt_cutoff = MAX(0,vel_forward_alt_cutoff);
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);

        vel_forward.last_pct = linear_interpolate(0, vel_forward.integrator,
                                                  height_above_ground, alt_cutoff, alt_cutoff+2);
    }
    if (vel_forward.last_pct == 0) {
        // if the percent is 0 then decay the integrator
        vel_forward.integrator *= 0.95f;
    }

    return vel_forward.last_pct;
}

/*
  get weathervaning yaw rate in cd/s
 */
float QuadPlane::get_weathervane_yaw_rate_cds(void)
{
    /*
      we only do weathervaning in modes where we are doing VTOL
      position control. We also don't do it if the pilot has given any
      yaw input in the last 3 seconds.
    */
    if (!in_vtol_mode() ||
        !motors->armed() ||
        weathervane.gain <= 0 ||
        plane.control_mode == &plane.mode_qstabilize ||
        plane.control_mode == &plane.mode_qhover ||
        plane.control_mode == &plane.mode_qautotune) {
        weathervane.last_output = 0;
        return 0;
    }
    const uint32_t tnow = millis();
    if (plane.channel_rudder->get_control_in() != 0) {
        weathervane.last_pilot_input_ms = tnow;
        weathervane.last_output = 0;
        return 0;
    }
    if (tnow - weathervane.last_pilot_input_ms < 3000) {
        weathervane.last_output = 0;
        return 0;
    }

    float roll = wp_nav->get_roll() / 100.0f;
    if (fabsf(roll) < weathervane.min_roll) {
        weathervane.last_output = 0;
        return 0;        
    }
    if (roll > 0) {
        roll -= weathervane.min_roll;
    } else {
        roll += weathervane.min_roll;
    }
    
    float output = constrain_float((roll/45.0f) * weathervane.gain, -1, 1);
    if (should_relax()) {
        output = 0;
    }
    weathervane.last_output = 0.98f * weathervane.last_output + 0.02f * output;

    // scale over half of yaw_rate_max. This gives the pilot twice the
    // authority of the weathervane controller
    return weathervane.last_output * (yaw_rate_max/2) * 100;
}

/*
  start guided mode control
 */
void QuadPlane::guided_start(void)
{
    guided_takeoff = false;
    setup_target_position();
    int32_t from_alt;
    int32_t to_alt;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,from_alt) && plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,to_alt)) {
        poscontrol.slow_descent = from_alt > to_alt;
    } else {
        // default back to old method
        poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    }
    poscontrol_init_approach();
}

/*
  update guided mode control
 */
void QuadPlane::guided_update(void)
{
    if (plane.control_mode == &plane.mode_guided && guided_takeoff && plane.current_loc.alt < plane.next_WP_loc.alt) {
        throttle_wait = false;
        set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        takeoff_controller();
    } else {
        guided_takeoff = false;
        // run VTOL position controller
        vtol_position_controller();
    }
}

void QuadPlane::afs_terminate(void)
{
    if (available()) {
        set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->output();
    }
}

/*
  return true if we should do guided mode loitering using VTOL motors
 */
bool QuadPlane::guided_mode_enabled(void)
{
    if (!available()) {
        return false;
    }
    // only use quadplane guided when in AUTO or GUIDED mode
    if (plane.control_mode != &plane.mode_guided && plane.control_mode != &plane.mode_auto) {
        return false;
    }
    if (plane.control_mode == &plane.mode_auto &&
        plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LOITER_TURNS) {
        // loiter turns is a fixed wing only operation
        return false;
    }
    return guided_mode != 0;
}

/*
  set altitude target to current altitude
 */
void QuadPlane::set_alt_target_current(void)
{
    pos_control->set_pos_target_z_cm(inertial_nav.get_altitude());
}

// user initiated takeoff for guided mode
bool QuadPlane::do_user_takeoff(float takeoff_altitude)
{
    if (plane.control_mode != &plane.mode_guided) {
        gcs().send_text(MAV_SEVERITY_INFO, "User Takeoff only in GUIDED mode");
        return false;
    }
    if (!hal.util->get_soft_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Must be armed for takeoff");
        return false;
    }
    if (is_flying()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already flying - no takeoff");
        return false;
    }
    plane.auto_state.vtol_loiter = true;
    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = plane.current_loc;
    plane.next_WP_loc.alt += takeoff_altitude*100;
    set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    guided_start();
    guided_takeoff = true;
    if ((options & OPTION_DISABLE_GROUND_EFFECT_COMP) == 0) {
        ahrs.set_takeoff_expected(true);
    }
    return true;
}

// return true if the wp_nav controller is being updated
bool QuadPlane::using_wp_nav(void) const
{
    if (plane.control_mode == &plane.mode_qloiter || plane.control_mode == &plane.mode_qland) {
        return true;
    }
    if (plane.control_mode == &plane.mode_qrtl && poscontrol.get_state() >= QPOS_POSITION2) {
        return true;
    }
    return false;
}

/*
  return mav_type for heartbeat
 */
MAV_TYPE QuadPlane::get_mav_type(void) const
{
    if (mav_type.get() == 0) {
        return MAV_TYPE_FIXED_WING;
    }
    return MAV_TYPE(mav_type.get());
}

/*
  return true if current mission item is a vtol takeoff
*/
bool QuadPlane::is_vtol_takeoff(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_TAKEOFF) {
        return true;
    }
    if (id == MAV_CMD_NAV_TAKEOFF && available() && (options & OPTION_ALLOW_FW_TAKEOFF) == 0) {
        // treat fixed wing takeoff as VTOL takeoff
        return true;
    }
    return false;
}

/*
  return true if current mission item is a vtol land
*/
bool QuadPlane::is_vtol_land(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_LAND) {
        if (options & QuadPlane::OPTION_MISSION_LAND_FW_APPROACH) {
            return plane.vtol_approach_s.approach_stage == Plane::Landing_ApproachStage::VTOL_LANDING;
        } else {
            return true;
        }
    }
    if (id == MAV_CMD_NAV_LAND && available() && (options & OPTION_ALLOW_FW_LAND) == 0) {
        // treat fixed wing land as VTOL land
        return true;
    }
    return false;
}

/*
  return true if we are in a transition to fwd flight from hover
 */
bool QuadPlane::in_transition(void) const
{
    return available() && assisted_flight &&
        (transition_state == TRANSITION_AIRSPEED_WAIT ||
         transition_state == TRANSITION_TIMER);
}

/*
  calculate current stopping distance for a quadplane in fixed wing flight
 */
float QuadPlane::stopping_distance(void)
{
    // use v^2/(2*accel). This is only quite approximate as the drag
    // varies with pitch, but it gives something for the user to
    // control the transition distance in a reasonable way
    return plane.ahrs.groundspeed_vector().length_squared() / (2 * transition_decel);
}

#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error to be considered landing
#define LAND_CHECK_LARGE_ANGLE_CD   1500.0f     // maximum angle target to be considered landing
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration after subtracting gravity

void QuadPlane::update_throttle_mix(void)
{
    // update filtered acceleration
    Vector3f accel_ef = ahrs.get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;
    throttle_mix_accel_ef_filter.apply(accel_ef, plane.scheduler.get_loop_period_s());

    // transition will directly manage the mix
    if (in_transition()) {
      return;
    }

    // if disarmed or landed prioritise throttle
    if (!motors->armed()) {
        attitude_control->set_throttle_mix_min();
        return;
    }

    if (plane.control_mode->is_vtol_man_throttle()) {
        // manual throttle
        if ((plane.get_throttle_input() <= 0) && (air_mode == AirMode::OFF)) {
            attitude_control->set_throttle_mix_min();
        } else {
            attitude_control->set_throttle_mix_man();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
        bool large_angle_request = (norm(angle_target.x, angle_target.y) > LAND_CHECK_LARGE_ANGLE_CD);

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // check for large acceleration - falling or high turbulence
        bool accel_moving = (throttle_mix_accel_ef_filter.get().length() > LAND_CHECK_ACCEL_MOVING);

        // check for requested descent
        bool descent_not_demanded = pos_control->get_vel_desired_cms().z >= 0.0f;

        if (large_angle_request || large_angle_error || accel_moving || descent_not_demanded) {
            attitude_control->set_throttle_mix_max(1.0);
        } else {
            attitude_control->set_throttle_mix_min();
        }
    }
}

/*
  see if we are in the approach phase of a VTOL landing
 */
bool QuadPlane::in_vtol_land_approach(void) const
{
    if (plane.control_mode == &plane.mode_qrtl &&
        poscontrol.get_state() <= QPOS_POSITION2) {
        return true;
    }
    if (in_vtol_auto()) {
        if (is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
            (poscontrol.get_state() == QPOS_APPROACH ||
             poscontrol.get_state() == QPOS_AIRBRAKE ||
             poscontrol.get_state() == QPOS_POSITION1 ||
             poscontrol.get_state() == QPOS_POSITION2)) {
            return true;
        }
    }
    return false;
}

/*
  see if we are in the descent phase of a VTOL landing
 */
bool QuadPlane::in_vtol_land_descent(void) const
{
    if (in_vtol_auto() && is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
        (poscontrol.get_state() == QPOS_LAND_DESCEND || poscontrol.get_state() == QPOS_LAND_FINAL)) {
        return true;
    }
    return false;
}

/*
  see if we are in the final phase of a VTOL landing
 */
bool QuadPlane::in_vtol_land_final(void) const
{
    return in_vtol_land_descent() && poscontrol.get_state() == QPOS_LAND_FINAL;
}

/*
  see if we are in any of the phases of a VTOL landing
 */
bool QuadPlane::in_vtol_land_sequence(void) const
{
    return in_vtol_land_approach() || in_vtol_land_descent() || in_vtol_land_final();
}

/*
  see if we are in the VTOL position control phase of a landing
 */
bool QuadPlane::in_vtol_land_poscontrol(void) const
{
    if (in_vtol_auto() && is_vtol_land(plane.mission.get_current_nav_cmd().id) &&
        poscontrol.get_state() >= QPOS_POSITION1) {
        return true;
    }
    return false;
}

// return true if we should show VTOL view
bool QuadPlane::show_vtol_view() const
{
    bool show_vtol = in_vtol_mode();

    if (is_tailsitter()) {
        if (show_vtol && (transition_state == TRANSITION_ANGLE_WAIT_VTOL)) {
            // in a vtol mode but still transitioning from forward flight
            return false;
        }

        if (!show_vtol && (transition_state == TRANSITION_ANGLE_WAIT_FW)) {
            // not in VTOL mode but still transitioning from VTOL
            return true;
        }
    }
    if (!show_vtol && tilt.is_vectored && transition_state <= TRANSITION_TIMER) {
        // we use multirotor controls during fwd transition for
        // vectored yaw vehicles
        return true;
    }

    return show_vtol;
}

// return the PILOT_VELZ_MAX_DN value if non zero, otherwise returns the PILOT_VELZ_MAX value.
uint16_t QuadPlane::get_pilot_velocity_z_max_dn() const
{
    if (pilot_velocity_z_max_dn == 0) {
        return abs(pilot_velocity_z_max_up);
   }
    return abs(pilot_velocity_z_max_dn);
}

/*
  should we use the fixed wing attitude controllers for roll/pitch control
 */
bool QuadPlane::use_fw_attitude_controllers(void) const
{
    if (available() &&
        motors->armed() &&
        motors->get_desired_spool_state() >= AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED &&
        in_vtol_mode() &&
        !is_tailsitter() &&
        poscontrol.get_state() != QPOS_AIRBRAKE) {
        // we want the desired rates for fixed wing slaved to the
        // multicopter rates
        return false;
    }
    return true;
}

/*
  calculate our closing velocity vector on the landing point. In the
  future this will take account of the landing point having a
  velocity
*/
Vector2f QuadPlane::landing_closing_velocity()
{
    Vector2f vel = ahrs.groundspeed_vector();
    return vel;
}

/*
  calculate our desired closing velocity vector on the landing point.
*/
Vector2f QuadPlane::landing_desired_closing_velocity()
{
    if (poscontrol.get_state() >= QPOS_LAND_DESCEND) {
        return Vector2f(0,0);
    }
    const Vector2f diff_wp = plane.current_loc.get_distance_NE(plane.next_WP_loc);
    float dist = diff_wp.length();
    if (dist < 1) {
        return Vector2f(0,0);
    }

    // base target speed based on sqrt of distance
    float target_speed = safe_sqrt(2*transition_decel*dist);
    Vector2f target_speed_xy = diff_wp.normalized() * target_speed;

    return target_speed_xy;
}

/*
  get target airspeed for landing, for use by TECS
*/
float QuadPlane::get_land_airspeed(void)
{
    if (poscontrol.get_state() == QPOS_APPROACH ||
        plane.control_mode == &plane.mode_rtl) {
        float land_airspeed = plane.SpdHgt_Controller->get_land_airspeed();
        if (!is_positive(land_airspeed)) {
            land_airspeed = plane.aparm.airspeed_cruise_cm * 0.01;
        }
        float cruise_airspeed = plane.aparm.airspeed_cruise_cm * 0.01;
        float time_to_landing = plane.auto_state.wp_distance / MAX(land_airspeed, 5);
        /*
          slow down to landing approach speed as we get closer to landing
         */
        land_airspeed = linear_interpolate(land_airspeed, cruise_airspeed,
                                           time_to_landing,
                                           20, 60);
        return land_airspeed;
    }
    Vector2f vel = landing_desired_closing_velocity();

    const float eas2tas = plane.ahrs.get_EAS2TAS();
    const Vector3f wind = plane.ahrs.wind_estimate();
    vel.x -= wind.x;
    vel.y -= wind.y;
    vel /= eas2tas;
    return vel.length();
}

void QuadPlane::set_desired_spool_state(AP_Motors::DesiredSpoolState state)
{
    if (motors->get_desired_spool_state() != state) {
        motors->set_desired_spool_state(state);
    }
}

QuadPlane *QuadPlane::_singleton = nullptr;
