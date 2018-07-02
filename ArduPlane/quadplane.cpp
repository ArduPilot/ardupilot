#include "Plane.h"

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
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("VELZ_MAX", 18, QuadPlane, pilot_velocity_z_max, 250),
    
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
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 7:Tri, 10: TailSitter
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 46, QuadPlane, frame_class, 1),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X or V)
    // @Description: Controls motor mixing for multicopter component
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F
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
    // @Description: If this is set to 1 then an RTL will change to QRTL when within RTL_RADIUS meters of the RTL destination
    // @Values: 0:Disabled,1:Enabled
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
    // @Description: This is used to calibrate the throttle range of the VTOL motors. Please read http://ardupilot.org/plane/docs/quadplane-esc-calibration.html before using. This parameter is automatically set back to 0 on every boot. This parameter only takes effect in QSTABILIZE mode. When set to 1 the output of all motors will come directly from the throttle stick when armed, and will be zero when disarmed. When set to 2 the output of all motors will be maximum when armed and zero when disarmed. Make sure you remove all properllers before using.
    // @Values: 0:Disabled,1:ThrottleInput,2:FullInput
    // @User: Standard
    AP_GROUPINFO("ESC_CAL", 42, QuadPlane, esc_calibration,  0),

    // @Param: VFWD_ALT
    // @DisplayName: Forward velocity alt cutoff
    // @Description: Controls altitude to disable forward velocity assist when below this relative altitude. This is useful to keep the forward velocity propeller from hitting the ground. Rangefinder height data is incorporated when available.
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
    // @Description: This is the angular error in attitude beyond which the quadplane VTOL motors will provide stability assistance. This will only be used if Q_ASSIST_SPEED is also non-zero. Assistance will be given if the attitude is outside the normal attitude limits by at least 5 degrees and the angular error in roll or pitch is greater than this angle for at least 1 second. Set to zero to disable angle assistance.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ANGLE", 45, QuadPlane, assist_angle, 30),

    // @Param: TILT_TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw
    AP_GROUPINFO("TILT_TYPE", 47, QuadPlane, tilt.tilt_type, TILT_TYPE_CONTINUOUS),

    // @Param: TAILSIT_ANGLE
    // @DisplayName: Tailsitter transition angle
    // @Description: This is the angle at which tailsitter aircraft will change from VTOL control to fixed wing control.
    // @Range: 5 80
    AP_GROUPINFO("TAILSIT_ANGLE", 48, QuadPlane, tailsitter.transition_angle, 45),

    // @Param: TILT_RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("TILT_RATE_DN", 49, QuadPlane, tilt.max_rate_down_dps, 0),
        
    // @Param: TAILSIT_INPUT
    // @DisplayName: Tailsitter input type
    // @Description: This controls whether stick input when hovering as a tailsitter follows the conventions for fixed wing hovering or multicopter hovering. When multicopter input is selected the roll stick will roll the aircraft in earth frame and yaw stick will yaw in earth frame. When using fixed wing input the roll and yaw sticks will control the aircraft in body frame.
    // @Values: 0:MultiCopterInput,1:FixedWingInput
    AP_GROUPINFO("TAILSIT_INPUT", 50, QuadPlane, tailsitter.input_type, TAILSITTER_INPUT_MULTICOPTER),

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
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output. This needs to be set for Q_TILT_TYPE=3 to enable vectored control for yaw of tricopter tilt quadplanes.
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
    // @Description: This provides a set of additional control options for quadplanes. LevelTransition means that the wings should be held level to within LEVEL_ROLL_LIMIT degrees during transition to fixed wing flight, and the vehicle will not use the vertical lift motors to climb during the transition. If AllowFWTakeoff bit is not set then fixed wing takeoff on quadplanes will instead perform a VTOL takeoff. If AllowFWLand bit is not set then fixed wing land on quadplanes will instead perform a VTOL land. If respect takeoff frame is not set the vehicle will interpret all takeoff waypoints as an altitude above the corrent position.
    // @Bitmask: 0:LevelTransition,1:AllowFWTakeoff,2:AllowFWLand,3:Respect takeoff frame types,4:Use a fixed wing approach for VTOL landings
    AP_GROUPINFO("OPTIONS", 58, QuadPlane, options, 0),

    AP_SUBGROUPEXTENSION("",59, QuadPlane, var_info2),

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

    // @Param: TAILSIT_THSCMX
    // @DisplayName: Maximum control throttle scaling value
    // @Description: Maximum value of throttle scaling for tailsitter velocity scaling, reduce this value to remove low thorottle D ossilaitons 
    // @Range: 1 5
    // @User: Standard
    AP_GROUPINFO("TAILSIT_THSCMX", 3, QuadPlane, tailsitter.throttle_scale_max, 5),

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
    // @Path: qautotune.cpp
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

    AP_GROUPEND
};

/*
  defaults for all quadplanes
 */
static const struct AP_Param::defaults_table_struct defaults_table[] = {
    { "Q_A_RAT_RLL_P",    0.25 },
    { "Q_A_RAT_RLL_I",    0.25 },
    { "Q_A_RAT_RLL_FILT", 10.0 },
    { "Q_A_RAT_PIT_P",    0.25 },
    { "Q_A_RAT_PIT_I",    0.25 },
    { "Q_A_RAT_PIT_FILT", 10.0 },
    { "Q_M_SPOOL_TIME",   0.25 },
    { "Q_LOIT_ANG_MAX",   15.0 },
    { "Q_LOIT_ACC_MAX",   250.0 },
    { "Q_LOIT_BRK_ACCEL", 50.0 },
    { "Q_LOIT_BRK_JERK",  250 },
    { "Q_LOIT_SPEED",     500 },
};

/*
  extra defaults for tailsitters
 */
static const struct AP_Param::defaults_table_struct defaults_table_tailsitter[] = {
    { "KFF_RDDRMIX",       0.02 },
    { "Q_A_RAT_PIT_FF",    0.2 },
    { "Q_A_RAT_YAW_FF",    0.2 },
    { "Q_A_RAT_YAW_I",    0.18 },
    { "LIM_PITCH_MAX",    3000 },
    { "LIM_PITCH_MIN",    -3000 },
    { "MIXING_GAIN",      1.0 },
    { "RUDD_DT_GAIN",      10 },
    { "Q_TRANSITION_MS",   2000 },
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
    { Parameters::k_param_quadplane, 400,  AP_PARAM_FLOAT, "Q_P_ACCZ_FILT"},   //  Q_AZ_FILT
    { Parameters::k_param_quadplane, 464,  AP_PARAM_FLOAT, "Q_P_ACCZ_FF"},     //  Q_AZ_FF
    { Parameters::k_param_quadplane, 276,  AP_PARAM_FLOAT, "Q_LOIT_SPEED"},    //  Q_WP_LOIT_SPEED
    { Parameters::k_param_quadplane, 468,  AP_PARAM_FLOAT, "Q_LOIT_BRK_JERK" },//  Q_WP_LOIT_JERK
    { Parameters::k_param_quadplane, 532,  AP_PARAM_FLOAT, "Q_LOIT_ACC_MAX" }, //  Q_WP_LOIT_MAXA
    { Parameters::k_param_quadplane, 596,  AP_PARAM_FLOAT, "Q_LOIT_BRK_ACCEL" },// Q_WP_LOIT_MINA
};


QuadPlane::QuadPlane(AP_AHRS_NavEKF &_ahrs) :
    ahrs(_ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);
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

    enum AP_Motors::motor_frame_class motor_class;
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
        gcs().send_text(MAV_SEVERITY_INFO, "Not enough memory for quadplane");
        goto failed;
    }

    /*
      dynamically allocate the key objects for quadplane. This ensures
      that the objects don't affect the vehicle unless enabled and
      also saves memory when not in use
     */
    motor_class = (enum AP_Motors::motor_frame_class)frame_class.get();
    switch (motor_class) {
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
        hal.console->printf("Unknown frame class %u - using QUAD\n", (unsigned)frame_class.get());
        frame_class.set(AP_Motors::MOTOR_FRAME_QUAD);
        setup_default_channels(4);
        break;
    }

    switch (motor_class) {
    case AP_Motors::MOTOR_FRAME_TRI:
        motors = new AP_MotorsTri(plane.scheduler.get_loop_rate_hz(), rc_speed);
        motors_var_info = AP_MotorsTri::var_info;
        break;
    case AP_Motors::MOTOR_FRAME_TAILSITTER:
        motors = new AP_MotorsTailsitter(plane.scheduler.get_loop_rate_hz(), rc_speed);
        motors_var_info = AP_MotorsTailsitter::var_info;
        rotation = ROTATION_PITCH_90;
        break;
    default:
        motors = new AP_MotorsMatrix(plane.scheduler.get_loop_rate_hz(), rc_speed);
        motors_var_info = AP_MotorsMatrix::var_info;
        break;
    }
    const static char *strUnableToAllocate = "Unable to allocate";
    if (!motors) {
        hal.console->printf("%s motors\n", strUnableToAllocate);
        goto failed;
    }

    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    // create the attitude view used by the VTOL code
    ahrs_view = ahrs.create_view(rotation, ahrs_trim_pitch);
    if (ahrs_view == nullptr) {
        goto failed;
    }

    attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors, loop_delta_t);
    if (!attitude_control) {
        hal.console->printf("%s attitude_control\n", strUnableToAllocate);
        goto failed;
    }
    AP_Param::load_object_from_eeprom(attitude_control, attitude_control->var_info);
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control);
    if (!pos_control) {
        hal.console->printf("%s pos_control\n", strUnableToAllocate);
        goto failed;
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);
    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (!wp_nav) {
        hal.console->printf("%s wp_nav\n", strUnableToAllocate);
        goto failed;
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (!loiter_nav) {
        hal.console->printf("%s loiter_nav\n", strUnableToAllocate);
        goto failed;
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

    motors->init((AP_Motors::motor_frame_class)frame_class.get(), (AP_Motors::motor_frame_type)frame_type.get());
    motors->set_throttle_range(thr_min_pwm, thr_max_pwm);
    motors->set_update_rate(rc_speed);
    motors->set_interlock(true);
    pos_control->set_dt(loop_delta_t);
    attitude_control->parameter_sanity_check();

    // setup the trim of any motors used by AP_Motors so px4io
    // failsafe will disable motors
    for (uint8_t i=0; i<8; i++) {
        SRV_Channel::Aux_servo_function_t func = SRV_Channels::get_motor_function(i);
        SRV_Channels::set_failsafe_pwm(func, thr_min_pwm);
    }

#if HAVE_PX4_MIXER
    // redo failsafe mixing on px4
    plane.setup_failsafe_mixing();
#endif
    
    transition_state = TRANSITION_DONE;

    if (tilt.tilt_mask != 0 && tilt.tilt_type == TILT_TYPE_VECTORED_YAW) {
        // setup tilt servos for vectored yaw
        SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
        SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);
    }

    
    setup_defaults();

    AP_Param::convert_old_parameters(&q_conversion_table[0], ARRAY_SIZE(q_conversion_table));
    
    gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane initialised");
    initialised = true;
    return true;
    
failed:
    initialised = false;
    enable.set(0);
    gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane setup failed");
    return false;
}

/*
  setup default parameters from defaults_table
 */
void QuadPlane::setup_defaults(void)
{
    AP_Param::set_defaults_from_table(defaults_table, ARRAY_SIZE(defaults_table));

    enum AP_Motors::motor_frame_class motor_class;
    motor_class = (enum AP_Motors::motor_frame_class)frame_class.get();
    if (motor_class == AP_Motors::MOTOR_FRAME_TAILSITTER) {
        AP_Param::set_defaults_from_table(defaults_table_tailsitter, ARRAY_SIZE(defaults_table_tailsitter));
    }
    
    // reset ESC calibration
    if (esc_calibration != 0) {
        esc_calibration.set_and_save(0);
    }
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
  ask the multicopter attitude control to match the roll and pitch rates being demanded by the
  fixed wing controller if not in a pure VTOL mode
 */
void QuadPlane::multicopter_attitude_rate_update(float yaw_rate_cds)
{
    check_attitude_relax();

    if (in_vtol_mode() || is_tailsitter()) {
        // use euler angle attitude control
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      yaw_rate_cds);
    } else {
        // use the fixed wing desired rates
        float roll_rate = plane.rollController.get_pid_info().desired;
        float pitch_rate = plane.pitchController.get_pid_info().desired;
        attitude_control->input_rate_bf_roll_pitch_yaw_2(roll_rate*100.0f, pitch_rate*100.0f, yaw_rate_cds);
    }
}

// hold in stabilize with given throttle
void QuadPlane::hold_stabilize(float throttle_in)
{    
    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds());

    if (throttle_in <= 0) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        if (is_tailsitter()) {
            // always stabilize with tailsitters so we can do belly takeoffs
            attitude_control->set_throttle_out(0, true, 0);
        } else {
            attitude_control->set_throttle_out_unstabilized(0, true, 0);
        }
    } else {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        attitude_control->set_throttle_out(throttle_in, true, 0);
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
    float pilot_throttle_scaled = plane.get_throttle_input() / 100.0f;
    hold_stabilize(pilot_throttle_scaled);

}

// run the multicopter Z controller
void QuadPlane::run_z_controller(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_pidz_active_ms > 2000) {
        // set alt target to current height on transition. This
        // starts the Z controller off with the right values
        gcs().send_text(MAV_SEVERITY_INFO, "Reset alt target to %.1f", (double)inertial_nav.get_altitude() / 100);
        set_alt_target_current();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());

        // initialize vertical speeds and leash lengths
        pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
        pos_control->set_max_accel_z(pilot_accel_z);
        
        // it has been two seconds since we last ran the Z
        // controller. We need to assume the integrator may be way off

        // the base throttle we start at is the current throttle we are using
        float base_throttle = constrain_float(motors->get_throttle() - motors->get_throttle_hover(), -1, 1) * 1000;
        pos_control->get_accel_z_pid().set_integrator(base_throttle);

        last_pidz_init_ms = now;
    }
    last_pidz_active_ms = now;
    pos_control->update_z_controller();    
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
        attitude_control->relax_attitude_controllers();
    }
    last_att_control_ms = now;
}

// init quadplane hover mode 
void QuadPlane::init_hover(void)
{
    // initialize vertical speeds and leash lengths
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    set_alt_target_current();
    pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());

    init_throttle_wait();
}

/*
  check for an EKF yaw reset
 */
void QuadPlane::check_yaw_reset(void)
{
    float yaw_angle_change_rad = 0.0f;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control->inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "EKF yaw reset %.2f", (double)degrees(yaw_angle_change_rad));
    }
}

/*
  hold hover with target climb rate
 */
void QuadPlane::hold_hover(float target_climb_rate)
{
    // motors use full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds());

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, plane.G_Dt, false);
    run_z_controller();
}

/*
  control QHOVER mode
 */
void QuadPlane::control_hover(void)
{
    if (throttle_wait) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0, true, 0);
        pos_control->relax_alt_hold_controllers(0);
    } else {
        hold_hover(get_pilot_desired_climb_rate_cms());
    }
}

void QuadPlane::init_loiter(void)
{
    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    set_alt_target_current();
    pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());

    init_throttle_wait();

    // remember initial pitch
    loiter_initial_pitch_cd = MAX(plane.ahrs.pitch_sensor, 0);

    // prevent re-init of target position
    last_loiter_ms = AP_HAL::millis();
}

void QuadPlane::init_land(void)
{
    init_loiter();
    throttle_wait = false;
    poscontrol.state = QPOS_LAND_DESCEND;
    landing_detect.lower_limit_start_ms = 0;
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
    if (motors->get_throttle() > 0.01f) {
        // if we are demanding more than 1% throttle then don't consider aircraft landed
        return true;
    }
    if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        return true;
    }
    if (plane.control_mode == &plane.mode_qstabilize || plane.control_mode == &plane.mode_qhover || plane.control_mode == &plane.mode_qloiter || plane.control_mode == &plane.mode_qautotune) {
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
    float ret = linear_interpolate(land_speed_cms, wp_nav->get_speed_down(),
                                   height_above_ground,
                                   land_final_alt, land_final_alt+6);
    return ret;
}


// run quadplane loiter controller
void QuadPlane::control_loiter()
{
    if (throttle_wait) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0, true, 0);
        pos_control->relax_alt_hold_controllers(0);
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        return;
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
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // process pilot's roll and pitch input
    loiter_nav->set_pilot_desired_acceleration(plane.channel_roll->get_control_in(),
                                               plane.channel_pitch->get_control_in(),
                                               plane.G_Dt);

    // run loiter controller
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
            pos_control->set_limit_accel_xy();            
        }
    }
    
    
    // call attitude controller with conservative smoothing gain of 4.0f
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  get_desired_yaw_rate_cds());

    if (plane.control_mode == &plane.mode_qland) {
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        if (height_above_ground < land_final_alt && poscontrol.state < QPOS_LAND_FINAL) {
            poscontrol.state = QPOS_LAND_FINAL;
            // cut IC engine if enabled
            if (land_icengine_cut != 0) {
                plane.g2.ice_control.engine_control(0, 0, 0);
            }
        }
        float descent_rate = (poscontrol.state == QPOS_LAND_FINAL)? land_speed_cms:landing_descent_rate_cms(height_above_ground);
        pos_control->set_alt_target_from_climb_rate(-descent_rate, plane.G_Dt, true);
        check_land_complete();
    } else if (plane.control_mode == &plane.mode_guided && guided_takeoff) {
        pos_control->set_alt_target_from_climb_rate_ff(0, plane.G_Dt, false);
    } else {
        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(get_pilot_desired_climb_rate_cms(), plane.G_Dt, false);
    }
    run_z_controller();
}

/*
  get pilot input yaw rate in cd/s
 */
float QuadPlane::get_pilot_input_yaw_rate_cds(void) const
{
    if (plane.get_throttle_input() <= 0 && !plane.auto_throttle_mode) {
        // the user may be trying to disarm
        return 0;
    }

    // add in rudder input
    return plane.channel_rudder->get_control_in() * yaw_rate_max / 45;
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
    if (plane.get_throttle_input() <= 0 && !plane.auto_throttle_mode) {
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
    return pilot_velocity_z_max * plane.channel_throttle->pwm_to_angle_dz_trim(dead_zone, trim) / 100.0f;
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
    if (plane.auto_throttle_mode) {
        // use altitude_error_cm, spread over 10s interval
        climb_rate = plane.altitude_error_cm * 0.1f;
    } else {
        // otherwise estimate from pilot input
        climb_rate = plane.g.flybywire_climb_rate * (plane.nav_pitch_cd/(float)plane.aparm.pitch_limit_max_cd);
        climb_rate *= plane.get_throttle_input();
    }
    climb_rate = constrain_float(climb_rate, -wp_nav->get_speed_down(), wp_nav->get_speed_up());

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
    if (!ahrs.airspeed_estimate(&aspeed) || aspeed < plane.aparm.airspeed_min) {
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
bool QuadPlane::assistance_needed(float aspeed)
{
    if (assist_speed <= 0) {
        // assistance disabled
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }
    if (aspeed < assist_speed) {
        // assistance due to Q_ASSIST_SPEED
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return true;
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
    const uint32_t now = AP_HAL::millis();
    if (angle_error_start_ms == 0) {
        angle_error_start_ms = now;
    }
    bool ret = (now - angle_error_start_ms) >= 1000U;
    if (ret && !in_angle_assist) {
        in_angle_assist = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Angle assist r=%d p=%d",
                                         (int)(ahrs.roll_sensor/100),
                                         (int)(ahrs.pitch_sensor/100));
    }
    return ret;
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
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
            motors->output();
        }
        transition_state = TRANSITION_DONE;
        assisted_flight = false;
        return;
    }

    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(&aspeed);

    // tailsitters use angle wait, not airspeed wait
    if (is_tailsitter() && transition_state == TRANSITION_AIRSPEED_WAIT) {
        transition_state = TRANSITION_ANGLE_WAIT_FW;
    }
    
    /*
      see if we should provide some assistance
     */
    if (have_airspeed &&
        assistance_needed(aspeed) &&
        !is_tailsitter() &&
        hal.util->get_soft_armed() &&
        ((plane.auto_throttle_mode && !plane.throttle_suppressed) ||
         plane.get_throttle_input()>0 ||
         plane.is_flying())) {
        // the quad should provide some assistance to the plane
        if (transition_state != TRANSITION_AIRSPEED_WAIT) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition started airspeed %.1f", (double)aspeed);
        }
        transition_state = TRANSITION_AIRSPEED_WAIT;
        transition_start_ms = millis();
        assisted_flight = true;
    } else {
        assisted_flight = false;
    }

    if (is_tailsitter()) {
        if (transition_state == TRANSITION_ANGLE_WAIT_FW &&
            tailsitter_transition_fw_complete()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
            transition_state = TRANSITION_DONE;
        }
    }
    
    // if rotors are fully forward then we are not transitioning,
    // unless we are waiting for airspeed to increase (in which case
    // the tilt will decrease rapidly)
    if (tiltrotor_fully_fwd() && transition_state != TRANSITION_AIRSPEED_WAIT) {
        transition_state = TRANSITION_DONE;
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
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // we hold in hover until the required airspeed is reached
        if (transition_start_ms == 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed wait");
            transition_start_ms = millis();
        }

        if (have_airspeed && aspeed > plane.aparm.airspeed_min && !assisted_flight) {
            transition_start_ms = millis();
            transition_state = TRANSITION_TIMER;
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed reached %.1f", (double)aspeed);
        }
        assisted_flight = true;

        // do not allow a climb on the quad motors during transition
        // a climb would add load to the airframe, and prolongs the
        // transition
        float climb_rate_cms = assist_climb_rate_cms();
        if (options & OPTION_LEVEL_TRANSITION) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
        }
        hold_hover(climb_rate_cms);

        // set desired yaw to current yaw in both desired angle and
        // rate request. This reduces wing twist in transition due to
        // multicopter yaw demands
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->rate_bf_yaw_target(ahrs.get_gyro().z);

        last_throttle = motors->get_throttle();

        // reset integrators while we are below target airspeed as we
        // may build up too much while still primarily under
        // multicopter control
        plane.pitchController.reset_I();
        plane.rollController.reset_I();

        // give full authority to attitude control
        attitude_control->set_throttle_mix_max();
        break;
    }
        
    case TRANSITION_TIMER: {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // after airspeed is reached we degrade throttle over the
        // transition time, but continue to stabilize
        if (millis() - transition_start_ms > (unsigned)transition_time_ms) {
            transition_state = TRANSITION_DONE;
            gcs().send_text(MAV_SEVERITY_INFO, "Transition done");
        }
        float trans_time_ms = (float)transition_time_ms.get();
        float transition_scale = (trans_time_ms - (millis() - transition_start_ms)) / trans_time_ms;
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
        // control surfaces at this stage
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->rate_bf_yaw_target(ahrs.get_gyro().z);
        break;
    }

    case TRANSITION_ANGLE_WAIT_FW: {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        assisted_flight = true;
        // calculate transition rate in degrees per
        // millisecond. Assume we want to get to the transition angle
        // in half the transition time
        float transition_rate = tailsitter.transition_angle / float(transition_time_ms/2);
        uint32_t dt = AP_HAL::millis() - transition_start_ms;
        plane.nav_pitch_cd = constrain_float((-transition_rate * dt)*100, -8500, 0);
        plane.nav_roll_cd = 0;
        check_attitude_relax();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      0);
        attitude_control->set_throttle_out(motors->get_throttle_hover(), true, 0);
        break;
    }

    case TRANSITION_ANGLE_WAIT_VTOL:
        // nothing to do, this is handled in the fw attitude controller
        return;

    case TRANSITION_DONE:
        if (!tilt.motors_active && !is_tailsitter()) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
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
    
    if (plane.afs.should_crash_vehicle()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        motors->output();
        return;
    }
    
    if (motor_test.running) {
        motor_test_output();
        return;
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
        pos_control->relax_alt_hold_controllers(0);
    }
    
    check_yaw_reset();
    
    if (!in_vtol_mode()) {
        update_transition();
    } else {
        const uint32_t now = AP_HAL::millis();

        assisted_flight = false;
        
        // give full authority to attitude control
        attitude_control->set_throttle_mix_max();

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
        } else if (is_tailsitter() &&
                   transition_state == TRANSITION_ANGLE_WAIT_VTOL) {
            if (tailsitter_transition_vtol_complete()) {
                /*
                  we have completed transition to VTOL as a tailsitter,
                  setup for the back transition when needed
                */
                gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL done");
                transition_state = TRANSITION_ANGLE_WAIT_FW;
                transition_start_ms = now;
            }
        } else {
            /*
              setup the transition state appropriately for next time we go into a non-VTOL mode
            */
            transition_start_ms = 0;
            if (throttle_wait && !plane.is_flying()) {
                transition_state = TRANSITION_DONE;
            } else if (is_tailsitter()) {
                /*
                  setup for the transition back to fixed wing for later
                 */
                transition_state = TRANSITION_ANGLE_WAIT_FW;
                transition_start_ms = now;
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
}

/*
  see if motors should be shutdown. If they should be then change AP_Motors state to 
  AP_Motors::DESIRED_SHUT_DOWN

  This is a safety check to prevent accidental motor runs on the
  ground, such as if RC fails and QRTL is started
 */
void QuadPlane::check_throttle_suppression(void)
{
    // if the motors have been running in the last 2 seconds then
    // allow them to run now
    if (AP_HAL::millis() - last_motors_active_ms < 2000) {
        return;
    }

    // see if motors are already disabled
    if (motors->get_desired_spool_state() < AP_Motors::DESIRED_THROTTLE_UNLIMITED) {
        return;
    }

    // if the users throttle is above zero then allow motors to run
    if (plane.get_throttle_input() != 0) {
        return;
    }

    // if we are in a fixed wing auto throttle mode and we have
    // unsuppressed the throttle then allow motors to run
    if (plane.auto_throttle_mode && !plane.throttle_suppressed) {
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
    motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    motors->set_throttle(0);
    last_motors_active_ms = 0;
}

/*
  output motors and do any copter needed
 */
void QuadPlane::motors_output(bool run_rate_controller)
{
    if (run_rate_controller) {
        attitude_control->rate_controller_run();
    }

    if (!hal.util->get_soft_armed() || plane.afs.should_crash_vehicle()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        motors->output();
        return;
    }
    if (esc_calibration && AP_Notify::flags.esc_calibration && plane.control_mode == &plane.mode_qstabilize) {
        // output is direct from run_esc_calibration()
        return;
    }

    if (in_tailsitter_vtol_transition()) {
        /*
          don't run the motor outputs while in tailsitter->vtol
          transition. That is taken care of by the fixed wing
          stabilisation code
         */
        return;
    }

    // see if motors should be shut down
    check_throttle_suppression();
    
    motors->output();
    if (motors->armed() && motors->get_throttle() > 0) {
        plane.DataFlash.Log_Write_Rate(ahrs_view, *motors, *attitude_control, *pos_control);
        Log_Write_QControl_Tuning();
        const uint32_t now = AP_HAL::millis();
        if (now - last_ctrl_log_ms > 100) {
            attitude_control->control_monitor_log();
        }
    }

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
    case QAUTOTUNE:
        qautotune.run();
        break;
#endif
    default:
        break;
    }
    // we also stabilize using fixed wing surfaces
    float speed_scaler = plane.get_speed_scaler();
    plane.stabilize_roll(speed_scaler);
    plane.stabilize_pitch(speed_scaler);
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
        init_land();
        break;
    case Mode::Number::QRTL:
        init_qrtl();
        break;
    case Mode::Number::GUIDED:
        guided_takeoff = false;
        break;
#if QAUTOTUNE_ENABLED
    case QAUTOTUNE:
        return qautotune.init();
#endif
    default:
        break;
    }
    return true;
}

/*
  handle a MAVLink DO_VTOL_TRANSITION
 */
bool QuadPlane::handle_do_vtol_transition(enum MAV_VTOL_STATE state)
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
    if (!enable) {
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
  are we in a VTOL mode?
 */
bool QuadPlane::in_vtol_mode(void) const
{
    if (!enable) {
        return false;
    }
    return (plane.control_mode == &plane.mode_qstabilize ||
            plane.control_mode == &plane.mode_qhover ||
            plane.control_mode == &plane.mode_qloiter ||
            plane.control_mode == &plane.mode_qland ||
            plane.control_mode == &plane.mode_qrtl ||
            plane.control_mode == &plane.mode_qautotune ||
            ((plane.control_mode == &plane.mode_guided || plane.control_mode == &plane.mode_avoidADSB) && plane.auto_state.vtol_loiter) ||
            in_vtol_auto());
}


/*
  main landing controller. Used for landing and RTL.
 */
void QuadPlane::vtol_position_controller(void)
{
    if (!setup()) {
        return;
    }

    setup_target_position();

    const Location &loc = plane.next_WP_loc;

    check_attitude_relax();

    // horizontal position control
    switch (poscontrol.state) {

    case QPOS_POSITION1: {
        Vector2f diff_wp = location_diff(plane.current_loc, loc);
        float distance = diff_wp.length();

        if (poscontrol.speed_scale <= 0) {
            // initialise scaling so we start off targeting our
            // current linear speed towards the target. If this is
            // less than the wpnav speed then the wpnav speed is used
            // land_speed_scale is then used to linearly change
            // velocity as we approach the waypoint, aiming for zero
            // speed at the waypoint
            Vector2f groundspeed = ahrs.groundspeed_vector();
            float speed_towards_target = distance>1?(diff_wp.normalized() * groundspeed):0;
            // setup land_speed_scale so at current distance we
            // maintain speed towards target, and slow down as we
            // approach

            // max_speed will control how fast we will fly. It will always decrease
            poscontrol.max_speed = MAX(speed_towards_target, wp_nav->get_speed_xy() * 0.01);
            poscontrol.speed_scale = poscontrol.max_speed / MAX(distance, 1);
        }

        // run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, loc);

        /*
          calculate target velocity, not dropping it below 2m/s
         */
        const float final_speed = 2.0f;
        Vector2f target_speed_xy = diff_wp * poscontrol.speed_scale;
        float target_speed = target_speed_xy.length();
        if (distance < 1) {
            // prevent numerical error before switching to POSITION2
            target_speed_xy(0.1, 0.1);
        }
        if (target_speed < final_speed) {
            // until we enter the loiter we always aim for at least 2m/s
            target_speed_xy = target_speed_xy.normalized() * final_speed;
            poscontrol.max_speed = final_speed;
        } else if (target_speed > poscontrol.max_speed) {
            // we never speed up during landing approaches
            target_speed_xy = target_speed_xy.normalized() * poscontrol.max_speed;
        } else {
            poscontrol.max_speed = target_speed;
        }
        pos_control->set_desired_velocity_xy(target_speed_xy.x*100,
                                             target_speed_xy.y*100);

        // reset position controller xy target to current position
        // because we only want velocity control (no position control)
        const Vector3f& curr_pos = inertial_nav.get_position();
        pos_control->set_xy_target(curr_pos.x, curr_pos.y);
        pos_control->set_desired_accel_xy(0.0f,0.0f);

        // run horizontal velocity controller
        pos_control->update_vel_controller_xy();

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll();
        plane.nav_pitch_cd = pos_control->get_pitch();

        /*
          limit the pitch down with an expanding envelope. This
          prevents the velocity controller demanding nose down during
          the initial slowdown if the target velocity curve is higher
          than the actual velocity curve (for a high drag
          aircraft). Nose down will cause a lot of downforce on the
          wings which will draw a lot of current and also cause the
          aircraft to lose altitude rapidly.
         */
        float pitch_limit_cd = linear_interpolate(-300, plane.aparm.pitch_limit_min_cd,
                                                  plane.auto_state.wp_proportion, 0, 1);
        if (plane.nav_pitch_cd < pitch_limit_cd) {
            plane.nav_pitch_cd = pitch_limit_cd;
            // tell the pos controller we have limited the pitch to
            // stop integrator buildup
            pos_control->set_limit_accel_xy();
        }
        
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                             plane.nav_pitch_cd,
                                                                             desired_auto_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        if (plane.auto_state.wp_proportion >= 1 ||
            plane.auto_state.wp_distance < 5) {
            poscontrol.state = QPOS_POSITION2;
            loiter_nav->clear_pilot_desired_acceleration();
            loiter_nav->init_target();
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position2 started v=%.1f d=%.1f",
                                    (double)ahrs.groundspeed(), (double)plane.auto_state.wp_distance);
        }
        break;
    }

    case QPOS_POSITION2:
    case QPOS_LAND_DESCEND:
        /*
          for final land repositioning and descent we run the position controller
         */

        // also run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, loc);
        FALLTHROUGH;

    case QPOS_LAND_FINAL:

        // set position controller desired velocity and acceleration to zero
        pos_control->set_desired_velocity_xy(0.0f,0.0f);
        pos_control->set_desired_accel_xy(0.0f,0.0f);

        // set position control target and update
        pos_control->set_xy_target(poscontrol.target.x, poscontrol.target.y);
        pos_control->update_xy_controller();

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll();
        plane.nav_pitch_cd = pos_control->get_pitch();

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
    switch (poscontrol.state) {
    case QPOS_POSITION1:
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
        if (plane.control_mode == &plane.mode_qrtl || plane.control_mode == &plane.mode_guided || vtol_loiter_auto) {
            plane.ahrs.get_position(plane.current_loc);
            float target_altitude = plane.next_WP_loc.alt;
            if (poscontrol.slow_descent) {
                // gradually descend as we approach target
                plane.auto_state.wp_proportion = location_path_proportion(plane.current_loc, 
                                                                          plane.prev_WP_loc, plane.next_WP_loc);
                target_altitude = linear_interpolate(plane.prev_WP_loc.alt,
                                                     plane.next_WP_loc.alt,
                                                     plane.auto_state.wp_proportion,
                                                     0, 1);
            }
            adjust_alt_target(target_altitude - plane.home.alt);
        } else {
            pos_control->set_alt_target_from_climb_rate(0, plane.G_Dt, false);
        }
        break;
    }

    case QPOS_LAND_DESCEND: {
        float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
        pos_control->set_alt_target_from_climb_rate(-landing_descent_rate_cms(height_above_ground),
                                                    plane.G_Dt, true);
        break;
    }

    case QPOS_LAND_FINAL:
        pos_control->set_alt_target_from_climb_rate(-land_speed_cms, plane.G_Dt, true);
        break;
        
    case QPOS_LAND_COMPLETE:
        break;
    }
    
    run_z_controller();
}


/*
  setup the target position based on plane.next_WP_loc
 */
void QuadPlane::setup_target_position(void)
{
    const Location &loc = plane.next_WP_loc;
    Location origin = inertial_nav.get_origin();
    Vector2f diff2d;

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    diff2d = location_diff(origin, loc);
    poscontrol.target.x = diff2d.x * 100;
    poscontrol.target.y = diff2d.y * 100;
    poscontrol.target.z = plane.next_WP_loc.alt - origin.alt;

    const uint32_t now = AP_HAL::millis();
    if (!locations_are_same(loc, last_auto_target) ||
        plane.next_WP_loc.alt != last_auto_target.alt ||
        now - last_loiter_ms > 500) {
        wp_nav->set_wp_destination(poscontrol.target);
        last_auto_target = loc;
    }
    last_loiter_ms = now;
    
    // setup vertical speed and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);
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
    pos_control->set_desired_velocity_xy(0.0f,0.0f);
    pos_control->set_desired_accel_xy(0.0f,0.0f);

    // set position control target and update
    pos_control->set_xy_target(poscontrol.target.x, poscontrol.target.y);
    pos_control->update_xy_controller();

    // nav roll and pitch are controller by position controller
    plane.nav_roll_cd = pos_control->get_roll();
    plane.nav_pitch_cd = pos_control->get_pitch();

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());

    pos_control->set_alt_target_from_climb_rate(wp_nav->get_speed_up(), plane.G_Dt, true);
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
    pos_control->set_alt_target_from_climb_rate_ff(assist_climb_rate_cms(), plane.G_Dt, false);
    run_z_controller();
}


/*
  handle auto-mode when auto_state.vtol_mode is true
 */
void QuadPlane::control_auto(const Location &loc)
{
    if (!setup()) {
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

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
    if (poscontrol.state >= QPOS_POSITION2) {
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
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    poscontrol.state = QPOS_POSITION1;
    poscontrol.speed_scale = 0;
    pos_control->set_desired_accel_xy(0.0f, 0.0f);
    pos_control->init_xy_controller();
}

/*
  start a VTOL takeoff
 */
bool QuadPlane::do_vtol_takeoff(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }

    plane.set_next_WP(cmd.content.location);
    if (options & OPTION_RESPECT_TAKEOFF_FRAME) {
        if (plane.current_loc.alt >= plane.next_WP_loc.alt) {
            // we are above the takeoff already, no need to do anything
            return false;
        }
    } else {
        plane.next_WP_loc.alt = plane.current_loc.alt + cmd.content.location.alt;
    }
    throttle_wait = false;

    // set target to current position
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    set_alt_target_current();
    pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    
    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
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
    pos_control->get_accel_z_pid().reset_I();
    pos_control->get_vel_xy_pid().reset_I();
    
    plane.set_next_WP(cmd.content.location);
    // initially aim for current altitude
    plane.next_WP_loc.alt = plane.current_loc.alt;
    poscontrol.state = QPOS_POSITION1;
    poscontrol.speed_scale = 0;
    pos_control->set_desired_accel_xy(0.0f, 0.0f);
    pos_control->init_xy_controller();

    throttle_wait = false;
    landing_detect.lower_limit_start_ms = 0;
    set_alt_target_current();

    plane.crash_state.is_crashed = false;
    
    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
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
    if (plane.current_loc.alt < plane.next_WP_loc.alt) {
        return false;
    }
    transition_state = is_tailsitter() ? TRANSITION_ANGLE_WAIT_FW : TRANSITION_AIRSPEED_WAIT;
    plane.TECS_controller.set_pitch_max_limit(transition_pitch_max);
    set_alt_target_current();

    plane.complete_auto_takeoff();
    
    return true;
}

/*
  check if a landing is complete
 */
void QuadPlane::check_land_complete(void)
{
    if (poscontrol.state != QPOS_LAND_FINAL) {
        // only apply to final landing phase
        return;
    }
    const uint32_t now = AP_HAL::millis();
    bool might_be_landed =  (landing_detect.lower_limit_start_ms != 0 &&
                             now - landing_detect.lower_limit_start_ms > 1000);
    if (!might_be_landed) {
        landing_detect.land_start_ms = 0;
        return;
    }
    float height = inertial_nav.get_altitude()*0.01f;
    if (landing_detect.land_start_ms == 0) {
        landing_detect.land_start_ms = now;
        landing_detect.vpos_start_m = height;
    }
    // we only consider the vehicle landed when the motors have been
    // at minimum for 5s and the vertical position estimate has not
    // changed by more than 20cm for 4s
    if (fabsf(height - landing_detect.vpos_start_m) > 0.2) {
        // height has changed, call off landing detection
        landing_detect.land_start_ms = 0;
        return;
    }
           
    if ((now - landing_detect.land_start_ms) < 4000 ||
        (now - landing_detect.lower_limit_start_ms) < 5000) {
        // not landed yet
        return;
    }
    landing_detect.land_start_ms = 0;
    // motors have been at zero for 5s, and we have had less than 0.3m
    // change in altitude for last 4s. We are landed.
    plane.disarm_motors();
    poscontrol.state = QPOS_LAND_COMPLETE;
    gcs().send_text(MAV_SEVERITY_INFO,"Land complete");
    // reload target airspeed which could have been modified by the mission
    plane.aparm.airspeed_cruise_cm.load();
}

/*
  check if a VTOL landing has completed
 */
bool QuadPlane::verify_vtol_land(void)
{
    if (!available()) {
        return true;
    }
    if (poscontrol.state == QPOS_POSITION2 &&
        plane.auto_state.wp_distance < 2) {
        poscontrol.state = QPOS_LAND_DESCEND;
        gcs().send_text(MAV_SEVERITY_INFO,"Land descend started");
        plane.set_next_WP(plane.next_WP_loc);
    }

    if (should_relax()) {
        loiter_nav->soften_for_landing();
    }

    // at land_final_alt begin final landing
    float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    if (poscontrol.state == QPOS_LAND_DESCEND && height_above_ground < land_final_alt) {
        poscontrol.state = QPOS_LAND_FINAL;

        // cut IC engine if enabled
        if (land_icengine_cut != 0) {
            plane.g2.ice_control.engine_control(0, 0, 0);
        }
        gcs().send_text(MAV_SEVERITY_INFO,"Land final started");
    }

    check_land_complete();
    return false;
}

// Write a control tuning packet
void QuadPlane::Log_Write_QControl_Tuning()
{
    float des_alt_m = 0.0f;
    int16_t target_climb_rate_cms = 0;
    if (plane.control_mode != QSTABILIZE) {
        des_alt_m = pos_control->get_alt_target() / 100.0f;
        target_climb_rate_cms = pos_control->get_vel_target_z();
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
    };
    plane.DataFlash.WriteBlock(&pkt, sizeof(pkt));

    // write multicopter position control message
    pos_control->write_log();
}


/*
  calculate the forward throttle percentage. The forward throttle can
  be used to assist with position hold and with landing approach. It
  reduces the need for down pitch which reduces load on the vertical
  lift motors.
 */
int8_t QuadPlane::forward_throttle_pct(void)
{
    /*
      in non-VTOL modes or modes without a velocity controller. We
      don't use it in QHOVER or QSTABILIZE as they are the primary
      recovery modes for a quadplane and need to be as simple as
      possible. They will drift with the wind
    */
    if (!in_vtol_mode() ||
        !motors->armed() ||
        vel_forward.gain <= 0 ||
        plane.control_mode == &plane.mode_qstabilize ||
        plane.control_mode == &plane.mode_qhover ||
        plane.control_mode == &plane.mode_qautotune) {
        return 0;
    }

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
    const Vector3f &desired_velocity_cms = pos_control->get_desired_velocity();
    Vector3f vel_ned;
    if (!plane.ahrs.get_velocity_NED(vel_ned)) {
        // we don't know our velocity? EKF must be pretty sick
        vel_forward.last_pct = 0;
        vel_forward.integrator = 0;
        return 0;
    }
    Vector3f vel_error_body = ahrs.get_rotation_body_to_ned().transposed() * ((desired_velocity_cms*0.01f) - vel_ned);

    // find component of velocity error in fwd body frame direction
    float fwd_vel_error = vel_error_body * Vector3f(1,0,0);

    // scale forward velocity error by maximum airspeed
    fwd_vel_error /= MAX(plane.aparm.airspeed_max, 5);

    // add in a component from our current pitch demand. This tends to
    // move us to zero pitch. Assume that LIM_PITCH would give us the
    // WP nav speed.
    fwd_vel_error -= (wp_nav->get_speed_xy() * 0.01f) * plane.nav_pitch_cd / (float)plane.aparm.pitch_limit_max_cd;

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
    
    // If we are below alt_cutoff then scale down the effect until it turns off at alt_cutoff and decay the integrator
    float alt_cutoff = MAX(0,vel_forward_alt_cutoff);
    float height_above_ground = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    vel_forward.last_pct = linear_interpolate(0, vel_forward.integrator,
                                   height_above_ground, alt_cutoff, alt_cutoff+2);
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
    poscontrol.state = QPOS_POSITION1;
    poscontrol.speed_scale = 0;
    guided_takeoff = false;
    setup_target_position();
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
}

/*
  update guided mode control
 */
void QuadPlane::guided_update(void)
{
    if (plane.control_mode == &plane.mode_guided && guided_takeoff && plane.current_loc.alt < plane.next_WP_loc.alt) {
        throttle_wait = false;
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
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
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
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
    return guided_mode != 0;
}

/*
  set altitude target to current altitude
 */
void QuadPlane::set_alt_target_current(void)
{
    pos_control->set_alt_target(inertial_nav.get_altitude());
}

/*
  adjust the altitude target to the given target, moving it slowly
 */
void QuadPlane::adjust_alt_target(float altitude_cm)
{
    float current_alt = inertial_nav.get_altitude();
    // don't let it get beyond 50cm from current altitude
    float target_cm = constrain_float(altitude_cm, current_alt-50, current_alt+50);
    pos_control->set_alt_target(target_cm);
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
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    guided_start();
    guided_takeoff = true;
    return true;
}

// return true if the wp_nav controller is being updated
bool QuadPlane::using_wp_nav(void) const
{
    return plane.control_mode == QLOITER || plane.control_mode == QLAND || plane.control_mode == QRTL;
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
