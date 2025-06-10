#include "AC_Autorotation.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

extern const AP_HAL::HAL& hal;

// Autorotation controller defaults
#define HEAD_SPEED_TARGET_RATIO  1.0    // Normalised target main rotor head speed
#define AP_ALPHA_TPP             20.0  // (deg) Maximum angle of the Tip Path Plane

const AP_Param::GroupInfo AC_Autorotation::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_Autorotation, _param_enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HS_P
    // @DisplayName: P gain for head speed controller
    // @Description: Increase value to increase sensitivity of head speed controller during autonomous autorotation.
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_hs, "HS_", 2, AC_Autorotation, AC_P),

    // @Param: HS_SET_PT
    // @DisplayName: Target Head Speed
    // @Description: The target head speed in RPM during autorotation. Start by setting to desired hover speed and tune from there as necessary.
    // @Units: RPM
    // @Range: 1000 2800
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HS_SET_PT", 3, AC_Autorotation, _param_head_speed_set_point, 1500),

    // @Param: FWD_SP_TARG
    // @DisplayName: Target Glide Body Frame Forward Speed
    // @Description: Target ground speed in cm/s for the autorotation controller to try and achieve/ maintain during the glide phase.
    // @Units: m/s
    // @Range: 8 20
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("FWD_SP_TARG", 4, AC_Autorotation, _param_target_speed, 11),

    // @Param: COL_FILT_E
    // @DisplayName: Entry Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter. For the entry phase. Acts as a following trim.  Must be higher than AROT_COL_FILT_G.
    // @Units: Hz
    // @Range: 0.2 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("COL_FILT_E", 5, AC_Autorotation, _param_col_entry_cutoff_freq, 0.7),

    // @Param: COL_FILT_G
    // @DisplayName: Glide Phase Collective Filter
    // @Description: Cut-off frequency for collective low pass filter. For the glide phase. Acts as a following trim.  Must be lower than AROT_COL_FILT_E.
    // @Units: Hz
    // @Range: 0.03 0.15
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("COL_FILT_G", 6, AC_Autorotation, _param_col_glide_cutoff_freq, 0.1),

    // @Param: XY_ACC_MAX
    // @DisplayName: Body Frame XY Acceleration Limit
    // @Description: Maximum body frame acceleration allowed in the in speed controller. This limit defines a circular constraint in accel. Minimum used is 0.5 m/s/s.
    // @Units: m/s/s
    // @Range: 0.5 8.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("XY_ACC_MAX", 7, AC_Autorotation, _param_accel_max, 5.0),

    // @Param: HS_SENSOR
    // @DisplayName: Main Rotor RPM Sensor
    // @Description: Allocate the RPM sensor instance to use for measuring head speed.
    // @Values: 0:RPM1,1:RPM2
    // @User: Standard
    AP_GROUPINFO("HS_SENSOR", 8, AC_Autorotation, _param_rpm_instance, 0),

    // @Param: ROT_SOL
    // @DisplayName: rotor solidity
    // @Description: helicopter specific main rotor solidity
    // @Range: 0.01 0.1
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("ROT_SOL", 10, AC_Autorotation, _param_solidity, 0.05),

    // @Param: ROT_DIAM
    // @DisplayName: rotor diameter
    // @Description: helicopter specific main rotor diameter
    // @Units: m
    // @Range: 0.001 0.01
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("ROT_DIAM", 11, AC_Autorotation, _param_diameter, 1.25),

    // @Param: TD_TIME
    // @DisplayName: Touchdown Time
    // @Description: Desired time for the touch down phase to last. Using the measured vertical velocity, this parameter is used to calculate the height that the vehicle will transition from the flare to the touch down phase. Minimum value used is 0.3 s.
    // @Units: s
    // @Range: 0.3 2.0
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("TD_TIME", 12, AC_Autorotation, _param_touchdown_time, 1.0),

    // @Param: FLR_MIN_HGT
    // @DisplayName: Minimum Flare Height
    // @Description: A safety cutoff feature to ensure that the calculated flare height cannot go below this value. This is the absolute minimum height that the flare will initiate. Ensure that this is appropriate for your vehicle.
    // @Units: m
    // @Range: 10 30
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLR_MIN_HGT", 13, AC_Autorotation, _flare_hgt.min_height, 6),

    // @Param: TD_MIN_HGT
    // @DisplayName: Minimum Touch Down Height
    // @Description: A safety cutoff feature to ensure that the calculated touch down height cannot go below this value. This is the absolute minimum height that the touch down will initiate. Touch down height must be less than flare height. Ensure that this is appropriate for your vehicle.
    // @Units: m
    // @Range: 0.5 10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TD_MIN_HGT", 14, AC_Autorotation, _touch_down_hgt.min_height, 0.5),

    // @Param: TD_COL_P
    // @DisplayName: P gain for vertical touchdown controller
    // @Description: Proportional term based on sink rate error
    // @Range: 0.3 1
    // @Increment: 0.01
    // @User: Advanced
    AP_SUBGROUPINFO(_p_col_td, "TD_COL_", 15, AC_Autorotation, AC_P),

    // @Param: FLR_MAX_HGT
    // @DisplayName: Maximum Flare Height
    // @Description: A safety cutoff feature to ensure that the calculated flare height cannot go above this value. This is the absolute maximum height above ground that the flare will initiate. Ensure that this is appropriate for your vehicle.
    // @Units: m
    // @Range: 10 30
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLR_MAX_HGT", 17, AC_Autorotation, _flare_hgt.max_height, 30),

    // @Param: NAV_MODE
    // @DisplayName: Autorotation Navigation Mode
    // @Description: Select the navigation mode to use when in the autonomous autorotation.
    // @Values: 0:Pilot lateral accel control,1:Turn into wind,2:Maintain velocity vector with cross track.
    // @User: Standard
    AP_GROUPINFO("NAV_MODE", 18, AC_Autorotation, _param_nav_mode, 0),

    AP_GROUPEND
};

// Constructor
AC_Autorotation::AC_Autorotation(AP_MotorsHeli*& motors, AC_AttitudeControl*& att_crtl, AP_InertialNav& inav, AC_PosControl*& pos_ctrl) :
    _motors_heli(motors),
    _attitude_control(att_crtl),
    _pos_control(pos_ctrl)
    {
    #if AP_RANGEFINDER_ENABLED
        _ground_surface = new AP_SurfaceDistance(ROTATION_PITCH_270, inav, 2U); // taking the 3rd instance of SurfaceDistance to not conflict with the first two already in copter
    #endif
        AP_Param::setup_object_defaults(this, var_info);
    }

void AC_Autorotation::init(void)
{
    const AP_AHRS &ahrs = AP::ahrs();

    // Calculate the direction vector for use in CROSS_TRACK navigation control
    Vector2f ground_speed_NE = ahrs.groundspeed_vector();

    // TODO add a constrain so that the track vector must be within +/- 90 of heading (heli going backwards aint pretty)

    if (ground_speed_NE.length() > MIN_MANOEUVERING_SPEED) {
        // We are moving with sufficient speed that the vehicle is moving "with purpose" and the
        // ground velocity vector can be used to define the track vector
        _track_vector = ground_speed_NE.normalized();

    } else {
        // We are better off using the vehicle's current heading to define the track vector.
        Vector2f unit_vec = {1.0, 0.0};
        _track_vector = ahrs.body_to_earth2D(unit_vec);
    }

    // Initialisation of head speed controller
    // Set initial collective position to be the current collective position for smooth init
    const float collective_out = _motors_heli->get_throttle();

    // Reset feed forward filter
    col_trim_lpf.reset(collective_out);

    // Configure the lagged velocity filter so that we can estimate if we are in steady conditions in the flare height calc
    _lagged_vel_z.set_cutoff_frequency(0.5);
    _lagged_vel_z.reset(get_ef_velocity_up());

    // Protect against divide by zero TODO: move this to an accessor function
    _param_head_speed_set_point.set(MAX(_param_head_speed_set_point, 500.0));

    // Set limits and initialise NE pos controller
    _pos_control->set_max_speed_accel_NE_cm(_param_target_speed.get()*100.0, _param_accel_max.get()*100.0);
    _pos_control->set_correction_speed_accel_NE_cm(_param_target_speed.get()*100.0, _param_accel_max.get()*100.0);
    _pos_control->set_pos_error_max_NE_cm(1000);
    _pos_control->init_NE_controller();

    // Reset the landed reason
    _landed_reason.min_speed = false;
    _landed_reason.land_col = false;
    _landed_reason.is_still = false;

    // Reset the guarded height measurements to ensure that they init to the min value
    _flare_hgt.reset();
    _touch_down_hgt.reset();

    // set the guarded heights
    _touch_down_hgt.max_height = _flare_hgt.min_height;

    // ensure the AP_SurfaceDistance object is enabled
#if AP_RANGEFINDER_ENABLED
    _ground_surface->enabled = true;
#endif
    _last_gnd_surf_update = 0;

    // Calc initial estimate of what height we think we should flare at
    initial_flare_hgt_estimate();
}

// Functions and config that are only to be done once at the beginning of the entry
void AC_Autorotation::init_entry(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AROT: Entry Phase");

    // Target head speed is set to rpm at initiation to prevent steps in controller
    if (!get_norm_head_speed(_target_head_speed)) {
        // Cannot get a valid RPM sensor reading so we default to not slewing the head speed target
        _target_head_speed = HEAD_SPEED_TARGET_RATIO;
    }

    // The rate to move the head speed from the current measurement to the target
    _hs_accel = (HEAD_SPEED_TARGET_RATIO - _target_head_speed) / (float(entry_time_ms)*1e-3);

    // Set collective following trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_entry_cutoff_freq.get());

    // Set collective low-pass cut off filter at 2 Hz
    _motors_heli->set_throttle_filter_cutoff(2.0);

    // Set speed target to maintain the current speed whilst we enter the autorotation
    _desired_vel = _param_target_speed.get();

    // When entering the autorotation we can have some roll target applied depending on what condition that
    // the position controller initialised on. If we are in the TURN_INTO_WIND or CROSS_TRACK navigation mode
    // we want to prevent changes in heading initially as the roll target may turn the aircraft in the wrong direction
    _heading_hold = Nav_Mode(_param_nav_mode.get()) == Nav_Mode::TURN_INTO_WIND ||
                    Nav_Mode(_param_nav_mode.get()) == Nav_Mode::CROSS_TRACK;
}

// The entry controller just a special case of the glide controller with head speed target slewing
void AC_Autorotation::run_entry(float pilot_norm_accel)
{
    // Slew the head speed target from the initial condition to the target head speed ratio for the glide
    const float max_change = _hs_accel * _dt;
    _target_head_speed = constrain_float(HEAD_SPEED_TARGET_RATIO, _target_head_speed - max_change, _target_head_speed + max_change);

    run_glide(pilot_norm_accel);
}

// Functions and config that are only to be done once at the beginning of the glide
void AC_Autorotation::init_glide(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AROT: Glide Phase");

    // Set collective following trim low pass filter cut off frequency
    col_trim_lpf.set_cutoff_frequency(_param_col_glide_cutoff_freq.get());

    // Ensure target head speed is set to setpoint, in case it didn't reach the target during entry
    _target_head_speed = HEAD_SPEED_TARGET_RATIO;

    // Ensure desired forward speed target is set to param value
    _desired_vel = _param_target_speed.get();

    // unlock any heading hold if we had one
    _heading_hold = false;
}

// Maintain head speed and forward speed as we glide to the ground
void AC_Autorotation::run_glide(float des_lat_accel_norm)
{
    update_headspeed_controller();

    update_navigation_controller(des_lat_accel_norm);

    // Keep flare altitude estimate up to date so state machine can decide when to flare
    update_flare_hgt();
}

// Functions and config that are only to be done once at the beginning of the flare
void AC_Autorotation::init_flare(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "AROT: Flare Phase");

    // Ensure target head speed, we may have skipped the glide phase if we did not have time to complete the
    // entry phase before hitting the flare height
    _target_head_speed = HEAD_SPEED_TARGET_RATIO;

    _flare_entry_fwd_speed = get_bf_speed_forward();

    // Lock heading for the flare if the navigation mode uses roll components in the calculations
    _heading_hold = Nav_Mode(_param_nav_mode.get()) == Nav_Mode::TURN_INTO_WIND ||
                    Nav_Mode(_param_nav_mode.get()) == Nav_Mode::CROSS_TRACK;
}

void AC_Autorotation::run_flare(float des_lat_accel_norm)
{
    // Update head speed/ collective controller
    update_headspeed_controller();

    // During the flare we want to linearly slow the aircraft to a stop as we
    // reach the touch down alt for the start of the touch down phase
    _desired_vel = linear_interpolate(0.0f, _flare_entry_fwd_speed, _hagl, _touch_down_hgt.get(), _flare_hgt.get());

    // Run forward speed controller
    update_navigation_controller(des_lat_accel_norm);
}

void AC_Autorotation::init_touchdown(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "AROT: Touch Down Phase");

    // store the descent speed and height at the start of the touch down
    _touchdown_init_climb_rate = get_ef_velocity_up();
    _touchdown_init_hgt = _hagl;

    // unlock any heading hold if we had one
    _heading_hold = false;

}

void AC_Autorotation::run_touchdown(float des_lat_accel_norm)
{
    const float climb_rate = get_ef_velocity_up();

    // Calc the desired climb rate based on the height above the ground, ideally we 
    // want to smoothly touch down with zero speed at the point we touch the ground
    float target_climb_rate = 0.0;
    if (is_positive(_hagl)) {
        target_climb_rate = linear_interpolate(0.0, _touchdown_init_climb_rate, _hagl, 0.0, _touchdown_init_hgt);
    }

    // Update collective following trim component
    const float col_ff = col_trim_lpf.apply(_motors_heli->get_throttle(), _dt);

    // Update collective output
    const float error = target_climb_rate - climb_rate;
    const float col_p = _p_col_td.get_p(error);
    const float collective_out = constrain_value(col_p + col_ff, 0.0f, 1.0f);

    _motors_heli->set_throttle(collective_out);

    // We don't know exactly at what point we transitioned into the touch down phase, se we need to 
    // keep driving the desired speed to zero. This will help with getting the vehicle level for touch down
    _desired_vel *= 1 - (_dt / get_touchdown_time());

    update_navigation_controller(des_lat_accel_norm);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: ARCR
    // @Vehicles: Copter
    // @Description: Helicopter autorotation climb rate controller information
    // @Field: TimeUS: Time since system startup
    // @Field: Tar: Target climb rate
    // @Field: Act: Measured climb rate
    // @Field: Err: Controller error
    // @Field: P: P-term for climb rate controller response
    // @Field: FF: FF-term for climb rate controller response
    // @Field: Out: Output of the climb rate controller

    // Write to data flash log
    AP::logger().WriteStreaming("ARCR",
                       "TimeUS,Tar,Act,Err,P,FF,Out",
                        "Qffffff",
                        AP_HAL::micros64(),
                        target_climb_rate,
                        climb_rate,
                        error,
                        col_p,
                        col_ff,
                        collective_out);
#endif
}


void AC_Autorotation::update_headspeed_controller(void)
{
    // Get current rpm and update healthy signal counters
    float head_speed_norm;
    if (!get_norm_head_speed(head_speed_norm)) {
        // RPM sensor is bad, set collective to angle of -2 deg and hope for the best
         _motors_heli->set_coll_from_ang(-2.0);
         return;
    }

    // Calculate the head speed error.
    _head_speed_error = head_speed_norm - _target_head_speed;

    const float p_term_hs = _p_hs.get_p(_head_speed_error);

    // Adjusting collective trim using feed forward (not yet been updated, so this value is the previous time steps collective position)
    const float col_ff = col_trim_lpf.apply(_motors_heli->get_throttle(), _dt);

    // Calculate collective position to be set
    const float collective_out = constrain_value((p_term_hs + col_ff), 0.0f, 1.0f);

    // Send collective to setting to motors output library
    _motors_heli->set_throttle(collective_out);

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: ARHS
    // @Vehicles: Copter
    // @Description: helicopter AutoRotation Head Speed (ARHS) controller information
    // @Field: TimeUS: Time since system startup
    // @Field: Tar: Normalised target head speed
    // @Field: Act: Normalised measured head speed
    // @Field: Err: Head speed controller error
    // @Field: P: P-term for head speed controller response
    // @Field: FF: FF-term for head speed controller response
    // @Field: Out: Output from the head speed controller

    // Write to data flash log
    AP::logger().WriteStreaming("ARHS",
                                "TimeUS,Tar,Act,Err,P,FF,Out",
                                "s------",
                                "F000000",
                                "Qffffff",
                                AP_HAL::micros64(),
                                _target_head_speed,
                                head_speed_norm,
                                _head_speed_error,
                                p_term_hs,
                                col_ff,
                                collective_out
                                );
#endif
}

// Get measured head speed and normalise by head speed set point. Returns false if a valid rpm measurement cannot be obtained
bool AC_Autorotation::get_norm_head_speed(float& norm_rpm) const
{
    // Assuming zero rpm is safer as it will drive collective in the direction of increasing head speed
    float current_rpm = 0.0;

#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // Checking to ensure no nullptr, we do have a pre-arm check for this so it will be very bad if RPM has gone away
    if (rpm == nullptr) {
        return false;
    }

    // Check RPM sensor is returning a healthy status
    if (!rpm->get_rpm(_param_rpm_instance.get(), current_rpm)) {
        return false;
    }
#endif

    // Protect against div by zeros later in the code
    float head_speed_set_point = MAX(1.0, _param_head_speed_set_point.get());

    // Normalize the RPM by the setpoint
    norm_rpm = current_rpm/head_speed_set_point;
    return true;
}

void AC_Autorotation::calc_yaw_rate_from_roll_target(float& yaw_rate_rad, float& lat_accel) {
    // We will be rolling into the wind to resist the error introduced by the wind pushing us
    // Get the current roll angle target from the attitude controller
    float target_roll_deg = _attitude_control->get_att_target_euler_cd().x * 0.01;

    const float roll_deadzone_deg = 2.0;
    if (fabsf(target_roll_deg) < roll_deadzone_deg) {
        target_roll_deg = 0.0;

    } else if (is_positive(target_roll_deg)) {
        // smooth out steps in target accels when coming out of the dead zone
        target_roll_deg -= roll_deadzone_deg;
    } else {
        // smooth out steps in target accels when coming out of the dead zone
        target_roll_deg += roll_deadzone_deg;
    }

    // protect against math error in the angle_to_accel calc
    target_roll_deg = constrain_float(target_roll_deg, -85, 85);

    // Convert it to a lateral acceleration
    float accel = angle_to_accel(target_roll_deg);

    // Maintain the same accel limit that we impose on the pilot inputs so as to prioritize managing forward accels/speeds
    accel = MIN(accel, get_accel_max() * 0.5);

    // Calculate the yaw rate from the lateral acceleration
    if (fabsf(_desired_vel) > MIN_MANOEUVERING_SPEED) {
        // Calc yaw rate from desired body-frame accels
        // this seems suspiciously simple, but it is correct
        // accel = r * w^2, r = radius and w = angular rate
        // radius can be calculated as the distance traveled in the time it takes to do 360 deg
        // One rotation takes: (2*pi)/w seconds
        // Distance traveled in that time: (s*2*pi)/w
        // radius for that distance: ((s*2*pi)/w) / (2*pi)
        // r = s / w
        // accel = (s / w) * w^2
        // accel = s * w
        // w = accel / s
        yaw_rate_rad = accel / _desired_vel;

        // Only apply the acceleration if we are going to apply the yaw rate
        lat_accel = accel;
    }
}

// Update speed controller
void AC_Autorotation::update_navigation_controller(float pilot_norm_accel)
{
    const AP_AHRS &ahrs = AP::ahrs();

    // Accel limit the desired velocity
    // TODO?????????????????????????????????????????

    // Set up yaw rate
    AC_AttitudeControl::HeadingCommand desired_heading;
    desired_heading.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
    desired_heading.yaw_rate_cds = 0.0;

    Vector3f desired_velocity_bf;
    Vector3f desired_accel_bf;
    Vector3f desired_velocity_NED;
    Vector3f desired_accel_NED;

    switch (Nav_Mode(_param_nav_mode.get())) {
        case Nav_Mode::TURN_INTO_WIND: {
            // Mode seeks to maintain body frame velocity target whilst turning into wind.

            // Set body frame velocity targets
            desired_velocity_bf.x = _desired_vel;
            desired_velocity_bf.y = 0.0; // Start with the assumption that we want zero side slip
            desired_velocity_bf.z = get_bf_speed_down(); // Always match the current vz. We add this in because the vz is significant proportion of the speed that needs to be accounted for when we rotate from body frame to earth frame.

            // The position controller will be resisting the wind by rolling into the wind.
            // Calc the needed lateral accel and yaw rate to make a co-ordinated turn into roll.
            float yaw_rate_rad = 0.0;
            calc_yaw_rate_from_roll_target(yaw_rate_rad, desired_accel_bf.y);
            desired_heading.yaw_rate_cds = degrees(yaw_rate_rad) * 100.0;

            // Convert body frame targets into earth frame
            desired_velocity_NED = ahrs.body_to_earth(desired_velocity_bf);
            desired_accel_NED = ahrs.body_to_earth(desired_accel_bf);

            break;
        }
        case Nav_Mode::CROSS_TRACK: {
            // Mode seeks to maintain either the velocity vector or the heading (slow speed case)
            // recorded on mode init, then yawing into wind to cross-track, reducing drag and roll angle.

            // The position controller will be resisting the wind by rolling into the wind.
            // Calc the needed yaw rate needed to turn into wind and to cross track.
            float yaw_rate_rad = 0.0;
            float unused_lat_accel = 0.0;
            calc_yaw_rate_from_roll_target(yaw_rate_rad, unused_lat_accel);
            desired_heading.yaw_rate_cds = degrees(yaw_rate_rad) * 100.0;

            // We already have the earth-frame unit vector that we want to maintain the velocity vector along
            // so we just use that to calculate the earth frame target vel directly
            desired_velocity_NED = Vector3f{_track_vector, 0.0} * _desired_vel;

            break;
        }
        case Nav_Mode::PILOT_LAT_ACCEL:
        default: {
            // Mode lets pilot request a lateral acceleration. The roll and yaw rates
            // are then calculated to perform a coordinated turn.

            // Set body frame velocity targets
            desired_velocity_bf.x = _desired_vel;
            desired_velocity_bf.y = 0.0; // Start with the assumption that we want zero side slip
            desired_velocity_bf.z = get_bf_speed_down(); // Always match the current vz. We add this in because the vz is significant proportion of the speed that needs to be accounted for when we rotate from body frame to earth frame.

            // Pilot can request as much as 1/2 of the max accel laterally to perform a turn.
            // We only allow up to half as we need to prioritize building/maintaining airspeed.
            desired_accel_bf.y = pilot_norm_accel * get_accel_max() * 0.5;

            // In the case where we have low ground speed (e.g. touch down phase) we still want to let the
            // pilot yaw. We use the min manoeuvering speed as the default "time constant" so that the yaw
            // feels consistant below the min ground speed and to avoid a div by zero.
            float yaw_rate_tc = MAX(fabsf(desired_velocity_bf.x), MIN_MANOEUVERING_SPEED);

            // Calc yaw rate from desired body-frame accels
            // this seems suspiciously simple, but it is correct
            // accel = r * w^2, r = radius and w = angular rate
            // radius can be calculated as the distance traveled in the time it takes to do 360 deg
            // One rotation takes: (2*pi)/w seconds
            // Distance traveled in that time: (s*2*pi)/w
            // radius for that distance: ((s*2*pi)/w) / (2*pi)
            // r = s / w
            // accel = (s / w) * w^2
            // accel = s * w
            // w = accel / s
            desired_heading.yaw_rate_cds = degrees(desired_accel_bf.y / yaw_rate_tc) * 100.0;

            // Only perform coordinated turns when above the min manoeuvering speed
            if (fabsf(desired_velocity_bf.x) < MIN_MANOEUVERING_SPEED) {
                desired_accel_bf.y = 0.0;
            }

            // Convert body frame targets into earth frame
            desired_velocity_NED = ahrs.body_to_earth(desired_velocity_bf);
            desired_accel_NED = ahrs.body_to_earth(desired_accel_bf);
        }
    }

    // zero yaw rate if we are in a heading hold
    if (_heading_hold) {
        desired_heading.yaw_rate_cds = 0.0;
    }

    // We only use 2D NE position controller so discard z target and convert to cm
    Vector2f desired_velocity_NE_cm = {desired_velocity_NED.x * 100.0, desired_velocity_NED.y * 100.0};
    Vector2f desired_accel_NE_cm = {desired_accel_NED.x * 100.0, desired_accel_NED.y * 100.0};

    // Update the position controller
    _pos_control->input_vel_accel_NE_cm(desired_velocity_NE_cm, desired_accel_NE_cm, true);
    _pos_control->update_NE_controller();

    // Output to the attitude controller
    _attitude_control->input_thrust_vector_heading_cd(_pos_control->get_thrust_vector(), desired_heading);

    // Calculate the unit velocity vector for wp bearing telemetry data
    if (desired_velocity_NE_cm.length() > MIN_MANOEUVERING_SPEED * 100.0) {
        _bearing_vector = desired_velocity_NE_cm;
        _bearing_vector.normalize();
    } else {
        _bearing_vector = {0.0, 0.0};
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: ARSC
    // @Vehicles: Copter
    // @Description: Helicopter AutoRotation Speed Controller (ARSC) information 
    // @Field: TimeUS: Time since system startup
    // @Field: VX: Desired velocity X in body frame
    // @Field: VY: Desired velocity Y in body frame
    // @Field: AX: Desired Acceleration X in body frame
    // @Field: AY: Desired Acceleration Y in body frame

    AP::logger().WriteStreaming("ARSC",
                                "TimeUS,VX,VY,VZ,AX,AY,TX,TY",
                                "snnnoo--",
                                "F0000000",
                                "Qfffffff",
                                AP_HAL::micros64(),
                                desired_velocity_bf.x,
                                desired_velocity_bf.y,
                                desired_velocity_bf.z,
                                desired_accel_bf.x,
                                desired_accel_bf.y,
                                _track_vector.x,
                                _track_vector.y);

#endif
}

// Calculate an initial estimate of when the aircraft needs to flare
// This function calculates and stores a few constants that are used again in subsequent flare height update calculations
void AC_Autorotation::initial_flare_hgt_estimate(void)
{
    // Get blade pitch angle, accounting for non-zero zero thrust blade pitch for the asymmetric blade case
    float blade_pitch_hover_rad = radians(_motors_heli->get_hover_coll_ang() - _motors_heli->get_coll_zero_thrust_pitch());

    // Ensure safe math operations below by constraining blade_pitch_hover_rad to be > 0.
    // Assuming 0.1 deg will never be enough to blade pitch angle to maintain hover.
    blade_pitch_hover_rad = MAX(blade_pitch_hover_rad, radians(0.1));

    static const float CL_ALPHA = M_2PI;
    const float b = get_solidity() * CL_ALPHA;
    const float disc_area = M_PI * 0.25 * sq(_param_diameter.get()); // (m^2)

    // Calculating the equivalent inflow ratio (average across the whole blade)
    float lambda_eq = -b / 16.0 + safe_sqrt(sq(b) / 256.0 + b * blade_pitch_hover_rad / 12.0);

    // Tip speed = head speed (rpm) / 60 * 2pi * rotor diameter/2. Eq below is simplified.
    float tip_speed_auto = _param_head_speed_set_point.get() * M_PI * _param_diameter.get() / 60.0;

    // Calc the coefficient of thrust in the hover
    float c_t_hover = 0.5 * b * (blade_pitch_hover_rad / 3.0 - lambda_eq / 2.0);
    c_t_hover = MAX(c_t_hover, 0.00001); //TODO improve this constrain
    _hover_thrust = c_t_hover * SSL_AIR_DENSITY * disc_area * sq(tip_speed_auto);

    // Estimate rate of descent
    static const float ASSUMED_CD0 = 0.011;
    const float sink_rate = ((0.25 * get_solidity() * ASSUMED_CD0 / c_t_hover) + (sq(c_t_hover) / (get_solidity() * ASSUMED_CD0))) * tip_speed_auto;

    // Calc flare altitude
    // TODO: We need to come up with some way to modify the forward speed target here as the target speed is
    // body frame forward speed and the flare height calculation needs earth frame forward speed.  The trouble is
    // that this is a projection into the future so we cannot provide a measured value at this stage
    float des_spd_fwd = _param_target_speed.get();
    calc_flare_hgt(des_spd_fwd, -1.0 * sink_rate);

    gcs().send_text(MAV_SEVERITY_INFO, "Ct/sigma=%.4f W=%.2f kg flare_alt=%.2f", c_t_hover/get_solidity(), _hover_thrust/GRAVITY_MSS, _flare_hgt.get());
    gcs().send_text(MAV_SEVERITY_INFO, "sink rate=%.3f", sink_rate);
    gcs().send_text(MAV_SEVERITY_INFO, "inflow spd=%.3f", lambda_eq*tip_speed_auto);
}

void AC_Autorotation::calc_flare_hgt(const float fwd_speed, float climb_rate)
{
    // we must be descending for this maths to sensible
    if (!is_negative(climb_rate)) {
        return;
    }

    // Keep the slow filter of vel z up to date. We always update this with a fresh measurement as the
    // climb_rate argument maybe from an approximated climb rate not a measured one
    _lagged_vel_z.apply(get_ef_velocity_up(), _dt);

    // Estimate total rotor drag force coefficient in the descent
    // This is not the fully non-dimensionalized drag force to avoid having to constantly
    // dividing and multiply vehicle constants like rotor diameter
    float CR = _hover_thrust / sq(climb_rate);
    CR = constrain_float(CR, 0.01, 1.7); // TODO: confirm typical range of drag coefficients expected

    // Compute speed module and glide path angle during descent
    const float speed_module = MAX(norm(climb_rate, fwd_speed), 0.1); // (m/s)
    const float glide_angle = M_PI / 2 - safe_asin(fwd_speed / speed_module); // (rad)

    // Estimate inflow velocity at beginning of flare
    float entry_inflow = - speed_module * sinf(glide_angle + radians(AP_ALPHA_TPP));

    const float k_1 = safe_sqrt(_hover_thrust / CR); // TODO: discuss: in the initial estimate k1 will always equal climb rate, see definition of CR above
    const float ASYMPT_THRESHOLD = 0.05;
    const float final_sink_rate = climb_rate + ASYMPT_THRESHOLD; //relative to rotor ref system

    // Protect against div by 0 case
    if (is_zero(final_sink_rate + k_1)) {
        climb_rate -= ASYMPT_THRESHOLD;
    }
    if (is_zero(entry_inflow + k_1)) {
        entry_inflow -= ASYMPT_THRESHOLD;
    }

    // Estimate flare duration
    const float m = _hover_thrust / GRAVITY_MSS;
    const float k_3 = safe_sqrt((CR * GRAVITY_MSS) / m);
    const float k_2 = 1 / (2 * k_3) * logf(fabsf((entry_inflow - k_1)/(entry_inflow + k_1)));
    const float a = logf(fabsf((final_sink_rate - k_1)/(final_sink_rate + k_1)));
    const float b = logf(fabsf((entry_inflow - k_1)/(entry_inflow + k_1)));
    const float delta_t_flare = (1 / (2 * k_3)) * (a - b);

    // Estimate flare delta altitude
    const float k_4 = (2 * k_2 * k_3) + (2 * k_3 * delta_t_flare);
    const float flare_distance = ((k_1 / k_3) * (k_4 - logf(fabsf(1-expf(k_4))) - (2 * k_2 * k_3 - logf(fabsf(1 - expf(2 * k_2 * k_3)))))) - k_1 * delta_t_flare;
    const float delta_h = -flare_distance * cosf(radians(AP_ALPHA_TPP));

    // Estimate altitude to begin touch down phase. This value is preserved for logging without constraining it to the min/max allowable
    _calculated_touch_down_hgt = -1.0 * climb_rate * get_touchdown_time();

    // Ensure that we keep the calculated touch down height within the allowable min/max values
    // before it is used in the _calculated_flare_hgt. Not doing this can lead to a zero-ground speed 
    // condition before the state machine is permited to progress onto the touchdown phase.
    const float calc_td_hgt = constrain_float(_calculated_touch_down_hgt, _touch_down_hgt.min_height.get(), _touch_down_hgt.max_height.get());

    // Total delta altitude to ground
    _calculated_flare_hgt = calc_td_hgt + delta_h;

    // Save these calculated values for use, if the conditions were appropriate for us to consider the value reliable
    const float speed_error = fabsf(_desired_vel - get_bf_speed_forward());
    if (speed_error < 0.2 * _desired_vel &&  // Check that our forward speed is withing 20% of target
        fabsf(_lagged_vel_z.get() - get_ef_velocity_up()) < 1.0) // Sink rate can be considered approx steady
    {
        _touch_down_hgt.set(calc_td_hgt);
        _flare_hgt.set(_calculated_flare_hgt);
    }
}

void AC_Autorotation::update_flare_hgt(void)
{
    const float vel_z = get_ef_velocity_up();
    const float fwd_speed = get_ef_speed_forward(); // (m/s)

    // update the flare height calc
    calc_flare_hgt(fwd_speed, vel_z);

    // TODO figure out cause of spikes in flare height estimate
}

bool AC_Autorotation::below_flare_height(void) const
{
    // we cannot transition to the flare phase if we do not know what height we are at
    if (!_hagl_valid) {
        return false;
    }
    return _hagl < _flare_hgt.get();
}

// Determine if we are above the touchdown height using the descent rate and param values
bool AC_Autorotation::should_begin_touchdown(void) const
{
    // we cannot transition to the touchdown phase if we do not know what height we are at
    if (!_hagl_valid) {
        return false;
    }

    const float vz = get_ef_velocity_up();

    // We need to be descending for the touch down controller to interpolate the target
    // sensibly between the entry of the touch down phase zero.
    if (!is_negative(vz)) {
        return false;
    }

    // We maybe descending with significant descent rate that could lead to an early
    // progression into the touchdown phase. Either we still have significant ground speed
    // and we want to let the flare do more work to reduce the speed (both vertical and forward)
    // or we have low ground speed in which case we still want the head speed controller to manage
    // the energy in the head in the flare controller.  Hence we will not allow the touch down
    // phase to begin.
    if (_hagl > _touch_down_hgt.max_height) {
        return false;
    }

    // Calculate the time to impact assuming a constant descent speed
    float time_to_ground = fabsf(_hagl / vz);
    const bool time_check = time_to_ground <= get_touchdown_time();

    // force the flare if we are below the minimum guard height
    const bool min_height_check = _hagl < _touch_down_hgt.min_height;

    return time_check || min_height_check;
}

// Zero velocity and accel to bring all of the controls into position to trip Copter's landing detector.
void AC_Autorotation::run_landed(void)
{
    // Update the position controller with zero vels and accels
    Vector2f desired_velocity_NE_cm;
    Vector2f desired_accel_NE_cm;
    _pos_control->input_vel_accel_NE_cm(desired_velocity_NE_cm, desired_accel_NE_cm, true);
    _pos_control->soften_for_landing_NE();
    _pos_control->update_NE_controller();

    // Output to the attitude controller
    AC_AttitudeControl::HeadingCommand desired_heading;
    desired_heading.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
    desired_heading.yaw_rate_cds = 0.0;
    _attitude_control->input_thrust_vector_heading_cd(_pos_control->get_thrust_vector(), desired_heading);
}

// Determine the body frame forward speed in m/s
float AC_Autorotation::get_bf_speed_forward(void) const
{
    return get_bf_vel().x;
}

// Determine the body frame down speed in m/s
float AC_Autorotation::get_bf_speed_down(void) const
{
    return get_bf_vel().z;
}

// Determine the body frame forward speed in m/s
Vector3f AC_Autorotation::get_bf_vel(void) const
{
    Vector3f vel_NED = {0,0,0};
    const AP_AHRS &ahrs = AP::ahrs();
    if (ahrs.get_velocity_NED(vel_NED)) {
        vel_NED = ahrs.earth_to_body(vel_NED);
    }
    // TODO: need to improve the handling of the velocity NED not ok case
    return vel_NED;
}

// Determine the earth frame forward speed in m/s
float AC_Autorotation::get_ef_speed_forward(void) const
{
    const AP_AHRS &ahrs = AP::ahrs();
    Vector2f groundspeed_vector = ahrs.groundspeed_vector();
    float speed_forward = (groundspeed_vector.x*ahrs.cos_yaw() + groundspeed_vector.y * ahrs.sin_yaw()); // (m/s)
    return speed_forward;
}

// Get the earth frame vertical velocity in meters, positive is up
float AC_Autorotation::get_ef_velocity_up(void) const
{
    const AP_AHRS &ahrs = AP::ahrs();
    Vector3f vel_NED = {0,0,0};
    IGNORE_RETURN(ahrs.get_velocity_NED(vel_NED));
    // TODO: need to improve the handling of the velocity NED not ok case
    return vel_NED.z * -1.0;
}

// Update the height above ground estimate in meters
void AC_Autorotation::update_hagl(void)
{
    // Always reset the hagl valid flag
    _hagl_valid = false;

#if AP_RANGEFINDER_ENABLED
    const uint32_t now = AP_HAL::millis();

    // Keep the ground surface object up to date at 20 Hz
    if ((_last_gnd_surf_update == 0) || (now - _last_gnd_surf_update > 0.05)) {
        _ground_surface->update();
        _last_gnd_surf_update = now;
    }

    // Get the height above ground estimate from the surface tracker library. The rangefinder may go out of range low
    // as we are landing so we allow 5 s of grace whereby we use inertial nav to extrapolate from the last good measurement
    int32_t hagl = 0;
    static const uint32_t oor_low_timer_ms = 5000;
    if (_ground_surface->get_rangefinder_height_interpolated_cm(hagl, oor_low_timer_ms)) {
        _hagl = float(hagl) * 0.01;
        _hagl_valid = true;
    }

    // TODO: improve fail over to terrain and then home

#endif //AP_RANGEFINDER_ENABLED
}

#if HAL_LOGGING_ENABLED
// Logging of lower rate autorotation specific variables. This is meant for stuff that
// doesn't need a high rate, e.g. controller variables that are need for tuning.
void AC_Autorotation::log_write_autorotation(void) const
{
    // enum class for bitmask documentation in logging
    enum class AC_Autorotation_Landed_Reason : uint8_t {
        LOW_SPEED = 1<<0, // true if below 1 m/s
        LAND_COL  = 1<<1, // true if collective below land col min
        IS_STILL  = 1<<2, // passes inertial nav is_still() check
    };

    uint8_t reason = 0;
    if (_landed_reason.min_speed) {
        reason |= uint8_t(AC_Autorotation_Landed_Reason::LOW_SPEED);
    }
    if (_landed_reason.land_col) {
        reason |= uint8_t(AC_Autorotation_Landed_Reason::LAND_COL);
    }
    if (_landed_reason.is_still) {
        reason |= uint8_t(AC_Autorotation_Landed_Reason::IS_STILL);
    }

    // @LoggerMessage: AROT
    // @Vehicles: Copter
    // @Description: Helicopter AutoROTation (AROT) information
    // @Field: TimeUS: Time since system startup
    // @Field: MHgt: Measured Height
    // @Field: CFH: Unfiltered Calculated Flare Height
    // @Field: FHgt: Flare Height
    // @Field: CTDH: Unfiltered Calculated Touch Down Height
    // @Field: TDHgt: Touchdown Height
    // @Field: LR: Landed Reason state flags
    // @FieldBitmaskEnum: LR: AC_Autorotation_Landed_Reason

    // Write to data flash log
    AP::logger().WriteStreaming("AROT",
                                "TimeUS,MHgt,CFH,FHgt,CTDH,TDHgt,LR",
                                "smmmmm-",
                                "F00000-",
                                "QfffffB",
                                AP_HAL::micros64(),
                                _hagl,
                                _calculated_flare_hgt,
                                _flare_hgt.get(),
                                _calculated_touch_down_hgt,
                                _touch_down_hgt.get(),
                                reason);
}
#endif  // HAL_LOGGING_ENABLED

// Arming checks for autorotation, mostly checking for miss-configurations
bool AC_Autorotation::arming_checks(size_t buflen, char *buffer) const
{
    if (!enabled()) {
        // Don't run arming checks if not enabled
        return true;
    }

    // Check for correct RPM sensor config
#if AP_RPM_ENABLED
    // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    // Get current rpm, checking to ensure no nullptr
    if (rpm == nullptr) {
        hal.util->snprintf(buffer, buflen, "Can't access RPM");
        return false;
    }

    // Sanity check that the designated rpm sensor instance is there
    if (_param_rpm_instance.get() < 0) {
        hal.util->snprintf(buffer, buflen, "RPM instance <0");
        return false;
    }

    if (!rpm->enabled(_param_rpm_instance.get())) {
        hal.util->snprintf(buffer, buflen, "RPM%i not enabled", _param_rpm_instance.get()+1);
        return false;
    }
#endif

    // Check that heli motors is configured for autorotation
    if (!_motors_heli->rsc_autorotation_enabled()) {
        hal.util->snprintf(buffer, buflen, "H_RSC_AROT_* not configured");
        return false;
    }

#if AP_RANGEFINDER_ENABLED
    // Check that we can see a healthy rangefinder
    if (!_ground_surface->rangefinder_configured()) {
        hal.util->snprintf(buffer, buflen, "Downward rangefinder not configured");
        return false;
    }

    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        hal.util->snprintf(buffer, buflen, "Downward rangefinder not configured");
        return false;
    }

    // Sanity check that the rangefinder is adequate and can at least read up to the minimum height required to flare
    if (float(rangefinder->max_distance_orient(ROTATION_PITCH_270)) < _flare_hgt.min_height.get()) {
        hal.util->snprintf(buffer, buflen, "Rngfnd max distance < min flare height");
        return false;
    }
#else
    // We currently rely too heavily on rangefinders, thow an arming check if we do not have Rangefinder compiled in
    hal.util->snprintf(buffer, buflen, "Rngfnd not compiled in");
    return false;
#endif

    // Check that the blade pitch collective appears plausible. Would expect at least 1 deg of blade pitch required for hover.
    const float blade_pitch_hover_deg = _motors_heli->get_hover_coll_ang() - _motors_heli->get_coll_zero_thrust_pitch();
    if (blade_pitch_hover_deg < 1.0) {
        hal.util->snprintf(buffer, buflen, "Hover pit < 1 deg. Check H_COL_* setup");
        return false;
    }

    // Sanity check that min touch down height is less than min flare height
    if (_flare_hgt.min_height.get() < _touch_down_hgt.min_height) {
        hal.util->snprintf(buffer, buflen, "FLR_MIN_HGT < TD_MIN_HGT");
        return false;
    }

    return true;
}

// Check if we believe we have landed. We need the landed state to zero all
// controls and make sure that the copter landing detector will trip
bool AC_Autorotation::check_landed(void)
{
    // minimum speed (m/s) used for "is moving" check
    const float min_moving_speed = 1.0;

    Vector3f velocity;
    const AP_AHRS &ahrs = AP::ahrs();
    _landed_reason.min_speed = ahrs.get_velocity_NED(velocity) && (velocity.length() < min_moving_speed);
    _landed_reason.land_col = _motors_heli->get_below_land_min_coll();
    _landed_reason.is_still = AP::ins().is_still();

    return _landed_reason.min_speed && _landed_reason.land_col && _landed_reason.is_still;
}

// Dynamically update time step used in autorotation controllers
void AC_Autorotation::set_dt(float delta_sec)
{
    if (is_positive(delta_sec)) {
        _dt = delta_sec;
        return;
    }
    _dt = 2.5e-3; // Assume 400 Hz
}

int32_t AC_Autorotation::get_wp_bearing(void) const
{
    Vector2f origin;
    return get_bearing_cd(origin, _bearing_vector);
}

float AC_Autorotation::wp_distance_m(void) const
{
    if (is_positive(_hagl) && _hagl_valid) {
    // We don't have waypoints to fly towards in this mode, so instead we send a crude estimate of
    // where we think we might land if we continue on this course.  We will likely be getting a glide
    // ratio of roughly 1:1 so reporting the HAGL will give an approximate distance travelled.
        return _hagl;
    }

    // We may not be in rangefinder range in which case we just return height above origin
    const AP_AHRS &ahrs = AP::ahrs();
    float down;
    if (ahrs.get_relative_position_D_origin(down)) {
        return fabsf(down);
    }

    // If we got this far things went really wrong, just return a fixed value so a direction vector
    // at least displays in GCS
    return 200.0;
}

float AC_Autorotation::crosstrack_error(void) const
{
    return _pos_control->crosstrack_error();
}

void AC_Autorotation::exit(void)
{
#if AP_RANGEFINDER_ENABLED
    _ground_surface->enabled = false;
#endif
}

// Set height value with protections in place to ensure we do not exceed the minimum value
void AC_Autorotation::GuardedHeight::set(float hgt)
{
    // ensure that min and max heights are positive and remotely sensible
    const float min_hgt = MAX(min_height.get(), 0.2);
    const float max_hgt = MAX(max_height.get(), 1.0);
    height = constrain_float(hgt, min_hgt, max_hgt);
}
