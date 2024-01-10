#include "Blimp.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void failsafe_check_static()
{
    blimp.failsafe_check();
}

void Blimp::init_ardupilot()
{

#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

    BoardConfig.init();


    // initialise notify system
    notify.init();
    notify_flight_mode();

    // initialise battery monitor
    battery.init();

    // Init RSSI
    rssi.init();

    barometer.init();

    // setup telem slots with serial ports
    gcs().setup_uarts();

#if HAL_LOGGING_ENABLED
    log_init();
#endif

    init_rc_in();               // sets up rc channels from radio

    // allocate the motors class
    allocate_motors();
    loiter = new Loiter(blimp.scheduler.get_loop_rate_hz());

    // initialise rc channels including setting mode
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM);
    rc().init();

    // sets up motors and output to escs
    init_rc_out();


    // motors initialised so parameters can be sent
    ap.initialised_params = true;

#if AP_RELAY_ENABLED
    relay.init();
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

#if HAL_LOGGING_ENABLED
    // initialise AP_Logger library
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&blimp, &Blimp::Log_Write_Vehicle_Startup_Messages, void));
#endif

    startup_INS_ground();

#if AP_SCRIPTING_ENABLED
    g2.scripting.init();
#endif // AP_SCRIPTING_ENABLED

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // setup fin output
    motors->setup_fins();

    // enable output to motors
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    //Initialise fin filters
    vel_xy_filter.init(scheduler.get_loop_rate_hz(), motors->freq_hz, 0.5f, 15.0f);
    vel_z_filter.init(scheduler.get_loop_rate_hz(), motors->freq_hz, 1.0f, 15.0f);
    vel_yaw_filter.init(scheduler.get_loop_rate_hz(),motors->freq_hz, 5.0f, 15.0f);

    // attempt to switch to MANUAL, if this fails then switch to Land
    if (!set_mode((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED)) {
        // set mode to MANUAL will trigger mode change notification to pilot
        set_mode(Mode::Number::MANUAL, ModeReason::UNAVAILABLE);
    } else {
        // alert pilot to mode change
        AP_Notify::events.failsafe_mode_change = 1;
    }

    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Blimp::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Blimp::position_ok() const
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

// ekf_has_absolute_position - returns true if the EKF can provide an absolute WGS-84 position estimate
bool Blimp::ekf_has_absolute_position() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors->armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// ekf_has_relative_position - returns true if the EKF can provide a position estimate relative to it's starting position
bool Blimp::ekf_has_relative_position() const
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled
    bool enabled = false;
    if (!enabled) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal relative position
    if (!motors->armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    } else {
        return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    }
}

// returns true if the ekf has a good altitude estimate (required for modes which do AltHold)
bool Blimp::ekf_alt_ok() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow alt control with only dcm
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // require both vertical velocity and position
    return (filt_status.flags.vert_vel && filt_status.flags.vert_pos);
}

// update_auto_armed - update status of auto_armed flag
void Blimp::update_auto_armed()
{
    // disarm checks
    if (ap.auto_armed) {
        // if motors are disarmed, auto_armed should also be false
        if (!motors->armed()) {
            set_auto_armed(false);
            return;
        }
        // if in a manual flight mode and throttle is zero, auto-armed should become false
        if (flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
    }
}

#if HAL_LOGGING_ENABLED
/*
  should we log a message type now?
 */
bool Blimp::should_log(uint32_t mask)
{
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
}
#endif

// return MAV_TYPE corresponding to frame class
MAV_TYPE Blimp::get_frame_mav_type()
{
    return MAV_TYPE_AIRSHIP;
}

// return string corresponding to frame_class
const char* Blimp::get_frame_string()
{
    return "AIRFISH";  //TODO: Change to be able to change with different frame_classes
}

/*
  allocate the motors class
 */
void Blimp::allocate_motors(void)
{
    switch ((Fins::motor_frame_class)g2.frame_class.get()) {
    case Fins::MOTOR_FRAME_AIRFISH:
    default:
        motors = new Fins(blimp.scheduler.get_loop_rate_hz());
        break;
    }
    if (motors == nullptr) {
        AP_BoardConfig::allocation_error("FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    AP_Param::load_object_from_eeprom(motors, Fins::var_info);

    // reload lines from the defaults file that may now be accessible
    AP_Param::reload_defaults_file(true);

    // param count could have changed
    AP_Param::invalidate_count();
}
