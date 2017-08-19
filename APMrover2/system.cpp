/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
    We will determine later if we are actually on the ground and process a
    ground start in that case.

*****************************************************************************/

#include "Rover.h"
#include "version.h"

static void mavlink_delay_cb_static()
{
    rover.mavlink_delay_cb();
}

static void failsafe_check_static()
{
    rover.failsafe_check();
}

void Rover::init_ardupilot()
{
    // initialise console serial port
    serial_manager.init_console();

    hal.console->printf("\n\nInit %s"
                        "\n\nFree RAM: %u\n",
                        fwver.fw_string,
                        hal.util->available_memory());

    //
    // Check the EEPROM format version before loading any parameters from EEPROM.
    //

    load_parameters();

    // initialise stats module
    g2.stats.init();

    gcs().set_dataflash(&DataFlash);

    mavlink_system.sysid = g.sysid_this_mav;

    // initialise serial ports
    serial_manager.init();

    // setup first port early to allow BoardConfig to report errors
    gcs().chan(0).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    BoardConfig.init();
#if HAL_WITH_UAVCAN
    BoardConfig_CAN.init();
#endif

    // initialise notify system
    notify.init(false);
    AP_Notify::flags.failsafe_battery = false;
    notify_mode((enum mode)control_mode->mode_number());

    ServoRelayEvents.set_channel_mask(0xFFF0);

    battery.init();

    rssi.init();

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    usb_connected = true;
    check_usb_mux();

    // setup telem slots with serial ports
    gcs().setup_uarts(serial_manager);

    // setup frsky telemetry
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.init(serial_manager, fwver.fw_string, MAV_TYPE_GROUND_ROVER);
#endif

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise compass
    init_compass();

    // initialise rangefinder
    init_rangefinder();

    // init beacons used for non-gps position estimation
    init_beacon();

    // init visual odometry
    init_visual_odom();

    // and baro for EKF
    init_barometer(true);

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    set_control_channels();  // setup radio channels and ouputs ranges
    init_rc_in();            // sets up rc channels deadzone
    g2.motors.init();        // init motors including setting servo out channels ranges
    init_rc_out();           // enable output

    // init wheel encoders
    g2.wheel_encoder.init();

    relay.init();

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(&DataFlash, serial_manager);
#endif

    /*
      setup the 'main loop is dead' check. Note that this relies on
      the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // give AHRS the range beacon sensor
    ahrs.set_beacon(&g2.beacon);


    init_capabilities();

    startup_ground();

    Mode *initial_mode = control_mode_from_num((enum mode)g.initial_mode.get());
    if (initial_mode == nullptr) {
        initial_mode = &mode_initializing;
    }
    set_mode(*initial_mode, MODE_REASON_INITIALISED);


    // set the correct flight mode
    // ---------------------------
    reset_control_switch();
    init_aux_switch();

    // disable safety if requested
    BoardConfig.init_safety();

    // flag that initialisation has completed
    initialised = true;
}

//*********************************************************************************
// This function does all the calibrations, etc. that we need during a ground start
//*********************************************************************************
void Rover::startup_ground(void)
{
    set_mode(mode_initializing, MODE_REASON_INITIALISED);

    gcs().send_text(MAV_SEVERITY_INFO, "<startup_ground> Ground start");

    #if(GROUND_START_DELAY > 0)
        gcs().send_text(MAV_SEVERITY_NOTICE, "<startup_ground> With delay");
        delay(GROUND_START_DELAY * 1000);
    #endif

    // IMU ground start
    //------------------------
    //

    startup_INS_ground();

    // read the radio to set trims
    // ---------------------------
    trim_radio();

    // initialise mission library
    mission.init();

    // initialise DataFlash library
    DataFlash.set_mission(&mission);
    DataFlash.setVehicle_Startup_Log_Writer(
        FUNCTOR_BIND(&rover, &Rover::Log_Write_Vehicle_Startup_Messages, void)
        );

    // we don't want writes to the serial port to cause us to pause
    // so set serial ports non-blocking once we are ready to drive
    serial_manager.set_blocking_writes_all(false);

    gcs().send_text(MAV_SEVERITY_INFO, "Ready to drive");
}

/*
  set the in_reverse flag
  reset the throttle integrator if this changes in_reverse
 */
void Rover::set_reverse(bool reverse)
{
    if (in_reverse == reverse) {
        return;
    }
    in_reverse = reverse;
}

bool Rover::set_mode(Mode &new_mode, mode_reason_t reason)
{
    if (control_mode == &new_mode) {
        // don't switch modes if we are already in the correct mode.
        return true;
    }

    Mode &old_mode = *control_mode;
    if (!new_mode.enter()) {
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE, new_mode.mode_number());
        gcs().send_text(MAV_SEVERITY_WARNING, "Flight mode change failed");
        return false;
    }

    control_mode = &new_mode;

#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.update_control_mode(control_mode->mode_number());
#endif

    old_mode.exit();

    if (should_log(MASK_LOG_MODE)) {
        control_mode_reason = reason;
        DataFlash.Log_Write_Mode(control_mode->mode_number(), reason);
    }

    notify_mode((enum mode)control_mode->mode_number());
    return true;
}

void Rover::startup_INS_ground(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Beginning INS calibration. Do not move vehicle");
    hal.scheduler->delay(100);

    ahrs.init();
    // say to EKF that rover only move by goind forward
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();
}

// updates the notify state
// should be called at 50hz
void Rover::update_notify()
{
    notify.update();
}

void Rover::resetPerfData(void) {
    mainLoop_count = 0;
    G_Dt_max = 0;
    perf_mon_timer = millis();
}


void Rover::check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;
}

// update notify with mode change
void Rover::notify_mode(enum mode new_mode)
{
    notify.flags.flight_mode = new_mode;

    switch (new_mode) {
    case MANUAL:
        notify.set_flight_mode_str("MANU");
        break;
    case STEERING:
        notify.set_flight_mode_str("STER");
        break;
    case HOLD:
        notify.set_flight_mode_str("HOLD");
        break;
    case AUTO:
        notify.set_flight_mode_str("AUTO");
        break;
    case RTL:
        notify.set_flight_mode_str("RTL");
        break;
    case GUIDED:
        notify.set_flight_mode_str("GUID");
        break;
    case INITIALISING:
        notify.set_flight_mode_str("INIT");
        break;
    default:
        notify.set_flight_mode_str("----");
        break;
    }
}

/*
  check a digitial pin for high,low (1/0)
 */
uint8_t Rover::check_digital_pin(uint8_t pin)
{
    const int8_t dpin = hal.gpio->analogPinToDigitalPin(pin);
    if (dpin == -1) {
        return 0;
    }
    // ensure we are in input mode
    hal.gpio->pinMode(dpin, HAL_GPIO_INPUT);

    // enable pullup
    hal.gpio->write(dpin, 1);

    return hal.gpio->read(dpin);
}

/*
  should we log a message type now?
 */
bool Rover::should_log(uint32_t mask)
{
    return DataFlash.should_log(mask);
}

/*
  update AHRS soft arm state and log as needed
 */
void Rover::change_arm_state(void)
{
    Log_Arm_Disarm();
    update_soft_armed();
}

/*
  arm motors
 */
bool Rover::arm_motors(AP_Arming::ArmingMethod method)
{
    if (!arming.arm(method)) {
        return false;
    }

    change_arm_state();
    return true;
}

/*
  disarm motors
 */
bool Rover::disarm_motors(void)
{
    if (!arming.disarm()) {
        return false;
    }
    if (control_mode != &mode_auto) {
        // reset the mission on disarm if we are not in auto
        mission.reset();
    }

    // only log if disarming was successful
    change_arm_state();

    return true;
}
