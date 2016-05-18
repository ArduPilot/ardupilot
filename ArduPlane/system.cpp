// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"
#include "version.h"

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED

// This is the help function
int8_t Plane::main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Commands:\n"
                         "  logs        log readback/setup mode\n"
                         "  setup       setup mode\n"
                         "  test        test mode\n"
                         "  reboot      reboot to flight mode\n"
                         "\n");
    return(0);
}

// Command/function table for the top-level menu.
static const struct Menu::command main_menu_commands[] = {
//   command		function called
//   =======        ===============
    {"logs",        MENU_FUNC(process_logs)},
    {"setup",       MENU_FUNC(setup_mode)},
    {"test",        MENU_FUNC(test_mode)},
    {"reboot",      MENU_FUNC(reboot_board)},
    {"help",        MENU_FUNC(main_menu_help)},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

int8_t Plane::reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
void Plane::run_cli(AP_HAL::UARTDriver *port)
{
    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(NULL,1);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED


static void mavlink_delay_cb_static()
{
    plane.mavlink_delay_cb();
}

static void failsafe_check_static()
{
    plane.failsafe_check();
}

void Plane::init_ardupilot()
{
    // initialise serial port
    serial_manager.init_console();

    cliSerial->printf("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %u\n",
                      (unsigned)hal.util->available_memory());


    //
    // Check the EEPROM format version before loading any parameters from EEPROM
    //
    load_parameters();

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // set sensors to HIL mode
        ins.set_hil_mode();
        compass.set_hil_mode();
        barometer.set_hil_mode();
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // this must be before BoardConfig.init() so if
    // BRD_SAFETYENABLE==0 then we don't have safety off yet
    for (uint8_t tries=0; tries<10; tries++) {
        if (setup_failsafe_mixing()) {
            break;
        }
        hal.scheduler->delay(10);
    }
#endif

    BoardConfig.init();

    // initialise serial ports
    serial_manager.init();

    GCS_MAVLINK::set_dataflash(&DataFlash);

    // allow servo set on all channels except first 4
    ServoRelayEvents.set_channel_mask(0xFFF0);

    set_control_channels();

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // initialise rangefinder
    init_rangefinder();

    // initialise battery monitoring
    battery.init();

    rpm_sensor.init();

    // init the GCS
    gcs[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_Console, 0);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.    
    usb_connected = true;
    check_usb_mux();

    // setup all other telem slots with  serial ports
    for (uint8_t i = 1; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        gcs[i].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, (i - 1));
    }

    // setup frsky
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.init(serial_manager);
#endif

    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise airspeed sensor
    airspeed.init();

    if (g.compass_enabled==true) {
        bool compass_ok = compass.init() && compass.read();
#if HIL_SUPPORT
    if (g.hil_mode != 0) {
        compass_ok = true;
    }
#endif
        if (!compass_ok) {
            cliSerial->println("Compass initialisation failed!");
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }
    
#if OPTFLOW == ENABLED
    // make optflow available to libraries
    ahrs.set_optflow(&optflow);
#endif

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    // give AHRS the airspeed sensor
    ahrs.set_airspeed(&airspeed);

    // GPS Initialization
    gps.init(&DataFlash, serial_manager);

    init_rc_in();               // sets up rc channels from radio

    relay.init();

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(&DataFlash, serial_manager);
#endif

#if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(FENCE_TRIGGERED_PIN, 0);
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

#if CLI_ENABLED == ENABLED
    if (g.cli_enabled == 1) {
        const char *msg = "\nPress ENTER 3 times to start interactive setup\n";
        cliSerial->println(msg);
        if (gcs[1].initialised && (gcs[1].get_uart() != NULL)) {
            gcs[1].get_uart()->println(msg);
        }
        if (num_gcs > 2 && gcs[2].initialised && (gcs[2].get_uart() != NULL)) {
            gcs[2].get_uart()->println(msg);
        }
    }
#endif // CLI_ENABLED

    init_capabilities();

    quadplane.setup();

    startup_ground();

    // don't initialise rc output until after quadplane is setup as
    // that can change initial values of channels
    init_rc_out();
    
    // choose the nav controller
    set_nav_controller();

    set_mode((FlightMode)g.initial_mode.get());

    // set the correct flight mode
    // ---------------------------
    reset_control_switch();

    // initialise sensor
#if OPTFLOW == ENABLED
    optflow.init();
#endif

}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void Plane::startup_ground(void)
{
    set_mode(INITIALISING);

    gcs_send_text(MAV_SEVERITY_INFO,"<startup_ground> Ground start");

#if (GROUND_START_DELAY > 0)
    gcs_send_text(MAV_SEVERITY_NOTICE,"<startup_ground> With delay");
    delay(GROUND_START_DELAY * 1000);
#endif

    // Makes the servos wiggle
    // step 1 = 1 wiggle
    // -----------------------
    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        demo_servos(1);
    }

    //INS ground start
    //------------------------
    //
    startup_INS_ground();

    // read the radio to set trims
    // ---------------------------
    if (g.trim_rc_at_start != 0) {
        trim_radio();
    }

    // Save the settings for in-air restart
    // ------------------------------------
    //save_EEPROM_groundstart();

    // initialise mission library
    mission.init();

    // Makes the servos wiggle - 3 times signals ready to fly
    // -----------------------
    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        demo_servos(3);
    }

    // reset last heartbeat time, so we don't trigger failsafe on slow
    // startup
    failsafe.last_heartbeat_ms = millis();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    ins.set_dataflash(&DataFlash);

    gcs_send_text(MAV_SEVERITY_INFO,"Ready to fly");
}

enum FlightMode Plane::get_previous_mode() {
    return previous_mode; 
}

void Plane::set_mode(enum FlightMode mode)
{
    if(control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }
    if(g.auto_trim > 0 && control_mode == MANUAL)
        trim_control_surfaces();

    // perform any cleanup required for prev flight mode
    exit_mode(control_mode);

    // cancel inverted flight
    auto_state.inverted_flight = false;

    // don't cross-track when starting a mission
    auto_state.next_wp_no_crosstrack = true;

    // reset landing check
    auto_state.checked_for_autoland = false;

    // reset go around command
    auto_state.commanded_go_around = false;

    // not in pre-flare
    auto_state.land_pre_flare = false;
    
    // zero locked course
    steer_state.locked_course_err = 0;

    // reset crash detection
    crash_state.is_crashed = false;
    crash_state.impact_detected = false;

    // always reset this because we don't know who called set_mode. In evasion
    // behavior you should set this flag after set_mode so you know the evasion
    // logic is controlling the mode. This allows manual override of the mode
    // to exit evasion behavior automatically but if the mode is manually switched
    // then we won't resume AUTO after an evasion
    adsb_state.is_evading = false;

    // set mode
    previous_mode = control_mode;
    control_mode = mode;

    if (previous_mode == AUTOTUNE && control_mode != AUTOTUNE) {
        // restore last gains
        autotune_restore();
    }

    // zero initial pitch and highest airspeed on mode change
    auto_state.highest_airspeed = 0;
    auto_state.initial_pitch_cd = ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    prev_WP_loc = current_loc;

    // new mode means new loiter
    loiter.start_time_ms = 0;

    // assume non-VTOL mode
    auto_state.vtol_mode = false;
    auto_state.vtol_loiter = false;
    
    switch(control_mode)
    {
    case INITIALISING:
        auto_throttle_mode = true;
        break;

    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case FLY_BY_WIRE_A:
        auto_throttle_mode = false;
        break;

    case AUTOTUNE:
        auto_throttle_mode = false;
        autotune_start();
        break;

    case ACRO:
        auto_throttle_mode = false;
        acro_state.locked_roll = false;
        acro_state.locked_pitch = false;
        break;
        
    case CRUISE:
        auto_throttle_mode = true;
        cruise_state.locked_heading = false;
        cruise_state.lock_timer_ms = 0;
        set_target_altitude_current();
        break;

    case FLY_BY_WIRE_B:
        auto_throttle_mode = true;
        set_target_altitude_current();
        break;

    case CIRCLE:
        // the altitude to circle at is taken from the current altitude
        auto_throttle_mode = true;
        next_WP_loc.alt = current_loc.alt;
        break;

    case AUTO:
        auto_throttle_mode = true;
        if (quadplane.available() && quadplane.enable == 2) {
            auto_state.vtol_mode = true;
        } else {
            auto_state.vtol_mode = false;
        }
        next_WP_loc = prev_WP_loc = current_loc;
        // start or resume the mission, based on MIS_AUTORESET
        mission.start_or_resume();
        break;

    case RTL:
        auto_throttle_mode = true;
        prev_WP_loc = current_loc;
        do_RTL(get_RTL_altitude());
        break;

    case LOITER:
        auto_throttle_mode = true;
        do_loiter_at_location();
        break;

    case GUIDED:
        auto_throttle_mode = true;
        guided_throttle_passthru = false;
        /*
          when entering guided mode we set the target as the current
          location. This matches the behaviour of the copter code
        */
        guided_WP_loc = current_loc;
        set_guided_WP();
        break;

    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL:
        if (!quadplane.init_mode()) {
            control_mode = previous_mode;
        } else {
            auto_throttle_mode = false;
            auto_state.vtol_mode = true;
        }
        break;
    }

    // start with throttle suppressed in auto_throttle modes
    throttle_suppressed = auto_throttle_mode;

    if (should_log(MASK_LOG_MODE))
        DataFlash.Log_Write_Mode(control_mode);

    // reset attitude integrators on mode change
    rollController.reset_I();
    pitchController.reset_I();
    yawController.reset_I();    
    steerController.reset_I();    
}

/*
  set_mode() wrapper for MAVLink SET_MODE
 */
bool Plane::mavlink_set_mode(uint8_t mode)
{
    switch (mode) {
    case MANUAL:
    case CIRCLE:
    case STABILIZE:
    case TRAINING:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CRUISE:
    case GUIDED:
    case AUTO:
    case RTL:
    case LOITER:
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL:
        set_mode((enum FlightMode)mode);
        return true;
    }
    return false;
}

// exit_mode - perform any cleanup required when leaving a flight mode
void Plane::exit_mode(enum FlightMode mode)
{
    // stop mission when we leave auto
    if (mode == AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();

            if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND)
            {
                restart_landing_sequence();
            }
        }
        auto_state.started_flying_in_auto_ms = 0;
    }
}

void Plane::check_long_failsafe()
{
    uint32_t tnow = millis();
    // only act on changes
    // -------------------
    if(failsafe.state != FAILSAFE_LONG && failsafe.state != FAILSAFE_GCS &&
            flight_stage != AP_SpdHgtControl::FLIGHT_LAND_FINAL &&
            flight_stage != AP_SpdHgtControl::FLIGHT_LAND_PREFLARE &&
            flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
        if (failsafe.state == FAILSAFE_SHORT &&
                   (tnow - failsafe.ch3_timer_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_LONG);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_AUTO && control_mode == AUTO &&
                   failsafe.last_heartbeat_ms != 0 &&
                   (tnow - failsafe.last_heartbeat_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HEARTBEAT &&
                   failsafe.last_heartbeat_ms != 0 &&
                   (tnow - failsafe.last_heartbeat_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS);
        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI && 
                   gcs[0].last_radio_status_remrssi_ms != 0 &&
                   (tnow - gcs[0].last_radio_status_remrssi_ms) > g.long_fs_timeout*1000) {
            failsafe_long_on_event(FAILSAFE_GCS);
        }
    } else {
        // We do not change state but allow for user to change mode
        if (failsafe.state == FAILSAFE_GCS && 
            (tnow - failsafe.last_heartbeat_ms) < g.short_fs_timeout*1000) {
            failsafe.state = FAILSAFE_NONE;
        } else if (failsafe.state == FAILSAFE_LONG && 
                   !failsafe.ch3_failsafe) {
            failsafe.state = FAILSAFE_NONE;
        }
    }
}

void Plane::check_short_failsafe()
{
    // only act on changes
    // -------------------
    if(failsafe.state == FAILSAFE_NONE &&
            flight_stage != AP_SpdHgtControl::FLIGHT_LAND_FINAL &&
            flight_stage != AP_SpdHgtControl::FLIGHT_LAND_PREFLARE &&
            flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
        if(failsafe.ch3_failsafe) {                                              // The condition is checked and the flag ch3_failsafe is set in radio.pde
            failsafe_short_on_event(FAILSAFE_SHORT);
        }
    }

    if(failsafe.state == FAILSAFE_SHORT) {
        if(!failsafe.ch3_failsafe) {
            failsafe_short_off_event();
        }
    }
}


void Plane::startup_INS_ground(void)
{
#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        while (barometer.get_last_update() == 0) {
            // the barometer begins updating when we get the first
            // HIL_STATE message
            gcs_send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
            hal.scheduler->delay(1000);
        }
    }
#endif

    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        gcs_send_text(MAV_SEVERITY_ALERT, "Beginning INS calibration. Do not move plane");
        hal.scheduler->delay(100);
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
    ahrs.set_wind_estimation(true);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    // read Baro pressure at ground
    //-----------------------------
    init_barometer();

    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        zero_airspeed(true);
    } else {
        gcs_send_text(MAV_SEVERITY_WARNING,"No airspeed");
    }
}

// updates the status of the notify objects
// should be called at 50hz
void Plane::update_notify()
{
    notify.update();
}

void Plane::resetPerfData(void) 
{
    perf.mainLoop_count = 0;
    perf.G_Dt_max       = 0;
    perf.G_Dt_min       = 0;
    perf.num_long       = 0;
    perf.start_ms       = millis();
    perf.last_log_dropped = DataFlash.num_dropped();
}


void Plane::check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;
}


void Plane::print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        port->print("Manual");
        break;
    case CIRCLE:
        port->print("Circle");
        break;
    case STABILIZE:
        port->print("Stabilize");
        break;
    case TRAINING:
        port->print("Training");
        break;
    case ACRO:
        port->print("ACRO");
        break;
    case FLY_BY_WIRE_A:
        port->print("FBW_A");
        break;
    case AUTOTUNE:
        port->print("AUTOTUNE");
        break;
    case FLY_BY_WIRE_B:
        port->print("FBW_B");
        break;
    case CRUISE:
        port->print("CRUISE");
        break;
    case AUTO:
        port->print("AUTO");
        break;
    case RTL:
        port->print("RTL");
        break;
    case LOITER:
        port->print("Loiter");
        break;
    case GUIDED:
        port->print("Guided");
        break;
    case QSTABILIZE:
        port->print("QStabilize");
        break;
    case QHOVER:
        port->print("QHover");
        break;
    case QLOITER:
        port->print("QLoiter");
        break;
    case QLAND:
        port->print("QLand");
        break;
    case QRTL:
        port->print("QRTL");
        break;
    default:
        port->printf("Mode(%u)", (unsigned)mode);
        break;
    }
}

#if CLI_ENABLED == ENABLED
void Plane::print_comma(void)
{
    cliSerial->print(",");
}
#endif

/*
  write to a servo
 */
void Plane::servo_write(uint8_t ch, uint16_t pwm)
{
#if HIL_SUPPORT
    if (g.hil_mode==1 && !g.hil_servos) {
        if (ch < 8) {
            RC_Channel::rc_channel(ch)->set_radio_out(pwm);
        }
        return;
    }
#endif
    hal.rcout->enable_ch(ch);
    hal.rcout->write(ch, pwm);
}

/*
  should we log a message type now?
 */
bool Plane::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = hal.util->get_soft_armed() || DataFlash.log_while_disarmed();
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        // we have to set in_mavlink_delay to prevent logging while
        // writing headers
        start_logging();
    }
    return ret;
#else
    return false;
#endif
}

/*
  send FrSky telemetry. Should be called at 5Hz by scheduler
 */
#if FRSKY_TELEM_ENABLED == ENABLED
void Plane::frsky_telemetry_send(void)
{
    frsky_telemetry.send_frames((uint8_t)control_mode);
}
#endif


/*
  return throttle percentage from 0 to 100 for normal use and -100 to 100 when using reverse thrust
 */
int8_t Plane::throttle_percentage(void)
{
    if (quadplane.in_vtol_mode()) {
        return quadplane.throttle_percentage();
    }
    // to get the real throttle we need to use norm_output() which
    // returns a number from -1 to 1.
    if (aparm.throttle_min >= 0) {
        return constrain_int16(50*(channel_throttle->norm_output()+1), 0, 100);
    } else {
        // reverse thrust
        return constrain_int16(100*channel_throttle->norm_output(), -100, 100);
    }
}

/*
  update AHRS soft arm state and log as needed
 */
void Plane::change_arm_state(void)
{
    Log_Arm_Disarm();
    hal.util->set_soft_armed(arming.is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    quadplane.set_armed(hal.util->get_soft_armed());
}

/*
  arm motors
 */
bool Plane::arm_motors(AP_Arming::ArmingMethod method)
{
    if (!arming.arm(method)) {
        return false;
    }

    // only log if arming was successful
    channel_throttle->enable_out();

    change_arm_state();
    return true;
}

/*
  disarm motors
 */
bool Plane::disarm_motors(void)
{
    if (!arming.disarm()) {
        return false;
    }
    if (arming.arming_required() == AP_Arming::YES_ZERO_PWM) {
        channel_throttle->disable_out();  
    }
    if (control_mode != AUTO) {
        // reset the mission on disarm if we are not in auto
        mission.reset();
    }

    // suppress the throttle in auto-throttle modes
    throttle_suppressed = auto_throttle_mode;
    
    //only log if disarming was successful
    change_arm_state();

    // reload target airspeed which could have been modified by a mission
    plane.g.airspeed_cruise_cm.load();
    
    return true;
}
