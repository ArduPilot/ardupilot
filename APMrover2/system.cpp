// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
The init_ardupilot function processes everything we need for an in - air restart
    We will determine later if we are actually on the ground and process a
    ground start in that case.

*****************************************************************************/

#include "Rover.h"
#include "version.h"

#if CLI_ENABLED == ENABLED

// This is the help function
int8_t Rover::main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Commands:\n"
                      "  logs        log readback/setup mode\n"
                      "  setup       setup mode\n"
                      "  test        test mode\n"
                      "\n"
                      "Move the slide switch and reset to FLY.\n"
                      "\n");
    return(0);
}

// Command/function table for the top-level menu.

static const struct Menu::command main_menu_commands[] = {
//   command        function called
//   =======        ===============
    {"logs",        MENU_FUNC(process_logs)},
    {"setup",       MENU_FUNC(setup_mode)},
    {"test",        MENU_FUNC(test_mode)},
    {"reboot",      MENU_FUNC(reboot_board)},
    {"help",        MENU_FUNC(main_menu_help)}
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

int8_t Rover::reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
void Rover::run_cli(AP_HAL::UARTDriver *port)
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

    cliSerial->printf("\n\nInit " FIRMWARE_STRING
                      "\n\nFree RAM: %u\n",
                      hal.util->available_memory());

    //
    // Check the EEPROM format version before loading any parameters from EEPROM.
    //

    load_parameters();

    GCS_MAVLINK::set_dataflash(&DataFlash);

    // initialise serial ports
    serial_manager.init();

    // setup first port early to allow BoardConfig to report errors
    gcs[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);
    
    BoardConfig.init();

    ServoRelayEvents.set_channel_mask(0xFFF0);

    set_control_channels();

    battery.init();

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    usb_connected = true;
    check_usb_mux();

    // setup telem slots with serial ports
    for (uint8_t i = 1; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        gcs[i].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, i);
    }

    // setup frsky telemetry
#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.init(serial_manager, FIRMWARE_STRING, MAV_TYPE_GROUND_ROVER);
#endif

    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    if (g.compass_enabled==true) {
        if (!compass.init()|| !compass.read()) {
            cliSerial->println("Compass initialisation failed!");
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
            //compass.get_offsets();                        // load offsets to account for airframe magnetic interference
        }
    }

    // initialise sonar
    init_sonar();

    // and baro for EKF
    init_barometer(true);

    // Do GPS init
    gps.init(&DataFlash, serial_manager);

    rc_override_active = hal.rcin->set_overrides(rc_override, 8);

    init_rc_in();        // sets up rc channels from radio
    init_rc_out();        // sets up the timer libs

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


#if CLI_ENABLED == ENABLED
    // If the switch is in 'menu' mode, run the main menu.
    //
    // Since we can't be sure that the setup or test mode won't leave
    // the system in an odd state, we don't let the user exit the top
    // menu; they must reset in order to fly.
    //
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
#endif

    init_capabilities();

    startup_ground();

    set_mode((enum mode)g.initial_mode.get());

    // set the correct flight mode
    // ---------------------------
    reset_control_switch();
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void Rover::startup_ground(void)
{
    set_mode(INITIALISING);

    gcs_send_text(MAV_SEVERITY_INFO,"<startup_ground> Ground start");

    #if(GROUND_START_DELAY > 0)
        gcs_send_text(MAV_SEVERITY_NOTICE,"<startup_ground> With delay");
        delay(GROUND_START_DELAY * 1000);
    #endif

    //IMU ground start
    //------------------------
    //

    startup_INS_ground();

    // read the radio to set trims
    // ---------------------------
    trim_radio();

    // initialise mission library
    mission.init();

    // we don't want writes to the serial port to cause us to pause
    // so set serial ports non-blocking once we are ready to drive
    serial_manager.set_blocking_writes_all(false);

    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    ins.set_dataflash(&DataFlash);

    gcs_send_text(MAV_SEVERITY_INFO,"Ready to drive");
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
    g.pidSpeedThrottle.reset_I();
    steerController.reset_I();
    nav_controller->set_reverse(reverse);
    steerController.set_reverse(reverse);
    in_reverse = reverse;
}

void Rover::set_mode(enum mode mode)
{

    if (control_mode == mode){
        // don't switch modes if we are already in the correct mode.
        return;
    }

    // If we are changing out of AUTO mode reset the loiter timer
    if (control_mode == AUTO) {
        loiter_time = 0;
    }

    control_mode = mode;
    throttle_last = 0;
    throttle = 500;
    g.pidSpeedThrottle.reset_I();

#if FRSKY_TELEM_ENABLED == ENABLED
    frsky_telemetry.update_control_mode(control_mode);
#endif
    
    if (control_mode != AUTO) {
        auto_triggered = false;
    }

    switch(control_mode) {
    case MANUAL:
    case HOLD:
    case LEARNING:
    case STEERING:
        auto_throttle_mode = false;
        break;

    case AUTO:
        auto_throttle_mode = true;
        rtl_complete = false;
        restart_nav();
        break;

    case RTL:
        auto_throttle_mode = true;
        do_RTL();
        break;

    case GUIDED:
        auto_throttle_mode = true;
        rtl_complete = false;
        /*
           when entering guided mode we set the target as the current
           location. This matches the behaviour of the copter code.
           */
        guided_WP = current_loc;
        set_guided_WP();
        break;

    default:
        auto_throttle_mode = true;
        do_RTL();
        break;
    }

    if (should_log(MASK_LOG_MODE)) {
        DataFlash.Log_Write_Mode(control_mode);
    }
}

/*
  set_mode() wrapper for MAVLink SET_MODE
 */
bool Rover::mavlink_set_mode(uint8_t mode)
{
    switch (mode) {
    case MANUAL:
    case HOLD:
    case LEARNING:
    case STEERING:
    case GUIDED:
    case AUTO:
    case RTL:
        set_mode((enum mode)mode);
        return true;
    }
    return false;
}

/*
  called to set/unset a failsafe event.
 */
void Rover::failsafe_trigger(uint8_t failsafe_type, bool on)
{
    uint8_t old_bits = failsafe.bits;
    if (on) {
        failsafe.bits |= failsafe_type;
    } else {
        failsafe.bits &= ~failsafe_type;
    }
    if (old_bits == 0 && failsafe.bits != 0) {
        // a failsafe event has started
        failsafe.start_time = millis();
    }
    if (failsafe.triggered != 0 && failsafe.bits == 0) {
        // a failsafe event has ended
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Failsafe ended");
    }

    failsafe.triggered &= failsafe.bits;

    if (failsafe.triggered == 0 &&
        failsafe.bits != 0 &&
        millis() - failsafe.start_time > g.fs_timeout*1000 &&
        control_mode != RTL &&
        control_mode != HOLD) {
        failsafe.triggered = failsafe.bits;
        gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Failsafe trigger 0x%x", (unsigned)failsafe.triggered);
        switch (g.fs_action) {
        case 0:
            break;
        case 1:
            set_mode(RTL);
            break;
        case 2:
            set_mode(HOLD);
            break;
        }
    }
}

void Rover::startup_INS_ground(void)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Warming up ADC");
    mavlink_delay(500);

    // Makes the servos wiggle twice - about to begin INS calibration - HOLD LEVEL AND STILL!!
    // -----------------------
    gcs_send_text(MAV_SEVERITY_INFO, "Beginning INS calibration. Do not move vehicle");
    mavlink_delay(1000);

    ahrs.init();
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


void Rover::print_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case MANUAL:
        port->print("Manual");
        break;
    case HOLD:
        port->print("HOLD");
        break;
    case LEARNING:
        port->print("Learning");
        break;
    case STEERING:
        port->print("Steering");
        break;
    case AUTO:
        port->print("AUTO");
        break;
    case RTL:
        port->print("RTL");
        break;
    default:
        port->printf("Mode(%u)", (unsigned)mode);
        break;
    }
}

/*
  check a digitial pin for high,low (1/0)
 */
uint8_t Rover::check_digital_pin(uint8_t pin)
{
    int8_t dpin = hal.gpio->analogPinToDigitalPin(pin);
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
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = hal.util->get_soft_armed() || DataFlash.log_while_disarmed();
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        start_logging();
    }
    return ret;
}

/*
  update AHRS soft arm state and log as needed
 */
void Rover::change_arm_state(void)
{
    Log_Arm_Disarm();
    hal.util->set_soft_armed(arming.is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
}

/*
  arm motors
 */
bool Rover::arm_motors(AP_Arming::ArmingMethod method)
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
bool Rover::disarm_motors(void)
{
    if (!arming.disarm()) {
        return false;
    }
    if (arming.arming_required() == AP_Arming::YES_ZERO_PWM) {
        channel_throttle->disable_out();
        if (g.skid_steer_out) {
            channel_steer->disable_out();
        }
    }
    if (control_mode != AUTO) {
        // reset the mission on disarm if we are not in auto
        mission.reset();
    }

    //only log if disarming was successful
    change_arm_state();

    return true;
}
