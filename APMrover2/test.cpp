#include "Rover.h"

#if CLI_ENABLED == ENABLED

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] = {
    {"passthru",        MENU_FUNC(test_passthru)},
    {"failsafe",        MENU_FUNC(test_failsafe)},
    {"relay",           MENU_FUNC(test_relay)},
    {"waypoints",       MENU_FUNC(test_wp)},
    {"modeswitch",      MENU_FUNC(test_modeswitch)},

    // Tests below here are for hardware sensors only present
    // when real sensors are attached or they are emulated
    {"gps",             MENU_FUNC(test_gps)},
    {"ins",             MENU_FUNC(test_ins)},
    {"rngfndtest",      MENU_FUNC(test_rangefinder)},
    {"compass",         MENU_FUNC(test_mag)},
    {"logging",         MENU_FUNC(test_logging)},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    {"shell",           MENU_FUNC(test_shell)},
#endif
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

int8_t Rover::test_mode(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Test Mode\n\n");
    test_menu.run();
    return 0;
}

void Rover::print_hit_enter()
{
    cliSerial->printf("Hit Enter to exit.\n\n");
}

int8_t Rover::test_passthru(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while (1) {
        delay(20);

        // New radio frame? (we could use also if((millis()- timer) > 20)
        if (hal.rcin->new_input()) {
            cliSerial->printf("CH:");
            for (int i = 0; i < 8; i++) {
                cliSerial->printf("%u", hal.rcin->read(i));    // Print channel values
                cliSerial->printf(",");
                hal.rcout->write(i, hal.rcin->read(i));  // Copy input to Servos
            }
            cliSerial->printf("\n");
        }
        if (cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}


int8_t Rover::test_failsafe(uint8_t argc, const Menu::arg *argv)
{
    uint8_t fail_test = 0;
    print_hit_enter();
    for (int i = 0; i < 50; i++) {
        delay(20);
        read_radio();
    }

    // read the radio to set trims
    // ---------------------------
    trim_radio();

    oldSwitchPosition = readSwitch();

    cliSerial->printf("Unplug battery, throttle in neutral, turn off radio.\n");
    while (channel_throttle->get_control_in() > 0) {
        delay(20);
        read_radio();
    }

    while (1) {
        delay(20);
        read_radio();

        if (channel_throttle->get_control_in() > 0) {
            cliSerial->printf("THROTTLE CHANGED %d \n", channel_throttle->get_control_in());
            fail_test++;
        }

        if (oldSwitchPosition != readSwitch()) {
            cliSerial->printf("CONTROL MODE CHANGED: ");
            print_mode(cliSerial, readSwitch());
            cliSerial->printf("\n");
            fail_test++;
        }

        if (throttle_failsafe_active()) {
            cliSerial->printf("THROTTLE FAILSAFE ACTIVATED: %d, ", channel_throttle->get_radio_in());
            print_mode(cliSerial, readSwitch());
            cliSerial->printf("\n");
            fail_test++;
        }

        if (fail_test > 0) {
            return (0);
        }
        if (cliSerial->available() > 0) {
            cliSerial->printf("LOS caused no change in APM.\n");
            return (0);
        }
    }
}

int8_t Rover::test_relay(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while (1) {
        cliSerial->printf("Relay on\n");
        relay.on(0);
        delay(3000);
        if (cliSerial->available() > 0) {
            return (0);
        }

        cliSerial->printf("Relay off\n");
        relay.off(0);
        delay(3000);
        if (cliSerial->available() > 0) {
            return (0);
        }
    }
}

int8_t Rover::test_wp(uint8_t argc, const Menu::arg *argv)
{
    delay(1000);

    cliSerial->printf("%u waypoints\n", static_cast<uint32_t>(mission.num_commands()));
    cliSerial->printf("Hit radius: %f\n", static_cast<double>(g.waypoint_radius.get()));

    for (uint8_t i = 0; i < mission.num_commands(); i++) {
        AP_Mission::Mission_Command temp_cmd;
        if (mission.read_cmd_from_storage(i, temp_cmd)) {
            test_wp_print(temp_cmd);
        }
    }

    return (0);
}

void Rover::test_wp_print(const AP_Mission::Mission_Command& cmd)
{
    cliSerial->printf("command #: %d id:%d options:%d p1:%d p2:%d p3:%d p4:%d \n",
            static_cast<int32_t>(cmd.index),
            static_cast<int32_t>(cmd.id),
            static_cast<int32_t>(cmd.content.location.options),
            static_cast<int32_t>(cmd.p1),
            (cmd.content.location.alt),
            (cmd.content.location.lat),
            (cmd.content.location.lng));
}

int8_t Rover::test_modeswitch(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    cliSerial->printf("Control CH ");

    cliSerial->printf("%d\n", MODE_CHANNEL);

    while (1) {
        delay(20);
        uint8_t switchPosition = readSwitch();
        if (oldSwitchPosition != switchPosition) {
            cliSerial->printf("Position %d\n",  switchPosition);
            oldSwitchPosition = switchPosition;
        }
        if (cliSerial->available() > 0) {
            return (0);
        }
    }
}

/*
  test the dataflash is working
 */
int8_t Rover::test_logging(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Testing dataflash logging\n");
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}


//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

int8_t Rover::test_gps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    uint32_t last_message_time_ms = 0;
    while (1) {
        delay(100);

        gps.update();

        if (gps.last_message_time_ms() != last_message_time_ms) {
            last_message_time_ms = gps.last_message_time_ms();
            const Location &loc = gps.location();
            cliSerial->printf("Lat: %d, Lon %d, Alt: %dm, #sats: %d\n",
                    (loc.lat),
                    (loc.lng),
                    (loc.alt/100),
                    (gps.num_sats()));
        } else {
            cliSerial->printf(".");
        }
        if (cliSerial->available() > 0) {
            return (0);
        }
    }
}

int8_t Rover::test_ins(uint8_t argc, const Menu::arg *argv)
{
    // cliSerial->printf("Calibrating.");
    ahrs.init();
    ahrs.set_fly_forward(true);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    print_hit_enter();
    delay(1000);

    uint8_t medium_loopCounter = 0;

    while (1) {
        ins.wait_for_sample();

        ahrs.update();

        if (g.compass_enabled) {
            medium_loopCounter++;
            if (medium_loopCounter >= 5) {
                compass.read();
                medium_loopCounter = 0;
            }
        }

        // We are using the IMU
        // ---------------------
        Vector3f gyros = ins.get_gyro();
        Vector3f accels = ins.get_accel();
        cliSerial->printf("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n",
                static_cast<int32_t>(ahrs.roll_sensor / 100),
                static_cast<int32_t>(ahrs.pitch_sensor / 100),
                static_cast<uint16_t>(ahrs.yaw_sensor / 100),
                static_cast<double>(gyros.x), static_cast<double>(gyros.y), static_cast<double>(gyros.z),
                static_cast<double>(accels.x), static_cast<double>(accels.y), static_cast<double>(accels.z));
        if (cliSerial->available() > 0) {
            return (0);
        }
    }
}

void Rover::print_enabled(bool b)
{
       if (b) {
           cliSerial->printf("en");
       } else {
           cliSerial->printf("dis");
       }
       cliSerial->printf("abled\n");
}

int8_t Rover::test_mag(uint8_t argc, const Menu::arg *argv)
{
    if (!g.compass_enabled) {
        cliSerial->printf("Compass: ");
        print_enabled(false);
        return (0);
    }

    if (!compass.init()) {
        cliSerial->printf("Compass initialisation failed!\n");
        return 0;
    }
    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_compass(&compass);

    // we need the AHRS initialised for this test
    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    int counter = 0;
    float heading = 0;

    print_hit_enter();

    uint8_t medium_loopCounter = 0;

    while (1) {
        ins.wait_for_sample();
        ahrs.update();

        medium_loopCounter++;
        if (medium_loopCounter >= 5) {
            if (compass.read()) {
                // Calculate heading
                Matrix3f m = ahrs.get_rotation_body_to_ned();
                heading = compass.calculate_heading(m);
                compass.learn_offsets();
            }
            medium_loopCounter = 0;
        }

        counter++;
        if (counter > 20) {
            if (compass.healthy()) {
                const Vector3f mag_ofs = compass.get_offsets();
                const Vector3f mag = compass.get_field();
                cliSerial->printf("Heading: %f, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n",
                        static_cast<double>((wrap_360_cd(ToDeg(heading) * 100)) /100),
                        static_cast<double>(mag.x), static_cast<double>(mag.y), static_cast<double>(mag.z),
                        static_cast<double>(mag_ofs.x), static_cast<double>(mag_ofs.y), static_cast<double>(mag_ofs.z));
            } else {
                cliSerial->printf("compass not healthy\n");
            }
            counter = 0;
        }
        if (cliSerial->available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.
    cliSerial->printf("saving offsets\n");
    compass.save_offsets();
    return (0);
}

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

int8_t Rover::test_rangefinder(uint8_t argc, const Menu::arg *argv)
{
    init_rangefinder();
    delay(20);
    rangefinder.update();

    if (rangefinder.status(0) == RangeFinder::RangeFinder_NotConnected && rangefinder.status(1) == RangeFinder::RangeFinder_NotConnected) {
        cliSerial->printf("WARNING: Rangefinder is not enabled\n");
    }

    print_hit_enter();

    float rangefinder_dist_cm_min = 0.0f;
    float rangefinder_dist_cm_max = 0.0f;
    float voltage_min = 0.0f, voltage_max = 0.0f;
    float rangefinder2_dist_cm_min = 0.0f;
    float rangefinder2_dist_cm_max = 0.0f;
    float voltage2_min = 0.0f, voltage2_max = 0.0f;
    uint32_t last_print = 0;

    while (true) {
        delay(20);
        rangefinder.update();
        uint32_t now = millis();

        float dist_cm = rangefinder.distance_cm(0);
        float voltage = rangefinder.voltage_mv(0);
        if (is_zero(rangefinder_dist_cm_min)) {
            rangefinder_dist_cm_min = dist_cm;
            voltage_min = voltage;
        }
        rangefinder_dist_cm_max = MAX(rangefinder_dist_cm_max, dist_cm);
        rangefinder_dist_cm_min = MIN(rangefinder_dist_cm_min, dist_cm);
        voltage_min = MIN(voltage_min, voltage);
        voltage_max = MAX(voltage_max, voltage);

        dist_cm = rangefinder.distance_cm(1);
        voltage = rangefinder.voltage_mv(1);
        if (is_zero(rangefinder2_dist_cm_min)) {
            rangefinder2_dist_cm_min = dist_cm;
            voltage2_min = voltage;
        }
        rangefinder2_dist_cm_max = MAX(rangefinder2_dist_cm_max, dist_cm);
        rangefinder2_dist_cm_min = MIN(rangefinder2_dist_cm_min, dist_cm);
        voltage2_min = MIN(voltage2_min, voltage);
        voltage2_max = MAX(voltage2_max, voltage);

        if (now - last_print >= 200) {
            cliSerial->printf("rangefinder1 dist=%.1f:%.1fcm volt1=%.2f:%.2f   rangefinder2 dist=%.1f:%.1fcm volt2=%.2f:%.2f\n",
                    static_cast<double>(rangefinder_dist_cm_min),
                    static_cast<double>(rangefinder_dist_cm_max),
                    static_cast<double>(voltage_min),
                    static_cast<double>(voltage_max),
                    static_cast<double>(rangefinder2_dist_cm_min),
                    static_cast<double>(rangefinder2_dist_cm_max),
                    static_cast<double>(voltage2_min),
                    static_cast<double>(voltage2_max));
            voltage_min = voltage_max = 0.0f;
            voltage2_min = voltage2_max = 0.0f;
            rangefinder_dist_cm_min = rangefinder_dist_cm_max = 0.0f;
            rangefinder2_dist_cm_min = rangefinder2_dist_cm_max = 0.0f;
            last_print = now;
        }
        if (cliSerial->available() > 0) {
            break;
        }
    }
    return (0);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
/*
 *  run a debug shell
 */
int8_t Rover::test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

#endif  // CLI_ENABLED
