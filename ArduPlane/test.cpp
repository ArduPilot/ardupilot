#include "Plane.h"

#if CLI_ENABLED == ENABLED

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] = {
    // Tests below here are for hardware sensors only present
    // when real sensors are attached or they are emulated
    {"gps",                 MENU_FUNC(test_gps)},
    {"ins",                 MENU_FUNC(test_ins)},
    {"airspeed",            MENU_FUNC(test_airspeed)},
    {"airpressure",         MENU_FUNC(test_pressure)},
    {"compass",             MENU_FUNC(test_mag)},
    {"logging",             MENU_FUNC(test_logging)},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    {"shell", 				MENU_FUNC(test_shell)},
#endif

};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

int8_t Plane::test_mode(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Test Mode\n\n");
    test_menu.run();
    return 0;
}

void Plane::print_hit_enter()
{
    cliSerial->printf("Hit Enter to exit.\n\n");
}

/*
 *  test the dataflash is working
 */
int8_t Plane::test_logging(uint8_t argc, const Menu::arg *argv)
{
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
/*
 *  run a debug shell
 */
int8_t Plane::test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

int8_t Plane::test_gps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    hal.scheduler->delay(1000);

    uint32_t last_message_time_ms = 0;
    while(1) {
        hal.scheduler->delay(100);

        gps.update();

        if (gps.last_message_time_ms() != last_message_time_ms) {
            last_message_time_ms = gps.last_message_time_ms();
            const Location &loc = gps.location();
            cliSerial->printf("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n",
                                (long)loc.lat,
                                (long)loc.lng,
                                (long)loc.alt/100,
                                (int)gps.num_sats());
        } else {
            cliSerial->printf(".");
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

int8_t Plane::test_ins(uint8_t argc, const Menu::arg *argv)
{
    //cliSerial->printf("Calibrating.");
    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    print_hit_enter();
    hal.scheduler->delay(1000);
    
    uint8_t counter = 0;

    while(1) {
        hal.scheduler->delay(20);
        if (micros() - perf.fast_loopTimer_us > 19000UL) {
            perf.fast_loopTimer_us = micros();

            // INS
            // ---
            ahrs.update();

            if(g.compass_enabled) {
                counter++;
                if(counter == 5) {
                    compass.read();
                    counter = 0;
                }
            }

            // We are using the INS
            // ---------------------
            Vector3f gyros  = ins.get_gyro();
            Vector3f accels = ins.get_accel();
            cliSerial->printf("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n",
                            (int)ahrs.roll_sensor / 100,
                            (int)ahrs.pitch_sensor / 100,
                            (uint16_t)ahrs.yaw_sensor / 100,
                            (double)gyros.x, (double)gyros.y, (double)gyros.z,
                            (double)accels.x, (double)accels.y, (double)accels.z);
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


int8_t Plane::test_mag(uint8_t argc, const Menu::arg *argv)
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
    ahrs.set_wind_estimation(true);
    ahrs.set_compass(&compass);

    // we need the AHRS initialised for this test
    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

    uint16_t counter = 0;
    float heading = 0;

    print_hit_enter();

    while(1) {
        hal.scheduler->delay(20);
        if (micros() - perf.fast_loopTimer_us > 19000UL) {
            perf.fast_loopTimer_us = micros();

            // INS
            // ---
            ahrs.update();

            if(counter % 5 == 0) {
                if (compass.read()) {
                    // Calculate heading
                    const Matrix3f &m = ahrs.get_rotation_body_to_ned();
                    heading = compass.calculate_heading(m);
                    compass.learn_offsets();
                }
            }

            counter++;
            if (counter>20) {
                if (compass.healthy()) {
                    const Vector3f &mag_ofs = compass.get_offsets();
                    const Vector3f &mag = compass.get_field();
                    cliSerial->printf("Heading: %f, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n",
                                        (double)((wrap_360_cd(ToDeg(heading) * 100)) /100),
                                        (double)mag.x, (double)mag.y, (double)mag.z,
                                        (double)mag_ofs.x, (double)mag_ofs.y, (double)mag_ofs.z);
                } else {
                    cliSerial->printf("compass not healthy\n");
                }
                counter=0;
            }
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

int8_t Plane::test_airspeed(uint8_t argc, const Menu::arg *argv)
{
    if (!airspeed.enabled()) {
        cliSerial->printf("airspeed: ");
        print_enabled(false);
        return (0);
    }else{
        print_hit_enter();
        zero_airspeed(false);
        cliSerial->printf("airspeed: ");
        print_enabled(true);

        while(1) {
            hal.scheduler->delay(20);
            read_airspeed();
            cliSerial->printf("%.1f m/s\n", (double)airspeed.get_airspeed());

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    }
}


int8_t Plane::test_pressure(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Uncalibrated relative airpressure\n");
    print_hit_enter();

    init_barometer(true);

    while(1) {
        hal.scheduler->delay(100);
        barometer.update();

        if (!barometer.healthy()) {
            cliSerial->printf("not healthy\n");
        } else {
            cliSerial->printf("Alt: %0.2fm, Raw: %f Temperature: %.1f\n",
                                (double)barometer.get_altitude(),
                                (double)barometer.get_pressure(),
                                (double)barometer.get_temperature());
        }

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

void Plane::print_enabled(bool b)
{
    if (b) {
        cliSerial->printf("en");
    } else {
        cliSerial->printf("dis");
    }
    cliSerial->printf("abled\n");
}

#endif // CLI_ENABLED
