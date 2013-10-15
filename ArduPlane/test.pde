// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   test_radio_pwm(uint8_t argc,    const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_passthru(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_failsafe(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_gps(uint8_t argc,                  const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
static int8_t   test_adc(uint8_t argc,                  const Menu::arg *argv);
#endif
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_wp(uint8_t argc,                   const Menu::arg *argv);
static int8_t   test_airspeed(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_pressure(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_mag(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_xbee(uint8_t argc,                 const Menu::arg *argv);
static int8_t   test_eedump(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_rawgps(uint8_t argc,                       const Menu::arg *argv);
static int8_t   test_modeswitch(uint8_t argc,           const Menu::arg *argv);
static int8_t   test_logging(uint8_t argc,              const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static int8_t   test_shell(uint8_t argc,              const Menu::arg *argv);
#endif

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] PROGMEM = {
    {"pwm",                         test_radio_pwm},
    {"radio",                       test_radio},
    {"passthru",            test_passthru},
    {"failsafe",            test_failsafe},
    {"relay",                       test_relay},
    {"waypoints",           test_wp},
    {"xbee",                        test_xbee},
    {"eedump",                      test_eedump},
    {"modeswitch",          test_modeswitch},

    // Tests below here are for hardware sensors only present
    // when real sensors are attached or they are emulated
#if HIL_MODE == HIL_MODE_DISABLED
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM1
    {"adc",                 test_adc},
 #endif
    {"gps",                 test_gps},
    {"rawgps",              test_rawgps},
    {"ins",                 test_ins},
    {"airspeed",    test_airspeed},
    {"airpressure", test_pressure},
    {"compass",             test_mag},
#else
    {"gps",                 test_gps},
    {"ins",                 test_ins},
    {"compass",             test_mag},
#endif
    {"logging",             test_logging},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    {"shell", 				test_shell},
#endif

};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("Test Mode\n\n"));
    test_menu.run();
    return 0;
}

static void print_hit_enter()
{
    cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

static int8_t
test_eedump(uint8_t argc, const Menu::arg *argv)
{
    uint16_t i, j;

    // hexdump the EEPROM
    for (i = 0; i < EEPROM_MAX_ADDR; i += 16) {
        cliSerial->printf_P(PSTR("%04x:"), i);
        for (j = 0; j < 16; j++)
            cliSerial->printf_P(PSTR(" %02x"), hal.storage->read_byte(i + j));
        cliSerial->println();
    }
    return(0);
}

static int8_t
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);

        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        cliSerial->printf_P(PSTR("IN:\t1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        (int)channel_roll->radio_in,
                        (int)channel_pitch->radio_in,
                        (int)channel_throttle->radio_in,
                        (int)channel_rudder->radio_in,
                        (int)g.rc_5.radio_in,
                        (int)g.rc_6.radio_in,
                        (int)g.rc_7.radio_in,
                        (int)g.rc_8.radio_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


static int8_t
test_passthru(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);

        // New radio frame? (we could use also if((millis()- timer) > 20)
        if (hal.rcin->valid_channels() > 0) {
            cliSerial->print_P(PSTR("CH:"));
            for(int16_t i = 0; i < 8; i++) {
                cliSerial->print(hal.rcin->read(i));        // Print channel values
                print_comma();
                servo_write(i, hal.rcin->read(i)); // Copy input to Servos
            }
            cliSerial->println();
        }
        if (cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    // read the radio to set trims
    // ---------------------------
    trim_radio();

    while(1) {
        delay(20);
        read_radio();

        channel_roll->calc_pwm();
        channel_pitch->calc_pwm();
        channel_throttle->calc_pwm();
        channel_rudder->calc_pwm();

        // write out the servo PWM values
        // ------------------------------
        set_servos();

        cliSerial->printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        (int)channel_roll->control_in,
                        (int)channel_pitch->control_in,
                        (int)channel_throttle->control_in,
                        (int)channel_rudder->control_in,
                        (int)g.rc_5.control_in,
                        (int)g.rc_6.control_in,
                        (int)g.rc_7.control_in,
                        (int)g.rc_8.control_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_failsafe(uint8_t argc, const Menu::arg *argv)
{
    uint8_t fail_test;
    print_hit_enter();
    for(int16_t i = 0; i < 50; i++) {
        delay(20);
        read_radio();
    }

    // read the radio to set trims
    // ---------------------------
    trim_radio();

    oldSwitchPosition = readSwitch();

    cliSerial->printf_P(PSTR("Unplug battery, throttle in neutral, turn off radio.\n"));
    while(channel_throttle->control_in > 0) {
        delay(20);
        read_radio();
    }

    while(1) {
        delay(20);
        read_radio();

        if(channel_throttle->control_in > 0) {
            cliSerial->printf_P(PSTR("THROTTLE CHANGED %d \n"), (int)channel_throttle->control_in);
            fail_test++;
        }

        if(oldSwitchPosition != readSwitch()) {
            cliSerial->printf_P(PSTR("CONTROL MODE CHANGED: "));
            print_flight_mode(cliSerial, readSwitch());
            cliSerial->println();
            fail_test++;
        }

        if(g.throttle_fs_enabled && channel_throttle->get_failsafe()) {
            cliSerial->printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), (int)channel_throttle->radio_in);
            print_flight_mode(cliSerial, readSwitch());
            cliSerial->println();
            fail_test++;
        }

        if(fail_test > 0) {
            return (0);
        }
        if(cliSerial->available() > 0) {
            cliSerial->printf_P(PSTR("LOS caused no change in APM.\n"));
            return (0);
        }
    }
}

static int8_t
test_relay(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        cliSerial->printf_P(PSTR("Relay on\n"));
        relay.on();
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }

        cliSerial->printf_P(PSTR("Relay off\n"));
        relay.off();
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
    delay(1000);

    // save the alitude above home option
    if (g.RTL_altitude_cm < 0) {
        cliSerial->printf_P(PSTR("Hold current altitude\n"));
    }else{
        cliSerial->printf_P(PSTR("Hold altitude of %dm\n"), (int)g.RTL_altitude_cm/100);
    }

    cliSerial->printf_P(PSTR("%d waypoints\n"), (int)g.command_total);
    cliSerial->printf_P(PSTR("Hit radius: %d\n"), (int)g.waypoint_radius);
    cliSerial->printf_P(PSTR("Loiter radius: %d\n\n"), (int)g.loiter_radius);

    for(uint8_t i = 0; i <= g.command_total; i++) {
        struct Location temp = get_cmd_with_index(i);
        test_wp_print(&temp, i);
    }

    return (0);
}

static void
test_wp_print(const struct Location *cmd, uint8_t wp_index)
{
    cliSerial->printf_P(PSTR("command #: %d id:%d options:%d p1:%d p2:%ld p3:%ld p4:%ld \n"),
                    (int)wp_index,
                    (int)cmd->id,
                    (int)cmd->options,
                    (int)cmd->p1,
                    (long)cmd->alt,
                    (long)cmd->lat,
                    (long)cmd->lng);
}

static int8_t
test_xbee(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);
    cliSerial->printf_P(PSTR("Begin XBee X-CTU Range and RSSI Test:\n"));

    while(1) {

        if (hal.uartC->available())
            hal.uartC->write(hal.uartC->read());

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


static int8_t
test_modeswitch(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    cliSerial->printf_P(PSTR("Control CH "));

    cliSerial->println(FLIGHT_MODE_CHANNEL, BASE_DEC);

    while(1) {
        delay(20);
        uint8_t switchPosition = readSwitch();
        if (oldSwitchPosition != switchPosition) {
            cliSerial->printf_P(PSTR("Position %d\n"),  (int)switchPosition);
            oldSwitchPosition = switchPosition;
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

/*
 *  test the dataflash is working
 */
static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
/*
 *  run a debug shell
 */
static int8_t
test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

#if CONFIG_INS_TYPE == CONFIG_INS_OILPAN || CONFIG_HAL_BOARD == HAL_BOARD_APM1
static int8_t
test_adc(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    apm1_adc.Init();
    delay(1000);
    cliSerial->printf_P(PSTR("ADC\n"));
    delay(1000);

    while(1) {
        for (int8_t i=0; i<9; i++) cliSerial->printf_P(PSTR("%.1f\t"),apm1_adc.Ch(i));
        cliSerial->println();
        delay(100);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}
#endif // CONFIG_INS_TYPE

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(100);

        g_gps->update();

        if (g_gps->new_data) {
            cliSerial->printf_P(PSTR("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n"),
                            (long)g_gps->latitude,
                            (long)g_gps->longitude,
                            (long)g_gps->altitude_cm/100,
                            (int)g_gps->num_sats);
        }else{
            cliSerial->printf_P(PSTR("."));
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
    //cliSerial->printf_P(PSTR("Calibrating."));
    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);

    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ahrs.reset();

    print_hit_enter();
    delay(1000);
    
    uint8_t counter = 0;

    while(1) {
        delay(20);
        if (hal.scheduler->micros() - fast_loopTimer_us > 19000UL) {
            fast_loopTimer_us       = hal.scheduler->micros();

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
            cliSerial->printf_P(PSTR("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n"),
                            (int)ahrs.roll_sensor / 100,
                            (int)ahrs.pitch_sensor / 100,
                            (uint16_t)ahrs.yaw_sensor / 100,
                            gyros.x, gyros.y, gyros.z,
                            accels.x, accels.y, accels.z);
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
    if (!g.compass_enabled) {
        cliSerial->printf_P(PSTR("Compass: "));
        print_enabled(false);
        return (0);
    }

    if (!compass.init()) {
        cliSerial->println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }
    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);
    ahrs.set_compass(&compass);
    report_compass();

    // we need the AHRS initialised for this test
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ahrs.reset();

    uint16_t counter = 0;
    float heading = 0;

    print_hit_enter();

    while(1) {
        delay(20);
        if (hal.scheduler->micros() - fast_loopTimer_us > 19000UL) {
            fast_loopTimer_us       = hal.scheduler->micros();

            // INS
            // ---
            ahrs.update();

            if(counter % 5 == 0) {
                if (compass.read()) {
                    // Calculate heading
                    const Matrix3f &m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.null_offsets();
                }
            }

            counter++;
            if (counter>20) {
                if (compass.healthy) {
                    Vector3f maggy = compass.get_offsets();
                    cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %d, %d, %d,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                    (wrap_360_cd(ToDeg(heading) * 100)) /100,
                                    (int)compass.mag_x,
                                    (int)compass.mag_y,
                                    (int)compass.mag_z,
                                    maggy.x,
                                    maggy.y,
                                    maggy.z);
                } else {
                    cliSerial->println_P(PSTR("compass not healthy"));
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
    cliSerial->println_P(PSTR("saving offsets"));
    compass.save_offsets();
    return (0);
}

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

#if HIL_MODE == HIL_MODE_DISABLED

static int8_t
test_airspeed(uint8_t argc, const Menu::arg *argv)
{
    if (!airspeed.enabled()) {
        cliSerial->printf_P(PSTR("airspeed: "));
        print_enabled(false);
        return (0);
    }else{
        print_hit_enter();
        zero_airspeed();
        cliSerial->printf_P(PSTR("airspeed: "));
        print_enabled(true);

        while(1) {
            delay(20);
            read_airspeed();
            cliSerial->printf_P(PSTR("%.1f m/s\n"), airspeed.get_airspeed());

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    }
}


static int8_t
test_pressure(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("Uncalibrated relative airpressure\n"));
    print_hit_enter();

    home.alt        = 0;
    wp_distance = 0;
    init_barometer();

    while(1) {
        delay(100);
        current_loc.alt = read_barometer() + home.alt;

        if (!barometer.healthy) {
            cliSerial->println_P(PSTR("not healthy"));
        } else {
            cliSerial->printf_P(PSTR("Alt: %0.2fm, Raw: %f Temperature: %.1f\n"),
                                current_loc.alt / 100.0,
                                barometer.get_pressure(), 
                                barometer.get_temperature());
        }

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_rawgps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        // Blink Yellow LED if we are sending data to GPS
        if (hal.uartC->available()) {
            hal.uartB->write(hal.uartC->read());
        }
        // Blink Red LED if we are receiving data from GPS
        if (hal.uartB->available()) {
            hal.uartC->write(hal.uartB->read());
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}
#endif // HIL_MODE == HIL_MODE_DISABLED

#endif // CLI_ENABLED
