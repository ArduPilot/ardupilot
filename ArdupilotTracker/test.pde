// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   test_radio_pwm(uint8_t argc,    const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_passthru(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_gps(uint8_t argc,                  const Menu::arg *argv);
#if CONFIG_ADC == ENABLED
static int8_t   test_adc(uint8_t argc,                  const Menu::arg *argv);
#endif
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_battery(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_pressure(uint8_t argc,     const Menu::arg *argv);
static int8_t   test_mag(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_xbee(uint8_t argc,                 const Menu::arg *argv);
static int8_t   test_eedump(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_rawgps(uint8_t argc,                       const Menu::arg *argv);
static int8_t   test_modeswitch(uint8_t argc,           const Menu::arg *argv);
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
    {"relay",                       test_relay},
    {"battery",     test_battery},
    {"xbee",                        test_xbee},
    {"eedump",                      test_eedump},
    {"modeswitch",          test_modeswitch},

    // Tests below here are for hardware sensors only present
    // when real sensors are attached or they are emulated
#if HIL_MODE == HIL_MODE_DISABLED
 #if CONFIG_ADC == ENABLED
    {"adc",                 test_adc},
 #endif
    {"gps",                 test_gps},
    {"rawgps",              test_rawgps},
    {"ins",                 test_ins},
    {"airpressure", test_pressure},
    {"compass",             test_mag},
#elif HIL_MODE == HIL_MODE_SENSORS
    {"gps",                 test_gps},
    {"ins",                 test_ins},
    {"compass",             test_mag},
#elif HIL_MODE == HIL_MODE_ATTITUDE
#endif
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

        cliSerial->printf_P(PSTR("IN:\t1: %d\t2: %d\t3: %d\t4: %d\n"),
                        (int)g.channel_azimuth.radio_in,
                        (int)g.channel_elevation.radio_in,
                        (int)g.rc_3.radio_in,
                        (int)g.rc_4.radio_in);

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
        if (hal.rcin->valid() > 0) {
            cliSerial->print_P(PSTR("CH:"));
            for(int16_t i = 0; i < 8; i++) {
                cliSerial->print(hal.rcin->read(i));        // Print channel values
                print_comma();
                hal.rcout->write(i, hal.rcin->read(i)); // Copy input to Servos
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

        g.channel_azimuth.calc_pwm();
        g.channel_elevation.calc_pwm();
        //g.channel_throttle.calc_pwm();
        //g.channel_rudder.calc_pwm();

        // write out the servo PWM values
        // ------------------------------
        set_servos();

        cliSerial->printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\n"),
                        (int)g.channel_azimuth.control_in,
                        (int)g.channel_elevation.control_in,
                        (int)g.rc_3.control_in,
                        (int)g.rc_4.control_in
                        );

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_battery(uint8_t argc, const Menu::arg *argv)
{
    if (g.battery_monitoring == 3 || g.battery_monitoring == 4) {
        print_hit_enter();
        delta_ms_medium_loop = 100;

        while(1) {
            delay(100);
            read_radio();
            read_battery();
            if (g.battery_monitoring == 3) {
                cliSerial->printf_P(PSTR("V: %4.4f\n"),
                                battery_voltage1,
                                current_amps1,
                                current_total1);
            } else {
                cliSerial->printf_P(PSTR("V: %4.4f, A: %4.4f, mAh: %4.4f\n"),
                                battery_voltage1,
                                current_amps1,
                                current_total1);
            }

            // write out the servo PWM values
            // ------------------------------
            set_servos();

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("Not enabled\n"));
        return (0);
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

    cliSerial->println(FLIGHT_MODE_CHANNEL, DEC);

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
static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}
*/

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

#if HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS

 #if CONFIG_ADC == ENABLED
static int8_t
test_adc(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    adc.Init();
    delay(1000);
    cliSerial->printf_P(PSTR("ADC\n"));
    delay(1000);

    while(1) {
        for (int8_t i=0; i<9; i++) cliSerial->printf_P(PSTR("%.1f\t"),adc.Ch(i));
        cliSerial->println();
        delay(100);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}
 #endif // CONFIG_ADC

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(333);

        // Blink GPS LED if we don't have a fix
        // ------------------------------------
        update_GPS_light();

        g_gps->update();

        if (g_gps->new_data) {
            cliSerial->printf_P(PSTR("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n"),
                            (long)g_gps->latitude,
                            (long)g_gps->longitude,
                            (long)g_gps->altitude/100,
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

    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
    ahrs.reset();

    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer_ms > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer_ms;
            G_Dt                            = (float)delta_ms_fast_loop / 1000.f;                       // used by DCM integrator
            fast_loopTimer_ms       = millis();

            // INS
            // ---
            ahrs.update();

            if(g.compass_enabled) {
                medium_loopCounter++;
                if(medium_loopCounter == 5) {
                    compass.read();
                    medium_loopCounter = 0;
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

    compass.set_orientation(MAG_ORIENTATION);
    if (!compass.init()) {
        cliSerial->println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }
    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_compass(&compass);
    report_compass();

    // we need the AHRS initialised for this test
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             flash_leds);
    ahrs.reset();

    int16_t counter = 0;
    float heading = 0;

    //cliSerial->printf_P(PSTR("MAG_ORIENTATION: %d\n"), MAG_ORIENTATION);

    print_hit_enter();

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer_ms > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer_ms;
            G_Dt                            = (float)delta_ms_fast_loop / 1000.f;                       // used by DCM integrator
            fast_loopTimer_ms       = millis();

            // INS
            // ---
            ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5) {
                if (compass.read()) {
                    // Calculate heading
                    Matrix3f m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.null_offsets();
                }
                medium_loopCounter = 0;
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

#endif // HIL_MODE == HIL_MODE_DISABLED || HIL_MODE == HIL_MODE_SENSORS

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

#if HIL_MODE == HIL_MODE_DISABLED

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
                            barometer.get_pressure(), 0.1*barometer.get_temperature());
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
            digitalWrite(B_LED_PIN, LED_ON);
            hal.uartB->write(hal.uartC->read());
            digitalWrite(B_LED_PIN, LED_OFF);
        }
        // Blink Red LED if we are receiving data from GPS
        if (hal.uartB->available()) {
            digitalWrite(C_LED_PIN, LED_ON);
            hal.uartC->write(hal.uartB->read());
            digitalWrite(C_LED_PIN, LED_OFF);
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}
#endif // HIL_MODE == HIL_MODE_DISABLED

#endif // CLI_ENABLED
