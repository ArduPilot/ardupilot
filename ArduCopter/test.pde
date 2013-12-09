// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
static int8_t   test_baro(uint8_t argc,                 const Menu::arg *argv);
#endif
static int8_t   test_compass(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_gps(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_logging(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_motors(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_optflow(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_radio_pwm(uint8_t argc,            const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static int8_t   test_shell(uint8_t argc,                const Menu::arg *argv);
#endif
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
static int8_t   test_sonar(uint8_t argc,                const Menu::arg *argv);
#endif

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
    {"baro",                test_baro},
#endif
    {"compass",             test_compass},
    {"gps",                 test_gps},
    {"ins",                 test_ins},
    {"logging",             test_logging},
    {"motors",              test_motors},
    {"optflow",             test_optflow},
    {"pwm",                 test_radio_pwm},
    {"radio",               test_radio},
    {"relay",               test_relay},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    {"shell", 				test_shell},
#endif
#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
    {"sonar",               test_sonar},
#endif
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
    test_menu.run();
    return 0;
}

#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
static int8_t
test_baro(uint8_t argc, const Menu::arg *argv)
{
    int32_t alt;
    print_hit_enter();
    init_barometer();

    while(1) {
        delay(100);
        alt = read_barometer();

        if (!barometer.healthy) {
            cliSerial->println_P(PSTR("not healthy"));
        } else {
            cliSerial->printf_P(PSTR("Alt: %0.2fm, Raw: %f Temperature: %.1f\n"),
                                alt / 100.0,
                                barometer.get_pressure(), 
                                barometer.get_temperature());
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}
#endif

static int8_t
test_compass(uint8_t argc, const Menu::arg *argv)
{
    uint8_t delta_ms_fast_loop;
    uint8_t medium_loopCounter = 0;

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
    ahrs.set_compass(&compass);
    report_compass();

    // we need the AHRS initialised for this test
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ahrs.reset();
    int16_t counter = 0;
    float heading = 0;

    print_hit_enter();

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer;
            G_Dt                    = (float)delta_ms_fast_loop / 1000.f;                       // used by DCM integrator
            fast_loopTimer          = millis();

            // INS
            // ---
            ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5) {
                if (compass.read()) {
                    // Calculate heading
                    const Matrix3f &m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.null_offsets();
                }
                medium_loopCounter = 0;
            }

            counter++;
            if (counter>20) {
                if (compass.healthy()) {
                    const Vector3f &mag_ofs = compass.get_offsets();
                    const Vector3f &mag = compass.get_field();
                    cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                        (wrap_360_cd(ToDeg(heading) * 100)) /100,
                                        mag.x,
                                        mag.y,
                                        mag.z,
                                        mag_ofs.x,
                                        mag_ofs.y,
                                        mag_ofs.z);
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

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(100);

        g_gps->update();

        if (g_gps->new_data) {
            cliSerial->printf_P(PSTR("Lat: "));
            print_latlon(cliSerial, g_gps->latitude);
            cliSerial->printf_P(PSTR(", Lon "));
            print_latlon(cliSerial, g_gps->longitude);
            cliSerial->printf_P(PSTR(", Alt: %ldm, #sats: %d\n"),
                            g_gps->altitude_cm/100,
                            g_gps->num_sats);
            g_gps->new_data = false;
        }else{
            cliSerial->print_P(PSTR("."));
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
    Vector3f gyro, accel;
    print_hit_enter();
    cliSerial->printf_P(PSTR("INS\n"));
    delay(1000);

    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    cliSerial->printf_P(PSTR("...done\n"));

    delay(50);

    while(1) {
        ins.update();
        gyro = ins.get_gyro();
        accel = ins.get_accel();

        float test = accel.length() / GRAVITY_MSS;

        cliSerial->printf_P(PSTR("a %7.4f %7.4f %7.4f g %7.4f %7.4f %7.4f t %7.4f \n"),
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            test);

        delay(40);
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
    cliSerial->println_P(PSTR("Testing dataflash logging"));
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}

static int8_t
test_motors(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR(
                        "Connect battery for this test.\n"
                        "Motors will spin by frame position order.\n"
                        "Front (& right of centerline) motor first, then in clockwise order around frame.\n"
                        "Remember to disconnect battery after this test.\n"
                        "Any key to exit.\n"));

    // ensure all values have been sent to motors
    motors.set_update_rate(g.rc_speed);
    motors.set_frame_orientation(g.frame_orientation);
    motors.set_min_throttle(g.throttle_min);
    motors.set_mid_throttle(g.throttle_mid);

    // enable motors
    init_rc_out();

    while(1) {
        delay(20);
        read_radio();
        motors.output_test();
        if(cliSerial->available() > 0) {
            g.esc_calibrate.set_and_save(0);
            return(0);
        }
    }
}

static int8_t
test_optflow(uint8_t argc, const Menu::arg *argv)
{
#if OPTFLOW == ENABLED
    if(g.optflow_enabled) {
        cliSerial->printf_P(PSTR("man id: %d\t"),optflow.read_register(ADNS3080_PRODUCT_ID));
        print_hit_enter();

        while(1) {
            delay(200);
            optflow.update();
            cliSerial->printf_P(PSTR("x/dx: %d/%d\t y/dy %d/%d\t squal:%d\n"),
                            optflow.x,
                            optflow.dx,
                            optflow.y,
                            optflow.dy,
                            optflow.surface_quality);

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("OptFlow: "));
        print_enabled(false);
    }
    return (0);
#else
    return (0);
#endif      // OPTFLOW == ENABLED
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

        // servo Yaw
        //APM_RC.OutputCh(CH_7, g.rc_4.radio_out);

        cliSerial->printf_P(PSTR("IN: 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
                        g.rc_1.radio_in,
                        g.rc_2.radio_in,
                        g.rc_3.radio_in,
                        g.rc_4.radio_in,
                        g.rc_5.radio_in,
                        g.rc_6.radio_in,
                        g.rc_7.radio_in,
                        g.rc_8.radio_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_radio(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(20);
        read_radio();


        cliSerial->printf_P(PSTR("IN  1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\n"),
                        g.rc_1.control_in,
                        g.rc_2.control_in,
                        g.rc_3.control_in,
                        g.rc_4.control_in,
                        g.rc_5.control_in,
                        g.rc_6.control_in,
                        g.rc_7.control_in);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t test_relay(uint8_t argc, const Menu::arg *argv)
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

#if HIL_MODE != HIL_MODE_ATTITUDE && HIL_MODE != HIL_MODE_SENSORS
/*
 *  test the sonar
 */
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
#if CONFIG_SONAR == ENABLED
    if(g.sonar_enabled == false) {
        cliSerial->printf_P(PSTR("Sonar disabled\n"));
        return (0);
    }

    // make sure sonar is initialised
    init_sonar();

    print_hit_enter();
    while(1) {
        delay(100);

        cliSerial->printf_P(PSTR("Sonar: %d cm\n"), sonar->read());

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
    return (0);
}
#endif

static void print_hit_enter()
{
    cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

#endif // CLI_ENABLED
