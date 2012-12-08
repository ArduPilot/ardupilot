// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   test_radio_pwm(uint8_t argc,    const Menu::arg *argv);
static int8_t   test_radio(uint8_t argc,                const Menu::arg *argv);
//static int8_t	test_failsafe(uint8_t argc,     const Menu::arg *argv);
//static int8_t	test_stabilize(uint8_t argc,    const Menu::arg *argv);
static int8_t   test_gps(uint8_t argc,                  const Menu::arg *argv);
//static int8_t	test_tri(uint8_t argc,          const Menu::arg *argv);
//static int8_t	test_adc(uint8_t argc,          const Menu::arg *argv);
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
//static int8_t	test_imu(uint8_t argc,          const Menu::arg *argv);
//static int8_t	test_dcm_eulers(uint8_t argc,   const Menu::arg *argv);
//static int8_t	test_dcm(uint8_t argc,          const Menu::arg *argv);
//static int8_t	test_omega(uint8_t argc,        const Menu::arg *argv);
//static int8_t	test_stab_d(uint8_t argc,       const Menu::arg *argv);
static int8_t   test_battery(uint8_t argc,              const Menu::arg *argv);
//static int8_t	test_toy(uint8_t argc,      const Menu::arg *argv);
static int8_t   test_wp_nav(uint8_t argc,               const Menu::arg *argv);
//static int8_t	test_reverse(uint8_t argc,      const Menu::arg *argv);
static int8_t   test_tuning(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
static int8_t   test_wp(uint8_t argc,                   const Menu::arg *argv);
#if HIL_MODE != HIL_MODE_ATTITUDE
static int8_t   test_baro(uint8_t argc,                 const Menu::arg *argv);
static int8_t   test_sonar(uint8_t argc,                const Menu::arg *argv);
#endif
static int8_t   test_mag(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_optflow(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_logging(uint8_t argc,              const Menu::arg *argv);
//static int8_t	test_xbee(uint8_t argc,         const Menu::arg *argv);
static int8_t   test_eedump(uint8_t argc,               const Menu::arg *argv);
static int8_t   test_rawgps(uint8_t argc,               const Menu::arg *argv);
//static int8_t	test_mission(uint8_t argc,      const Menu::arg *argv);

// this is declared here to remove compiler errors
extern void             print_latlon(BetterStream *s, int32_t lat_or_lon);      // in Log.pde

// This is the help function
// PSTR is an AVR macro to read strings from flash memory
// printf_P is a version of printf that reads from flash memory
/*static int8_t	help_test(uint8_t argc,             const Menu::arg *argv)
 *  {
 *       cliSerial->printf_P(PSTR("\n"
 *                                                "Commands:\n"
 *                                                "  radio\n"
 *                                                "  servos\n"
 *                                                "  g_gps\n"
 *                                                "  imu\n"
 *                                                "  battery\n"
 *                                                "\n"));
 *  }*/

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
    {"pwm",                 test_radio_pwm},
    {"radio",               test_radio},
//	{"failsafe",	test_failsafe},
//	{"stabilize",	test_stabilize},
    {"gps",                 test_gps},
//	{"adc",         test_adc},
    {"ins",                 test_ins},
//	{"dcm",			test_dcm_eulers},
    //{"omega",		test_omega},
//	{"stab_d",		test_stab_d},
    {"battery",             test_battery},
    {"tune",                test_tuning},
    //{"tri",			test_tri},
    {"relay",               test_relay},
    {"wp",                  test_wp},
//	{"toy",			test_toy},
#if HIL_MODE != HIL_MODE_ATTITUDE
    {"altitude",    test_baro},
    {"sonar",               test_sonar},
#endif
    {"compass",             test_mag},
    {"optflow",             test_optflow},
    //{"xbee",		test_xbee},
    {"eedump",              test_eedump},
    {"logging",             test_logging},
//	{"rawgps",		test_rawgps},
//	{"mission",		test_mission},
    //{"reverse",		test_reverse},
    {"nav",                 test_wp_nav},
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
    //cliSerial->printf_P(PSTR("Test Mode\n\n"));
    test_menu.run();
    return 0;
}

static int8_t
test_eedump(uint8_t argc, const Menu::arg *argv)
{
    uintptr_t i, j;

    // hexdump the EEPROM
    for (i = 0; i < EEPROM_MAX_ADDR; i += 16) {
        cliSerial->printf_P(PSTR("%04x:"), i);
        for (j = 0; j < 16; j++)
            cliSerial->printf_P(PSTR(" %02x"), eeprom_read_byte((const uint8_t *)(i + j)));
        cliSerial->println();
    }
    return(0);
}


static int8_t
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else
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
#endif
}

/*
 *  //static int8_t
 *  //test_tri(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *
 *       while(1){
 *               delay(20);
 *
 *               // Filters radio input - adjust filters in the radio.pde file
 *               // ----------------------------------------------------------
 *               read_radio();
 *               g.rc_4.servo_out = g.rc_4.control_in;
 *               g.rc_4.calc_pwm();
 *
 *               cliSerial->printf_P(PSTR("input: %d\toutput%d\n"),
 *                                                       g.rc_4.control_in,
 *                                                       g.rc_4.radio_out);
 *
 *               APM_RC.OutputCh(CH_TRI_YAW, g.rc_4.radio_out);
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }*/


/*
//static int8_t
//test_toy(uint8_t argc, const Menu::arg *argv)
{
	set_alt_change(ASCENDING)

 	for(altitude_error = 2000; altitude_error > -100; altitude_error--){
 		int16_t temp = get_desired_climb_rate();
		cliSerial->printf("%ld, %d\n", altitude_error, temp);
	}
 	return 0;
}
{	wp_distance = 0;
	int16_t max_speed = 0;

 	for(int16_t i = 0; i < 200; i++){
	 	int32_t temp = 2 * 100 * (wp_distance - g.waypoint_radius * 100);
		max_speed = sqrt((float)temp);
		max_speed = min(max_speed, g.waypoint_speed_max);
		cliSerial->printf("Zspeed: %ld, %d, %ld\n", temp, max_speed, wp_distance);
	 	wp_distance += 100;
	}
 	return 0;
 }
//*/

/*static int8_t
 *  //test_toy(uint8_t argc, const Menu::arg *argv)
 *  {
 *       int16_t yaw_rate;
 *       int16_t roll_rate;
 *       g.rc_1.control_in = -2500;
 *       g.rc_2.control_in = 2500;
 *
 *       g.toy_yaw_rate = 3;
 *       yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;
 *       roll_rate = ((int32_t)g.rc_2.control_in * (yaw_rate/100)) /40;
 *       cliSerial->printf("yaw_rate, %d, roll_rate, %d\n", yaw_rate, roll_rate);
 *
 *       g.toy_yaw_rate = 2;
 *       yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;
 *       roll_rate = ((int32_t)g.rc_2.control_in * (yaw_rate/100)) /40;
 *       cliSerial->printf("yaw_rate, %d, roll_rate, %d\n", yaw_rate, roll_rate);
 *
 *       g.toy_yaw_rate = 1;
 *       yaw_rate = g.rc_1.control_in / g.toy_yaw_rate;
 *       roll_rate = ((int32_t)g.rc_2.control_in * (yaw_rate/100)) /40;
 *       cliSerial->printf("yaw_rate, %d, roll_rate, %d\n", yaw_rate, roll_rate);
 *  }*/

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

        //cliSerial->printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d\n"), (g.rc_1.servo_out / 100), (g.rc_2.servo_out / 100), g.rc_3.servo_out, (g.rc_4.servo_out / 100));

        /*cliSerial->printf_P(PSTR(	"min: %d"
         *                                               "\t in: %d"
         *                                               "\t pwm_in: %d"
         *                                               "\t sout: %d"
         *                                               "\t pwm_out %d\n"),
         *                                               g.rc_3.radio_min,
         *                                               g.rc_3.control_in,
         *                                               g.rc_3.radio_in,
         *                                               g.rc_3.servo_out,
         *                                               g.rc_3.pwm_out
         *                                               );
         */
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

/*
 *  //static int8_t
 *  //test_failsafe(uint8_t argc, const Menu::arg *argv)
 *  {
 *
 * #if THROTTLE_FAILSAFE
 *       byte fail_test;
 *       print_hit_enter();
 *       for(int16_t i = 0; i < 50; i++){
 *               delay(20);
 *               read_radio();
 *       }
 *
 *       oldSwitchPosition = readSwitch();
 *
 *       cliSerial->printf_P(PSTR("Unplug battery, throttle in neutral, turn off radio.\n"));
 *       while(g.rc_3.control_in > 0){
 *               delay(20);
 *               read_radio();
 *       }
 *
 *       while(1){
 *               delay(20);
 *               read_radio();
 *
 *               if(g.rc_3.control_in > 0){
 *                       cliSerial->printf_P(PSTR("THROTTLE CHANGED %d \n"), g.rc_3.control_in);
 *                       fail_test++;
 *               }
 *
 *               if(oldSwitchPosition != readSwitch()){
 *                       cliSerial->printf_P(PSTR("CONTROL MODE CHANGED: "));
 *                       cliSerial->println(flight_mode_strings[readSwitch()]);
 *                       fail_test++;
 *               }
 *
 *               if(g.throttle_fs_enabled && g.rc_3.get_failsafe()){
 *                       cliSerial->printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), g.rc_3.radio_in);
 *                       cliSerial->println(flight_mode_strings[readSwitch()]);
 *                       fail_test++;
 *               }
 *
 *               if(fail_test > 0){
 *                       return (0);
 *               }
 *               if(cliSerial->available() > 0){
 *                       cliSerial->printf_P(PSTR("LOS caused no change in ACM.\n"));
 *                       return (0);
 *               }
 *       }
 * #else
 *               return (0);
 * #endif
 *  }
 */

/*
 *  //static int8_t
 *  //test_stabilize(uint8_t argc, const Menu::arg *argv)
 *  {
 *       static byte ts_num;
 *
 *
 *       print_hit_enter();
 *       delay(1000);
 *
 *       // setup the radio
 *       // ---------------
 *       init_rc_in();
 *
 *       control_mode = STABILIZE;
 *       cliSerial->printf_P(PSTR("g.pi_stabilize_roll.kP: %4.4f\n"), g.pi_stabilize_roll.kP());
 *       cliSerial->printf_P(PSTR("max_stabilize_dampener:%d\n\n "), max_stabilize_dampener);
 *
 *       motors.auto_armed(false);
 *       motors.armed(true);
 *
 *       while(1){
 *               // 50 hz
 *               if (millis() - fast_loopTimer > 19) {
 *                       delta_ms_fast_loop     = millis() - fast_loopTimer;
 *                       fast_loopTimer		= millis();
 *                       G_Dt               = (float)delta_ms_fast_loop / 1000.f;
 *
 *                       if(g.compass_enabled){
 *                               medium_loopCounter++;
 *                               if(medium_loopCounter == 5){
 *                   Matrix3f m = dcm.get_dcm_matrix();
 *                                       compass.read();		                // Read magnetometer
 *                   compass.null_offsets();
 *                                       medium_loopCounter = 0;
 *                               }
 *                       }
 *
 *                       // for trim features
 *                       read_trim_switch();
 *
 *                       // Filters radio input - adjust filters in the radio.pde file
 *                       // ----------------------------------------------------------
 *                       read_radio();
 *
 *                       // IMU
 *                       // ---
 *                       read_AHRS();
 *
 *                       // allow us to zero out sensors with control switches
 *                       if(g.rc_5.control_in < 600){
 *                               dcm.roll_sensor = dcm.pitch_sensor = 0;
 *                       }
 *
 *                       // custom code/exceptions for flight modes
 *                       // ---------------------------------------
 *                       update_current_flight_mode();
 *
 *                       // write out the servo PWM values
 *                       // ------------------------------
 *                       set_servos_4();
 *
 *                       ts_num++;
 *                       if (ts_num > 10){
 *                               ts_num = 0;
 *                               cliSerial->printf_P(PSTR("r: %d, p:%d, rc1:%d, "),
 *                                       (int)(dcm.roll_sensor/100),
 *                                       (int)(dcm.pitch_sensor/100),
 *                                       g.rc_1.pwm_out);
 *
 *                               print_motor_out();
 *                       }
 *                       // R: 1417,  L: 1453  F: 1453  B: 1417
 *
 *                       //cliSerial->printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)delta_ms_fast_loop, ((int)dcm.roll_sensor/100), ((int)dcm.pitch_sensor/100), ((uint16_t)dcm.yaw_sensor/100));
 *                       //cliSerial->printf_P(PSTR("timer: %d, r: %d\tp: %d\t y: %d\n"), (int)delta_ms_fast_loop, ((int)dcm.roll_sensor/100), ((int)dcm.pitch_sensor/100), ((uint16_t)dcm.yaw_sensor/100));
 *
 *                       if(cliSerial->available() > 0){
 *                               if(g.compass_enabled){
 *                                       compass.save_offsets();
 *                                       report_compass();
 *                               }
 *                               return (0);
 *                       }
 *
 *               }
 *       }
 *  }
 */


/*
 * #if HIL_MODE != HIL_MODE_ATTITUDE && CONFIG_ADC == ENABLED
 *  //static int8_t
 *  //test_adc(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       cliSerial->printf_P(PSTR("ADC\n"));
 *       delay(1000);
 *
 *  adc.Init(&timer_scheduler);
 *
 *  delay(50);
 *
 *       while(1){
 *               for(int16_t i = 0; i < 9; i++){
 *                       cliSerial->printf_P(PSTR("%.1f,"),adc.Ch(i));
 *               }
 *               cliSerial->println();
 *               delay(20);
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }
 * #endif
 */

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else
    Vector3f gyro, accel;
    float temp;
    print_hit_enter();
    cliSerial->printf_P(PSTR("INS\n"));
    delay(1000);

    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate,
             delay, flash_leds, &timer_scheduler);

    delay(50);

    while(1) {
        ins.update();
        gyro = ins.get_gyro();
        accel = ins.get_accel();
        temp = ins.temperature();

        float test = sqrt(sq(accel.x) + sq(accel.y) + sq(accel.z)) / 9.80665;

        cliSerial->printf_P(PSTR("a %7.4f %7.4f %7.4f g %7.4f %7.4f %7.4f t %74f | %7.4f\n"),
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            temp, test);

        delay(40);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
}

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
    // test disabled to save code size for 1280
#if defined( __AVR_ATmega1280__ ) || HIL_MODE != HIL_MODE_DISABLED
    print_test_disabled();
    return (0);
#else
    print_hit_enter();
    delay(1000);

    while(1) {
        delay(333);

        // Blink GPS LED if we don't have a fix
        // ------------------------------------
        update_GPS_light();

        g_gps->update();

        if (g_gps->new_data) {
            cliSerial->printf_P(PSTR("Lat: "));
            print_latlon(&Serial, g_gps->latitude);
            cliSerial->printf_P(PSTR(", Lon "));
            print_latlon(&Serial, g_gps->longitude);
            cliSerial->printf_P(PSTR(", Alt: %ldm, #sats: %d\n"),
                            g_gps->altitude/100,
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
#endif
}

/*
 *  //static int8_t
 *  //test_dcm(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *       cliSerial->printf_P(PSTR("Gyro | Accel\n"));
 *       Vector3f   _cam_vector;
 *       Vector3f   _out_vector;
 *
 *       G_Dt = .02;
 *
 *       while(1){
 *               for(byte i = 0; i <= 50; i++){
 *                       delay(20);
 *                       // IMU
 *                       // ---
 *                       read_AHRS();
 *               }
 *
 *               Matrix3f temp = dcm.get_dcm_matrix();
 *               Matrix3f temp_t = dcm.get_dcm_transposed();
 *
 *               cliSerial->printf_P(PSTR("dcm\n"
 *                                                        "%4.4f \t %4.4f \t %4.4f \n"
 *                                                        "%4.4f \t %4.4f \t %4.4f \n"
 *                                                        "%4.4f \t %4.4f \t %4.4f \n\n"),
 *                                                       temp.a.x, temp.a.y, temp.a.z,
 *                                                       temp.b.x, temp.b.y, temp.b.z,
 *                                                       temp.c.x, temp.c.y, temp.c.z);
 *
 *               int16_t _pitch         = degrees(-asin(temp.c.x));
 *               int16_t _roll      = degrees(atan2(temp.c.y, temp.c.z));
 *               int16_t _yaw       = degrees(atan2(temp.b.x, temp.a.x));
 *               cliSerial->printf_P(PSTR(	"angles\n"
 *                                                               "%d \t %d \t %d\n\n"),
 *                                                               _pitch,
 *                                                               _roll,
 *                                                               _yaw);
 *
 *               //_out_vector = _cam_vector * temp;
 *               //cliSerial->printf_P(PSTR(	"cam\n"
 *               //						"%d \t %d \t %d\n\n"),
 *               //						(int)temp.a.x * 100, (int)temp.a.y * 100, (int)temp.a.x * 100);
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }
 */
/*
 *  //static int8_t
 *  //test_dcm(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *       cliSerial->printf_P(PSTR("Gyro | Accel\n"));
 *       delay(1000);
 *
 *       while(1){
 *               Vector3f accels = dcm.get_accel();
 *               cliSerial->print("accels.z:");
 *               cliSerial->print(accels.z);
 *               cliSerial->print("omega.z:");
 *               cliSerial->print(omega.z);
 *               delay(100);
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }
 */

/*static int8_t
 *  //test_omega(uint8_t argc, const Menu::arg *argv)
 *  {
 *       static byte ts_num;
 *       float old_yaw;
 *
 *       print_hit_enter();
 *       delay(1000);
 *       cliSerial->printf_P(PSTR("Omega"));
 *       delay(1000);
 *
 *       G_Dt = .02;
 *
 *       while(1){
 *               delay(20);
 *               // IMU
 *               // ---
 *               read_AHRS();
 *
 *               float my_oz = (dcm.yaw - old_yaw) * 50;
 *
 *               old_yaw = dcm.yaw;
 *
 *               ts_num++;
 *               if (ts_num > 2){
 *                       ts_num = 0;
 *                       //cliSerial->printf_P(PSTR("R: %4.4f\tP: %4.4f\tY: %4.4f\tY: %4.4f\n"), omega.x, omega.y, omega.z, my_oz);
 *                       cliSerial->printf_P(PSTR(" Yaw: %ld\tY: %4.4f\tY: %4.4f\n"), dcm.yaw_sensor, omega.z, my_oz);
 *               }
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *       return (0);
 *  }
 *  //*/

static int8_t
test_tuning(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();

    while(1) {
        delay(200);
        read_radio();
        tuning();
        cliSerial->printf_P(PSTR("tune: %1.3f\n"), tuning_value);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_battery(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // disable this test if we are using 1280
    print_test_disabled();
    return (0);
#else
    cliSerial->printf_P(PSTR("\nCareful! Motors will spin! Press Enter to start.\n"));
    cliSerial->flush();
    while(!cliSerial->available()) {
        delay(100);
    }
    cliSerial->flush();
    print_hit_enter();

    // allow motors to spin
    motors.enable();
    motors.armed(true);

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
            cliSerial->printf_P(PSTR("V: %4.4f, A: %4.4f, Ah: %4.4f\n"),
                            battery_voltage1,
                            current_amps1,
                            current_total1);
        }
        motors.throttle_pass_through();

        if(cliSerial->available() > 0) {
            motors.armed(false);
            return (0);
        }
    }
    motors.armed(false);
    return (0);
#endif
}

static int8_t test_relay(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else

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
#endif
}


static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
    delay(1000);

    // save the alitude above home option
    cliSerial->printf_P(PSTR("Hold alt "));
    if(g.rtl_altitude < 0) {
        cliSerial->printf_P(PSTR("\n"));
    }else{
        cliSerial->printf_P(PSTR("of %dm\n"), (int)g.rtl_altitude / 100);
    }

    cliSerial->printf_P(PSTR("%d wp\n"), (int)g.command_total);
    cliSerial->printf_P(PSTR("Hit rad: %dm\n"), (int)g.waypoint_radius);
    //cliSerial->printf_P(PSTR("Loiter radius: %d\n\n"), (int)g.loiter_radius);

    report_wp();

    return (0);
}

//static int8_t test_rawgps(uint8_t argc, const Menu::arg *argv) {
/*
 *  print_hit_enter();
 *  delay(1000);
 *  while(1){
 *          if (Serial3.available()){
 *                          digitalWrite(B_LED_PIN, LED_ON); // Blink Yellow LED if we are sending data to GPS
 *                          Serial1.write(Serial3.read());
 *                          digitalWrite(B_LED_PIN, LED_OFF);
 *          }
 *          if (Serial1.available()){
 *                          digitalWrite(C_LED_PIN, LED_ON); // Blink Red LED if we are receiving data from GPS
 *                          Serial3.write(Serial1.read());
 *                          digitalWrite(C_LED_PIN, LED_OFF);
 *          }
 *          if(cliSerial->available() > 0){
 *                          return (0);
 *  }
 *  }
 */
//}

/*static int8_t
 *  //test_xbee(uint8_t argc, const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *       cliSerial->printf_P(PSTR("Begin XBee X-CTU Range and RSSI Test:\n"));
 *
 *       while(1){
 *               if (Serial3.available())
 *                       Serial3.write(Serial3.read());
 *
 *               if(cliSerial->available() > 0){
 *                       return (0);
 *               }
 *       }
 *  }
 */

#if HIL_MODE != HIL_MODE_ATTITUDE
static int8_t
test_baro(uint8_t argc, const Menu::arg *argv)
{
 #if defined( __AVR_ATmega1280__ )         // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
 #else
    print_hit_enter();
    init_barometer();

    while(1) {
        delay(100);
        int32_t alt = read_barometer();                 // calls barometer.read()

        int32_t pres = barometer.get_pressure();
        int16_t temp = barometer.get_temperature();
        int32_t raw_pres = barometer.get_raw_pressure();
        int32_t raw_temp = barometer.get_raw_temp();
        cliSerial->printf_P(PSTR("alt: %ldcm, pres: %ldmbar, temp: %d/100degC,"
                             " raw pres: %ld, raw temp: %ld\n"),
                        alt, pres,temp, raw_pres, raw_temp);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
 #endif
}
#endif


static int8_t
test_mag(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else
    if(g.compass_enabled) {
        print_hit_enter();

        while(1) {
            delay(100);
            if (compass.read()) {
                float heading = compass.calculate_heading(ahrs.get_dcm_matrix());
                cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %d, %d, %d\n"),
                                (wrap_360(ToDeg(heading) * 100)) /100,
                                compass.mag_x,
                                compass.mag_y,
                                compass.mag_z);
            } else {
                cliSerial->println_P(PSTR("not healthy"));
            }

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("Compass: "));
        print_enabled(false);
        return (0);
    }
    return (0);
#endif
}

/*
 *  //static int8_t
 *  //test_reverse(uint8_t argc,        const Menu::arg *argv)
 *  {
 *       print_hit_enter();
 *       delay(1000);
 *
 *       while(1){
 *               delay(20);
 *
 *               // Filters radio input - adjust filters in the radio.pde file
 *               // ----------------------------------------------------------
 *               g.rc_4.set_reverse(0);
 *               g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
 *               g.rc_4.servo_out = g.rc_4.control_in;
 *               g.rc_4.calc_pwm();
 *               cliSerial->printf_P(PSTR("PWM:%d input: %d\toutput%d "),
 *                                                       APM_RC.InputCh(CH_4),
 *                                                       g.rc_4.control_in,
 *                                                       g.rc_4.radio_out);
 *               APM_RC.OutputCh(CH_6, g.rc_4.radio_out);
 *
 *
 *               g.rc_4.set_reverse(1);
 *               g.rc_4.set_pwm(APM_RC.InputCh(CH_4));
 *               g.rc_4.servo_out = g.rc_4.control_in;
 *               g.rc_4.calc_pwm();
 *               cliSerial->printf_P(PSTR("\trev input: %d\toutput%d\n"),
 *                                                       g.rc_4.control_in,
 *                                                       g.rc_4.radio_out);
 *
 *               APM_RC.OutputCh(CH_7, g.rc_4.radio_out);
 *
 *               if(cliSerial->available() > 0){
 *                       g.rc_4.set_reverse(0);
 *                       return (0);
 *               }
 *       }
 *  }*/

#if HIL_MODE != HIL_MODE_ATTITUDE
/*
 *  test the sonar
 */
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
    if(g.sonar_enabled == false) {
        cliSerial->printf_P(PSTR("Sonar disabled\n"));
        return (0);
    }

    // make sure sonar is initialised
    init_sonar();

    print_hit_enter();
    while(1) {
        delay(100);

        cliSerial->printf_P(PSTR("Sonar: %d cm\n"), sonar.read());
        //cliSerial->printf_P(PSTR("Sonar, %d, %d\n"), sonar.read(), sonar.raw_value);

        if(cliSerial->available() > 0) {
            return (0);
        }
    }

    return (0);
}
#endif


static int8_t
test_optflow(uint8_t argc, const Menu::arg *argv)
{
#if OPTFLOW == ENABLED
    if(g.optflow_enabled) {
        cliSerial->printf_P(PSTR("man id: %d\t"),optflow.read_register(ADNS3080_PRODUCT_ID));
        print_hit_enter();

        while(1) {
            delay(200);
            optflow.update(millis());
            Log_Write_Optflow();
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
    print_test_disabled();
    return (0);
#endif      // OPTFLOW == ENABLED
}


static int8_t
test_wp_nav(uint8_t argc, const Menu::arg *argv)
{
    current_loc.lat = 389539260;
    current_loc.lng = -1199540200;

    next_WP.lat = 389538528;
    next_WP.lng = -1199541248;

    // got 23506;, should be 22800
    update_navigation();
    cliSerial->printf_P(PSTR("bear: %ld\n"), wp_bearing);
    return 0;
}

/*
 *  test the dataflash is working
 */

static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
#if defined( __AVR_ATmega1280__ )          // test disabled to save code size for 1280
    print_test_disabled();
    return (0);
#else
    cliSerial->println_P(PSTR("Testing dataflash logging"));
    if (!DataFlash.CardInserted()) {
        cliSerial->println_P(PSTR("ERR: No dataflash inserted"));
        return 0;
    }
    DataFlash.ReadManufacturerID();
    cliSerial->printf_P(PSTR("Manufacturer: 0x%02x   Device: 0x%04x\n"),
                    (unsigned)DataFlash.df_manufacturer,
                    (unsigned)DataFlash.df_device);
    cliSerial->printf_P(PSTR("NumPages: %u  PageSize: %u\n"),
                    (unsigned)DataFlash.df_NumPages+1,
                    (unsigned)DataFlash.df_PageSize);
    DataFlash.StartRead(DataFlash.df_NumPages+1);
    cliSerial->printf_P(PSTR("Format version: %lx  Expected format version: %lx\n"),
                    (unsigned long)DataFlash.ReadLong(), (unsigned long)DF_LOGGING_FORMAT);
    return 0;
#endif
}


/*
 *  static int8_t
 *  //test_mission(uint8_t argc, const Menu::arg *argv)
 *  {
 *       //write out a basic mission to the EEPROM
 *
 *  //{
 *  //	uint8_t		id;					///< command id
 *  //	uint8_t		options;			///< options bitmask (1<<0 = relative altitude)
 *  //	uint8_t		p1;					///< param 1
 *  //	int32_t		alt;				///< param 2 - Altitude in centimeters (meters * 100)
 *  //	int32_t		lat;				///< param 3 - Lattitude * 10**7
 *  //	int32_t		lng;				///< param 4 - Longitude * 10**7
 *  //}
 *
 *       // clear home
 *       {Location t = {0,      0,	  0,        0,      0,          0};
 *       set_cmd_with_index(t,0);}
 *
 *       // CMD										opt						pitch       alt/cm
 *       {Location t = {MAV_CMD_NAV_TAKEOFF,        WP_OPTION_RELATIVE,	  0,        100,        0,          0};
 *       set_cmd_with_index(t,1);}
 *
 *       if (!strcmp_P(argv[1].str, PSTR("wp"))) {
 *
 *               // CMD											opt
 *               {Location t = {MAV_CMD_NAV_WAYPOINT,           WP_OPTION_RELATIVE,		15, 0, 0, 0};
 *               set_cmd_with_index(t,2);}
 *               // CMD											opt
 *               {Location t = {MAV_CMD_NAV_RETURN_TO_LAUNCH,       WP_OPTION_YAW,	  0,        0,      0,		0};
 *               set_cmd_with_index(t,3);}
 *
 *               // CMD											opt
 *               {Location t = {MAV_CMD_NAV_LAND,				0,	  0,        0,      0,		0};
 *               set_cmd_with_index(t,4);}
 *
 *       } else {
 *               //2250 = 25 meteres
 *               // CMD										opt		p1		//alt		//NS		//WE
 *               {Location t = {MAV_CMD_NAV_LOITER_TIME,    0,	  10,   0,          0,			0}; // 19
 *               set_cmd_with_index(t,2);}
 *
 *               // CMD										opt		dir		angle/deg	deg/s	relative
 *               {Location t = {MAV_CMD_CONDITION_YAW,		0,	  1,        360,        60,     1};
 *               set_cmd_with_index(t,3);}
 *
 *               // CMD										opt
 *               {Location t = {MAV_CMD_NAV_LAND,			0,	  0,        0,          0,      0};
 *               set_cmd_with_index(t,4);}
 *
 *       }
 *
 *       g.rtl_altitude.set_and_save(300);
 *       g.command_total.set_and_save(4);
 *       g.waypoint_radius.set_and_save(3);
 *
 *       test_wp(NULL, NULL);
 *       return (0);
 *  }
 */

static void print_hit_enter()
{
    cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

static void print_test_disabled()
{
    cliSerial->printf_P(PSTR("Sorry, not 1280 compat.\n"));
}

/*
 *  //static void fake_out_gps()
 *  {
 *       static float rads;
 *       g_gps->new_data    = true;
 *       g_gps->fix	    = true;
 *
 *       //int length = g.rc_6.control_in;
 *       rads += .05;
 *
 *       if (rads > 6.28){
 *               rads = 0;
 *       }
 *
 *       g_gps->latitude	= 377696000;	// Y
 *       g_gps->longitude	= -1224319000;	// X
 *       g_gps->altitude	= 9000;			// meters * 100
 *
 *       //next_WP.lng	    = home.lng - length * sin(rads);   // X
 *       //next_WP.lat  = home.lat + length * cos(rads);   // Y
 *  }
 *
 */
/*
 *  //static void print_motor_out(){
 *       cliSerial->printf("out: R: %d,  L: %d  F: %d  B: %d\n",
 *                               (motor_out[CH_1]   - g.rc_3.radio_min),
 *                               (motor_out[CH_2]   - g.rc_3.radio_min),
 *                               (motor_out[CH_3]   - g.rc_3.radio_min),
 *                               (motor_out[CH_4]   - g.rc_3.radio_min));
 *  }
 */
#endif // CLI_ENABLED
