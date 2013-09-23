// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t	test_radio_pwm(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_radio(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_passthru(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_failsafe(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_gps(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_ins(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_battery(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_relay(uint8_t argc,	 	const Menu::arg *argv);
static int8_t	test_wp(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_sonar(uint8_t argc, 	const Menu::arg *argv);
static int8_t	test_mag(uint8_t argc, 			const Menu::arg *argv);
static int8_t	test_modeswitch(uint8_t argc, 		const Menu::arg *argv);
static int8_t	test_logging(uint8_t argc, 		const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static int8_t   test_shell(uint8_t argc,              const Menu::arg *argv);
#endif

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] PROGMEM = {
	{"pwm",				test_radio_pwm},
	{"radio",			test_radio},
	{"passthru",		test_passthru},
	{"failsafe",		test_failsafe},
	{"battery",	test_battery},
	{"relay",			test_relay},
	{"waypoints",		test_wp},
	{"modeswitch",		test_modeswitch},

	// Tests below here are for hardware sensors only present
	// when real sensors are attached or they are emulated
	{"gps",			test_gps},
	{"ins",			test_ins},
	{"sonartest",	test_sonar},
	{"compass",		test_mag},
	{"logging",		test_logging},
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
test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);

		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();

		cliSerial->printf_P(PSTR("IN:\t1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
							channel_steer->radio_in,
							g.rc_2.radio_in,
							channel_throttle->radio_in,
							g.rc_4.radio_in,
							g.rc_5.radio_in,
							g.rc_6.radio_in,
							g.rc_7.radio_in,
							g.rc_8.radio_in);

		if(cliSerial->available() > 0){
			return (0);
		}
	}
}


static int8_t
test_passthru(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);

        // New radio frame? (we could use also if((millis()- timer) > 20)
        if (hal.rcin->valid_channels() > 0) {
            cliSerial->print("CH:");
            for(int i = 0; i < 8; i++){
                cliSerial->print(hal.rcin->read(i));	// Print channel values
                cliSerial->print(",");
                hal.rcout->write(i, hal.rcin->read(i)); // Copy input to Servos
            }
            cliSerial->println();
        }
        if (cliSerial->available() > 0){
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

	while(1){
		delay(20);
		read_radio();

		channel_steer->calc_pwm();
		channel_throttle->calc_pwm();

		// write out the servo PWM values
		// ------------------------------
		set_servos();

		cliSerial->printf_P(PSTR("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n"),
							channel_steer->control_in,
							g.rc_2.control_in,
							channel_throttle->control_in,
							g.rc_4.control_in,
							g.rc_5.control_in,
							g.rc_6.control_in,
							g.rc_7.control_in,
							g.rc_8.control_in);

		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

static int8_t
test_failsafe(uint8_t argc, const Menu::arg *argv)
{
	uint8_t fail_test;
	print_hit_enter();
	for(int i = 0; i < 50; i++){
		delay(20);
		read_radio();
	}

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	oldSwitchPosition = readSwitch();

	cliSerial->printf_P(PSTR("Unplug battery, throttle in neutral, turn off radio.\n"));
	while(channel_throttle->control_in > 0){
		delay(20);
		read_radio();
	}

	while(1){
		delay(20);
		read_radio();

		if(channel_throttle->control_in > 0){
			cliSerial->printf_P(PSTR("THROTTLE CHANGED %d \n"), channel_throttle->control_in);
			fail_test++;
		}

		if (oldSwitchPosition != readSwitch()){
			cliSerial->printf_P(PSTR("CONTROL MODE CHANGED: "));
            print_mode(cliSerial, readSwitch());
            cliSerial->println();
			fail_test++;
		}

		if (g.fs_throttle_enabled && channel_throttle->get_failsafe()){
			cliSerial->printf_P(PSTR("THROTTLE FAILSAFE ACTIVATED: %d, "), channel_throttle->radio_in);
            print_mode(cliSerial, readSwitch());
            cliSerial->println();
			fail_test++;
		}

		if(fail_test > 0){
			return (0);
		}
		if(cliSerial->available() > 0){
			cliSerial->printf_P(PSTR("LOS caused no change in APM.\n"));
			return (0);
		}
	}
}

static int8_t
test_battery(uint8_t argc, const Menu::arg *argv)
{
if (g.battery_monitoring == 3 || g.battery_monitoring == 4) {
	print_hit_enter();

	while(1){
		delay(100);
		read_radio();
		read_battery();
		if (g.battery_monitoring == 3){
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

		if(cliSerial->available() > 0){
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

	while(1){
		cliSerial->printf_P(PSTR("Relay on\n"));
		relay.on();
		delay(3000);
		if(cliSerial->available() > 0){
			return (0);
		}

		cliSerial->printf_P(PSTR("Relay off\n"));
		relay.off();
		delay(3000);
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

static int8_t
test_wp(uint8_t argc, const Menu::arg *argv)
{
	delay(1000);

	cliSerial->printf_P(PSTR("%u waypoints\n"), (unsigned)g.command_total);
	cliSerial->printf_P(PSTR("Hit radius: %f\n"), g.waypoint_radius);

	for(uint8_t i = 0; i <= g.command_total; i++){
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
		cmd->alt,
		cmd->lat,
		cmd->lng);
}

static int8_t
test_modeswitch(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	cliSerial->printf_P(PSTR("Control CH "));

	cliSerial->println(MODE_CHANNEL, BASE_DEC);

	while(1){
		delay(20);
		uint8_t switchPosition = readSwitch();
		if (oldSwitchPosition != switchPosition){
			cliSerial->printf_P(PSTR("Position %d\n"),  switchPosition);
			oldSwitchPosition = switchPosition;
		}
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

/*
  test the dataflash is working
 */
static int8_t
test_logging(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->println_P(PSTR("Testing dataflash logging"));
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}


//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

static int8_t
test_gps(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(100);

		g_gps->update();

		if (g_gps->new_data){
			cliSerial->printf_P(PSTR("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n"),
					g_gps->latitude,
					g_gps->longitude,
					g_gps->altitude_cm/100,
					g_gps->num_sats);
		}else{
			cliSerial->printf_P(PSTR("."));
		}
		if(cliSerial->available() > 0){
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
             ins_sample_rate);
    ahrs.reset();

	print_hit_enter();
	delay(1000);

    uint8_t medium_loopCounter = 0;

	while(1){
		delay(20);
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();

			// INS
			// ---
			ahrs.update();

			if(g.compass_enabled) {
				medium_loopCounter++;
				if(medium_loopCounter >= 5){
					compass.read();
                    medium_loopCounter = 0;
				}
			}

			// We are using the IMU
			// ---------------------
            Vector3f gyros 	= ins.get_gyro();
            Vector3f accels = ins.get_accel();
			cliSerial->printf_P(PSTR("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n"),
                            (int)ahrs.roll_sensor / 100,
                            (int)ahrs.pitch_sensor / 100,
                            (uint16_t)ahrs.yaw_sensor / 100,
                            gyros.x, gyros.y, gyros.z,
                            accels.x, accels.y, accels.z);
		}
		if(cliSerial->available() > 0){
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
    ahrs.set_compass(&compass);
    report_compass();

    // we need the AHRS initialised for this test
	ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ahrs.reset();

	int counter = 0;
    float heading = 0;

    print_hit_enter();

    uint8_t medium_loopCounter = 0;

    while(1) {
		delay(20);
		if (millis() - fast_loopTimer > 19) {
			delta_ms_fast_loop 	= millis() - fast_loopTimer;
			G_Dt 				= (float)delta_ms_fast_loop / 1000.f;		// used by DCM integrator
			fast_loopTimer		= millis();

			// IMU
			// ---
			ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter >= 5){
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

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
    if (!sonar.enabled()) {
        cliSerial->println_P(PSTR("WARNING: Sonar is not enabled"));
    }

    print_hit_enter();
    init_sonar();
    
    float sonar_dist_cm_min = 0.0f;
    float sonar_dist_cm_max = 0.0f;
    float voltage_min=0.0f, voltage_max = 0.0f;
    float sonar2_dist_cm_min = 0.0f;
    float sonar2_dist_cm_max = 0.0f;
    float voltage2_min=0.0f, voltage2_max = 0.0f;
    uint32_t last_print = 0;

	while (true) {
        delay(20);
        uint32_t now = millis();

        float dist_cm = sonar.distance_cm();
        float voltage = sonar.voltage();
        if (sonar_dist_cm_min == 0.0f) {
            sonar_dist_cm_min = dist_cm;
            voltage_min = voltage;
        }
        sonar_dist_cm_max = max(sonar_dist_cm_max, dist_cm);
        sonar_dist_cm_min = min(sonar_dist_cm_min, dist_cm);
        voltage_min = min(voltage_min, voltage);
        voltage_max = max(voltage_max, voltage);

        dist_cm = sonar2.distance_cm();
        voltage = sonar2.voltage();
        if (sonar2_dist_cm_min == 0.0f) {
            sonar2_dist_cm_min = dist_cm;
            voltage2_min = voltage;
        }
        sonar2_dist_cm_max = max(sonar2_dist_cm_max, dist_cm);
        sonar2_dist_cm_min = min(sonar2_dist_cm_min, dist_cm);
        voltage2_min = min(voltage2_min, voltage);
        voltage2_max = max(voltage2_max, voltage);

        if (now - last_print >= 200) {
            cliSerial->printf_P(PSTR("sonar1 dist=%.1f:%.1fcm volt1=%.2f:%.2f   sonar2 dist=%.1f:%.1fcm volt2=%.2f:%.2f\n"), 
                                sonar_dist_cm_min, 
                                sonar_dist_cm_max, 
                                voltage_min,
                                voltage_max,
                                sonar2_dist_cm_min, 
                                sonar2_dist_cm_max, 
                                voltage2_min,
                                voltage2_max);
            voltage_min = voltage_max = 0.0f;
            voltage2_min = voltage2_max = 0.0f;
            sonar_dist_cm_min = sonar_dist_cm_max = 0.0f;
            sonar2_dist_cm_min = sonar2_dist_cm_max = 0.0f;
            last_print = now;
        }
        if (cliSerial->available() > 0) {
            break;
	    }
    }
    return (0);
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

#endif // CLI_ENABLED
