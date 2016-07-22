// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

#if CLI_ENABLED == ENABLED

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] = {
	{"pwm",				MENU_FUNC(test_radio_pwm)},
	{"radio",			MENU_FUNC(test_radio)},
	{"passthru",		MENU_FUNC(test_passthru)},
	{"failsafe",		MENU_FUNC(test_failsafe)},
	{"relay",			MENU_FUNC(test_relay)},
	{"waypoints",		MENU_FUNC(test_wp)},
	{"modeswitch",		MENU_FUNC(test_modeswitch)},

	// Tests below here are for hardware sensors only present
	// when real sensors are attached or they are emulated
	{"gps",			MENU_FUNC(test_gps)},
	{"ins",			MENU_FUNC(test_ins)},
	{"sonartest",	MENU_FUNC(test_sonar)},
	{"compass",		MENU_FUNC(test_mag)},
	{"logging",		MENU_FUNC(test_logging)},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    {"shell", 				MENU_FUNC(test_shell)},
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

int8_t Rover::test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);

		// Filters radio input - adjust filters in the radio.cpp file
		// ----------------------------------------------------------
		read_radio();

		cliSerial->printf("IN:\t1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n",
							channel_steer->get_radio_in(),
							g.rc_2.get_radio_in(),
							channel_throttle->get_radio_in(),
							g.rc_4.get_radio_in(),
							g.rc_5.get_radio_in(),
							g.rc_6.get_radio_in(),
							g.rc_7.get_radio_in(),
							g.rc_8.get_radio_in());

		if(cliSerial->available() > 0){
			return (0);
		}
	}
}


int8_t Rover::test_passthru(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		delay(20);

        // New radio frame? (we could use also if((millis()- timer) > 20)
        if (hal.rcin->new_input()) {
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

int8_t Rover::test_radio(uint8_t argc, const Menu::arg *argv)
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

		cliSerial->printf("IN 1: %d\t2: %d\t3: %d\t4: %d\t5: %d\t6: %d\t7: %d\t8: %d\n",
							channel_steer->get_control_in(),
							g.rc_2.get_control_in(),
							channel_throttle->get_control_in(),
							g.rc_4.get_control_in(),
							g.rc_5.get_control_in(),
							g.rc_6.get_control_in(),
							g.rc_7.get_control_in(),
							g.rc_8.get_control_in());

		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

int8_t Rover::test_failsafe(uint8_t argc, const Menu::arg *argv)
{
	uint8_t fail_test = 0;
	print_hit_enter();
	for(int i = 0; i < 50; i++){
		delay(20);
		read_radio();
	}

	// read the radio to set trims
	// ---------------------------
	trim_radio();

	oldSwitchPosition = readSwitch();

	cliSerial->printf("Unplug battery, throttle in neutral, turn off radio.\n");
	while(channel_throttle->get_control_in() > 0){
		delay(20);
		read_radio();
	}

	while(1){
		delay(20);
		read_radio();

		if(channel_throttle->get_control_in() > 0){
			cliSerial->printf("THROTTLE CHANGED %d \n", channel_throttle->get_control_in());
			fail_test++;
		}

		if (oldSwitchPosition != readSwitch()){
			cliSerial->printf("CONTROL MODE CHANGED: ");
            print_mode(cliSerial, readSwitch());
            cliSerial->println();
			fail_test++;
		}

        if(throttle_failsafe_active()) {
			cliSerial->printf("THROTTLE FAILSAFE ACTIVATED: %d, ", channel_throttle->get_radio_in());
            print_mode(cliSerial, readSwitch());
            cliSerial->println();
			fail_test++;
		}

		if(fail_test > 0){
			return (0);
		}
		if(cliSerial->available() > 0){
			cliSerial->printf("LOS caused no change in APM.\n");
			return (0);
		}
	}
}

int8_t Rover::test_relay(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	while(1){
		cliSerial->printf("Relay on\n");
		relay.on(0);
		delay(3000);
		if(cliSerial->available() > 0){
			return (0);
		}

		cliSerial->printf("Relay off\n");
		relay.off(0);
		delay(3000);
		if(cliSerial->available() > 0){
			return (0);
		}
	}
}

int8_t Rover::test_wp(uint8_t argc, const Menu::arg *argv)
{
	delay(1000);

	cliSerial->printf("%u waypoints\n", (unsigned)mission.num_commands());
	cliSerial->printf("Hit radius: %f\n", (double)g.waypoint_radius.get());

	for(uint8_t i = 0; i < mission.num_commands(); i++){
        AP_Mission::Mission_Command temp_cmd;
        if (mission.read_cmd_from_storage(i,temp_cmd)) {
            test_wp_print(temp_cmd);
        }
	}

	return (0);
}

void Rover::test_wp_print(const AP_Mission::Mission_Command& cmd)
{
    cliSerial->printf("command #: %d id:%d options:%d p1:%d p2:%ld p3:%ld p4:%ld \n",
                    (int)cmd.index,
                    (int)cmd.id,
                    (int)cmd.content.location.options,
                    (int)cmd.p1,
                    (long)cmd.content.location.alt,
                    (long)cmd.content.location.lat,
                    (long)cmd.content.location.lng);
}

int8_t Rover::test_modeswitch(uint8_t argc, const Menu::arg *argv)
{
	print_hit_enter();
	delay(1000);

	cliSerial->printf("Control CH ");

	cliSerial->println(MODE_CHANNEL, BASE_DEC);

	while(1){
		delay(20);
		uint8_t switchPosition = readSwitch();
		if (oldSwitchPosition != switchPosition){
			cliSerial->printf("Position %d\n",  switchPosition);
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
int8_t Rover::test_logging(uint8_t argc, const Menu::arg *argv)
{
	cliSerial->println("Testing dataflash logging");
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
    while(1) {
        delay(100);

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

int8_t Rover::test_ins(uint8_t argc, const Menu::arg *argv)
{
	//cliSerial->printf("Calibrating.");
	ahrs.init();
    ahrs.set_fly_forward(true);

    ins.init(scheduler.get_loop_rate_hz());
    ahrs.reset();

	print_hit_enter();
	delay(1000);

    uint8_t medium_loopCounter = 0;

	while(1){
        ins.wait_for_sample();

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
        cliSerial->printf("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n",
                            (int)ahrs.roll_sensor / 100,
                            (int)ahrs.pitch_sensor / 100,
                            (uint16_t)ahrs.yaw_sensor / 100,
                            (double)gyros.x, (double)gyros.y, (double)gyros.z,
                            (double)accels.x, (double)accels.y, (double)accels.z);
        if(cliSerial->available() > 0){
            return (0);
        }
    }
}

void Rover::print_enabled(bool b)
{
       if(b)
               cliSerial->printf("en");
       else
               cliSerial->printf("dis");
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
        cliSerial->println("Compass initialisation failed!");
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

    while(1) {
        ins.wait_for_sample();
        ahrs.update();

        medium_loopCounter++;
        if(medium_loopCounter >= 5){
            if (compass.read()) {
                // Calculate heading
                Matrix3f m = ahrs.get_rotation_body_to_ned();
                heading = compass.calculate_heading(m);
                compass.learn_offsets();
            }
            medium_loopCounter = 0;
        }
        
        counter++;
        if (counter>20) {
            if (compass.healthy()) {
                const Vector3f mag_ofs = compass.get_offsets();
                const Vector3f mag = compass.get_field();
                cliSerial->printf("Heading: %f, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n",
                                    (wrap_360_cd(ToDeg(heading) * 100)) /100,
                                    (double)mag.x, (double)mag.y, (double)mag.z,
                                    (double)mag_ofs.x, (double)mag_ofs.y, (double)mag_ofs.z);
            } else {
                cliSerial->println("compass not healthy");
            }
            counter=0;
        }
        if (cliSerial->available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.    
    cliSerial->println("saving offsets");
    compass.save_offsets();
    return (0);
}

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

int8_t Rover::test_sonar(uint8_t argc, const Menu::arg *argv)
{
    init_sonar();
    delay(20);
    sonar.update();

    if (sonar.status() == RangeFinder::RangeFinder_NotConnected) {
        cliSerial->println("WARNING: Sonar is not enabled");
    }

    print_hit_enter();
    
    float sonar_dist_cm_min = 0.0f;
    float sonar_dist_cm_max = 0.0f;
    float voltage_min=0.0f, voltage_max = 0.0f;
    float sonar2_dist_cm_min = 0.0f;
    float sonar2_dist_cm_max = 0.0f;
    float voltage2_min=0.0f, voltage2_max = 0.0f;
    uint32_t last_print = 0;

	while (true) {
        delay(20);
        sonar.update();
        uint32_t now = millis();
    
        float dist_cm = sonar.distance_cm(0);
        float voltage = sonar.voltage_mv(0);
        if (is_zero(sonar_dist_cm_min)) {
            sonar_dist_cm_min = dist_cm;
            voltage_min = voltage;
        }
        sonar_dist_cm_max = MAX(sonar_dist_cm_max, dist_cm);
        sonar_dist_cm_min = MIN(sonar_dist_cm_min, dist_cm);
        voltage_min = MIN(voltage_min, voltage);
        voltage_max = MAX(voltage_max, voltage);

        dist_cm = sonar.distance_cm(1);
        voltage = sonar.voltage_mv(1);
        if (is_zero(sonar2_dist_cm_min)) {
            sonar2_dist_cm_min = dist_cm;
            voltage2_min = voltage;
        }
        sonar2_dist_cm_max = MAX(sonar2_dist_cm_max, dist_cm);
        sonar2_dist_cm_min = MIN(sonar2_dist_cm_min, dist_cm);
        voltage2_min = MIN(voltage2_min, voltage);
        voltage2_max = MAX(voltage2_max, voltage);

        if (now - last_print >= 200) {
            cliSerial->printf("sonar1 dist=%.1f:%.1fcm volt1=%.2f:%.2f   sonar2 dist=%.1f:%.1fcm volt2=%.2f:%.2f\n",
                                (double)sonar_dist_cm_min,
                                (double)sonar_dist_cm_max,
                                (double)voltage_min,
                                (double)voltage_max,
                                (double)sonar2_dist_cm_min,
                                (double)sonar2_dist_cm_max,
                                (double)voltage2_min,
                                (double)voltage2_max);
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

#endif // CLI_ENABLED
