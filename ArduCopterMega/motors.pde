/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY 10	// one secon
#define DISARM_DELAY 10	// one secon
#define LEVEL_DELAY 120 // twelve seconds
#define AUTO_LEVEL_DELAY 250 // twentyfive seconds


// called at 10hz
static void arm_motors()
{
	static int arming_counter;

	// Arm motor output : Throttle down and full yaw right for more than 2 seconds
	if (g.rc_3.control_in == 0){

		// full right
		if (g.rc_4.control_in > 4000) {

			// don't allow arming in anything but manual
			if (control_mode < ALT_HOLD){

				if (arming_counter > AUTO_LEVEL_DELAY){
					auto_level_counter = 155;
					arming_counter = 0;

				}else if (arming_counter == ARM_DELAY){
#if HIL_MODE != HIL_MODE_DISABLED
                    hil.send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif
					motor_armed 	= true;
					arming_counter 	= ARM_DELAY;

					// Clear throttle slew
					// -------------------
					//throttle_slew = 0;

					// Remember Orientation
					// --------------------
					init_simple_bearing();

					// Reset home position
					// ----------------------
					if(home_is_set)
						init_home();

					if(did_ground_start == false){
						did_ground_start = true;
						startup_ground();
					}

					// tune down compass
					// -----------------
					dcm.kp_yaw(0.08);
					dcm.ki_yaw(0);

					arming_counter++;
				} else{
					arming_counter++;
				}
			}

		// full left
		}else if (g.rc_4.control_in < -4000) {
			//Serial.print(arming_counter, DEC);
			if (arming_counter > LEVEL_DELAY){
				//Serial.print("init");
				imu.init_accel(mavlink_delay);
				arming_counter = 0;

			}else if (arming_counter == DISARM_DELAY){
#if HIL_MODE != HIL_MODE_DISABLED
                hil.send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif
				motor_armed 	= false;
				arming_counter 	= DISARM_DELAY;
				compass.save_offsets();

				// tune up compass
				// -----------------
				dcm.kp_yaw(0.8);
				dcm.ki_yaw(0.00004);

				// Clear throttle slew
				// -------------------
				//throttle_slew = 0;

				arming_counter++;

			}else{
				arming_counter++;
			}
		// centered
		}else{
			arming_counter = 0;
		}
	}else{
		arming_counter = 0;
	}
}


/*****************************************
 * Set the flight control servos based on the current calculated values
 *****************************************/
static void
set_servos_4()
{
	if (motor_armed == true && motor_auto_armed == true) {
		// creates the radio_out and pwm_out values
		output_motors_armed();
	} else{
		output_motors_disarmed();
	}
}

/*****************************************
 * Set the flight control servos based on the current calculated values
 *****************************************/





		//if (num++ > 25){
		//	num = 0;

			//Serial.print("kP: ");
			//Serial.println(g.pid_stabilize_roll.kP(),3);
			//*/


			/*
			Serial.printf("yaw: %d, lat_e: %ld, lng_e: %ld, \tnlat: %ld, nlng: %ld,\tnrll: %ld, nptc: %ld, \tcx: %.2f, sy: %.2f, \ttber: %ld, \tnber: %ld\n",
					(int)(dcm.yaw_sensor / 100),
					lat_error,
					long_error,
					nav_lat,
					nav_lon,
					nav_roll,
					nav_pitch,
					cos_yaw_x,
					sin_yaw_y,
					target_bearing,
					nav_bearing);
			//*/

			/*

			gcs_simple.write_byte(control_mode);
			//gcs_simple.write_int(motor_out[CH_1]);
			//gcs_simple.write_int(motor_out[CH_2]);
			//gcs_simple.write_int(motor_out[CH_3]);
			//gcs_simple.write_int(motor_out[CH_4]);

			gcs_simple.write_int(g.rc_3.servo_out);

			gcs_simple.write_int((int)(dcm.yaw_sensor 	/ 100));

			gcs_simple.write_int((int)nav_lat);
			gcs_simple.write_int((int)nav_lon);
			gcs_simple.write_int((int)nav_roll);
			gcs_simple.write_int((int)nav_pitch);

			//gcs_simple.write_int((int)(cos_yaw_x * 100));
			//gcs_simple.write_int((int)(sin_yaw_y * 100));

			gcs_simple.write_long(current_loc.lat);	//28
			gcs_simple.write_long(current_loc.lng);	//32
			gcs_simple.write_int((int)current_loc.alt);	//34

			gcs_simple.write_long(next_WP.lat);
			gcs_simple.write_long(next_WP.lng);
			gcs_simple.write_int((int)next_WP.alt);		//44

			gcs_simple.write_int((int)(target_bearing 	/ 100));
			gcs_simple.write_int((int)(nav_bearing 		/ 100));
			gcs_simple.write_int((int)(nav_yaw 			/ 100));

			if(altitude_sensor == BARO){
				gcs_simple.write_int((int)g.pid_baro_throttle.get_integrator());
			}else{
				gcs_simple.write_int((int)g.pid_sonar_throttle.get_integrator());
			}

			gcs_simple.write_int(g.throttle_cruise);
			gcs_simple.write_int(g.throttle_cruise);

			//24
			gcs_simple.flush(10); // Message ID

			//*/
			//Serial.printf("\n tb  %d\n", (int)(target_bearing 	/ 100));
			//Serial.printf("\n nb  %d\n", (int)(nav_bearing 	/ 100));
			//Serial.printf("\n dcm %d\n", (int)(dcm.yaw_sensor 	/ 100));

			/*Serial.printf("a %ld, e %ld, i %d, t %d, b %4.2f\n",
					current_loc.alt,
					altitude_error,
					(int)g.pid_baro_throttle.get_integrator(),
					nav_throttle,
					angle_boost());
			*/
		//}
