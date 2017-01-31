// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

// Functions that will handle joystick/gamepad input
// ----------------------------------------------------------------------------

// Anonymous namespace to hold variables used only in this file
namespace {
	int16_t mode_switch_pwm = 1100;
	float cam_tilt = 1500.0;
	float cam_tilt_goal = 1500.0;
	float cam_tilt_alpha = 0.97;
	int16_t lights1 = 1100;
	int16_t lights2 = 1100;
	int16_t rollTrim = 0;
	int16_t pitchTrim = 0;
	int16_t zTrim = 0;
	int16_t xTrim = 0;
	int16_t yTrim = 0;
	int16_t video_switch = 1100;
	int16_t x_last, y_last, z_last;
	uint16_t buttons_prev;
	float gain;
	bool toggle_mode = true;
}

void Sub::init_joystick() {
	default_js_buttons();

	set_mode((control_mode_t)flight_modes[0].get(), MODE_REASON_TX_COMMAND); // Initialize flight mode

    if(g.numGainSettings < 1) g.numGainSettings.set_and_save(1);

    if(g.numGainSettings == 1 || (g.gain_default < g.maxGain + 0.01 && g.gain_default > g.minGain - 0.01)) {
    	gain = constrain_float(g.gain_default, g.minGain, g.maxGain); // Use default gain parameter
    } else {
    	// Use setting closest to average of minGain and maxGain
        gain = g.minGain + (g.numGainSettings/2 - 1) * (g.maxGain - g.minGain) / (g.numGainSettings - 1);
    }

    gain = constrain_float(gain, 0.1, 1.0);
}

void Sub::transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons) {
	int16_t channels[11];

	uint32_t tnow_ms = millis();

	float rpyScale = 0.4*gain; // Scale -1000-1000 to -400-400 with gain
	float throttleScale = 0.8*gain*g.throttle_gain; // Scale 0-1000 to 0-800 times gain
	int16_t rpyCenter = 1500;
	int16_t throttleBase = 1500-500*throttleScale;

	bool shift = false;
	static uint32_t buttonDebounce;

	// Debouncing timer
	if ( tnow_ms - buttonDebounce > 100 ) {
		// Detect if any shift button is pressed
		for ( uint8_t i = 0 ; i < 16 ; i++ ) {
			if ( (buttons & (1 << i)) && get_button(i)->function() == JSButton::button_function_t::k_shift ) { shift = true; }
		}

		// Act if button is pressed
		// Only act upon pressing button and ignore holding. This provides compatibility with Taranis as joystick.
		for ( uint8_t i = 0 ; i < 16 ; i++ ) {
			if ( (buttons & (1 << i)) ) {
				handle_jsbutton_press(i,shift,(buttons_prev & (1 << i)));
				buttonDebounce = tnow_ms;
			}
		}

		buttons_prev = buttons;
	}

	// Set channels to override
	channels[0] = 1500 + pitchTrim;                           // pitch
	channels[1] = 1500 + rollTrim;                            // roll
	channels[2] = constrain_int16((z+zTrim)*throttleScale+throttleBase,1100,1900);  // throttle
	channels[3] = constrain_int16(r*rpyScale+rpyCenter,1100,1900);                       // yaw
	channels[4] = mode_switch_pwm;                                       // for testing only
	channels[5] = constrain_int16((x+xTrim)*rpyScale+rpyCenter,1100,1900);           // forward for ROV
	channels[6] = constrain_int16((y+yTrim)*rpyScale+rpyCenter,1100,1900);           // lateral for ROV
	channels[7] = 0xffff;                                     // camera tilt (sent in camera_tilt_smooth)
	channels[8] = lights1;                                    // lights 1
	channels[9] = lights2;                                    // lights 2
	channels[10] = video_switch;                              // video switch

	// Store old x, y, z values for use in input hold logic
	x_last = x;
	y_last = y;
	z_last = z;

	// record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
	failsafe.rc_override_active = hal.rcin->set_overrides(channels, 10);
}

void Sub::handle_jsbutton_press(uint8_t button, bool shift, bool held) {
	// For attempts to change control mode
	control_mode_t next_mode = control_mode;
	uint16_t next_mode_switch_pwm = mode_switch_pwm;

	// Act based on the function assigned to this button
	switch ( get_button(button)->function(shift) ) {
		case JSButton::button_function_t::k_arm_toggle:
			if ( motors.armed() ) {
				init_disarm_motors();
			} else {
				init_arm_motors(true);
			}
			break;
		case JSButton::button_function_t::k_arm:
			init_arm_motors(true);
			break;
		case JSButton::button_function_t::k_disarm:
			init_disarm_motors();
			break;
		case JSButton::button_function_t::k_mode_toggle:
			if( !held ) {
				next_mode = (control_mode_t)flight_modes[toggle_mode?1:0].get();
				next_mode_switch_pwm = toggle_mode?1300:1100;
				toggle_mode = !toggle_mode;
			}
			break;
		case JSButton::button_function_t::k_mode_1:
			next_mode = (control_mode_t)flight_modes[0].get();
			next_mode_switch_pwm = 1100;
			toggle_mode = true;
			break;
		case JSButton::button_function_t::k_mode_2:
			next_mode = (control_mode_t)flight_modes[1].get();
			next_mode_switch_pwm = 1300;
			toggle_mode = false;
			break;
		case JSButton::button_function_t::k_mode_3:
			next_mode = (control_mode_t)flight_modes[2].get();
			next_mode_switch_pwm = 1420;
			toggle_mode = false;
			break;
		case JSButton::button_function_t::k_mode_4:
			next_mode = (control_mode_t)flight_modes[3].get();
			next_mode_switch_pwm = 1550;
			toggle_mode = false;
			break;
		case JSButton::button_function_t::k_mode_5:
			next_mode = (control_mode_t)flight_modes[4].get();
			next_mode_switch_pwm = 1690;
			toggle_mode = false;
			break;
		case JSButton::button_function_t::k_mode_6:
			next_mode = (control_mode_t)flight_modes[5].get();
			next_mode_switch_pwm = 1900;
			toggle_mode = false;
			break;
		case JSButton::button_function_t::k_mount_center:
			cam_tilt_goal = g.cam_tilt_center;
			break;
		case JSButton::button_function_t::k_mount_tilt_up: {
			uint8_t i;

			// Find the first aux channel configured as mount tilt, if any
			if(SRV_Channels::find_channel(SRV_Channel::k_mount_tilt, i)) {

				// Get the channel output limits
				SRV_Channel *ch = SRV_Channels::srv_channel(i);
				uint16_t min = ch->get_output_min();
				uint16_t max = ch->get_output_max();

				cam_tilt_goal = constrain_int16(cam_tilt_goal-g.cam_tilt_step,min,max);
			}
		}
			break;
		case JSButton::button_function_t::k_mount_tilt_down: {
			uint8_t i;

			// Find the first aux channel configured as mount tilt, if any
			if(SRV_Channels::find_channel(SRV_Channel::k_mount_tilt, i)) {

				// Get the channel output limits
				SRV_Channel *ch = SRV_Channels::srv_channel(i);
				uint16_t min = ch->get_output_min();
				uint16_t max = ch->get_output_max();

				cam_tilt_goal = constrain_int16(cam_tilt_goal+g.cam_tilt_step,min,max);
			}
		}
			break;
		case JSButton::button_function_t::k_camera_trigger:
			break;
		case JSButton::button_function_t::k_camera_source_toggle:
			if ( !held ) {
				static bool video_toggle = false;
				video_toggle = !video_toggle;
				if ( video_toggle ) {
					video_switch = 1900;
					gcs_send_text(MAV_SEVERITY_INFO,"Video Toggle: Source 2");
				} else {
					video_switch = 1100;
					gcs_send_text(MAV_SEVERITY_INFO,"Video Toggle: Source 1");
				}
			}
			break;
		case JSButton::button_function_t::k_mount_pan_right:
			// Not implemented
			break;
		case JSButton::button_function_t::k_mount_pan_left:
			// Not implemented
			break;
		case JSButton::button_function_t::k_lights1_cycle:
			if ( !held ) {
				static bool increasing = true;
				if ( increasing ) {
					lights1 = constrain_float(lights1+g.lights_step,1100,1900);
				} else {
					lights1 = constrain_float(lights1-g.lights_step,1100,1900);
				}
				if ( lights1 >= 1900 || lights1 <= 1100 ) {
					increasing = !increasing;
				}
			}
			break;
		case JSButton::button_function_t::k_lights1_brighter:
			if ( !held ) {
				lights1 = constrain_float(lights1+g.lights_step,1100,1900);
			}
			break;
		case JSButton::button_function_t::k_lights1_dimmer:
			if ( !held ) {
				lights1 = constrain_float(lights1-g.lights_step,1100,1900);
			}
			break;
		case JSButton::button_function_t::k_lights2_cycle:
			if ( !held ) {
				static bool increasing = true;
				if ( increasing ) {
					lights2 = constrain_float(lights2+g.lights_step,1100,1900);
				} else {
					lights2 = constrain_float(lights2-g.lights_step,1100,1900);
				}
				if ( lights2 >= 1900 || lights2 <= 1100 ) {
					increasing = !increasing;
				}
			}
			break;
		case JSButton::button_function_t::k_lights2_brighter:
			if ( !held ) {
				lights2 = constrain_float(lights2+g.lights_step,1100,1900);
			}
			break;
		case JSButton::button_function_t::k_lights2_dimmer:
			if ( !held ) {
				lights2 = constrain_float(lights2-g.lights_step,1100,1900);
			}
			break;
		case JSButton::button_function_t::k_gain_toggle:
			if ( !held ) {
				static bool lowGain = false;
				lowGain = !lowGain;
				if ( lowGain ) {
					gain = 0.5f;
				} else {
					gain = 1.0f;
				}
				gcs_send_text_fmt(MAV_SEVERITY_INFO,"#Gain: %2.0f%%",gain*100);
			}
			break;
		case JSButton::button_function_t::k_gain_inc:
			if ( !held ) {
				// check that our gain parameters are in correct range, update in eeprom and notify gcs if needed
				g.minGain.set_and_save(constrain_float(g.minGain, 0.10, 0.80));
				g.maxGain.set_and_save(constrain_float(g.maxGain, g.minGain, 1.0));
				g.numGainSettings.set_and_save(constrain_int16(g.numGainSettings, 1, 10));

				if(g.numGainSettings == 1) {
					gain = constrain_float(g.gain_default, g.minGain, g.maxGain);
				} else {
					gain = constrain_float(gain + (g.maxGain-g.minGain)/(g.numGainSettings-1), g.minGain, g.maxGain);
				}

				gcs_send_text_fmt(MAV_SEVERITY_INFO,"#Gain is %2.0f%%",gain*100);
			}
			break;
		case JSButton::button_function_t::k_gain_dec:
			if ( !held ) {
				// check that our gain parameters are in correct range, update in eeprom and notify gcs if needed
				g.minGain.set_and_save(constrain_float(g.minGain, 0.10, 0.80));
				g.maxGain.set_and_save(constrain_float(g.maxGain, g.minGain, 1.0));
				g.numGainSettings.set_and_save(constrain_int16(g.numGainSettings, 1, 10));

				if(g.numGainSettings == 1) {
					gain = constrain_float(g.gain_default, g.minGain, g.maxGain);
				} else {
					gain = constrain_float(gain - (g.maxGain-g.minGain)/(g.numGainSettings-1), g.minGain, g.maxGain);
				}

				gcs_send_text_fmt(MAV_SEVERITY_INFO,"#Gain is %2.0f%%",gain*100);
			}
			break;
		case JSButton::button_function_t::k_trim_roll_inc:
			rollTrim = constrain_float(rollTrim+10,-200,200);
			break;
		case JSButton::button_function_t::k_trim_roll_dec:
			rollTrim = constrain_float(rollTrim-10,-200,200);
			break;
		case JSButton::button_function_t::k_trim_pitch_inc:
			pitchTrim = constrain_float(pitchTrim+10,-200,200);
			break;
		case JSButton::button_function_t::k_trim_pitch_dec:
			pitchTrim = constrain_float(pitchTrim-10,-200,200);
			break;
		case JSButton::button_function_t::k_input_hold_toggle:
			if ( !held ) {
				zTrim = z_last-500;
				xTrim = x_last;
				yTrim = y_last;
				gcs_send_text(MAV_SEVERITY_INFO,"#Input Hold Set");
			}
			break;
		case JSButton::button_function_t::k_relay_1_on:
			// Not implemented
			break;
		case JSButton::button_function_t::k_relay_1_off:
			// Not implemented
			break;
		case JSButton::button_function_t::k_relay_1_toggle:
			// Not implemented
			break;
		case JSButton::button_function_t::k_relay_2_on:
			// Not implemented
			break;
		case JSButton::button_function_t::k_relay_2_off:
			// Not implemented
			break;
		case JSButton::button_function_t::k_relay_2_toggle:
			// Not implemented
			break;
		case JSButton::button_function_t::k_custom_1:
			// Not implemented
			break;
		case JSButton::button_function_t::k_custom_2:
			// Not implemented
			break;
		case JSButton::button_function_t::k_custom_3:
			// Not implemented
			break;
		case JSButton::button_function_t::k_custom_4:
			// Not implemented
			break;
		case JSButton::button_function_t::k_custom_5:
			// Not implemented
			break;
		case JSButton::button_function_t::k_custom_6:
			// Not implemented
			break;
	}

	// Update the control mode if needed
	if(control_mode != next_mode) {
		if(set_mode(next_mode, MODE_REASON_TX_COMMAND)) {
			// Notify user
	        if (ap.initialised) {
	            AP_Notify::events.user_mode_change = 1;
	        }
	        // Update CH5 pwm value (For GCS)
	        mode_switch_pwm = next_mode_switch_pwm;
		} else {
		    // Notify user
			if(ap.initialised) {
				AP_Notify::events.user_mode_change_failed = 1;
			}
		}
	}
}

JSButton* Sub::get_button(uint8_t index) {
	// Help to access appropriate parameter
	switch (index) {
		case 0: return &g.jbtn_0;
		case 1: return &g.jbtn_1;
		case 2: return &g.jbtn_2;
		case 3: return &g.jbtn_3;
		case 4: return &g.jbtn_4;
		case 5: return &g.jbtn_5;
		case 6: return &g.jbtn_6;
		case 7: return &g.jbtn_7;
		case 8: return &g.jbtn_8;
		case 9: return &g.jbtn_9;
		case 10: return &g.jbtn_10;
		case 11: return &g.jbtn_11;
		case 12: return &g.jbtn_12;
		case 13: return &g.jbtn_13;
		case 14: return &g.jbtn_14;
		case 15: return &g.jbtn_15;
		default: return &g.jbtn_0;
	}
}

void Sub::camera_tilt_smooth() {
	int16_t channels[11];

	for ( uint8_t i = 0 ; i < 11 ; i++ ) {
		channels[i] = 0xffff;
	}
	// Camera tilt low pass filter
	cam_tilt = cam_tilt*cam_tilt_alpha+cam_tilt_goal*(1-cam_tilt_alpha);
	channels[7] = cam_tilt;

	failsafe.rc_override_active = hal.rcin->set_overrides(channels, 10);
}

void Sub::default_js_buttons() {
	JSButton::button_function_t defaults[16][2] = {
			{JSButton::button_function_t::k_none, 					JSButton::button_function_t::k_none},
			{JSButton::button_function_t::k_mode_1, 				JSButton::button_function_t::k_none},
			{JSButton::button_function_t::k_mode_3, 				JSButton::button_function_t::k_none},
			{JSButton::button_function_t::k_mode_2, 				JSButton::button_function_t::k_none},

			{JSButton::button_function_t::k_disarm, 				JSButton::button_function_t::k_none},
			{JSButton::button_function_t::k_shift, 					JSButton::button_function_t::k_none},
			{JSButton::button_function_t::k_arm, 					JSButton::button_function_t::k_none},
			{JSButton::button_function_t::k_mount_center, 			JSButton::button_function_t::k_none},

			{JSButton::button_function_t::k_input_hold_toggle, 		JSButton::button_function_t::k_none},
			{JSButton::button_function_t::k_mount_tilt_down, 		JSButton::button_function_t::k_none},
			{JSButton::button_function_t::k_mount_tilt_up, 			JSButton::button_function_t::k_none},
			{JSButton::button_function_t::k_gain_inc, 				JSButton::button_function_t::k_trim_pitch_dec},

			{JSButton::button_function_t::k_gain_dec, 				JSButton::button_function_t::k_trim_pitch_inc},
			{JSButton::button_function_t::k_lights1_dimmer, 		JSButton::button_function_t::k_trim_roll_dec},
			{JSButton::button_function_t::k_lights1_brighter, 		JSButton::button_function_t::k_trim_roll_inc},
			{JSButton::button_function_t::k_none, 					JSButton::button_function_t::k_none},
	};

	for(int i = 0; i < 16; i++) {
		get_button(i)->set_default(defaults[i][0], defaults[i][1]);
	}
}
