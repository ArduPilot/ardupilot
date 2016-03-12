// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

// Functions that will handle joystick/gamepad input
// ----------------------------------------------------------------------------

namespace {
	int16_t mode;
	int16_t camTilt = 1500;
	int16_t lights1 = 1100;
	int16_t lights2 = 1100;
	int16_t rollTrim = 0;
	int16_t pitchTrim = 0;
}

void Sub::transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons) {
	int16_t channels[10];

	uint32_t tnow_ms = millis();

	float rpyScale = 0.5;
	float throttleScale = 0.8;
	int16_t rpyCenter = 1500;
	int16_t throttleBase = 1500-500*throttleScale;
	uint8_t shift = 0;
	static uint32_t buttonDebounce;

	// Debouncing timer
	if ( tnow_ms - buttonDebounce > 50 ) {
		buttonDebounce = tnow_ms;

		if ( (buttons & (1 << 0)) && g.jbtn_0.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 1)) && g.jbtn_1.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 2)) && g.jbtn_2.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 3)) && g.jbtn_3.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 4)) && g.jbtn_4.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 5)) && g.jbtn_5.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 6)) && g.jbtn_6.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 7)) && g.jbtn_7.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 8)) && g.jbtn_8.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 9)) && g.jbtn_9.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 10)) && g.jbtn_10.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 11)) && g.jbtn_11.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 12)) && g.jbtn_12.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 13)) && g.jbtn_13.function() == JSButton::button_function_t::k_shift ) { shift = 1; }
		if ( (buttons & (1 << 14)) && g.jbtn_14.function() == JSButton::button_function_t::k_shift ) { shift = 1; }

		for ( uint8_t i = 0 ; i < 16 ; i++ ) {
			if ( buttons & (1 << i) ) {
				handle_jsbutton_press(i,shift);
			}
		}
	}

	channels[0] = 1500;                           // pitch
	channels[1] = 1500 + rollTrim;                // roll
	channels[2] = z*throttleScale+throttleBase;   // throttle
	channels[3] = r*rpyScale+rpyCenter;           // yaw
	channels[4] = mode;                           // for testing only
	channels[5] = x*rpyScale+rpyCenter;           // forward for ROV
	channels[6] = y*rpyScale+rpyCenter;           // strafe for ROV
	channels[7] = camTilt;                        // camera tilt
	channels[8] = lights1;
	channels[9] = 0;

	// record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
	failsafe.rc_override_active = hal.rcin->set_overrides(channels, 10);
}

void Sub::handle_jsbutton_press(uint8_t button, uint8_t shift) {
	uint8_t func;

	switch ( button ) {
		case 0: if ( shift ) func = g.jbtn_0.function(); else func = g.jbtn_0.function(true); break;
		case 1: if ( shift ) func = g.jbtn_1.function(); else func = g.jbtn_1.function(true); break;
		case 2: if ( shift ) func = g.jbtn_2.function(); else func = g.jbtn_2.function(true); break;
		case 3: if ( shift ) func = g.jbtn_3.function(); else func = g.jbtn_3.function(true); break;
		case 4: if ( shift ) func = g.jbtn_4.function(); else func = g.jbtn_4.function(true); break;
		case 5: if ( shift ) func = g.jbtn_5.function(); else func = g.jbtn_5.function(true); break;
		case 6: if ( shift ) func = g.jbtn_6.function(); else func = g.jbtn_6.function(true); break;
		case 7: if ( shift ) func = g.jbtn_7.function(); else func = g.jbtn_7.function(true); break;
		case 8: if ( shift ) func = g.jbtn_8.function(); else func = g.jbtn_8.function(true); break;
		case 9: if ( shift ) func = g.jbtn_9.function(); else func = g.jbtn_9.function(true); break;
		case 10: if ( shift ) func = g.jbtn_10.function(); else func = g.jbtn_10.function(true); break;
		case 11: if ( shift ) func = g.jbtn_11.function(); else func = g.jbtn_11.function(true); break;
		case 12: if ( shift ) func = g.jbtn_12.function(); else func = g.jbtn_12.function(true); break;
		case 13: if ( shift ) func = g.jbtn_13.function(); else func = g.jbtn_13.function(true); break;
		case 14: if ( shift ) func = g.jbtn_14.function(); else func = g.jbtn_14.function(true); break;
		case 15: if ( shift ) func = g.jbtn_15.function(); else func = g.jbtn_15.function(true); break;
	}

	switch ( func ) {
		case JSButton::button_function_t::k_arm_toggle:
			break;
		case JSButton::button_function_t::k_arm:
			init_arm_motors(true);
			break;
		case JSButton::button_function_t::k_disarm:
			init_disarm_motors();
			break;
		case JSButton::button_function_t::k_mode_toggle:
			init_disarm_motors();
			break;
		case JSButton::button_function_t::k_mode_1:
			mode = 1100;
			break;
		case JSButton::button_function_t::k_mode_2:
			mode = 1300;
			break;
		case JSButton::button_function_t::k_mode_3:
			mode = 1420;
			break;
		case JSButton::button_function_t::k_mode_4:
			mode = 1550;
			break;
		case JSButton::button_function_t::k_mode_5:
			mode = 1690;
			break;
		case JSButton::button_function_t::k_mode_6:
			mode = 1900;
			break;
		case JSButton::button_function_t::k_mount_center:
			camTilt = 1500;
			break;
		case JSButton::button_function_t::k_mount_tilt_up:
			camTilt = constrain_float(camTilt-30,800,2200);
			break;
		case JSButton::button_function_t::k_mount_tilt_down:
			camTilt = constrain_float(camTilt+30,800,2200);
			break;
		case JSButton::button_function_t::k_camera_trigger:
			break;
		case JSButton::button_function_t::k_lights1_cycle:
			{
				static bool increasing = true;
				if ( increasing ) {
					lights1 = constrain_float(lights1+100,1100,1900);
				} else {
					lights1 = constrain_float(lights1-100,1100,1900);
				}
				if ( lights1 >= 1900 || lights1 <= 1100 ) {
					increasing = !increasing;
				}
			}
			break;
		case JSButton::button_function_t::k_lights1_brighter:
			lights1 = constrain_float(lights1+100,1100,1900);
			break;
		case JSButton::button_function_t::k_lights1_dimmer:
			lights1 = constrain_float(lights1-100,1100,1900);
			break;
		case JSButton::button_function_t::k_lights2_cycle:
			{
				static bool increasing = true;
				if ( increasing ) {
					lights2 = constrain_float(lights2+100,1100,1900);
				} else {
					lights2 = constrain_float(lights2-100,1100,1900);
				}
				if ( lights2 >= 1900 || lights2 <= 1100 ) {
					increasing = !increasing;
				}
			}
			break;
		case JSButton::button_function_t::k_lights2_brighter:
			lights2 = constrain_float(lights2+100,1100,1900);
			break;
		case JSButton::button_function_t::k_lights2_dimmer:
			lights2 = constrain_float(lights2-100,1100,1900);
			break;
		case JSButton::button_function_t::k_gain_toggle:
			break;
		case JSButton::button_function_t::k_gain_inc:
			break;
		case JSButton::button_function_t::k_gain_dec:
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
	}
}
