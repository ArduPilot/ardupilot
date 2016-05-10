// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

// Functions that will handle joystick/gamepad input
// ----------------------------------------------------------------------------

// Anonymous namespace to hold variables used only in this file
namespace {
	int16_t mode = 1100;
	int16_t camTilt = 1500;
	int16_t lights1 = 1100;
	int16_t lights2 = 1100;
	int16_t rollTrim = 0;
	int16_t pitchTrim = 0;
	float gain = 0.5;
	float maxGain = 1.0;
	float minGain = 0.25;
	int8_t numGainSettings = 4;
}

void Sub::transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons) {
	int16_t channels[10];

	uint32_t tnow_ms = millis();

	float rpyScale = 0.5*gain; // Scale -1000-1000 to -500-500 with gain
	float throttleScale = 0.8*gain; // Scale 0-1000 to 0-800 with gain
	int16_t rpyCenter = 1500;
	int16_t throttleBase = 1500-500*throttleScale;
	bool shift = false;
	static uint32_t buttonDebounce;

	// Debouncing timer
	if ( tnow_ms - buttonDebounce > 50 ) {
		buttonDebounce = tnow_ms;

		// Detect if any shift button is pressed
		for ( uint8_t i = 0 ; i < 16 ; i++ ) {
			if ( (buttons & (1 << i)) && get_button(i)->function() == JSButton::button_function_t::k_shift ) { shift = true; }
		}

		// Act if button is pressed
		for ( uint8_t i = 0 ; i < 16 ; i++ ) {
			if ( buttons & (1 << i) ) {
				handle_jsbutton_press(i,shift);
			}
		}
	}

	// Set channels to override
	channels[0] = 1500 + pitchTrim;               // pitch
	channels[1] = 1500 + rollTrim;                // roll
	channels[2] = z*throttleScale+throttleBase;   // throttle
	channels[3] = r*rpyScale+rpyCenter;           // yaw
	channels[4] = mode;                           // for testing only
	channels[5] = x*rpyScale+rpyCenter;           // forward for ROV
	channels[6] = y*rpyScale+rpyCenter;           // lateral for ROV
	channels[7] = camTilt;                        // camera tilt
	channels[8] = lights1;
	channels[9] = lights2;

	// record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
	failsafe.rc_override_active = hal.rcin->set_overrides(channels, 10);
}

void Sub::handle_jsbutton_press(uint8_t button, bool shift) {
	// Act based on the function assigned to this button
	switch ( get_button(button)->function(shift) ) {
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
		case JSButton::button_function_t::k_camera_source_toggle:
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
			{
				static bool lowGain = false;
				lowGain = !lowGain;
				if ( lowGain ) {
					gain = 0.5f;
				} else {
					gain = 1.0f;
				}
				gcs_send_text_fmt(MAV_SEVERITY_INFO,"Gain: %2.0f%%",gain*100);
			}
			break;
		case JSButton::button_function_t::k_gain_inc:
			gain = constrain_float(gain + (maxGain-minGain)/(numGainSettings-1), minGain, maxGain);
			gcs_send_text_fmt(MAV_SEVERITY_INFO,"Gain: %2.0f%%",gain*100);
			break;
		case JSButton::button_function_t::k_gain_dec:
			gain = constrain_float(gain - (maxGain-minGain)/(numGainSettings-1), minGain, maxGain);
			gcs_send_text_fmt(MAV_SEVERITY_INFO,"Gain: %2.0f%%",gain*100);
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
			break;
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
