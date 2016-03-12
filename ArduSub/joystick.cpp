// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

void Sub::transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons) {
	int16_t channels[10];

	uint32_t tnow_ms = millis();

	float rpyScale = 0.5;
	float throttleScale = 0.8;
	int16_t rpyCenter = 1500;
	int16_t throttleBase = 1500-500*throttleScale;
	static int16_t rollTrim = 0;
	static int16_t mode;
	static int16_t camTilt = 1500;
	static int16_t lights = 1100;
	static uint32_t buttonDebounce;

	// Debouncing timer
	if ( tnow_ms - buttonDebounce > 50 ) {
		buttonDebounce = tnow_ms;

		for ( uint8_t i = 0 ; i < 16 ; i++ ) {
			if ( buttons & (1 << i) ) {
//				handle_jsbutton_press(i);
			}
		}

		// Button logic to arm/disarm motors (Start and back buttons)
		if ( buttons & (1 << 4) ) {
			init_arm_motors(true);
		} else if ( buttons & (1 << 5) ) {
			init_disarm_motors();
		}

		// Button logic to change camera tilt (D-pad up and down + left joystick click to center)
		if ( buttons & (1 << 0) ) {
			camTilt = constrain_float(camTilt-30,800,2200);
		} else if ( buttons & (1 << 1) ) {
			camTilt = constrain_float(camTilt+30,800,2200);
		} else if ( buttons & (1 << 6) ) {
			camTilt = 1500; // Reset camera tilt
		}

		// Button logic for roll trim (D-pad left and right)
		if ( (buttons & ( 1 << 2 )) && rollTrim > -200 ) {
			rollTrim -= 10;
		} else if ( (buttons & ( 1 << 3 )) && rollTrim < 200 ) {
			rollTrim += 10;
		}

		// Button logic for mode changes (B for stabilize, Y for altitude hold)
		if ( buttons & (1 << 14) ) {
			mode = 2000;
		} else if ( buttons & (1 << 12)) {
			mode = 1000;
		}

		// Button logic for lights with dimming (right bumper brighter, left bumper dimmer)
		if ( buttons & (1 << 8) ) {
			lights = constrain_float(lights-100,1100,1900);
		} else if ( buttons & (1 << 9) ) {
			lights = constrain_float(lights+100,1100,1900);
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
	channels[8] = lights;
	channels[9] = 0;

	// record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
	failsafe.rc_override_active = hal.rcin->set_overrides(channels, 10);
}

//void Sub::handle_jsbutton_press(uint8_t button);
