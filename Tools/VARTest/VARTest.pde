/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  new variable scheme
  Andrew Tridgell February 2012
*/

#include <FastSerial.h>
#include <AP_Common.h>
#include <PID.h>            // PID library
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <I2C.h>
#include <AP_Math.h>
#include <config.h>
#include <Parameters.h>

static Parameters g;
static AP_Compass_HMC5843 compass;

FastSerialPort0(Serial);

#define SERIAL0_BAUD 115200

void setup() {
	Serial.begin(SERIAL0_BAUD, 128, 128);

	load_parameters();

	Serial.printf_P(PSTR("sizeof(RC_Channel)=%u\n"), (unsigned)sizeof(RC_Channel));
	Serial.printf_P(PSTR("sizeof(g)=%u\n"), (unsigned)sizeof(g));
	Serial.printf_P(PSTR("sizeof(g.throttle_min)=%u\n"), (unsigned)sizeof(g.throttle_min));
	Serial.printf_P(PSTR("sizeof(g.channel_roll)=%u\n"), (unsigned)sizeof(g.channel_roll));
	Serial.printf_P(PSTR("throttle_max now: %u\n"), (unsigned)g.throttle_max);

	// try set interfaces
	g.throttle_min.set(g.throttle_min+1);
	g.throttle_min.save();
	g.throttle_min.set_and_save(g.throttle_min+1);

	Serial.printf_P(PSTR("throttle_min now: %u\n"), (unsigned)g.throttle_min);

	// find a variable by name
	AP_Param                  *vp;
	vp = AP_Param::find("RLL2SRV_P");
	((AP_Float *)vp)->set(23);

	Serial.printf_P(PSTR("RLL2SRV_P=%f\n"),
					g.pidServoRoll.kP());

	char s[AP_MAX_NAME_SIZE+1];

	g.throttle_min.copy_name(s, sizeof(s));
	s[AP_MAX_NAME_SIZE] = 0;
	Serial.printf_P(PSTR("THROTTLE_MIN.copy_name()->%s\n"), s);

	g.channel_roll.radio_min.copy_name(s, sizeof(s));
	s[AP_MAX_NAME_SIZE] = 0;
	Serial.printf_P(PSTR("RC1_MIN.copy_name()->%s %p\n"), s, &g.channel_roll.radio_min);
}

void loop()
{
}
