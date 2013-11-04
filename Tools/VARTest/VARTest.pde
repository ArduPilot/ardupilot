/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  new variable scheme
  Andrew Tridgell February 2012
*/

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_Baro.h>        // ArduPilot barometer library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h>     // Range finder library
#include <Filter.h>                     // Filter library
#include <AP_Buffer.h>      // APM FIFO Buffer
#include <AP_Relay.h>       // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_SpdHgtControl.h>
#include <memcheck.h>

#include <APM_OBC.h>
#include <APM_Control.h>
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <DataFlash.h>
#include <SITL.h>
#include <AP_Notify.h>

#include "config.h"
#include "Parameters.h"

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

AP_HAL::BetterStream* cliSerial;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// this sets up the parameter table, and sets the default values. This
// must be the first AP_Param variable declared to ensure its
// constructor runs before the constructors of the other AP_Param
// variables
extern const AP_Param::Info var_info[];
AP_Param param_loader(var_info, WP_START_BYTE);

static Parameters g;

static GPS         *g_gps;
AP_GPS_Auto     g_gps_driver(&g_gps);
AP_InertialSensor_MPU6000 ins;
AP_AHRS_DCM  ahrs(ins, g_gps);

static AP_Compass_HIL compass;
AP_Baro_HIL      barometer;
SITL					sitl;

#define SERIAL0_BAUD 115200

#define Debug(fmt, args...)  cliSerial->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args)


void setup() {
	cliSerial = hal.uartA;

	hal.uartA->begin(SERIAL0_BAUD, 128, 128);

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

	load_parameters();

	// show some sizes
	cliSerial->printf_P(PSTR("sizeof(RC_Channel)=%u\n"), (unsigned)sizeof(RC_Channel));
	cliSerial->printf_P(PSTR("sizeof(g)=%u\n"), (unsigned)sizeof(g));
	cliSerial->printf_P(PSTR("sizeof(g.throttle_min)=%u\n"), (unsigned)sizeof(g.throttle_min));
	cliSerial->printf_P(PSTR("sizeof(g.channel_roll)=%u\n"), (unsigned)sizeof(g.channel_roll));
	cliSerial->printf_P(PSTR("throttle_max now: %u\n"), (unsigned)g.throttle_max);

	// some ad-hoc testing

	// try set interfaces
	g.throttle_min.set(g.throttle_min+1);
	g.throttle_min.save();
	g.throttle_min.set_and_save(g.throttle_min+1);

	cliSerial->printf_P(PSTR("throttle_min now: %u\n"), (unsigned)g.throttle_min);

	// find a variable by name
	AP_Param                  *vp;
	enum ap_var_type type;
	vp = AP_Param::find("RLL2SRV_P", &type);
	((AP_Float *)vp)->set(23);

	cliSerial->printf_P(PSTR("RLL2SRV_P=%f\n"),
					g.pidServoRoll.kP());

#if 0
	char s[AP_MAX_NAME_SIZE+1];

	g.throttle_min.copy_name(s, sizeof(s));
	s[AP_MAX_NAME_SIZE] = 0;
	cliSerial->printf_P(PSTR("THROTTLE_MIN.copy_name()->%s\n"), s);

	g.channel_roll.radio_min.copy_name(s, sizeof(s));
	s[AP_MAX_NAME_SIZE] = 0;
	cliSerial->printf_P(PSTR("RC1_MIN.copy_name()->%s %p\n"), s, &g.channel_roll.radio_min);
#endif

	Vector3f ofs;
	ofs = compass.get_offsets();
	cliSerial->printf_P(PSTR("Compass: %f %f %f\n"),
					ofs.x, ofs.y, ofs.z);
	ofs.x += 1.1;
	ofs.y += 1.2;
	ofs.z += 1.3;
	compass.set_offsets(ofs);
	compass.save_offsets();
	cliSerial->printf_P(PSTR("Compass: %f %f %f\n"),
					ofs.x, ofs.y, ofs.z);

	test_vector3f();

	// full testing of all variables
	AP_Param::ParamToken token;
	for (AP_Param *ap = AP_Param::first(&token, &type);
		 ap;
		 ap=AP_Param::next(&token, &type)) {
		//test_variable(ap, type);
	}

	AP_Param::show_all(cliSerial);

	cliSerial->println_P(PSTR("All done."));
}

void loop()
{
}

// test vector3f handling
void test_vector3f(void)
{
	enum ap_var_type type;

	AP_Float *f;
	AP_Vector3f *v;
	Vector3f vec;

	v = (AP_Vector3f *)AP_Param::find("COMPASS_OFS", &type);

	f = (AP_Float *)AP_Param::find("COMPASS_OFS_X", &type);
	f->set_and_save(10);
	f = (AP_Float *)AP_Param::find("COMPASS_OFS_Y", &type);
	f->set_and_save(11);
	f = (AP_Float *)AP_Param::find("COMPASS_OFS_Z", &type);
	f->set_and_save(12);

	v->load();

	vec = *v;
	cliSerial->printf_P(PSTR("vec %f %f %f\n"),
					vec.x, vec.y, vec.z);

	if (vec.x != 10 ||
		vec.y != 11 ||
		vec.z != 12) {
		cliSerial->printf_P(PSTR("wrong value for compass vector\n"));
	}
}

#if 0
// test all interfaces for a variable
void test_variable(AP_Param *ap, enum ap_var_type type)
{
	static int8_t value;
	char s[AP_MAX_NAME_SIZE+1];

	value++;

	ap->copy_name(s, sizeof(s), type==AP_PARAM_FLOAT);
	cliSerial->printf_P(PSTR("Testing variable '%s' of type %u\n"),
					s, type);
	enum ap_var_type type2;
	if (AP_Param::find(s, &type2) != ap ||
		type2 != type) {
		Debug("find failed");
	}
	if (strcmp(s, "FORMAT_VERSION") == 0) {
		// don't wipe the version
		return;
	}
	switch (type) {
	case AP_PARAM_INT8: {
		AP_Int8 *v = (AP_Int8 *)ap;
		if (sizeof(*v) != 1) {
			Debug("incorrect size %u", (unsigned)sizeof(*v));
		}
		v->set(value);
		if (!v->save()) {
			Debug("failed set_and_save");
		}
		if (!v->load()) {
			Debug("failed load");
		}
		if (v->get() != value) {
			Debug("wrong value %d %d", (int)v->get(), (int)value);
		}
		if (!v->set_and_save(value+1)) {
			Debug("failed set_and_save");
		}
		if (!v->load()) {
			Debug("failed load");
		}
		if (v->vtype != type) {
			Debug("wrong type");
		}
		if (v->get() != value+1) {
			Debug("wrong value %u %u", (unsigned)v->get(), value+1);
		}
		if (*v != value+1) {
			Debug("wrong direct value %u %u", (unsigned)v->get(), value+1);
		}
		*v = value+2;
		if (v->get() != value+2) {
			Debug("wrong copy assignment value %u %u", (unsigned)v->get(), value+2);
		}
		break;
	}
	case AP_PARAM_INT16: {
		AP_Int16 *v = (AP_Int16 *)ap;
		if (sizeof(*v) != 2) {
			Debug("incorrect size %u", (unsigned)sizeof(*v));
		}
		v->set(value);
		if (!v->save()) {
			Debug("failed set_and_save");
		}
		if (!v->load()) {
			Debug("failed load");
		}
		if (v->get() != value) {
			Debug("wrong value %u %u", (unsigned)v->get(), value);
		}
		if (!v->set_and_save(value+1)) {
			Debug("failed set_and_save");
		}
		if (!v->load()) {
			Debug("failed load");
		}
		if (v->vtype != type) {
			Debug("wrong type");
		}
		if (v->get() != value+1) {
			Debug("wrong value %u %u", (unsigned)v->get(), value+1);
		}
		if (*v != value+1) {
			Debug("wrong direct value %u %u", (unsigned)v->get(), value+1);
		}
		*v = value+2;
		if (v->get() != value+2) {
			Debug("wrong copy assignment value %u %u", (unsigned)v->get(), value+2);
		}
		break;
	}
	case AP_PARAM_INT32: {
		AP_Int32 *v = (AP_Int32 *)ap;
		if (sizeof(*v) != 4) {
			Debug("incorrect size %u", (unsigned)sizeof(*v));
		}
		v->set(value);
		if (!v->save()) {
			Debug("failed set_and_save");
		}
		if (!v->load()) {
			Debug("failed load");
		}
		if (v->get() != value) {
			Debug("wrong value %u %u", (unsigned)v->get(), value);
		}
		if (!v->set_and_save(value+1)) {
			Debug("failed set_and_save");
		}
		if (!v->load()) {
			Debug("failed load");
		}
		if (v->vtype != type) {
			Debug("wrong type");
		}
		if (v->get() != value+1) {
			Debug("wrong value %u %u", (unsigned)v->get(), value+1);
		}
		if (*v != value+1) {
			Debug("wrong direct value %u %u", (unsigned)v->get(), value+1);
		}
		*v = value+2;
		if (v->get() != value+2) {
			Debug("wrong copy assignment value %u %u", (unsigned)v->get(), value+2);
		}
		break;
	}
	case AP_PARAM_FLOAT: {
		AP_Float *v = (AP_Float *)ap;
		if (sizeof(*v) != 4) {
			Debug("incorrect size %u", (unsigned)sizeof(*v));
		}
		v->set(value);
		if (!v->save()) {
			Debug("failed set_and_save");
		}
		if (!v->load()) {
			Debug("failed load");
		}
		if (v->get() != value) {
			Debug("wrong value %u %u", (unsigned)v->get(), value);
		}
		if (!v->set_and_save(value+1)) {
			Debug("failed set_and_save");
		}
		if (!v->load()) {
			Debug("failed load");
		}
		if (v->vtype != type) {
			Debug("wrong type");
		}
		if (v->get() != value+1) {
			Debug("wrong value %u %u", (unsigned)v->get(), value+1);
		}
		if (*v != value+1) {
			Debug("wrong direct value %u %u", (unsigned)v->get(), value+1);
		}
		*v = value+2;
		if (v->get() != value+2) {
			Debug("wrong copy assignment value %u %u", (unsigned)v->get(), value+2);
		}
		break;
	}
	case AP_PARAM_VECTOR3F: {
		AP_Vector3f *v = (AP_Vector3f *)ap;
		if (sizeof(*v) != 12) {
			Debug("incorrect size %u", (unsigned)sizeof(*v));
		}
		break;
	}
	case AP_PARAM_VECTOR6F: {
		AP_Vector6f *v = (AP_Vector6f *)ap;
		if (sizeof(*v) != 24) {
			Debug("incorrect size %u", (unsigned)sizeof(*v));
		}
		break;
	}
	case AP_PARAM_MATRIX3F: {
		AP_Matrix3f *v = (AP_Matrix3f *)ap;
		if (sizeof(*v) != 36) {
			Debug("incorrect size %u", (unsigned)sizeof(*v));
		}
		break;
	}
	default:
		break;
	}
}
#endif

AP_HAL_MAIN();
