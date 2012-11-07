/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  new variable scheme
  Andrew Tridgell February 2012
*/

#include <FastSerial.h>
#include <AP_Common.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <PID.h>            // PID library
#include <APM_RC.h>         // ArduPilot Mega RC Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <SPI.h>			// Arduino SPI lib
#include <I2C.h>
#include <AP_Semaphore.h>   // for removing conflict between optical flow and dataflash on SPI3 bus
#include <DataFlash.h>
#include <AP_AHRS.h>
#include <AP_ADC.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_Buffer.h>
#include <GCS_MAVLink.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h> // Inertial Sensor (uncalibated IMU) Library
#include <AP_GPS.h>
#include <AP_Math.h>
#include <SITL.h>
#include <GCS_MAVLink.h>
#include <AP_Declination.h>
#include <AP_AnalogSource.h>
#include <AP_Airspeed.h>
#include "config.h"
#include "Parameters.h"

// this sets up the parameter table, and sets the default values. This
// must be the first AP_Param variable declared to ensure its
// constructor runs before the constructors of the other AP_Param
// variables
extern const AP_Param::Info var_info[];
AP_Param param_loader(var_info, WP_START_BYTE);

static Parameters g;

static AP_ADC_ADS7844          adc;
static GPS         *g_gps;
AP_GPS_Auto     g_gps_driver(&Serial1, &g_gps);
# if CONFIG_IMU_TYPE == CONFIG_IMU_MPU6000
  AP_InertialSensor_MPU6000 ins( CONFIG_MPU6000_CHIP_SELECT_PIN );
# else
  AP_InertialSensor_Oilpan ins( &adc );
#endif // CONFIG_IMU_TYPE
AP_AHRS_DCM  ahrs(&ins, g_gps);

Arduino_Mega_ISR_Registry isr_registry;
#ifdef DESKTOP_BUILD
AP_Compass_HIL          compass;
#else
static AP_Compass_HMC5843 compass;
#endif
AP_Baro_BMP085_HIL      barometer;

SITL					sitl;

FastSerialPort0(Serial);
FastSerialPort1(Serial1);       // GPS port

#define SERIAL0_BAUD 115200

#define Debug(fmt, args...)  Serial.printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__ , ##args)


void setup() {
	Serial.begin(SERIAL0_BAUD, 128, 128);

	load_parameters();

	// show some sizes
	Serial.printf_P(PSTR("sizeof(RC_Channel)=%u\n"), (unsigned)sizeof(RC_Channel));
	Serial.printf_P(PSTR("sizeof(g)=%u\n"), (unsigned)sizeof(g));
	Serial.printf_P(PSTR("sizeof(g.throttle_min)=%u\n"), (unsigned)sizeof(g.throttle_min));
	Serial.printf_P(PSTR("sizeof(g.channel_roll)=%u\n"), (unsigned)sizeof(g.channel_roll));
	Serial.printf_P(PSTR("throttle_max now: %u\n"), (unsigned)g.throttle_max);

	// some ad-hoc testing

	// try set interfaces
	g.throttle_min.set(g.throttle_min+1);
	g.throttle_min.save();
	g.throttle_min.set_and_save(g.throttle_min+1);

	Serial.printf_P(PSTR("throttle_min now: %u\n"), (unsigned)g.throttle_min);

	// find a variable by name
	AP_Param                  *vp;
	enum ap_var_type type;
	vp = AP_Param::find("RLL2SRV_P", &type);
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

	Vector3f ofs;
	ofs = compass.get_offsets();
	Serial.printf_P(PSTR("Compass: %f %f %f\n"),
					ofs.x, ofs.y, ofs.z);
	ofs.x += 1.1;
	ofs.y += 1.2;
	ofs.z += 1.3;
	compass.set_offsets(ofs);
	compass.save_offsets();
	Serial.printf_P(PSTR("Compass: %f %f %f\n"),
					ofs.x, ofs.y, ofs.z);

	test_vector3f();

	// full testing of all variables
	AP_Param::ParamToken token;
	for (AP_Param *ap = AP_Param::first(&token, &type);
		 ap;
		 ap=AP_Param::next(&token, &type)) {
		//test_variable(ap, type);
	}

	AP_Param::show_all();

	Serial.println_P(PSTR("All done."));
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
	Serial.printf_P(PSTR("vec %f %f %f\n"),
					vec.x, vec.y, vec.z);

	if (vec.x != 10 ||
		vec.y != 11 ||
		vec.z != 12) {
		Serial.printf_P(PSTR("wrong value for compass vector\n"));
	}
}


// test all interfaces for a variable
void test_variable(AP_Param *ap, enum ap_var_type type)
{
	static int8_t value;
	char s[AP_MAX_NAME_SIZE+1];

	value++;

	ap->copy_name(s, sizeof(s), type==AP_PARAM_FLOAT);
	Serial.printf_P(PSTR("Testing variable '%s' of type %u\n"),
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
