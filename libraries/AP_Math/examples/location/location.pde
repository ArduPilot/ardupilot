/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Unit tests for the AP_Math polygon code
//

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>

#ifdef DESKTOP_BUILD
// all of this is needed to build with SITL
#include <DataFlash.h>
#include <APM_RC.h>
#include <GCS_MAVLink.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_ADC.h>
#include <AP_Baro.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
#include <Filter.h>
#include <SITL.h>
#include <I2C.h>
#include <SPI.h>
#include <AP_Declination.h>
Arduino_Mega_ISR_Registry isr_registry;
AP_Baro_BMP085_HIL      barometer;
AP_Compass_HIL     compass;
SITL sitl;
BetterStream *mavlink_comm_0_port;
BetterStream *mavlink_comm_1_port;
mavlink_system_t mavlink_system;
#endif

FastSerialPort(Serial, 0);

static const struct {
	Vector2f wp1, wp2, location;
	bool passed;
} test_points[] = {
	{ Vector2f(-35.3647759314918, 149.16265692810987), 
	  Vector2f(-35.36279922658029, 149.16352169591426), 
	  Vector2f(-35.36214956969903, 149.16461410046492), true },
	{ Vector2f(-35.36438601157189, 149.16613916088568),
	  Vector2f(-35.364432558610254, 149.16287313113048),
	  Vector2f(-35.36491510034746, 149.16365837225004), false },
	{ Vector2f(0, 0),
	  Vector2f(0, 1),
	  Vector2f(0, 2), true },
	{ Vector2f(0, 0),
	  Vector2f(0, 2),
	  Vector2f(0, 1), false },
	{ Vector2f(0, 0),
	  Vector2f(1, 0),
	  Vector2f(2, 0), true },
	{ Vector2f(0, 0),
	  Vector2f(2, 0),
	  Vector2f(1, 0), false },
	{ Vector2f(0, 0),
	  Vector2f(-1, 1),
	  Vector2f(-2, 2), true },
};

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

static struct Location location_from_point(Vector2f pt)
{
	struct Location loc = {0};
	loc.lat = pt.x * 1.0e7;
	loc.lng = pt.y * 1.0e7;
	return loc;
}

static void test_passed_waypoint(void)
{
	Serial.println("waypoint tests starting");
	for (uint8_t i=0; i<ARRAY_LENGTH(test_points); i++) {
		struct Location loc = location_from_point(test_points[i].location);
		struct Location wp1 = location_from_point(test_points[i].wp1);
		struct Location wp2 = location_from_point(test_points[i].wp2);
		if (location_passed_point(loc, wp1, wp2) != test_points[i].passed) {
			Serial.printf("Failed waypoint test %u\n", (unsigned)i);
			return;
		}
	}
	Serial.println("waypoint tests OK");
}

/*
  polygon tests
 */
void setup(void)
{
    Serial.begin(115200);
    test_passed_waypoint();
}

void
loop(void)
{
}
