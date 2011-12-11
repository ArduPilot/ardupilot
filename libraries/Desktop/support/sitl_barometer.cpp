/*
  SITL handling

  This simulates a barometer

  Andrew Tridgell November 2011
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <AP_Baro.h>     // ArduPilot Mega BMP085 Library
#include "desktop.h"
#include "util.h"

/*
  setup the barometer with new input
  altitude is in meters
 */
void sitl_update_barometer(float altitude)
{
	extern AP_Baro_BMP085_HIL barometer;
	double Temp, Press, y;

	Temp = 312;

	y = ((altitude-584.0) * 1000.0) / 29271.267;
	y /= (Temp / 10.0) + 273.15;
	y = 1.0/exp(y);
	y *= 95446.0;

	Press = y;

	barometer.setHIL(Temp, Press);
}
