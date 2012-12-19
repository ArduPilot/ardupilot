/*
  SITL handling

  This simulates a barometer

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "AP_HAL_AVR_SITL.h"

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/*
  setup the barometer with new input
  altitude is in meters
 */
void SITL_State::_update_barometer(float altitude)
{
	double Temp, Press, y;
	static uint32_t last_update;

	if (_barometer == NULL) {
		// this sketch doesn't use a barometer
		return;
	}

	// 80Hz, to match the real APM2 barometer
	if (hal.scheduler->millis() - last_update < 12) {
		return;
	}
	last_update = hal.scheduler->millis();

	Temp = 312;

	y = ((altitude-584.0) * 1000.0) / 29271.267;
	y /= (Temp / 10.0) + 273.15;
	y = 1.0/exp(y);
	y *= 95446.0;

	Press = y + (_rand_float() * _sitl->baro_noise);

	_barometer->setHIL(Temp, Press);
}

#endif
