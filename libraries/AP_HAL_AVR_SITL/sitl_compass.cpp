/*
  SITL handling

  This simulates a compass

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"

#include <AP_Math.h>
#include "../AP_Compass/AP_Compass.h"
#include "../AP_Declination/AP_Declination.h"
#include "../SITL/SITL.h"

using namespace AVR_SITL;

/*
  setup the compass with new input
  all inputs are in degrees
 */
void SITL_State::_update_compass(float rollDeg, float pitchDeg, float yawDeg)
{
	if (_compass == NULL) {
		// no compass in this sketch
		return;
	}
	yawDeg += _sitl->mag_error;
	if (yawDeg > 180.0f) {
		yawDeg -= 360.0f;
	}
	if (yawDeg < -180.0f) {
		yawDeg += 360.0f;
	}
	_compass->setHIL(radians(rollDeg), radians(pitchDeg), radians(yawDeg));
	Vector3f noise = _rand_vec3f() * _sitl->mag_noise;
    Vector3f motor = _sitl->mag_mot.get() * _current;
    _compass->setHIL(_compass->getHIL() + noise+motor);
}

#endif
