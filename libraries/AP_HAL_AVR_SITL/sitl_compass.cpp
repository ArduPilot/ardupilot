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

#define MAG_OFS_X 5.0
#define MAG_OFS_Y 13.0
#define MAG_OFS_Z -18.0

// inclination in Canberra (degrees)
#define MAG_INCLINATION -66

// magnetic field strength in Canberra as observed
// using an APM1 with 5883L compass
#define MAG_FIELD_STRENGTH 818

/*
  given a magnetic heading, and roll, pitch, yaw values,
  calculate consistent magnetometer components

  All angles are in radians
 */
Vector3f SITL_State::_heading_to_mag(float roll, float pitch, float yaw)
{
	Vector3f Bearth, m;
	Matrix3f R;
	float declination = AP_Declination::get_declination(_sitl->state.latitude, _sitl->state.longitude);

	// Bearth is the magnetic field in Canberra. We need to adjust
	// it for inclination and declination
	Bearth(MAG_FIELD_STRENGTH, 0, 0);
	R.from_euler(0, -ToRad(MAG_INCLINATION), ToRad(declination));
	Bearth = R * Bearth;

	// create a rotation matrix for the given attitude
	R.from_euler(roll, pitch, yaw);

	// convert the earth frame magnetic vector to body frame, and
	// apply the offsets
	m = R.transposed() * Bearth - Vector3f(MAG_OFS_X, MAG_OFS_Y, MAG_OFS_Z);
	return m + (_rand_vec3f() * _sitl->mag_noise);
}



/*
  setup the compass with new input
  all inputs are in degrees
 */
void SITL_State::_update_compass(float roll, float pitch, float yaw)
{
	if (_compass == NULL) {
		// no compass in this sketch
		return;
	}
	Vector3f m = _heading_to_mag(ToRad(roll),
				     ToRad(pitch),
				     ToRad(yaw));
	_compass->setHIL(m.x, m.y, m.z);
}

#endif
