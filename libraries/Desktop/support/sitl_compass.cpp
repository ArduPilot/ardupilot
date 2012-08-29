/*
  SITL handling

  This simulates a compass

  Andrew Tridgell November 2011
 */
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <AP_Math.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <SITL.h>
#include "desktop.h"
#include "util.h"

#define MAG_OFS_X 5.0
#define MAG_OFS_Y 13.0
#define MAG_OFS_Z -18.0

// inclination in Canberra (degrees)
#define MAG_INCLINATION -66

// magnetic field strength in Canberra as observed
// using an APM1 with 5883L compass
#define MAG_FIELD_STRENGTH 818

extern SITL sitl;

/*
  given a magnetic heading, and roll, pitch, yaw values,
  calculate consistent magnetometer components

  All angles are in radians
 */
static Vector3f heading_to_mag(float roll, float pitch, float yaw)
{
	Vector3f Bearth, m;
	Matrix3f R;
	float declination = AP_Declination::get_declination(sitl.state.latitude, sitl.state.longitude);

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
	return m + (rand_vec3f() * sitl.mag_noise);
}



/*
  setup the compass with new input
  all inputs are in degrees
 */
void sitl_update_compass(float roll, float pitch, float yaw)
{
	extern AP_Compass_HIL compass;
	Vector3f m = heading_to_mag(ToRad(roll),
				    ToRad(pitch),
				    ToRad(yaw));
	compass.setHIL(m.x, m.y, m.z);
}
