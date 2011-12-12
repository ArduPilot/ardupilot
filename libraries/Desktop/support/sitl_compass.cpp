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
#include "desktop.h"
#include "util.h"

/*
  given a magnetic heading, and roll, pitch, yaw values,
  calculate consistent magnetometer components

  All angles are in radians
 */
static Vector3f heading_to_mag(float heading, float roll, float pitch, float yaw)
{
	Vector3f v;
	double headX, headY, cos_roll, sin_roll, cos_pitch, sin_pitch, scale;
	const double magnitude = 665;

	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);


	headY = -sin(heading);
	headX = cos(heading);

	if (fabs(cos_roll) < 1.0e-20) {
		cos_roll = 1.0e-10;
	}
	if (fabs(cos_pitch) < 1.0e-20) {
		cos_pitch = 1.0e-10;
	}

	v.z = -0.4;
	v.y = (headY + v.z*sin_roll) / cos_roll;
	v.x = (headX - (v.y*sin_roll*sin_pitch + v.z*cos_roll*sin_pitch)) / cos_pitch;
	scale = magnitude / sqrt((v.x*v.x) + (v.y*v.y) + (v.z*v.z));
	v *= scale;
	return v;
}



/*
  setup the compass with new input
  all inputs are in degrees
 */
void sitl_update_compass(float heading, float roll, float pitch, float yaw)
{
	extern AP_Compass_HIL compass;
	Vector3f m = heading_to_mag(ToRad(heading),
				    ToRad(roll),
				    ToRad(pitch),
				    ToRad(yaw));
	compass.setHIL(m.x, m.y, m.z);
}
