#include "AP_Math.h"

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
	if (isnan(v)) {
		return 0.0;
	}
	if (v >= 1.0) {
		return PI/2;
	}
	if (v <= -1.0) {
		return -PI/2;
	}
	return asin(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
	if (isnan(v) || v <= 0.0) {
		return 0.0;
	}
	return sqrt(v);
}


// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void rotation_matrix_from_euler(Matrix3f &m, float roll, float pitch, float yaw)
{
	float cp = cos(pitch);
	float sp = sin(pitch);
	float sr = sin(roll);
	float cr = cos(roll);
	float sy = sin(yaw);
	float cy = cos(yaw);

	m.a.x = cp * cy;
	m.a.y = (sr * sp * cy) - (cr * sy);
	m.a.z = (cr * sp * cy) + (sr * sy);
	m.b.x = cp * sy;
	m.b.y = (sr * sp * sy) + (cr * cy);
	m.b.z = (cr * sp * sy) - (sr * cy);
	m.c.x = -sp;
	m.c.y = sr * cp;
	m.c.z = cr * cp;
}

// calculate euler angles from a rotation matrix
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void calculate_euler_angles(Matrix3f &m, float *roll, float *pitch, float *yaw)
{
	if (pitch != NULL) {
		*pitch	= -safe_asin(m.c.x);
	}
	if (roll != NULL) {
		*roll 	= atan2(m.c.y, m.c.z);
	}
	if (yaw != NULL) {
		*yaw 	= atan2(m.b.x, m.a.x);
	}
}
