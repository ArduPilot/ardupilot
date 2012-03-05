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
	float ret = sqrt(v);
	if (isnan(ret)) {
		return 0;
	}
	return ret;
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


// create a quaternion from Euler angles
void quaternion_from_euler(Quaternion &q, float roll, float pitch, float yaw)
{
	float cr2 = cos(roll/2);
	float cp2 = cos(pitch/2);
	float cy2 = cos(yaw/2);
	// the sign reversal here is due to the different conventions
	// in the madgwick quaternion code and the rest of APM
	float sr2 = -sin(roll/2);
	float sp2 = -sin(pitch/2);
	float sy2 = sin(yaw/2);

	q.q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
	q.q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
	q.q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
	q.q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// create eulers from a quaternion
void euler_from_quaternion(Quaternion &q, float *roll, float *pitch, float *yaw)
{
	*roll = -(atan2(2.0*(q.q1*q.q2 + q.q3*q.q4),
			1 - 2.0*(q.q2*q.q2 + q.q3*q.q3)));
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	*pitch = -safe_asin(2.0*(q.q1*q.q3 - q.q4*q.q2));
	*yaw = atan2(2.0*(q.q1*q.q4 + q.q2*q.q3),
		     1 - 2.0*(q.q3*q.q3 + q.q4*q.q4));
}
