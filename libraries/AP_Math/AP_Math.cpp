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
void calculate_euler_angles(const Matrix3f &m, float *roll, float *pitch, float *yaw)
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
	float cr2 = cos(roll*0.5);
	float cp2 = cos(pitch*0.5);
	float cy2 = cos(yaw*0.5);
	// the sign reversal here is due to the different conventions
	// in the madgwick quaternion code and the rest of APM
	float sr2 = -sin(roll*0.5);
	float sp2 = -sin(pitch*0.5);
	float sy2 = sin(yaw*0.5);

	q.q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
	q.q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
	q.q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
	q.q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// create eulers from a quaternion
void euler_from_quaternion(const Quaternion &q, float *roll, float *pitch, float *yaw)
{
	*roll = -(atan2(2.0*(q.q1*q.q2 + q.q3*q.q4),
			1 - 2.0*(q.q2*q.q2 + q.q3*q.q3)));
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	*pitch = -safe_asin(2.0*(q.q1*q.q3 - q.q4*q.q2));
	*yaw = atan2(2.0*(q.q1*q.q4 + q.q2*q.q3),
		     1 - 2.0*(q.q3*q.q3 + q.q4*q.q4));
}

// convert a quaternion to a rotation matrix
void quaternion_to_rotation_matrix(const Quaternion &q, Matrix3f &m)
{
	float q3q3 = q.q3 * q.q3;
	float q3q4 = q.q3 * q.q4;
	float q2q2 = q.q2 * q.q2;
	float q2q3 = q.q2 * q.q3;
	float q2q4 = q.q2 * q.q4;
	float q1q2 = q.q1 * q.q2;
	float q1q3 = q.q1 * q.q3;
	float q1q4 = q.q1 * q.q4;
	float q4q4 = q.q4 * q.q4;

	m.a.x = 1-2*(q3q3 + q4q4);
	m.a.y =   2*(q2q3 - q1q4);
	m.a.z = - 2*(q2q4 + q1q3);
	m.b.x =   2*(q2q3 + q1q4);
	m.b.y = 1-2*(q2q2 + q4q4);
	m.b.z =  -2*(q3q4 - q1q2);
	m.c.x =  -2*(q2q4 - q1q3);
	m.c.y =  -2*(q3q4 + q1q2);
	m.c.z = 1-2*(q2q2 + q3q3);
}

// convert a vector in earth frame to a vector in body frame,
// assuming body current rotation is given by a quaternion
void quaternion_earth_to_body(const Quaternion &q, Vector3f &v)
{
	Matrix3f m;
	// we reverse z before and afterwards because of the differing
	// quaternion conventions from APM conventions.
	v.z = -v.z;
	quaternion_to_rotation_matrix(q, m);
	v = m * v;
	v.z = -v.z;
}
