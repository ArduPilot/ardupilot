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


// find a rotation that is the combination of two other
// rotations. This is used to allow us to add an overall board
// rotation to an existing rotation of a sensor such as the compass
// Note that this relies the set of rotations being complete. The
// optional 'found' parameter is for the test suite to ensure that it is.
enum Rotation rotation_combination(enum Rotation r1, enum Rotation r2, bool *found)
{
    Vector3f tv1, tv2;
    enum Rotation r;
    tv1(1,2,3);
    tv1.rotate(r1);
    tv1.rotate(r2);

    for (r=ROTATION_NONE; r<ROTATION_MAX;
         r = (enum Rotation)((uint8_t)r+1)) {
        Vector3f diff;
        tv2(1,2,3);
        tv2.rotate(r);
        diff = tv1 - tv2;
        if (diff.length() < 1.0e-6) {
            // we found a match
            if (found) {
                *found = true;
            }
            return r;
        }
    }

    // we found no matching rotation. Someone has edited the
    // rotations list and broken its completeness property ...
    if (found) {
        *found = false;
    }
    return ROTATION_NONE;
}

// constrain a value
float constrain(float amt, float low, float high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int32_t value
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// degrees -> radians
float radians(float deg) {
	return deg * DEG_TO_RAD;
}

// radians -> degrees
float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

// square
float sq(float v) {
	return v*v;
}

// 2D vector length
float pythagorous2(float a, float b) {
	return sqrt(sq(a)+sq(b));
}

// 3D vector length
float pythagorous3(float a, float b, float c) {
	return sqrt(sq(a)+sq(b)+sq(c));
}
