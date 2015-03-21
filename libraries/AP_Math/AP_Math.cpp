#include "AP_Math.h"

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return PI/2;
    }
    if (v <= -1.0f) {
        return -PI/2;
    }
    return asinf(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

// a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
}

#if HAL_CPU_CLASS < HAL_CPU_CLASS_75 || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    #define FAST_ATAN2_PIBY2_FLOAT  1.5707963f
    // fast_atan2 - faster version of atan2
    //      126 us on AVR cpu vs 199 for regular atan2
    //      absolute error is < 0.005 radians or 0.28 degrees
    //      origin source: https://gist.github.com/volkansalma/2972237/raw/
    float fast_atan2(float y, float x)
    {
        if (x == 0.0f) {
            if (y > 0.0f) {
                return FAST_ATAN2_PIBY2_FLOAT;
            }
            if (y == 0.0f) {
                return 0.0f;
            }
            return -FAST_ATAN2_PIBY2_FLOAT;
        }
        float atan;
        float z = y/x;
        if (fabs( z ) < 1.0f) {
            atan = z / (1.0f + 0.28f * z * z);
            if (x < 0.0f) {
                if (y < 0.0f) {
                    return atan - PI;
                }
                return atan + PI;
            }
        } else {
            atan = FAST_ATAN2_PIBY2_FLOAT - (z / (z * z + 0.28f));
            if (y < 0.0f) {
                return atan - PI;
            }
        }
        return atan;
    }
#else
    float fast_atan2(float y, float x)
    {
        return atan2f(y,x);
    }
#endif

#if ROTATION_COMBINATION_SUPPORT
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
        if (diff.length() < 1.0e-6f) {
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
#endif



