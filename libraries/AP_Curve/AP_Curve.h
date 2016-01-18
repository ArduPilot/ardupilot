// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Curve.h
/// @brief	used to transforms a pwm value to account for the non-linear pwm->thrust values of normal ESC+motors

#ifndef __AP_CURVE_H__
#define __AP_CURVE_H__

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_HAL.h>

/// @class      AP_Curve
template <class T, uint8_t SIZE>
class AP_Curve {
public:

    // Constructor
    AP_Curve();

    // clear - removes all points from the curve
    void clear();

    // add_point - adds a point to the curve.  returns TRUE if successfully added
    bool add_point( T x, T y );

    // get_y - returns the point on the curve at the given pwm_value (i.e. the new modified pwm_value)
    T get_y( T x );

    // displays the contents of the curve (for debugging)
    void dump_curve(AP_HAL::BetterStream*);

protected:
    uint8_t     _num_points;						// number of points in the cruve
    T           _x[SIZE];			// x values of each point on the curve
    T           _y[SIZE];			// y values of each point on the curve
    float       _slope[SIZE];		// slope between any two points.  i.e. slope[0] is the slope between points 0 and 1
    bool        _constrained;       // if true, first and last points added will constrain the y values returned by get_y function
};


/* Typedefs for template instansations of AP_Curve.
 * Only use the AP_Curve instances listed here!
 * If you need a different one, you must first instantiate the template at the
 * end of AP_Curve.cpp, then add a typedef here.  We can't leave the whole
 * template implementation in the header due to PSTR related issues.
 */

typedef AP_Curve<int16_t,3> AP_CurveInt16_Size3;
typedef AP_Curve<int16_t,4> AP_CurveInt16_Size4;
typedef AP_Curve<int16_t,5> AP_CurveInt16_Size5;
typedef AP_Curve<uint16_t,3> AP_CurveUInt16_Size3;
typedef AP_Curve<uint16_t,4> AP_CurveUInt16_Size4;
typedef AP_Curve<uint16_t,5> AP_CurveUInt16_Size5;

#endif  // __AP_CURVE_H__
