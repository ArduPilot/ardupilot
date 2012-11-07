// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	ThirdOrderCompFilter3D.h
/// @brief	A class to implement third order complementary filter (for combining barometer and GPS with accelerometer data)
/// math provided by Jonathan Challenger

#ifndef __THIRDORDERCOMPFILTER3D_H__
#define __THIRDORDERCOMPFILTER3D_H__

#include <inttypes.h>
#include <AP_Math.h>               // Math library for matrix and vector math
#include <AP_Buffer.h>          // ArduPilot general purpose FIFO buffer

// #defines to control how often historical accel based positions are saved
// so they can later be compared to laggy gps readings
#define THIRD_ORDER_SAVE_POS_10HZ   10
#define THIRD_ORDER_SAVE_POS_5HZ    20

#define THIRD_ORDER_COMP_FILTER_HISTORIC_XY_SAVE_COUNTER_DEFAULT THIRD_ORDER_SAVE_POS_10HZ

class ThirdOrderCompFilter3D
{
public:
    // constructor
    ThirdOrderCompFilter3D(float time_constant_seconds_xy, float time_constant_seconds_z)
        {
            update_gains(time_constant_seconds_xy, time_constant_seconds_z);
        };

    // update_gains - update gains from time constant (given in seconds)
    virtual void        update_gains(float time_constant_seconds_xy, float time_constant_seconds_z);

    // set_3rd_order - resets the first order value (i.e. position)
    virtual void        set_3rd_order_xy(float x, float y);
    virtual void        set_3rd_order_z(float z);

    // set_2nd_order - resets the second order value (i.e. velocity)
    virtual void        set_2nd_order_xy(float x, float y);
    virtual void        set_2nd_order_z(float z);

    // correct_3rd_order_z - correct accelerometer offsets using barometer or gps
    virtual void        correct_3rd_order_xy(float x, float y, Matrix3f& dcm_matrix, float deltat);
    virtual void        correct_3rd_order_z(float third_order_sample, Matrix3f& dcm_matrix, float deltat);

    // add_1st_order_sample - Add a new 1st order sample (i.e. acceleration) to the filter, but don't recalculate
    virtual void        add_1st_order_sample(Vector3f& sample) { _first_order_sample = sample; }

    // recalculate the 2nd and 3rd order estimates
    virtual void        calculate(float deltat, Matrix3f& dcm_matrix);

    // return the new estimate for the 3rd order (i.e. position)
    virtual Vector3f&   get_3rd_order_estimate() { _comp_h_total = _comp_h + _comp_h_correction; return _comp_h_total; }

    // return the new estimate for the 2nd order (i.e. velocity)
    virtual Vector3f&   get_2nd_order_estimate() { return _comp_v; }

    // set the 1st order correction vector (i.e. correction to be applied to the accelerometer)
    virtual void        set_1st_order_correction( const Vector3f &correction_vector) { _comp_k1o = correction_vector; }

    // get the 1st order correction vector (i.e. correction to be applied to the accelerometer)
    virtual Vector3f&   get_1st_order_correction( void ) { return _comp_k1o; }

//private:
    float           _k1_xy;                                 // 1st order error correction gain for horizontal position
    float           _k2_xy;                                 // 2nd order error correction gain for horizontal position
    float           _k3_xy;                                 // 3rd order error correction gain for horizontal position
    float           _k1_z;                                  // 1st order error correction gain for altitude
    float           _k2_z;                                  // 2nd order error correction gain for altitude
    float           _k3_z;                                  // 3rd order error correction gain for altitude
    Vector3f        _comp_k1o;                              // acceleration estimate
    Vector3f        _comp_v;                                // velocity estimate
    Vector3f        _comp_h;                                // position estimate
    Vector3f        _first_order_sample;                    // acceleration sample
    uint8_t         _historic_xy_counter;                   // historic positions saved when this counter reaches 10 
    AP_BufferFloat_Size10   _hist_3rd_order_estimates_x;    // buffer of historic accel based position to account for lag
    AP_BufferFloat_Size10   _hist_3rd_order_estimates_y;    // buffer of historic accel based position to account for lag
    AP_BufferFloat_Size15   _hist_3rd_order_estimates_z;    // buffer of historic accel based altitudes to account for lag
    Vector3f        _comp_h_correction;                     // sum of correction to _comp_h from delayed 1st order samples
    Vector3f        _comp_h_total;                          // sum of _comp_h + _comp_h_correction
    Vector3f        comp_k1o_ef;                            // accelerometer correction in earth frame (only z element is used).  here for debug purposes
};

#endif // __THIRDORDERCOMPFILTER3D_H__
