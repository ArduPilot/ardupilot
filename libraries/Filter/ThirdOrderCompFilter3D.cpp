/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <FastSerial.h>
#include <ThirdOrderCompFilter3D.h>

#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include <wiring.h>
#endif

// Public Methods //////////////////////////////////////////////////////////////

// update_gains - update gains from time constant (given in seconds)
void ThirdOrderCompFilter3D::update_gains(float time_constant_seconds_xy, float time_constant_seconds_z)
{
    static float last_time_constant_xy = 0;
    static float last_time_constant_z = 0;

    // X & Y axis time constant
    if( time_constant_seconds_xy == 0 ) {
        _k1_xy = _k2_xy = _k3_xy = 0;
    }else{
        if( time_constant_seconds_xy != last_time_constant_xy ) {
            _k1_xy = 3 / time_constant_seconds_xy;
            _k2_xy = 3 / (time_constant_seconds_xy*time_constant_seconds_xy);
            _k3_xy = 1 / (time_constant_seconds_xy*time_constant_seconds_xy*time_constant_seconds_xy);
            last_time_constant_xy = time_constant_seconds_xy;
        }
    }

    // Z axis time constant
    if( time_constant_seconds_z == 0 ) {
        _k1_z = _k2_z = _k3_z = 0;
    }else{
        if( time_constant_seconds_z != last_time_constant_z ) {
            _k1_z = 3 / time_constant_seconds_z;
            _k2_z = 3 / (time_constant_seconds_z*time_constant_seconds_z);
            _k3_z = 1 / (time_constant_seconds_z*time_constant_seconds_z*time_constant_seconds_z);
            last_time_constant_z = time_constant_seconds_z;
        }
    }
}

// set_3rd_order - resets the first order value (i.e. position)
void ThirdOrderCompFilter3D::set_3rd_order_xy(float x, float y)
{
    _comp_h.x = x;
    _comp_h.y = y;
    _comp_h_correction.x = 0;
    _comp_h_correction.y = 0;

    // clear historic estimates
    _hist_3rd_order_estimates_x.clear();
    _hist_3rd_order_estimates_y.clear();
}

// set_3rd_order - resets the first order value (i.e. position)
void ThirdOrderCompFilter3D::set_3rd_order_z(float z )
{
    _comp_h.z = z;
    _comp_h_correction.z = 0;
}

// set_2nd_order - resets the second order value (i.e. velocity)
void ThirdOrderCompFilter3D::set_2nd_order_xy(float x, float y)
{
    _comp_v.x = x;
    _comp_v.y = y;
}

// set_2nd_order - resets the second order value (i.e. velocity)
void ThirdOrderCompFilter3D::set_2nd_order_z(float z )
{
    _comp_v.z = z;
}

// correct_3rd_order_z - correct accelerometer offsets using barometer or gps
void ThirdOrderCompFilter3D::correct_3rd_order_xy(float x, float y, Matrix3f& dcm_matrix, float deltat)
{
    float hist_comp_h_x, hist_comp_h_y;
    
    // 3rd order samples (i.e. position from gps) are delayed by 500ms
    // we store historical position at 10hz so 5 iterations ago
    if( _hist_3rd_order_estimates_x.num_items() >= 4 ) {
        hist_comp_h_x = _hist_3rd_order_estimates_x.peek(3);
        hist_comp_h_y = _hist_3rd_order_estimates_y.peek(3);
    }else{
        hist_comp_h_x = _comp_h.x;
        hist_comp_h_y = _comp_h.y;
    }

    // calculate error in position from gps with our historical estimate
    float err_x = x - (hist_comp_h_x + _comp_h_correction.x);
    float err_y = y - (hist_comp_h_y + _comp_h_correction.y);

    // calculate correction to accelerometers and apply in the body frame
    _comp_k1o += dcm_matrix.mul_transpose(Vector3f((err_x*_k3_xy)*deltat,(err_y*_k3_xy)*deltat,0));

    // correct velocity
    _comp_v.x += (err_x*_k2_xy) * deltat;
    _comp_v.y += (err_y*_k2_xy) * deltat;

    // correct position
    _comp_h_correction.x += err_x*_k1_xy * deltat;
    _comp_h_correction.y += err_y*_k1_xy * deltat;
}

// correct_3rd_order_z - correct accelerometer offsets using barometer or gps
void ThirdOrderCompFilter3D::correct_3rd_order_z(float third_order_sample, Matrix3f& dcm_matrix, float deltat)
{
    float hist_comp_h_z;
    
    // 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
    // so we should calculate error using historical estimates
    if( _hist_3rd_order_estimates_z.num_items() >= 15 ) {
        //hist_comp_h_z = _hist_3rd_order_estimates_z.get();
        hist_comp_h_z = _hist_3rd_order_estimates_z.peek(14);
    }else{
        hist_comp_h_z = _comp_h.z;
    }

    // calculate error in position from baro with our estimate
    float err = third_order_sample - (hist_comp_h_z + _comp_h_correction.z);

    // calculate correction to accelerometers and apply in the body frame
    _comp_k1o += dcm_matrix.mul_transpose(Vector3f(0,0,(err*_k3_z) * deltat));

    // correct velocity
    _comp_v.z += (err*_k2_z) * deltat;

    // correct position
    _comp_h_correction.z += err*_k1_z * deltat;
}

// recalculate the 2nd and 3rd order estimates
void ThirdOrderCompFilter3D::calculate(float deltat, Matrix3f& dcm_matrix)
{
    // get earth frame accelerometer correction
    comp_k1o_ef = dcm_matrix * _comp_k1o;

    // calculate velocity by adding new acceleration from accelerometers
    _comp_v += (-_first_order_sample + comp_k1o_ef) * deltat;

    // calculate new estimate of position
    _comp_h += _comp_v * deltat;

    // store 3rd order estimate (i.e. estimated vertical position) for future use
    _hist_3rd_order_estimates_z.add(_comp_h.z);

    // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
    _historic_xy_counter++;
    if( _historic_xy_counter >= THIRD_ORDER_SAVE_POS_10HZ ) {
        _historic_xy_counter = 0;
        _hist_3rd_order_estimates_x.add(_comp_h.x);
        _hist_3rd_order_estimates_y.add(_comp_h.y);
    }
}
