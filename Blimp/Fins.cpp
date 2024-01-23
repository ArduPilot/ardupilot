#include "Blimp.h"

#include <SRV_Channel/SRV_Channel.h>

// This is the scale used for RC inputs so that they can be scaled to the float point values used in the sine wave code.
#define FIN_SCALE_MAX 1000

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo Fins::var_info[] = {

    // @Param: FREQ_HZ
    // @DisplayName: Fins frequency
    // @Description: This is the oscillation frequency of the fins
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("FREQ_HZ", 1, Fins, freq_hz, 3),

    // @Param: TURBO_MODE
    // @DisplayName: Enable turbo mode
    // @Description: Enables double speed on high offset.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TURBO_MODE", 2, Fins, turbo_mode, 0),

    AP_GROUPEND
};

//constructor
Fins::Fins(uint16_t loop_rate) :
    _loop_rate(loop_rate)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Fins::setup_fins()
{
    //fin   #   r   f   d     y,    r   f     d     y               right, front, down, yaw for amplitude then for offset
    add_fin(0,  0,  1, 0.5,   0,    0,  0,  0.5,    0); //Back
    add_fin(1,  0, -1, 0.5,   0,    0,  0,  0.5,    0); //Front
    add_fin(2, -1,  0,   0, 0.5,    0,  0,    0,  0.5); //Right
    add_fin(3,  1,  0,   0, 0.5,    0,  0,    0, -0.5); //Left

    SRV_Channels::set_angle(SRV_Channel::k_motor1, FIN_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, FIN_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, FIN_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, FIN_SCALE_MAX);
}

void Fins::add_fin(int8_t fin_num, float right_amp_fac, float front_amp_fac, float down_amp_fac, float yaw_amp_fac,
                   float right_off_fac, float front_off_fac, float down_off_fac, float yaw_off_fac)
{

    // ensure valid fin number is provided
    if (fin_num >= 0 && fin_num < NUM_FINS) {

        // set amplitude factors
        _right_amp_factor[fin_num] = right_amp_fac;
        _front_amp_factor[fin_num] = front_amp_fac;
        _down_amp_factor[fin_num] = down_amp_fac;
        _yaw_amp_factor[fin_num] = yaw_amp_fac;

        // set offset factors
        _right_off_factor[fin_num] = right_off_fac;
        _front_off_factor[fin_num] = front_off_fac;
        _down_off_factor[fin_num] = down_off_fac;
        _yaw_off_factor[fin_num] = yaw_off_fac;
    }
}

//B,F,R,L = 0,1,2,3
void Fins::output()
{
    if (!_armed) {
        // set everything to zero so fins stop moving
        right_out = 0;
        front_out = 0;
        down_out  = 0;
        yaw_out   = 0;
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_FINI(right_out, front_out, down_out, yaw_out);
#endif

    //Constrain after logging so as to still show when sub-optimal tuning is causing massive overshoots.
    right_out = constrain_float(right_out, -1, 1);
    front_out = constrain_float(front_out, -1, 1);
    down_out = constrain_float(down_out, -1, 1);
    yaw_out = constrain_float(yaw_out, -1, 1);

    _time = AP_HAL::micros() * 1.0e-6;

    for (int8_t i=0; i<NUM_FINS; i++) {
        _amp[i] =  fmaxf(0,_right_amp_factor[i]*right_out) + fmaxf(0,_front_amp_factor[i]*front_out) +
                   fabsf(_down_amp_factor[i]*down_out) + fabsf(_yaw_amp_factor[i]*yaw_out);
        _off[i] = _right_off_factor[i]*right_out + _front_off_factor[i]*front_out +
                  _down_off_factor[i]*down_out + _yaw_off_factor[i]*yaw_out;
        _freq[i] = 1;

        _num_added = 0;
        if (fmaxf(0,_right_amp_factor[i]*right_out) > 0.0f) {
            _num_added++;
        }
        if (fmaxf(0,_front_amp_factor[i]*front_out) > 0.0f) {
            _num_added++;
        }
        if (fabsf(_down_amp_factor[i]*down_out) > 0.0f) {
            _num_added++;
        }
        if (fabsf(_yaw_amp_factor[i]*yaw_out) > 0.0f) {
            _num_added++;
        }

        if (_num_added > 0) {
            _off[i] = _off[i]/_num_added; //average the offsets
        }

        if ((_amp[i]+fabsf(_off[i])) > 1) {
            _amp[i] = 1 - fabsf(_off[i]);
        }

        if (turbo_mode) {
            //double speed fins if offset at max...
            if (_amp[i] <= 0.6 && fabsf(_off[i]) >= 0.4) {
                _freq[i] = 2;
            }
        }
        // finding and outputting current position for each servo from sine wave
        _pos[i]= _amp[i]*cosf(freq_hz * _freq[i] * _time * 2 * M_PI) + _off[i];
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _pos[i] * FIN_SCALE_MAX);
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_FINO(_amp, _off);
#endif
}

void Fins::output_min()
{
    right_out = 0;
    front_out = 0;
    down_out  = 0;
    yaw_out   = 0;
    Fins::output();
}
