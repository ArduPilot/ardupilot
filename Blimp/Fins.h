//This class converts horizontal acceleration commands to fin flapping commands.
#pragma once
#include <AP_Notify/AP_Notify.h>

extern const AP_HAL::HAL& hal;

#define NUM_FINS 4 //Current maximum number of fins that can be added.
#define RC_SCALE 1000
class Fins
{
public:
    friend class Blimp;
    friend class Loiter;

    enum motor_frame_class {
        MOTOR_FRAME_UNDEFINED = 0,
        MOTOR_FRAME_AIRFISH = 1,
    };
    enum motor_frame_type {
        MOTOR_FRAME_TYPE_AIRFISH = 1,
    };

    //constructor
    Fins(uint16_t loop_rate);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

    bool initialised_ok() const
    {
        return true;
    }

    void armed(bool arm)
    {
        if (arm != _armed) {
            _armed = arm;
            AP_Notify::flags.armed = arm;
        }

    }
    bool armed() const
    {
        return _armed;
    }

protected:
    // internal variables
    const uint16_t      _loop_rate;                 // rate in Hz at which output() function is called (normally 400hz)
    uint16_t            _speed_hz;                  // speed in hz to send updates to motors
    float               _throttle_avg_max;          // last throttle input from set_throttle_avg_max

    float               _time;                       // current timestamp

    bool _armed;             // 0 if disarmed, 1 if armed

    float              _amp[NUM_FINS]; //amplitudes
    float              _off[NUM_FINS]; //offsets
    float              _freq[NUM_FINS]; //frequency multiplier
    float              _pos[NUM_FINS]; //servo positions

    float               _right_amp_factor[NUM_FINS];
    float               _front_amp_factor[NUM_FINS];
    float               _down_amp_factor[NUM_FINS];
    float               _yaw_amp_factor[NUM_FINS];

    float               _right_off_factor[NUM_FINS];
    float               _front_off_factor[NUM_FINS];
    float               _down_off_factor[NUM_FINS];
    float               _yaw_off_factor[NUM_FINS];

    int8_t              _num_added;

    //MIR This should probably become private in future.
public:
    float               right_out;                  //input right movement, negative for left, +1 to -1
    float               front_out;                  //input front/forwards movement, negative for backwards, +1 to -1
    float               yaw_out;                    //input yaw, +1 to -1
    float               down_out;                   //input height control, +1 to -1

    AP_Float            freq_hz;
    AP_Int8             turbo_mode;

    bool _interlock;         // 1 if the motor interlock is enabled (i.e. motors run), 0 if disabled (motors don't run)
    bool _initialised_ok;    // 1 if initialisation was successful

    void output_min();

    void add_fin(int8_t fin_num, float right_amp_fac, float front_amp_fac, float yaw_amp_fac, float down_amp_fac,
                 float right_off_fac, float front_off_fac, float yaw_off_fac, float down_off_fac);

    void setup_fins();

    void output();

    float get_throttle()
    {
        //Only for Mavlink - essentially just an indicator of how hard the fins are working.
        //Note that this is the unconstrained version, so if the higher level control gives too high input,
        //throttle will be displayed as more than 100.
        return fmaxf(fmaxf(fabsf(down_out),fabsf(front_out)), fmaxf(fabsf(right_out),fabsf(yaw_out)));
    }

    void rc_write(uint8_t chan, uint16_t pwm);
};
