//This class converts horizontal acceleration commands to fin flapping commands.
#pragma once
#include <AP_Notify/AP_Notify.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define NUM_FINS 4 //Current maximum number of fins that can be added.
#define RC_SCALE 1000
class Fins
{
public:
    friend class Blimp;

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

    enum class DesiredSpoolState : uint8_t {
        SHUT_DOWN = 0,              // all fins should move to stop
        THROTTLE_UNLIMITED = 2,     // all fins can move as needed
    };

    enum class SpoolState : uint8_t {
        SHUT_DOWN = 0,                      // all motors stop
        THROTTLE_UNLIMITED = 3,             // throttle is no longer constrained by start up procedure
    };

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
    DesiredSpoolState   _spool_desired;             // desired spool state
    SpoolState          _spool_state;               // current spool mode

    float               _time;                       //current timestep

    bool _armed;             // 0 if disarmed, 1 if armed

    float              _amp[NUM_FINS]; //amplitudes
    float              _off[NUM_FINS]; //offsets
    float              _omm[NUM_FINS]; //omega multiplier
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

    // get_spool_state - get current spool state
    enum SpoolState  get_spool_state(void) const
    {
        return _spool_state;
    }

    float max(float one, float two)
    {
        if (one >= two) {
            return one;
        } else {
            return two;
        }
    }

    void output_min();

    void add_fin(int8_t fin_num, float right_amp_fac, float front_amp_fac, float yaw_amp_fac, float down_amp_fac,
                 float right_off_fac, float front_off_fac, float yaw_off_fac, float down_off_fac);

    void setup_fins();

    float get_throttle_hover()
    {
        return 0;    //TODO
    }

    void set_desired_spool_state(DesiredSpoolState spool);

    void output();

    float get_throttle()
    {
        return 0.1f;    //TODO
    }

    void rc_write(uint8_t chan, uint16_t pwm);
};
