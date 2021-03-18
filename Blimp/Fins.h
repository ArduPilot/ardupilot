//This class converts horizontal acceleration commands to fin flapping commands.
#pragma once
// #include <AP_Common/AP_Common.h>
// #include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_Notify/AP_Notify.h>      // Notify library
#include <SRV_Channel/SRV_Channel.h>
// #include <Filter/Filter.h>         // filter library
// #include <AP_HAL/AP_HAL.h>
// #include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// #define FINS_SPEED_DEFAULT 10 //MIR what is this?
#define NUM_FINS 4
#define RC_SCALE 1000
class Fins
{
public:
    friend class Blimp;
    // Fins(void);

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

    // singleton support
    // static Fins    *get_singleton(void) { return _singleton; }

    // desired spool states
    // from AP_Motors_Class.h
    enum class DesiredSpoolState : uint8_t {
        SHUT_DOWN = 0,              // all motors should move to stop
        // GROUND_IDLE = 1,            // all motors should move to ground idle
        THROTTLE_UNLIMITED = 2,     // motors should move to being a state where throttle is unconstrained (e.g. by start up procedure)
    };

    // spool states
    enum class SpoolState : uint8_t {
        SHUT_DOWN = 0,                      // all motors stop
        // GROUND_IDLE = 1,                    // all motors at ground idle
        // SPOOLING_UP = 2,                       // increasing maximum throttle while stabilizing
        THROTTLE_UNLIMITED = 3,             // throttle is no longer constrained by start up procedure
        // SPOOLING_DOWN = 4,                     // decreasing maximum throttle while stabilizing
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
    // float               _roll_in;                   // desired roll control from attitude controllers, -1 ~ +1
    // float               _roll_in_ff;                // desired roll feed forward control from attitude controllers, -1 ~ +1
    // float               _pitch_in;                  // desired pitch control from attitude controller, -1 ~ +1
    // float               _pitch_in_ff;               // desired pitch feed forward control from attitude controller, -1 ~ +1
    // float               _yaw_in;                    // desired yaw control from attitude controller, -1 ~ +1
    // float               _yaw_in_ff;                 // desired yaw feed forward control from attitude controller, -1 ~ +1
    float               _throttle_in;               // last throttle input from set_throttle caller
    float               _down_out;                  // throttle after mixing is complete
    // float               _forward_in;                // last forward input from set_forward caller
    // float               _lateral_in;                // last lateral input from set_lateral caller
    float               _throttle_avg_max;          // last throttle input from set_throttle_avg_max
    // LowPassFilterFloat  _throttle_filter;           // throttle input filter
    DesiredSpoolState   _spool_desired;             // desired spool state
    SpoolState          _spool_state;               // current spool mode

    float               _air_density_ratio;         //air density as a proportion of sea level density

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
    // private:
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

    // float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    //     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }

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
        return 0;    //MIR temp
    }

    void set_desired_spool_state(DesiredSpoolState spool);

    void output();

    float get_throttle()
    {
        return 0.1f;    //MIR temp
    }

    void rc_write(uint8_t chan, uint16_t pwm);

    // set_density_ratio - sets air density as a proportion of sea level density
    void  set_air_density_ratio(float ratio)
    {
        _air_density_ratio = ratio;
    }

};
