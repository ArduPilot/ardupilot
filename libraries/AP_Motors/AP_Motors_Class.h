#pragma once

#include "AP_Motors_config.h"

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>         // filter library
#include <Filter/DerivativeFilter.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Logger/AP_Logger_config.h>
#include <SRV_Channel/SRV_Channel_config.h>

// offsets for motors in motor_out and _motor_filtered arrays
#define AP_MOTORS_MOT_1 0U
#define AP_MOTORS_MOT_2 1U
#define AP_MOTORS_MOT_3 2U
#define AP_MOTORS_MOT_4 3U
#define AP_MOTORS_MOT_5 4U
#define AP_MOTORS_MOT_6 5U
#define AP_MOTORS_MOT_7 6U
#define AP_MOTORS_MOT_8 7U
#define AP_MOTORS_MOT_9 8U
#define AP_MOTORS_MOT_10 9U
#define AP_MOTORS_MOT_11 10U
#define AP_MOTORS_MOT_12 11U
#define AP_MOTORS_MOT_13 12U
#define AP_MOTORS_MOT_14 13U
#define AP_MOTORS_MOT_15 14U
#define AP_MOTORS_MOT_16 15U
#define AP_MOTORS_MOT_17 16U
#define AP_MOTORS_MOT_18 17U
#define AP_MOTORS_MOT_19 18U
#define AP_MOTORS_MOT_20 19U
#define AP_MOTORS_MOT_21 20U
#define AP_MOTORS_MOT_22 21U
#define AP_MOTORS_MOT_23 22U
#define AP_MOTORS_MOT_24 23U
#define AP_MOTORS_MOT_25 24U
#define AP_MOTORS_MOT_26 25U
#define AP_MOTORS_MOT_27 26U
#define AP_MOTORS_MOT_28 27U
#define AP_MOTORS_MOT_29 28U
#define AP_MOTORS_MOT_30 29U
#define AP_MOTORS_MOT_31 30U
#define AP_MOTORS_MOT_32 31U

// motor update rate
#define AP_MOTORS_SPEED_DEFAULT     490 // default output rate to the motors

/// @class      AP_Motors
class AP_Motors {
public:

    enum motor_frame_class {
        MOTOR_FRAME_UNDEFINED = 0,
        MOTOR_FRAME_QUAD = 1,
        MOTOR_FRAME_HEXA = 2,
        MOTOR_FRAME_OCTA = 3,
        MOTOR_FRAME_OCTAQUAD = 4,
        MOTOR_FRAME_Y6 = 5,
        MOTOR_FRAME_HELI = 6,
        MOTOR_FRAME_TRI = 7,
        MOTOR_FRAME_SINGLE = 8,
        MOTOR_FRAME_COAX = 9,
        MOTOR_FRAME_TAILSITTER = 10,
        MOTOR_FRAME_HELI_DUAL = 11,
        MOTOR_FRAME_DODECAHEXA = 12,
        MOTOR_FRAME_HELI_QUAD = 13,
        MOTOR_FRAME_DECA = 14,
        MOTOR_FRAME_SCRIPTING_MATRIX = 15,
        MOTOR_FRAME_6DOF_SCRIPTING = 16,
        MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX = 17,
    };

    // return string corresponding to frame_class
    const char* get_frame_string() const;

    enum motor_frame_type {
        MOTOR_FRAME_TYPE_PLUS = 0,
        MOTOR_FRAME_TYPE_X = 1,
        MOTOR_FRAME_TYPE_V = 2,
        MOTOR_FRAME_TYPE_H = 3,
        MOTOR_FRAME_TYPE_VTAIL = 4,
        MOTOR_FRAME_TYPE_ATAIL = 5,
        MOTOR_FRAME_TYPE_PLUSREV = 6, // plus with reversed motor direction
        MOTOR_FRAME_TYPE_Y6B = 10,
        MOTOR_FRAME_TYPE_Y6F = 11, // for FireFlyY6
        MOTOR_FRAME_TYPE_BF_X = 12, // X frame, betaflight ordering
        MOTOR_FRAME_TYPE_DJI_X = 13, // X frame, DJI ordering
        MOTOR_FRAME_TYPE_CW_X = 14, // X frame, clockwise ordering
        MOTOR_FRAME_TYPE_I = 15, // (sideways H) octo only
        MOTOR_FRAME_TYPE_NYT_PLUS = 16, // plus frame, no differential torque for yaw
        MOTOR_FRAME_TYPE_NYT_X = 17, // X frame, no differential torque for yaw
        MOTOR_FRAME_TYPE_BF_X_REV = 18, // X frame, betaflight ordering, reversed motors
        MOTOR_FRAME_TYPE_Y4 = 19, //Y4 Quadrotor frame
    };


    // returns a formatted string into buffer, e.g. "QUAD/X"
    void get_frame_and_type_string(char *buffer, uint8_t buflen) const;

    // Constructor
    AP_Motors(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // singleton support
    static AP_Motors    *get_singleton(void) { return _singleton; }

    // check initialisation succeeded
    virtual bool        arming_checks(size_t buflen, char *buffer) const;
    virtual bool        motor_test_checks(size_t buflen, char *buffer) const;
    bool                initialised_ok() const { return _initialised_ok; }
    void                set_initialised_ok(bool val) { _initialised_ok = val; }

    // arm, disarm or check status status of motors
    bool                armed() const { return _armed; }
    void                armed(bool arm);

    // set motor interlock status
    void                set_interlock(bool set) { _interlock = set;}

    // get motor interlock status.  true means motors run, false motors don't run
    bool                get_interlock() const { return _interlock; }

    // get/set spoolup block
    bool                get_spoolup_block() const { return _spoolup_block; }
    void                set_spoolup_block(bool set) { _spoolup_block = set; }

    // set_roll, set_pitch, set_yaw, set_throttle
    void                set_roll(float roll_in) { _roll_in = roll_in; };        // range -1 ~ +1
    void                set_roll_ff(float roll_in) { _roll_in_ff = roll_in; };    // range -1 ~ +1
    void                set_pitch(float pitch_in) { _pitch_in = pitch_in; };    // range -1 ~ +1
    void                set_pitch_ff(float pitch_in) { _pitch_in_ff = pitch_in; };  // range -1 ~ +1
    void                set_yaw(float yaw_in) { _yaw_in = yaw_in; };            // range -1 ~ +1
    void                set_yaw_ff(float yaw_in) { _yaw_in_ff = yaw_in; };      // range -1 ~ +1
    void                set_throttle(float throttle_in) { _throttle_in = throttle_in; };   // range 0 ~ 1
    void                set_throttle_avg_max(float throttle_avg_max) { _throttle_avg_max = constrain_float(throttle_avg_max, 0.0f, 1.0f); };   // range 0 ~ 1
    void                set_throttle_filter_cutoff(float filt_hz) { _throttle_filter.set_cutoff_frequency(filt_hz); }
    void                set_slew_filter_cutoff(float filt_hz) { _throttle_slew_filter.set_cutoff_frequency(filt_hz); }
    void                set_forward(float forward_in) { _forward_in = forward_in; }; // range -1 ~ +1
    void                set_lateral(float lateral_in) { _lateral_in = lateral_in; };     // range -1 ~ +1

    // for 6DoF vehicles, sets the roll and pitch offset, this rotates the thrust vector in body frame
    virtual void        set_roll_pitch(float roll_deg, float pitch_deg) {};

    // accessors for roll, pitch, yaw and throttle inputs to motors
    float               get_roll() const { return _roll_in; }
    float               get_roll_ff() const { return _roll_in_ff; }
    float               get_pitch() const { return _pitch_in; }
    float               get_pitch_ff() const { return _pitch_in_ff; }
    float               get_yaw() const { return _yaw_in; }
    float               get_yaw_ff() const { return _yaw_in_ff; }
    float               get_throttle_out() const { return _throttle_out; }
    virtual bool        get_thrust(uint8_t motor_num, float& thr_out) const { return false; }
    virtual bool        get_raw_motor_throttle(uint8_t motor_num, float& thr_out) const { return false; }
    float               get_throttle() const { return constrain_float(_throttle_filter.get(), 0.0f, 1.0f); }
    float               get_throttle_bidirectional() const { return constrain_float(2 * (_throttle_filter.get() - 0.5f), -1.0f, 1.0f); }
    float               get_throttle_slew_rate() const { return _throttle_slew_rate; }
    float               get_forward() const { return _forward_in; }
    float               get_lateral() const { return _lateral_in; }
    virtual float       get_throttle_hover() const = 0;

    // motor failure handling
    void                set_thrust_boost(bool enable) { _thrust_boost = enable; }
    bool                get_thrust_boost() const { return _thrust_boost; }
    virtual uint8_t     get_lost_motor() const { return 0; }

    // desired spool states
    enum class DesiredSpoolState : uint8_t {
        SHUT_DOWN = 0,              // all motors should move to stop
        GROUND_IDLE = 1,            // all motors should move to ground idle
        THROTTLE_UNLIMITED = 2,     // motors should move to being a state where throttle is unconstrained (e.g. by start up procedure)
    };

    void set_desired_spool_state(enum DesiredSpoolState spool);

    enum DesiredSpoolState get_desired_spool_state(void) const { return _spool_desired; }

    // spool states
    enum class SpoolState : uint8_t {
        SHUT_DOWN = 0,                      // all motors stop
        GROUND_IDLE = 1,                    // all motors at ground idle
        SPOOLING_UP = 2,                       // increasing maximum throttle while stabilizing
        THROTTLE_UNLIMITED = 3,             // throttle is no longer constrained by start up procedure
        SPOOLING_DOWN = 4,                     // decreasing maximum throttle while stabilizing
    };

    // get_spool_state - get current spool state
    enum SpoolState  get_spool_state(void) const { return _spool_state; }

    // set_dt_s / get_dt_s - dt is the time since the last time (in seconds) the motor mixers were updated
    //   _dt should be set based on the time of the last IMU read used by these controllers
    //   the motor mixers should run on each loop to ensure normal operation
    void set_dt_s(float dt_s) { _dt_s = dt_s; }
    float get_dt_s() const { return _dt_s; }

    // structure for holding motor limit flags
    struct AP_Motors_limit {
        bool roll;           // we have reached roll or pitch limit
        bool pitch;          // we have reached roll or pitch limit
        bool yaw;            // we have reached yaw limit
        bool throttle_lower; // we have reached throttle's lower limit
        bool throttle_upper; // we have reached throttle's upper limit
    } limit;

    // set limit flag for pitch, roll and yaw
    void set_limit_flag_pitch_roll_yaw(bool flag);

#if AP_SCRIPTING_ENABLED
    // set limit flag for pitch, roll and yaw
    void set_external_limits(bool roll, bool pitch, bool yaw, bool throttle_lower, bool throttle_upper);
#endif

    //
    // virtual functions that should be implemented by child classes
    //

    // set update rate to motors - a value in hertz
    virtual void        set_update_rate( uint16_t speed_hz ) { _speed_hz = speed_hz; }

    // init
    virtual void        init(motor_frame_class frame_class, motor_frame_type frame_type) = 0;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    virtual void        set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) = 0;

    // output - sends commands to the motors
    virtual void        output() = 0;

    // output_min - sends minimum values out to the motors
    virtual void        output_min() = 0;

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    //  return true if output was successful, false if not possible
    bool                output_test_seq(uint8_t motor_seq, int16_t pwm);

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint32_t    get_motor_mask() = 0;

    // pilot input in the -1 ~ +1 range for roll, pitch and yaw. 0~1 range for throttle
    void                set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input);

    // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // using copter motors for forward flight
    virtual float       get_roll_factor(uint8_t i) { return 0.0f; }

    // return the pitch factor of any motor
    virtual float       get_pitch_factor(uint8_t i) { return 0.0f; }

    // return whether a motor is enabled or not
    virtual bool        is_motor_enabled(uint8_t i) { return false; }

    // This function required for tradheli. Tradheli initializes targets when going from unarmed to armed state.
    // This function is overriden in motors_heli class.   Always true for multicopters.
    virtual bool init_targets_on_arming() const { return true; }

    // returns true if the configured PWM type is digital and should have fixed endpoints
    bool is_digital_pwm_type() const;

    // returns true is pwm type is brushed
    bool is_brushed_pwm_type() const { return _pwm_type == PWMType::BRUSHED; }

    // returns true is pwm type is normal
    bool is_normal_pwm_type() const { return (_pwm_type == PWMType::NORMAL) || (_pwm_type == PWMType::PWM_RANGE) || (_pwm_type == PWMType::PWM_ANGLE); }

    MAV_TYPE get_frame_mav_type() const { return _mav_type; }

    // direct motor write
    virtual void        rc_write(uint8_t chan, uint16_t pwm);

#if AP_SCRIPTING_ENABLED
    void set_frame_string(const char * str);
#endif

#if HAL_LOGGING_ENABLED
    // write log, to be called at 10hz
    virtual void Log_Write() {};
#endif

    enum MotorOptions : uint8_t {
        BATT_RAW_VOLTAGE = (1 << 0U)
    };
    bool has_option(MotorOptions option) { return _options.get() & uint8_t(option); }

protected:
    // output functions that should be overloaded by child classes
    virtual void        output_armed_stabilizing() = 0;
    virtual void        rc_write_angle(uint8_t chan, int16_t angle_cd);
    virtual void        rc_set_freq(uint32_t mask, uint16_t freq_hz);


    /*
      map an internal motor mask to real motor mask, accounting for
      SERVOn_FUNCTION mappings, and allowing for multiple outputs per
      motor number
    */
    uint32_t    motor_mask_to_srv_channel_mask(uint32_t mask) const;

    // add a motor to the motor map
    void add_motor_num(int8_t motor_num);
    
    // update the throttle input filter
    virtual void update_throttle_filter() = 0;

    // save parameters as part of disarming
    virtual void save_params_on_disarm() {}

    // internal variables
    float               _dt_s;                      // time difference (in seconds) since the last loop time
    uint16_t            _speed_hz;                  // speed in hz to send updates to motors
    float               _roll_in;                   // desired roll control from attitude controllers, -1 ~ +1
    float               _roll_in_ff;                // desired roll feed forward control from attitude controllers, -1 ~ +1
    float               _pitch_in;                  // desired pitch control from attitude controller, -1 ~ +1
    float               _pitch_in_ff;               // desired pitch feed forward control from attitude controller, -1 ~ +1
    float               _yaw_in;                    // desired yaw control from attitude controller, -1 ~ +1
    float               _yaw_in_ff;                 // desired yaw feed forward control from attitude controller, -1 ~ +1
    float               _throttle_in;               // last throttle input from set_throttle caller
    float               _throttle_out;              // throttle after mixing is complete
    float               _throttle_slew_rate;        // throttle slew rate from input
    float               _forward_in;                // last forward input from set_forward caller
    float               _lateral_in;                // last lateral input from set_lateral caller
    float               _throttle_avg_max;          // last throttle input from set_throttle_avg_max
    LowPassFilterFloat  _throttle_filter;           // pilot throttle input filter
    DerivativeFilterFloat_Size7  _throttle_slew;    // throttle output slew detector
    LowPassFilterFloat  _throttle_slew_filter;      // filter for the output of the throttle slew
    DesiredSpoolState   _spool_desired;             // desired spool state
    SpoolState          _spool_state;               // current spool mode

    // mask of what channels need fast output
    uint32_t            _motor_fast_mask;

    // Used with PWMType::PWM_RANGE and PWMType::PWM_ANGLE
    struct {
        // Mask of motors using scaled output
        uint32_t mask;

        // Offset used to convert from PWM to scaled value
        float offset;
    } _motor_pwm_scaled;

    
    // pass through variables
    float _roll_radio_passthrough;     // roll input from pilot in -1 ~ +1 range.  used for setup and providing servo feedback while landed
    float _pitch_radio_passthrough;    // pitch input from pilot in -1 ~ +1 range.  used for setup and providing servo feedback while landed
    float _throttle_radio_passthrough; // throttle/collective input from pilot in 0 ~ 1 range.  used for setup and providing servo feedback while landed
    float _yaw_radio_passthrough;      // yaw input from pilot in -1 ~ +1 range.  used for setup and providing servo feedback while landed

    enum class PWMType : uint8_t {
        NORMAL     = 0,
        ONESHOT    = 1,
        ONESHOT125 = 2,
        BRUSHED    = 3,
        DSHOT150   = 4,
        DSHOT300   = 5,
        DSHOT600   = 6,
        DSHOT1200  = 7,
        PWM_RANGE  = 8,
        PWM_ANGLE  = 9,
    };

    AP_Enum<PWMType>             _pwm_type;            // PWM output type

    // motor failure handling
    bool                _thrust_boost;          // true if thrust boost is enabled to handle motor failure
    bool                _thrust_balanced;       // true when output thrust is well balanced
    float               _thrust_boost_ratio;    // choice between highest and second highest motor output for output mixing (0 ~ 1). Zero is normal operation

    // motor options
    AP_Int16            _options;

    MAV_TYPE _mav_type; // MAV_TYPE_GENERIC = 0;

    // return string corresponding to frame_class
    virtual const char* _get_frame_string() const = 0;

    // return string corresponding to frame_type
    virtual const char* get_type_string() const { return ""; }

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) = 0;

#if AP_SCRIPTING_ENABLED
    // Custom frame string set from scripting
    char* custom_frame_string;

    // external limits from scripting
    AP_Motors_limit external_limits;
#endif

private:

    bool _armed;             // 0 if disarmed, 1 if armed
    bool _interlock;         // 1 if the motor interlock is enabled (i.e. motors run), 0 if disabled (motors don't run)
    bool _initialised_ok;    // 1 if initialisation was successful
    bool _spoolup_block;     // true if spoolup is blocked

    static AP_Motors *_singleton;
};

namespace AP {
    AP_Motors *motors();
};
