/// @file	AP_MotorsHeli_Swash.h
/// @brief	Swashplate Library for traditional heli
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger_config.h>

// swashplate types
enum SwashPlateType {
    SWASHPLATE_TYPE_H3 = 0,
    SWASHPLATE_TYPE_H1,
    SWASHPLATE_TYPE_H3_140,
    SWASHPLATE_TYPE_H3_120,
    SWASHPLATE_TYPE_H4_90,
    SWASHPLATE_TYPE_H4_45
};

class AP_MotorsHeli_Swash {
public:

    AP_MotorsHeli_Swash(uint8_t mot_0, uint8_t mot_1, uint8_t mot_2, uint8_t mot_3, uint8_t instance);

    // configure - configure the swashplate settings for any updated parameters
    void configure();

    // get_swash_type - gets swashplate type
    SwashPlateType get_swash_type() const { return _swash_type; }

    // calculates servo output
    void calculate(float roll, float pitch, float collective);

    // Output calculated values to servos
    void output();

    // get_phase_angle - returns the rotor phase angle
    int16_t get_phase_angle() const { return _phase_angle; }

    // Get function output mask
    uint32_t get_output_mask() const;

#if HAL_LOGGING_ENABLED
    // Write SWSH log for this instance of swashplate
    void write_log(float cyclic_scaler, float col_ang_min, float col_ang_max, int16_t col_min, int16_t col_max) const;
#endif

    // Swashplate specific arming checks
    bool arming_checks(size_t buflen, char *buffer) const;

    // var_info
    static const struct AP_Param::GroupInfo var_info[];

private:

    // linearize mechanical output of swashplate servo
    void linearise_servo_output(float& input) const;

    // CCPM Mixers - calculate mixing scale factors by swashplate type
    void calculate_roll_pitch_collective_factors();

    // Setup a servo
    void add_servo_angle(uint8_t num, float angle, float collective);
    void add_servo_raw(uint8_t num, float roll, float pitch, float collective);

    // write to a swash servo. output value is pwm
    void rc_write(uint8_t chan, float swash_in);

    enum CollectiveDirection {
        COLLECTIVE_DIRECTION_NORMAL = 0,
        COLLECTIVE_DIRECTION_REVERSED
    };

    static const uint8_t _max_num_servos {4};

    // Currently configured setup
    SwashPlateType       _swash_type;                 // Swashplate type
    CollectiveDirection  _collective_direction;       // Collective control direction, normal or reversed

    // Internal variables
    bool                 _enabled[_max_num_servos];                 // True if this output servo is enabled
    float                _rollFactor[_max_num_servos];              // Roll axis scaling of servo output based on servo position
    float                _pitchFactor[_max_num_servos];             // Pitch axis scaling of servo output based on servo position
    float                _collectiveFactor[_max_num_servos];        // Collective axis scaling of servo output based on servo position
    float                _output[_max_num_servos];                  // Servo output value
    const uint8_t        _motor_num[_max_num_servos];               // Motor function to use for output
    const uint8_t        _instance;                                 // Swashplate instance. Used for logging.

    // Variables stored for logging
    float _roll_input;
    float _pitch_input;
    float _collective_input_scaled;

    // parameters
    AP_Int8  _swashplate_type;                   // Swash Type Setting
    AP_Int8  _swash_coll_dir;                    // Collective control direction, normal or reversed
    AP_Float  _linear_swash_servo_ang_deg;       // Swashplate servo horn max range level, used to linearize swashplate output
    AP_Int8  enable;
    AP_Int16 _servo1_pos;                        // servo1 azimuth position on swashplate with front of heli being 0 deg
    AP_Int16 _servo2_pos;                        // servo2 azimuth position on swashplate with front of heli being 0 deg
    AP_Int16 _servo3_pos;                        // servo3 azimuth position on swashplate with front of heli being 0 deg
    AP_Int16 _phase_angle;                       // Phase angle correction for rotor head.  If pitching the swash forward induces 
                                                 // a roll, this can be negative depending on mechanics.

};

