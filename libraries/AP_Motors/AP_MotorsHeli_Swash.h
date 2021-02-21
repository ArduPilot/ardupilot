/// @file	AP_MotorsHeli_Swash.h
/// @brief	Swashplate Library for traditional heli
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param/AP_Param.h>

// swashplate types
enum SwashPlateType {
    SWASHPLATE_TYPE_H3 = 0,
    SWASHPLATE_TYPE_H1,
    SWASHPLATE_TYPE_H3_140,
    SWASHPLATE_TYPE_H3_120,
    SWASHPLATE_TYPE_H4_90,
    SWASHPLATE_TYPE_H4_45
};

// collective direction
enum CollectiveDirection {
    COLLECTIVE_DIRECTION_NORMAL = 0,
    COLLECTIVE_DIRECTION_REVERSED
};

class AP_MotorsHeli_Swash {
public:

    AP_MotorsHeli_Swash() 
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // configure - configure the swashplate settings for any updated parameters
    void configure();

    // CCPM Mixers - calculate mixing scale factors by swashplate type
    void calculate_roll_pitch_collective_factors();

    // get_swash_type - gets swashplate type
    SwashPlateType get_swash_type() const { return _swash_type; }

    // get_servo_out - calculates servo output
    float get_servo_out(int8_t servo_num, float pitch, float roll, float collective) const;

    // linearize mechanical output of swashplate servo
    float get_linear_servo_output(float input) const;

    // get_phase_angle - returns the rotor phase angle
    int16_t get_phase_angle() const { return _phase_angle; }

    // var_info
    static const struct AP_Param::GroupInfo var_info[];

private:
    // internal variables
    SwashPlateType       _swash_type;                 // Swashplate type
    CollectiveDirection  _collective_direction;       // Collective control direction, normal or reversed
    float                _rollFactor[4];              // Roll axis scaling of servo output based on servo position
    float                _pitchFactor[4];             // Pitch axis scaling of servo output based on servo position
    float                _collectiveFactor[4];        // Collective axis scaling of servo output based on servo position
    int8_t               _make_servo_linear;          // Sets servo output to be linearized

    // parameters
    AP_Int8  _swashplate_type;                   // Swash Type Setting
    AP_Int8  _swash_coll_dir;                    // Collective control direction, normal or reversed
    AP_Int8  _linear_swash_servo;                // linearize swashplate output
    AP_Int8  enable;
    AP_Int16 _servo1_pos;                        // servo1 azimuth position on swashplate with front of heli being 0 deg
    AP_Int16 _servo2_pos;                        // servo2 azimuth position on swashplate with front of heli being 0 deg
    AP_Int16 _servo3_pos;                        // servo3 azimuth position on swashplate with front of heli being 0 deg
    AP_Int16 _phase_angle;                       // Phase angle correction for rotor head.  If pitching the swash forward induces 
                                                 // a roll, this can be negative depending on mechanics.

};

