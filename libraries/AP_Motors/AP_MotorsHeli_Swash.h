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
    friend class AP_MotorsHeli_Single;
    friend class AP_MotorsHeli_Dual;

    AP_MotorsHeli_Swash() {};

    // calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
    void calculate_roll_pitch_collective_factors();

    // set_swash_type - sets swashplate type
    void set_swash_type(SwashPlateType swash_type) { _swash_type = swash_type; }

    // set_collective_direction - sets swashplate collective direction
    void set_collective_direction(CollectiveDirection collective_direction) { _collective_direction = collective_direction; }

    // set_phase_angle - sets swashplate phase angle
    void set_phase_angle(int16_t phase_angle) { _phase_angle = phase_angle; }

    // set_phase_angle - sets swashplate phase angle
    float get_servo_out(int8_t servo_num, float pitch, float roll, float collective);

    void set_servo1_pos(int16_t servo_pos) { _servo1_pos = servo_pos; }
    void set_servo2_pos(int16_t servo_pos) { _servo2_pos = servo_pos; }
    void set_servo3_pos(int16_t servo_pos) { _servo3_pos = servo_pos; }
    void set_servo4_pos(int16_t servo_pos) { _servo4_pos = servo_pos; }

    // set_linear_servo_out - sets swashplate servo output to be linear
    void set_linear_servo_out(int8_t linear_servo) { _make_servo_linear = linear_servo; }

    //linearize mechanical output of swashplate servo
    float get_linear_servo_output(float input);

private:
    // internal variables
    SwashPlateType  _swash_type;             // Swashplate type
    CollectiveDirection  _collective_direction;  // Collective control direction, normal or reversed
    int16_t         _phase_angle;           // Phase angle correction for rotor head.  If pitching the swash forward induces a roll, this can be negative depending on mechanics.
    float           _rollFactor[4];
    float           _pitchFactor[4];
    float           _collectiveFactor[4];
    int16_t         _servo1_pos;
    int16_t         _servo2_pos;
    int16_t         _servo3_pos;
    int16_t         _servo4_pos;
    int8_t          _make_servo_linear;

};
class SwashInt16Param {
public:
    SwashInt16Param(void);

    static const struct AP_Param::GroupInfo var_info[];

    void set_enable(int8_t setenable) {enable = setenable; }
    int8_t get_enable() { return enable; }
    int16_t get_servo1_pos() const { return servo1_pos; }
    int16_t get_servo2_pos() const { return servo2_pos; }
    int16_t get_servo3_pos() const { return servo3_pos; }
    int16_t get_phase_angle() const { return phase_angle; }

private:
    AP_Int8 enable;
    AP_Int16 servo1_pos;
    AP_Int16 servo2_pos;
    AP_Int16 servo3_pos;
    AP_Int16 phase_angle;

};

