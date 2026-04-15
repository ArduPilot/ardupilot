/// @file   AP_MotorsHawk.h
/// @brief  Motor control class for HAWK vertical test frame

#pragma once

#include "AP_Motors_config.h"

#if AP_MOTORS_HAWK_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsMulticopter.h"

class AP_MotorsHawk : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsHawk(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
        _motor_out[0] = 0.0f;
        _motor_out[1] = 0.0f;
        _motor_out[2] = 0.0f;

        _encoder_angle[0] = 0.0f;
        _encoder_angle[1] = 0.0f;
        _encoder_angle[2] = 0.0f;

        _encoder_offset[0] = 0.0f;
        _encoder_offset[1] = 0.0f;
        _encoder_offset[2] = 0.0f;

        _hawk_roll_gain = 1.0f;
        _hawk_pitch_gain = 1.0f;
        _hawk_yaw_gain = 1.0f;
        _hawk_hover_throttle = 0.0f;

        _encoder_valid[0] = false;
        _encoder_valid[1] = false;
        _encoder_valid[2] = false;
    }

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class and type
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    void set_update_rate(uint16_t speed_hz) override;

    // output_to_motors - sends values to the motors
    void output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors
    uint32_t get_motor_mask() override;

    // run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override;

protected:

    // output - sends commands to the motors during stabilized armed operation
    void output_armed_stabilizing() override;

    // call vehicle supplied thrust compensation if set
    void thrust_compensation(void) override;

    const char* _get_frame_string() const override { return "HAWK"; }
    const char* get_type_string() const override { return ""; }

    // output_test_seq - spin a motor at the pwm value specified
    // motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    // -------------------------
    // HAWK-specific state
    // -------------------------

    // normalized output commands for the 3 motors
    float _motor_out[3];

    // encoder angles for each rotating station
    float _encoder_angle[3];

    // zero offsets for each encoder
    float _encoder_offset[3];

    // simple first-pass control gains
    float _hawk_roll_gain;
    float _hawk_pitch_gain;
    float _hawk_yaw_gain;

    // base hover throttle
    float _hawk_hover_throttle;

    // encoder validity flags
    bool _encoder_valid[3];
};

#endif  // AP_MOTORS_HAWK_ENABLED