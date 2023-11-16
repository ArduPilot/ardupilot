#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_RPM/AP_RPM.h>
#include "AP_MotorsMulticopter.h"

#define AP_MOTORS_COAX_POSITIVE      1
#define AP_MOTORS_COAX_NEGATIVE     -1

#define AP_MOTORS_COAX_SERVO_INPUT_RANGE    4500

//This class defines a helecopter frame that uses pulsing motors

class AP_MotorsPulsing_Heli : public AP_MotorsMulticopter {
public:
    //Create an instance of a Pulsing Heli object
    AP_MotorsPulsing_Heli(AP_AHRS_View  *ahrs_view, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz), _ahrs_view(ahrs_view)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    //Intialize the frame
    void            init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void            set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    void            set_update_rate( uint16_t speed_hz ) override;

    //Updates the output values to all motors
    virtual void    output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t        get_motor_mask() override;

    //Returns whether or not initialization was successful
    bool            arming_checks(size_t buflen, char *buffer) const override
    {
        return AP_Motors::arming_checks(buflen, buffer);
    }

    //Stores the group information for a pulsing helicopter
    static const struct AP_Param::GroupInfo        var_info[];

protected:
    // output - sends commands to the motors
    void            output_armed_stabilizing() override;

    float           _rotor_thrust;
    float           _tail_thrust;
    float           _pitch_action;
    float           _roll_action;

    AP_AHRS_View    *_ahrs_view;
    AP_RPM          *rpm;

    AP_Int8         _yaw_dir;
    AP_Float        _rotor_yaw_ff;
    AP_Float        _gyro_ff_gain;

    const char*     _get_frame_string() const override
    {
        return "PULSE";
    }

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void    _output_test_seq(uint8_t motor_seq, int16_t pwm) override;
};