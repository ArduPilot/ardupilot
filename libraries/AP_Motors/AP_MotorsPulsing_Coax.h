#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_RPM/AP_RPM.h>
#include "AP_MotorsMulticopter.h"

#define AP_MOTORS_COAX_POSITIVE      1
#define AP_MOTORS_COAX_NEGATIVE     -1

#define AP_MOTORS_COAX_SERVO_INPUT_RANGE    4500

//Defines the motors used in a Coax airframe using 2 Pulsing motors
class AP_MotorsPulsing_Coax : public AP_MotorsMulticopter {
public:
    //Construct a new pulsing coax motor object
    AP_MotorsPulsing_Coax(AP_AHRS_View  *ahrs_view, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz), _ahrs_view(ahrs_view)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    //Initialize the motors
    void            init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    //Set the vehicle's frame/type
    void            set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    //Set the rate at which commands will be sent to the motors
    void            set_update_rate( uint16_t speed_hz ) override;

    //Update the output commands to the motors
    virtual void    output_to_motors() override;
    
    //Determine which motor outputs are being used
    uint32_t        get_motor_mask() override;

    //Get the state of the arming checks
    bool            arming_checks(size_t buflen, char *buffer) const override { return AP_Motors::arming_checks(buflen, buffer); }
    
    static const struct AP_Param::GroupInfo        var_info[];

protected:
    // output - sends commands to the motors
    void            output_armed_stabilizing() override;

    float           _bottom_thrust;
    float           _top_thrust;
    float           _pitch_action;
    float           _roll_action;

    uint32_t        _last_update;

    AP_AHRS_View    *_ahrs_view;
    AP_RPM          *rpm;
    
    AP_Int8         _yaw_dir;

    const char*     _get_frame_string() const override { return "PULSE"; }

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void    _output_test_seq(uint8_t motor_seq, int16_t pwm) override;
};