// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsPX4.h
/// @brief	Motor control class using the PX4 mixing facility - not applicable to classic APM hardware!

#ifndef __AP_MOTORS_PX4_H__
#define __AP_MOTORS_PX4_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors_Class.h"

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <systemlib/mixer/mixer.h>

#define AP_MOTORS_PX4_PWM_OUTPUT        0
#define AP_MOTORS_PX4_UAVCAN_OUTPUT     1

#define AP_MOTORS_FRAME_LAYOUT          0                           // default frame layout (quad)
#define AP_MOTORS_OUTPUT_DEVICE         AP_MOTORS_PX4_PWM_OUTPUT    // default output device (PWM)


/// @class      AP_MotorsPX4
class AP_MotorsPX4 : public AP_Motors {
public:

    /// Constructor
    AP_MotorsPX4( RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz), _actuators_0_pub(-1), _armed_pub(-1), _old_armed(false)
    {
		AP_Param::setup_object_defaults(this, var_info);
        _armed = {};
        _actuators = {};
    };

    // init
    virtual void        Init();

    // enable - starts allowing signals to be sent to motors
    virtual void        enable();

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm);

    // output_min - sends minimum values out to the motors
    virtual void        output_min();

    // get_motor_mask - returns a bitmask of which outputs are being used for motors
    virtual uint16_t    get_motor_mask();

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // publish armed state if changed
    void publish_armed();

    // publish control state
    void publish_controls();

    // load mixer configuration
    bool load_mixer();

    // output - sends commands to the motors
    virtual void        output_armed();
    virtual void        output_disarmed();

    // PX4 ORB-related structs
    orb_advert_t        _actuators_0_pub;               /**< actuator control group 0 setpoint */
    orb_advert_t        _armed_pub;

    struct actuator_controls_s _actuators;             /**< actuator control inputs */
    struct actuator_armed_s _armed;

private:
    bool                _old_armed;

    AP_Int16            _frame_layout;                // Motor configuration number
    AP_Int16            _output_device;                // Mixer output file
};

#endif  // AP_MOTORSPX4
