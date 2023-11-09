#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsPulsing_Coax.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsPulsing_Coax::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    // @Param: YAW_DIR
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("YAW_DIR", 1, AP_MotorsPulsing_Coax, _yaw_dir, 1),

    AP_GROUPEND
};
// init
void AP_MotorsPulsing_Coax::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 1 - Bottom Throttle
    // 2 - Top Throttle
    // 3 - Bottom Pitch
    // 4 - Top Pitch
    // 5 - Bottom Roll
    // 6 - Top Roll 
    // make sure 4 output channels are mapped
    for (uint8_t i = 0; i < 6; i++) {
        add_motor_num(CH_1 + i);
    }


    // setup actuator scaling
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;
    motor_enabled[AP_MOTORS_MOT_3] = false;
    motor_enabled[AP_MOTORS_MOT_4] = false;
    motor_enabled[AP_MOTORS_MOT_5] = false;
    motor_enabled[AP_MOTORS_MOT_6] = false;

    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_MOTORS_MOT_3), AP_MOTORS_COAX_SERVO_INPUT_RANGE);
    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_MOTORS_MOT_4), AP_MOTORS_COAX_SERVO_INPUT_RANGE);
    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_MOTORS_MOT_5), AP_MOTORS_COAX_SERVO_INPUT_RANGE);
    SRV_Channels::set_angle(SRV_Channels::get_motor_function(AP_MOTORS_MOT_6), AP_MOTORS_COAX_SERVO_INPUT_RANGE);


    _mav_type = MAV_TYPE_QUADROTOR;
    rpm = AP_RPM::get_singleton();
    // record successful initialisation if what we setup was the desired frame_class
    // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Frame");
    set_initialised_ok(frame_class == MOTOR_FRAME_PULSING_COAX);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsPulsing_Coax::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    set_initialised_ok(frame_class == MOTOR_FRAME_PULSING_COAX);
}

// set update rate to motors - a value in hertz
void AP_MotorsPulsing_Coax::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    uint32_t mask =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 ;
    rc_set_freq(mask, _speed_hz);
}

void AP_MotorsPulsing_Coax::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            // sends minimum values out to the motors
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(0)); // bottom rotor
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(0)); // top rotor
            rc_write_angle(AP_MOTORS_MOT_3, 0); // pitch
            rc_write_angle(AP_MOTORS_MOT_4, 0); // pitch
            rc_write_angle(AP_MOTORS_MOT_5, 0); // roll
            rc_write_angle(AP_MOTORS_MOT_6, 0); // roll
            break;
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            rc_write_angle(AP_MOTORS_MOT_3, 0);
            rc_write_angle(AP_MOTORS_MOT_4, 0);
            rc_write_angle(AP_MOTORS_MOT_5, 0);
            rc_write_angle(AP_MOTORS_MOT_6, 0);
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], actuator_spin_up_to_ground_idle()); // spin up motors
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_2], actuator_spin_up_to_ground_idle());
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[AP_MOTORS_MOT_2]));
            
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            rc_write_angle(AP_MOTORS_MOT_3, -_pitch_action * AP_MOTORS_COAX_SERVO_INPUT_RANGE); // pitch
            rc_write_angle(AP_MOTORS_MOT_4, _pitch_action * AP_MOTORS_COAX_SERVO_INPUT_RANGE); // pitch
            rc_write_angle(AP_MOTORS_MOT_5, _roll_action * AP_MOTORS_COAX_SERVO_INPUT_RANGE); // roll
            rc_write_angle(AP_MOTORS_MOT_6, _roll_action * AP_MOTORS_COAX_SERVO_INPUT_RANGE); // roll
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_1], thrust_to_actuator(_bottom_thrust));
            set_actuator_with_slew(_actuator[AP_MOTORS_MOT_2], thrust_to_actuator(_top_thrust));
            rc_write(AP_MOTORS_MOT_1, output_to_pwm(_actuator[AP_MOTORS_MOT_1]));
            rc_write(AP_MOTORS_MOT_2, output_to_pwm(_actuator[AP_MOTORS_MOT_2]));
            // if ( AP_HAL::millis() > _last_update + 500)
            // {
            //     _last_update =  AP_HAL::millis();
            //     GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "t:%.2f\tr:%.2f\tp:%.2f\ty:%.2f", _throttle_out, _roll_action, _pitch_action, yaw_thrust);

            // }
            break;
    }
    
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsPulsing_Coax::get_motor_mask()
{
    uint32_t motor_mask =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2;
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

void AP_MotorsPulsing_Coax::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_avg_max;           // throttle thrust average maximum value, 0.0 - 1.0
    float   rp_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits


    // apply voltage and air pressure compensation
    // const float compensation_gain = get_compensation_gain();
    roll_thrust = _roll_in + _roll_in_ff;
    pitch_thrust = _pitch_in + _pitch_in_ff;
    yaw_thrust = _yaw_in + _yaw_in_ff;
    throttle_thrust = get_throttle();
    throttle_avg_max = _throttle_avg_max;


    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    
    _throttle_out = throttle_avg_max;

    _roll_action = roll_thrust * rp_scale;
    _pitch_action = pitch_thrust * rp_scale;

    _bottom_thrust = _throttle_out - _yaw_dir * yaw_thrust;
    _top_thrust = _throttle_out + _yaw_dir * yaw_thrust;
    
}


// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsPulsing_Coax::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // flap servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // flap servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // flap servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // flap servo 4
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        default:
            // do nothing
            break;
    }
}