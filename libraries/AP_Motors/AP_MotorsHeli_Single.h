/// @file	AP_MotorsHeli_Single.h
/// @brief	Motor control class for traditional heli
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"
#include "AP_MotorsHeli_Swash.h"
#include "AP_Motors_Thrust_Linearization.h"

// rsc and extgyro function output channels.
#define AP_MOTORS_HELI_SINGLE_EXTGYRO                          CH_7
#define AP_MOTORS_HELI_SINGLE_TAILRSC                          CH_7

// direct-drive variable pitch defaults
#define AP_MOTORS_HELI_SINGLE_DDVP_SPEED_DEFAULT               50

// default external gyro gain
#define AP_MOTORS_HELI_SINGLE_EXT_GYRO_GAIN                    350

// COLYAW parameter min and max values
#define AP_MOTORS_HELI_SINGLE_COLYAW_RANGE                     5.0f

// maximum number of swashplate servos
#define AP_MOTORS_HELI_SINGLE_NUM_SWASHPLATE_SERVOS            3

/// @class      AP_MotorsHeli_Single
class AP_MotorsHeli_Single : public AP_MotorsHeli {
public:
    // constructor
    AP_MotorsHeli_Single(uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(speed_hz),
        _tail_rotor(SRV_Channel::k_heli_tail_rsc, AP_MOTORS_HELI_SINGLE_TAILRSC, 1U),
        _swashplate(AP_MOTORS_MOT_1, AP_MOTORS_MOT_2, AP_MOTORS_MOT_3, AP_MOTORS_MOT_5, 1U)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // set update rate to motors - a value in hertz
    void set_update_rate(uint16_t speed_hz) override;

    // output_to_motors - sends values out to the motors
    void output_to_motors() override;

    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1
    void set_desired_rotor_speed(float desired_speed) override;

    // calculate_scalars - recalculates various scalars used
    void calculate_scalars() override;

    // calculate_armed_scalars - recalculates scalars that can change while armed
    void calculate_armed_scalars() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t get_motor_mask() override;

    // ext_gyro_gain - set external gyro gain in range 0 ~ 1000
    void ext_gyro_gain(float gain)  override { if (gain >= 0 && gain <= 1000) { _ext_gyro_gain_std.set(gain); }}

    // has_flybar - returns true if we have a mechical flybar
    bool has_flybar() const  override { return _flybar_mode; }

    // supports_yaw_passthrough - returns true if we support yaw passthrough
    bool supports_yaw_passthrough() const override { return get_tail_type() == TAIL_TYPE::SERVO_EXTGYRO; }

    void set_acro_tail(bool set) override { _acro_tail = set; }

    // Run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override;

    // Helper function for param conversions to be done in motors class
    void heli_motors_param_conversions(void) override;

    // Thrust Linearization handling
    Thrust_Linearization thr_lin {*this};

#if HAL_LOGGING_ENABLED
    // Blade angle logging - called at 10 Hz
    void Log_Write(void) override;
#endif

    // var_info
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_outputs - initialise Servo/PWM ranges and endpoints
    void init_outputs() override;

    // update_motor_controls - sends commands to motor controllers
    void update_motor_control(AP_MotorsHeli_RSC::RotorControlState state) override;

    // heli_move_actuators - moves swash plate and tail rotor
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out) override;

    // move_yaw - moves the yaw servo
    void move_yaw(float yaw_out);

    // Get yaw offset required to cancel out steady state main rotor torque
    float get_yaw_offset(float collective);

    // handle output limit flags and send throttle to servos lib
    void output_to_ddfp_tail(float throttle);

    // servo_test - move servos through full range of movement
    void servo_test() override;

    // Tail types
    enum class TAIL_TYPE {
        SERVO = 0,
        SERVO_EXTGYRO = 1,
        DIRECTDRIVE_VARPITCH = 2,
        DIRECTDRIVE_FIXEDPITCH_CW = 3,
        DIRECTDRIVE_FIXEDPITCH_CCW = 4,
        DIRECTDRIVE_VARPIT_EXT_GOV = 5
    };

    TAIL_TYPE get_tail_type() const { return TAIL_TYPE(_tail_type.get()); }

    // Helper to return true for direct drive fixed pitch tail, either CW or CCW
    bool have_DDFP_tail() const;

    // Helper to return true if the tail RSC should be used
    bool use_tail_RSC() const;

    // external objects we depend upon
    AP_MotorsHeli_RSC   _tail_rotor;            // tail rotor
    AP_MotorsHeli_Swash _swashplate;            // swashplate

    // internal variables
    float _oscillate_angle = 0.0f;              // cyclic oscillation angle, used by servo_test function
    float _servo_test_cycle_time = 0.0f;        // cycle time tracker, used by servo_test function
    float _collective_test = 0.0f;              // over-ride for collective output, used by servo_test function
    float _roll_test = 0.0f;                    // over-ride for roll output, used by servo_test function
    float _pitch_test = 0.0f;                   // over-ride for pitch output, used by servo_test function
    float _yaw_test = 0.0f;                     // over-ride for yaw output, used by servo_test function
    float _servo4_out = 0.0f;                   // output value sent to motor

    // parameters
    AP_Int16        _tail_type;                 // Tail type used: Servo, Servo with external gyro, direct drive variable pitch or direct drive fixed pitch
    AP_Int16        _ext_gyro_gain_std;         // PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro
    AP_Int16        _ext_gyro_gain_acro;        // PWM sent to external gyro on ch7 when tail type is Servo w/ ExtGyro in ACRO
    AP_Int8         _flybar_mode;               // Flybar present or not.  Affects attitude controller used during ACRO flight mode
    AP_Int16        _direct_drive_tailspeed;    // Direct Drive VarPitch Tail ESC speed (0 ~ 1000)
    AP_Float        _collective_yaw_scale;      // Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    AP_Float        _yaw_trim;                  // Fixed offset applied to yaw output to reduce yaw I.

    bool            _acro_tail = false;
};
