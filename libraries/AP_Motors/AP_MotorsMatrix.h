/// @file	AP_MotorsMatrix.h
/// @brief	Motor control class for Matrixcopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMulticopter.h"

#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

/// @class      AP_MotorsMatrix
class AP_MotorsMatrix : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsMatrix(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(loop_rate, speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix must be singleton");
        }
        _singleton = this;
    };

    // get singleton instance
    static AP_MotorsMatrix *get_singleton() {
        return _singleton;
    }

    // init
    virtual void        init(motor_frame_class frame_class, motor_frame_type frame_type) override;

#if AP_SCRIPTING_ENABLED
    // Init to be called from scripting
    virtual bool        init(uint8_t expected_num_motors);

    // Set throttle factor from scripting
    bool                set_throttle_factor(int8_t motor_num, float throttle_factor);

#endif // AP_SCRIPTING_ENABLED

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void                set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    // you must have setup_motors before calling this
    void                set_update_rate(uint16_t speed_hz) override;

    // output_test_num - spin a motor connected to the specified output channel
    //  (should only be performed during testing)
    //  If a motor output channel is remapped, the mapped channel is used.
    //  Returns true if motor output is set, false otherwise
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    bool                output_test_num(uint8_t motor, int16_t pwm);

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t            get_motor_mask() override;

    // return number of motor that has failed.  Should only be called if get_thrust_boost() returns true
    uint8_t             get_lost_motor() const override { return _motor_lost_index; }

    // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // using copter motors for forward flight
    float               get_roll_factor(uint8_t i) override { return _roll_factor[i]; }
    // return the pitch factor of any motor
    float               get_pitch_factor(uint8_t i) override { return _pitch_factor[i]; }

    // disable the use of motor torque to control yaw. Used when an external mechanism such
    // as vectoring is used for yaw control
    void                disable_yaw_torque(void) override;

    // add_motor using raw roll, pitch, throttle and yaw factors
    void                add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order, float throttle_factor = 1.0f);

    // same structure, but with floats.
    struct MotorDef {
        float angle_degrees;
        float yaw_factor;
        uint8_t testing_order;
    };

    // method to add many motors specified in a structure:
    void add_motors(const struct MotorDef *motors, uint8_t num_motors);

    // structure used for initialising motors that add have separate
    // roll/pitch/yaw factors.  Note that this does *not* include
    // the final parameter for the add_motor_raw call - throttle
    // factor as that is only used in the scripting binding, not in
    // the static motors at the moment.
    struct MotorDefRaw {
        float roll_fac;
        float pitch_fac;
        float yaw_fac;
        uint8_t testing_order;
    };
    void add_motors_raw(const struct MotorDefRaw *motors, uint8_t num_motors);

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

    // check for failed motor
    void                check_for_failed_motor(float throttle_thrust_best);

    // add_motor using just position and yaw_factor (or prop direction)
    void                add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

    // add_motor using separate roll and pitch factors (for asymmetrical frames) and prop direction
    void                add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order);

    // remove_motor
    void                remove_motor(int8_t motor_num);

    // configures the motors for the defined frame_class and frame_type
    virtual void        setup_motors(motor_frame_class frame_class, motor_frame_type frame_type);

    // normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
    void                normalise_rpy_factors();

    // call vehicle supplied thrust compensation if set
    void                thrust_compensation(void) override;

    const char*         _get_frame_string() const override { return _frame_class_string; }
    const char*         get_type_string() const override { return _frame_type_string; }

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to throttle 0~1
    float               _thrust_rpyt_out[AP_MOTORS_MAX_NUM_MOTORS]; // combined roll, pitch, yaw and throttle outputs to motors in 0~1 range
    uint8_t             _test_order[AP_MOTORS_MAX_NUM_MOTORS];  // order of the motors in the test sequence

    // motor failure handling
    float               _thrust_rpyt_out_filt[AP_MOTORS_MAX_NUM_MOTORS];    // filtered thrust outputs with 1 second time constant
    uint8_t             _motor_lost_index;  // index number of the lost motor

    motor_frame_class   _active_frame_class; // active frame class (i.e. quad, hexa, octa, etc)
    motor_frame_type    _active_frame_type;  // active frame type (i.e. plus, x, v, etc)

    const char*         _frame_class_string = ""; // string representation of frame class
    const char*         _frame_type_string = "";  //  string representation of frame type
private:
    static AP_MotorsMatrix *_singleton;
};
