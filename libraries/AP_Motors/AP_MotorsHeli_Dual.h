/// @file   AP_MotorsHeli_Dual.h
/// @brief  Motor control class for dual heli (tandem or transverse)
/// @author Fredrik Hedberg

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>

#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"
#include "AP_MotorsHeli_Swash.h"

// tandem modes
#define AP_MOTORS_HELI_DUAL_MODE_TANDEM                0 // tandem mode (rotors front and aft)
#define AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE            1 // transverse mode (rotors side by side)
#define AP_MOTORS_HELI_DUAL_MODE_INTERMESHING          2 // intermeshing mode (rotors side by side)

// tandem modes
#define AP_MOTORS_HELI_DUAL_SWASH_AXIS_PITCH           0 // swashplate pitch tilt axis
#define AP_MOTORS_HELI_DUAL_SWASH_AXIS_ROLL            1 // swashplate roll tilt axis
#define AP_MOTORS_HELI_DUAL_SWASH_AXIS_COLL            2 // swashplate collective axis

// default differential-collective-pitch scaler
#define AP_MOTORS_HELI_DUAL_DCP_SCALER             0.25f

// maximum number of swashplate servos
#define AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS    6

// default collective min, max and midpoints for the rear swashplate
#define AP_MOTORS_HELI_DUAL_COLLECTIVE2_MIN 1250
#define AP_MOTORS_HELI_DUAL_COLLECTIVE2_MAX 1750

/// @class AP_MotorsHeli_Dual
class AP_MotorsHeli_Dual : public AP_MotorsHeli {
public:
    // constructor
    AP_MotorsHeli_Dual(uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };


    // set_update_rate - set update rate to motors
    void set_update_rate( uint16_t speed_hz ) override;

    // output_to_motors - sends values out to the motors
    void output_to_motors() override;

    // calculate_scalars - recalculates various scalars used
    void calculate_scalars() override;

    // calculate_armed_scalars - recalculates scalars that can change while armed
    void calculate_armed_scalars() override;

    // has_flybar - returns true if we have a mechical flybar
    bool has_flybar() const  override { return AP_MOTORS_HELI_NOFLYBAR; }

    // supports_yaw_passthrought - returns true if we support yaw passthrough
    bool supports_yaw_passthrough() const  override { return false; }

    // servo_test - move servos through full range of movement
    void servo_test() override;

    // Run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_outputs
    void init_outputs () override;

    // update_motor_controls - sends commands to motor controllers
    void update_motor_control(RotorControlState state) override;

    // get_swashplate - calculate movement of each swashplate based on configuration
    float get_swashplate(int8_t swash_num, int8_t swash_axis, float pitch_input, float roll_input, float yaw_input, float coll_input);

    // move_actuators - moves swash plate to attitude of parameters passed in
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)  override;

    const char* _get_frame_string() const override { return "HELI_DUAL"; }

    //  objects we depend upon
    AP_MotorsHeli_Swash _swashplate1 { CH_1, CH_2, CH_3, CH_7 }; // swashplate1
    AP_MotorsHeli_Swash _swashplate2 { CH_4, CH_5, CH_6, CH_8 }; // swashplate2

    // internal variables
    float _oscillate_angle = 0.0f;                  // cyclic oscillation angle, used by servo_test function
    float _servo_test_cycle_time = 0.0f;            // cycle time tracker, used by servo_test function
    float _collective_test = 0.0f;                  // over-ride for collective output, used by servo_test function
    float _roll_test = 0.0f;                        // over-ride for roll output, used by servo_test function
    float _pitch_test = 0.0f;                       // over-ride for pitch output, used by servo_test function
    float _servo_out[8];                            // output value sent to motor

    // parameters
    AP_Int16        _collective2_min;               // Lowest possible servo position for the rear swashplate
    AP_Int16        _collective2_max;               // Highest possible servo position for the rear swashplate
    AP_Int8         _dual_mode;                     // which dual mode the heli is
    AP_Float        _dcp_scaler;                    // scaling factor applied to the differential-collective-pitch
    AP_Float        _dcp_yaw_effect;                // feed-forward compensation to automatically add yaw input when differential collective pitch is applied.
    AP_Float        _yaw_scaler;                    // scaling factor applied to the yaw mixing
    AP_Float        _dcp_trim;                      // used to easily trim dcp axis
    AP_Float        _yaw_rev_expo;                  // yaw reverser smoothing exponent, for intermeshing mode only.

    // internal variables
    float _collective2_zero_thrst_pct;
};
