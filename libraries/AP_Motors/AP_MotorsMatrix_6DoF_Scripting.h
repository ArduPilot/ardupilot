#pragma once
#ifdef ENABLE_SCRIPTING

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_MotorsMatrix.h"

class AP_MotorsMatrix_6DoF_Scripting : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsMatrix_6DoF_Scripting(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix 6DoF must be singleton");
        }
        _singleton = this;
    };

    // get singleton instance
    static AP_MotorsMatrix_6DoF_Scripting *get_singleton() {
        return _singleton;
    }

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    // sets the roll and pitch offset, this rotates the thrust vector in body frame
    // these are typically set such that the throttle thrust vector is earth frame up
    void set_roll_pitch(float roll_deg, float pitch_deg) override;

    // add_motor using raw roll, pitch, throttle and yaw factors, to be called from scripting
    void add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order);

    // if the expected number of motors have been setup then set as initalized
    bool init(uint8_t expected_num_motors) override;

    const char* get_frame_string() const override { return "6DoF scripting"; }

protected:
    // output - sends commands to the motors
    void output_armed_stabilizing() override;

    // nothing to do for setup, scripting will mark as initalized when done
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override {};

    float _forward_factor[AP_MOTORS_MAX_NUM_MOTORS];      // each motors contribution to forward thrust
    float _right_factor[AP_MOTORS_MAX_NUM_MOTORS];        // each motors contribution to right thrust

    // true if motor is revesible, it can go from -Spin max to +Spin max, if false motor is can go from Spin min to Spin max
    bool _reversible[AP_MOTORS_MAX_NUM_MOTORS];

    // store last values to allow deadzone
    float _last_thrust_out[AP_MOTORS_MAX_NUM_MOTORS];

    // Current offset angles, radians
    float _roll_offset;
    float _pitch_offset;

private:
    static AP_MotorsMatrix_6DoF_Scripting *_singleton;

};

#endif // ENABLE_SCRIPTING
