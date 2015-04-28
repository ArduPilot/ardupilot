// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file AP_MotorsHeli_Compound.h
/// @brief  Motor control class for compound helicopters

#ifndef __AP_MOTORS_HELI_COMPOUND_H__
#define __AP_MOTORS_HELI_COMPOUND_H__
#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <RC_Channel.h>

// TODO: Uncomment the AC_Booster implementation once that is merged to master.
// #include <AC_Booster.h>
#include "AP_MotorsHeli_Single.h"

// yaw channels for direct-drive fixed-pitch tail type
#define AP_MOTORS_HELI_COMPOUND_AUX_1       CH_7
#define AP_MOTORS_HELI_COMPOUND_AUX_2       CH_6

/// @class      AP_MotorsHeli_Compound
class AP_MotorsHeli_Compound : public AP_MotorsHeli_Single/*, public AC_BoosterBackend */ {
public:
    // constructor
    AP_MotorsHeli_Compound(RC_Channel&      rc_roll,
                           RC_Channel&      rc_pitch,
                           RC_Channel&      rc_throttle,
                           RC_Channel&      rc_yaw,
                           RC_Channel&      servo_aux_1,
                           RC_Channel&      servo_aux_2,
                           RC_Channel&      servo_rsc,
                           RC_Channel&      servo_1,
                           RC_Channel&      servo_2,
                           RC_Channel&      servo_3,
                           RC_Channel&      servo_yaw_1,
                           RC_Channel&      servo_yaw_2,
                           uint16_t         loop_rate,
                           uint16_t         speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli_Single(rc_roll, rc_pitch, rc_throttle, rc_yaw, servo_aux_1, servo_rsc, servo_1, servo_2, servo_3, servo_yaw_1, loop_rate, speed_hz),
        _servo_aux_1(servo_aux_1),
        _servo_aux_2(servo_aux_2),
        _servo_yaw_1(servo_yaw_1),
        _servo_yaw_2(servo_yaw_2)
    {
           AP_Param::setup_object_defaults(this, var_info);
    };

    // set_boost - engage the booster
    void set_boost(int16_t boost_in);

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_servos
    void init_servos();
    // yaw_control
    void output_yaw(int16_t yaw_out);

private:

    int16_t _boost_in;
    RC_Channel& _servo_aux_1;
    RC_Channel& _servo_aux_2;
    RC_Channel& _servo_yaw_1;
    RC_Channel& _servo_yaw_2;
};

#endif  // __AP_MOTORS_HELI_COMPOUND_H__