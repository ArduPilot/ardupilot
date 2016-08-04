// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_EPM.h
 *
 *  Created on: DEC 06, 2013
 *      Author: Andreas Jochum
 *              Pavel Kirienko <pavel.kirienko@zubax.com> - UAVCAN support
 *
 *      Set-up Wiki: http://copter.ardupilot.org/wiki/common-electro-permanent-magnet-gripper/
 *      EPM docs:    https://docs.zubax.com/opengrab_epm_v3
 */

/// @file	AP_EPM.h
/// @brief	AP_EPM control class
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <RC_Channel/RC_Channel.h>

// EPM PWM definitions
#define EPM_GRAB_PWM_DEFAULT        1900
#define EPM_RELEASE_PWM_DEFAULT     1100
#define EPM_NEUTRAL_PWM_DEFAULT     1500
#define EPM_RETURN_TO_NEUTRAL_MS    500         // EPM PWM returns to neutral position this many milliseconds after grab or release
#define EPM_REGRAB_DEFAULT          0           // default re-grab interval (in seconds) to ensure cargo is securely held

/// @class	AP_EPM
/// @brief	Class to manage the EPM_CargoGripper 
class AP_EPM {
public:
    AP_EPM();

    // initialise the EPM
    void        init();

    // enabled - true if the EPM is enabled
    bool        enabled() { return _enabled; }

    // grab - move the EPM pwm output to the grab position
    void        grab();

    // release - move the EPM pwm output to the release position
    void        release();

    // update - moves the pwm back to neutral after the timeout has passed
    // should be called at at least 10hz
    void        update();

    static const struct AP_Param::GroupInfo        var_info[];

private:

    // neutral - return the EPM pwm output to the neutral position
    void        neutral();

    bool should_use_uavcan() const { return _uavcan_fd >= 0; }

    // Refer to http://uavcan.org/Specification/7._List_of_standard_data_types/#uavcanequipmenthardpoint
    struct UAVCANCommand {
        uint8_t hardpoint_id = 0;
        uint16_t command = 0;
    };

    UAVCANCommand make_uavcan_command(uint16_t command) const
    {
        UAVCANCommand cmd;
        cmd.hardpoint_id = _uavcan_hardpoint_id;
        cmd.command = command;
        return cmd;
    }

    // EPM flags
    struct EPM_Flags {
        uint8_t grab    : 1;    // true if we think we have grabbed onto cargo, false if we think we've released it
        uint8_t active  : 1;    // true if we are actively sending grab or release PWM to EPM to activate grabbing or releasing, false if we are sending neutral pwm
    } _flags;

    // UAVCAN driver fd
    int _uavcan_fd = -1;

    // parameters
    AP_Int8     _enabled;               // EPM enable/disable
    AP_Int16    _grab_pwm;              // PWM value sent to EPM to initiate grabbing the cargo
    AP_Int16    _release_pwm;           // PWM value sent to EPM to release the cargo
    AP_Int16    _neutral_pwm;           // PWM value sent to EPM when not grabbing or releasing
    AP_Int8     _regrab_interval;       // Time in seconds that gripper will regrab the cargo to ensure grip has not weakend
    AP_Int16    _uavcan_hardpoint_id;

    // internal variables
    uint32_t    _last_grab_or_release;
};
