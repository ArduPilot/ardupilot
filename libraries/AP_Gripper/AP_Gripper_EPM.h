/*
 * AP_EPM.h
 *
 *  Created on: DEC 06, 2013
 *      Author: Andreas Jochum
 *              Pavel Kirienko <pavel.kirienko@zubax.com> - UAVCAN support
 *
 *      Set-up Wiki: https://copter.ardupilot.org/wiki/common-electro-permanent-magnet-gripper/
 *      EPM docs:    https://docs.zubax.com/opengrab_epm_v3
 */

/// @file	AP_EPM.h
/// @brief	AP_EPM control class
#pragma once

#include "AP_Gripper.h"
#include "AP_Gripper_Backend.h"

#include <SRV_Channel/SRV_Channel.h>

#define EPM_RETURN_TO_NEUTRAL_MS    500         // EPM PWM returns to neutral position this many milliseconds after grab or release

/// @class	AP_Gripper_EPM
/// @brief	Class to manage the EPM_CargoGripper 
class AP_Gripper_EPM : public AP_Gripper_Backend {
public:
    AP_Gripper_EPM(struct AP_Gripper::Backend_Config &_config);

    // initialise the EPM
    void        init_gripper() override;

    // grab - move the EPM pwm output to the grab position
    void        grab() override;

    // release - move the EPM pwm output to the release position
    void        release() override;

    // grabbed - returns true if gripper in grabbed state
    bool grabbed() const override;

    // released - returns true if gripper in released state
    bool released() const override;

    // update - moves the pwm back to neutral after the timeout has passed
    // should be called at at least 10hz
    void        update_gripper() override;

private:

    // neutral - return the EPM pwm output to the neutral position
    void        neutral();

    bool should_use_uavcan() const { return _uavcan_fd >= 0; }

    struct UAVCANCommand make_uavcan_command(uint16_t command) const;

    // UAVCAN driver fd
    int _uavcan_fd = -1;

    // internal variables
    uint32_t    _last_grab_or_release;
};
