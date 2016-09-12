// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_EPM.cpp
 *
 *  Created on: DEC 6, 2013
 *      Author: Andreas Jochum
 *              Pavel Kirienko <pavel.kirienko@zubax.com> - UAVCAN support
 */

#include "AP_EPM.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_EPM::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: EPM Enable/Disable
    // @Description: EPM enable/disable
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_EPM, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: GRAB
    // @DisplayName: EPM Grab PWM
    // @Description: PWM value sent to EPM to initiate grabbing the cargo
    // @User: Advanced
    // @Range: 1000 2000
    AP_GROUPINFO("GRAB",    1, AP_EPM, _grab_pwm, EPM_GRAB_PWM_DEFAULT),

    // @Param: RELEASE
    // @DisplayName: EPM Release PWM
    // @Description: PWM value sent to EPM to release the cargo
    // @User: Advanced
    // @Range: 1000 2000
    AP_GROUPINFO("RELEASE", 2, AP_EPM, _release_pwm, EPM_RELEASE_PWM_DEFAULT),

    // @Param: NEUTRAL
    // @DisplayName: EPM Neutral PWM
    // @Description: PWM value sent to EPM when not grabbing or releasing
    // @User: Advanced
    // @Range: 1000 2000
    AP_GROUPINFO("NEUTRAL", 3, AP_EPM, _neutral_pwm, EPM_NEUTRAL_PWM_DEFAULT),

    // @Param: REGRAB
    // @DisplayName: EPM Regrab interval
    // @Description: Time in seconds that gripper will regrab the cargo to ensure grip has not weakened; 0 to disable
    // @User: Advanced
    // @Values: 0 255
    AP_GROUPINFO("REGRAB",  4, AP_EPM, _regrab_interval, EPM_REGRAB_DEFAULT),

    // @Param: REGRAB
    // @DisplayName: EPM UAVCAN Hardpoint ID
    // @Description: Refer to https://docs.zubax.com/opengrab_epm_v3#UAVCAN_interface
    // @User: Standard
    // @Range: 0 255
    AP_GROUPINFO("UAVCAN_ID", 5, AP_EPM, _uavcan_hardpoint_id, 0),

    AP_GROUPEND
};

AP_EPM::AP_EPM(void) :
        _last_grab_or_release(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags.grab = false;
    _flags.active = false;
}

void AP_EPM::init() 
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

#ifdef UAVCAN_NODE_FILE
    _uavcan_fd = open(UAVCAN_NODE_FILE, 0);
    // http://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html
    ::printf("EPM: UAVCAN fd %d\n", _uavcan_fd);
#endif

    // initialise the EPM to the neutral position
    neutral();
}

// grab - move the epm pwm output to the grab position
void AP_EPM::grab()
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // flag we are active and grabbing cargo
    _flags.grab = true;
    _flags.active = true;

    // capture time
    _last_grab_or_release = AP_HAL::millis();

#ifdef UAVCAN_IOCS_HARDPOINT_SET
    if (should_use_uavcan()) {
        ::printf("EPM: UAVCAN GRAB\n");
        const UAVCANCommand cmd = make_uavcan_command(1);
        (void)ioctl(_uavcan_fd, UAVCAN_IOCS_HARDPOINT_SET, reinterpret_cast<unsigned long>(&cmd));
    }
    else
#endif
    {
        // move the servo to the release position
        RC_Channel_aux::set_radio(RC_Channel_aux::k_epm, _grab_pwm);
    }
}

// release - move the epm pwm output to the release position
void AP_EPM::release()
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // flag we are active and releasing cargo
    _flags.grab = false;
    _flags.active = true;

    // capture time
    _last_grab_or_release = AP_HAL::millis();

#ifdef UAVCAN_IOCS_HARDPOINT_SET
    if (should_use_uavcan()) {
        ::printf("EPM: UAVCAN RELEASE\n");
        const UAVCANCommand cmd = make_uavcan_command(0);
        (void)ioctl(_uavcan_fd, UAVCAN_IOCS_HARDPOINT_SET, reinterpret_cast<unsigned long>(&cmd));
    }
    else
#endif
    {
        // move the servo to the release position
        RC_Channel_aux::set_radio(RC_Channel_aux::k_epm, _release_pwm);
    }
}

// neutral - return the epm pwm output to the neutral position
void AP_EPM::neutral() 
{
    // return immediately if not enabled
    if (!_enabled) {
        return;
    }

    // flag we are inactive (i.e. not grabbing or releasing cargo)
    _flags.active = false;

    if (!should_use_uavcan()) {
        // move the servo to the off position
        RC_Channel_aux::set_radio(RC_Channel_aux::k_epm, _neutral_pwm);
    }
}

// update - moves the pwm back to neutral after the timeout has passed
// should be called at at least 10hz
void AP_EPM::update()
{
    // move EPM PWM output back to neutral 2 seconds after the last grab or release
    if (_flags.active && (AP_HAL::millis() - _last_grab_or_release > EPM_RETURN_TO_NEUTRAL_MS)) {
        neutral();
    }

    // re-grab the cargo intermittently
    if (_flags.grab && (_regrab_interval > 0) &&
        (AP_HAL::millis() - _last_grab_or_release > ((uint32_t)_regrab_interval * 1000))) {
        grab();
    }
}
