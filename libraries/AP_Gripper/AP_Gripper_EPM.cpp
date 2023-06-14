/*
 * AP_Gripper_EPM.cpp
 *
 *  Created on: DEC 6, 2013
 *      Author: Andreas Jochum
 *              Pavel Kirienko <pavel.kirienko@zubax.com> - UAVCAN support
 *              Peter Barker - moved into AP_Gripper_EPM
 */

#include "AP_Gripper_EPM.h"

#if AP_GRIPPER_EPM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <SRV_Channel/SRV_Channel.h>

#ifdef UAVCAN_NODE_FILE
#include <fcntl.h>
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

AP_Gripper_EPM::AP_Gripper_EPM(struct AP_Gripper::Backend_Config &_config) :
    AP_Gripper_Backend(_config) { }

void AP_Gripper_EPM::init_gripper()
{
#ifdef UAVCAN_NODE_FILE
    _uavcan_fd = ::open(UAVCAN_NODE_FILE, O_CLOEXEC);
    // https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html
    ::printf("EPM: DroneCAN fd %d\n", _uavcan_fd);
#endif

    // initialise the EPM to the neutral position
    neutral();
}

// Refer to http://uavcan.org/Specification/7._List_of_standard_data_types/#uavcanequipmenthardpoint
struct UAVCANCommand {
    uint8_t hardpoint_id = 0;
    uint16_t command = 0;
};

// grab - move the epm pwm output to the grab position
void AP_Gripper_EPM::grab()
{
    // flag we are active and grabbing cargo
    config.state = AP_Gripper::STATE_GRABBING;

    // capture time
    _last_grab_or_release = AP_HAL::millis();

#ifdef UAVCAN_IOCS_HARDPOINT_SET
    if (should_use_uavcan()) {
        const UAVCANCommand cmd = make_uavcan_command(1);
        (void)ioctl(_uavcan_fd, UAVCAN_IOCS_HARDPOINT_SET, reinterpret_cast<unsigned long>(&cmd));
    }
    else
#endif
    {
        // move the servo output to the grab position
        SRV_Channels::set_output_pwm(SRV_Channel::k_gripper, config.grab_pwm);
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Gripper load grabbing");
    AP::logger().Write_Event(LogEvent::GRIPPER_GRAB);
}

// release - move epm pwm output to the release position
void AP_Gripper_EPM::release()
{
    // flag we are releasing cargo
    config.state = AP_Gripper::STATE_RELEASING;

    // capture time
    _last_grab_or_release = AP_HAL::millis();

#ifdef UAVCAN_IOCS_HARDPOINT_SET
    if (should_use_uavcan()) {
        const UAVCANCommand cmd = make_uavcan_command(0);
        (void)ioctl(_uavcan_fd, UAVCAN_IOCS_HARDPOINT_SET, reinterpret_cast<unsigned long>(&cmd));
    }
    else
#endif
    {
        // move the servo to the release position
        SRV_Channels::set_output_pwm(SRV_Channel::k_gripper, config.release_pwm);
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Gripper load releasing");
    AP::logger().Write_Event(LogEvent::GRIPPER_RELEASE);
}

// neutral - return the epm pwm output to the neutral position
void AP_Gripper_EPM::neutral()
{
    if (!should_use_uavcan()) {
        // move the servo to the off position
        SRV_Channels::set_output_pwm(SRV_Channel::k_gripper, config.neutral_pwm);
    }
}

// update - moves the pwm back to neutral after the timeout has passed
// should be called at at least 10hz
void AP_Gripper_EPM::update_gripper()
{
    // move EPM PWM output back to neutral after the last grab or release
    if (AP_HAL::millis() - _last_grab_or_release > EPM_RETURN_TO_NEUTRAL_MS) {
        if (config.state == AP_Gripper::STATE_GRABBING) {
            neutral();
            config.state = AP_Gripper::STATE_GRABBED;
        } else if (config.state == AP_Gripper::STATE_RELEASING) {
            neutral();
            config.state = AP_Gripper::STATE_RELEASED;
        }
    }

    // re-grab the cargo intermittently
    if (config.state == AP_Gripper::STATE_GRABBED &&
        (config.regrab_interval > 0) &&
        (AP_HAL::millis() - _last_grab_or_release > ((uint32_t)config.regrab_interval * 1000))) {
        grab();
    }
}

UAVCANCommand AP_Gripper_EPM::make_uavcan_command(uint16_t command) const
{
    UAVCANCommand cmd;
    cmd.hardpoint_id = config.uavcan_hardpoint_id;
    cmd.command = command;
    return cmd;
}


bool AP_Gripper_EPM::released() const
{
    // we assume instanteous releasing ATM:
    return (config.state == AP_Gripper::STATE_GRABBED ||
            config.state == AP_Gripper::STATE_GRABBING);
}

bool AP_Gripper_EPM::grabbed() const
{
    // we assume instanteous grabbing ATM:
    return (config.state == AP_Gripper::STATE_GRABBED ||
            config.state == AP_Gripper::STATE_GRABBING);
}

#endif  // AP_GRIPPER_EPM_ENABLED
