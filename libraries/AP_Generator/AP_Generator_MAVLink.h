// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Generator_Backend.h"

#if HAL_GENERATOR_MAVLINK_ENABLED

#include <AP_Common/AP_Common.h>
#include <SRV_Channel/SRV_Channel.h>
#include <stdint.h>
#include <stdio.h>


/*
 *  Example setup:
 *  param set SERIAL2_PROTOCOL 30  # Generator protocol
 *  param set SERIAL2_BAUD 9600
 */

class AP_Generator_MAVLink : public AP_Generator_Backend
{

public:
    // constructor
    using AP_Generator_Backend::AP_Generator_Backend;

    // init should be called at vehicle startup to get the generator library ready
    void init(void) override {}
    // update should be called regularly to update the generator state
    void update(void) override;

    // methods to control the generator state:
    bool stop(void) override { return false; }
    bool idle(void) override { return false; }
    bool run(void) override { return false; }

    void handle_mavlink_msg(const GCS_MAVLINK &channel, const mavlink_message_t &msg) override;

    // method to send a GENERATOR_STATUS mavlink message
    void send_generator_status(const GCS_MAVLINK &channel) override;

    // prearm checks to ensure generator is good for arming.  Note
    // that if the generator has never sent us a message then these
    // automatically pass!
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const override { return  true; }

    // Update front end with voltage, current, and rpm values
    void update_frontend_readings(void) { return; }

    // healthy returns true if the generator is not present, or it is
    // present, providing telemetry and not indicating an errors.
    bool healthy() const override;

private:

    // we just store the most recent packet for the time being:
    mavlink_generator_status_t packet;

    uint32_t last_received_ms;
};
#endif  // HAL_GENERATOR_MAVLINK_ENABLED
