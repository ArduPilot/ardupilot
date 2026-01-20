/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#pragma once

#include "AP_Generator_config.h"

#if AP_GENERATOR_CORTEX_ENABLED

#include "AP_Generator_Backend.h"
#include <AP_Common/AP_Common.h>
#include <AP_PiccoloCAN/AP_PiccoloCAN.h>
#include <AP_PiccoloCAN/AP_PiccoloCAN_Device.h>
#include <AP_PiccoloCAN/piccolo_protocol/CortexPackets.h>


/**
 * Class defining the Cortex generator from Currawong Engineering
 */
class AP_Generator_Cortex : public AP_Generator_Backend
{
public:
    using AP_Generator_Backend::AP_Generator_Backend;

    // init should be called at vehicle startup to get the generator library ready
    __INITFUNC__ void init(void) override;
    // update should be called regularly to update the generator state
    virtual void update(void) override;

    // methods to control the generator state:
    virtual bool stop(void) override;
    virtual bool idle(void) override;
    virtual bool run(void) override;

    // method to send a GENERATOR_STATUS mavlink message
    virtual void send_generator_status(const GCS_MAVLINK &channel) override;

    // prearm checks to ensure generator is good for arming.
    virtual bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const override;

    // Update front end with voltage, current, and rpm values
    void update_frontend_readings(void);

    bool is_connected(void) const;

    bool is_inhibited(void) const {
        return telemetry.status.status.inhibited;
    }

    bool is_ready(void) const {
        return is_connected() && telemetry.status.status.readyToRun;
    }

    // healthy returns true if the generator is not present, or it is
    // present, providing telemetry and not indicating an errors.
    bool healthy() const override {
        return is_connected();
    }

    static AP_Generator_Cortex* get_instance(void)
    {
        return _singleton;
    }

    bool handle_message(AP_HAL::CANFrame &frame, AP_PiccoloCAN &can_iface);

private:
    static AP_Generator_Cortex* _singleton;

    // Internal data structures for received telemetry
    struct CortexTelemetry_t {
        Cortex_TelemetryStatus_t status;
        Cortex_TelemetryGenerator_t generator;
        Cortex_TelemetryBattery_t battery;
        Cortex_TelemetryOutputRail_t rails;
        Cortex_TelemetryController_t controller;
    } telemetry;

    float batteryCurrent(void) const { return telemetry.battery.current; }
    float batteryVoltage(void) const { return telemetry.battery.voltage; }

    float generatorCurrent(void) const { return telemetry.generator.current; }
    float generatorVoltage(void) const { return telemetry.generator.voltage; }
    float generatorPower(void) const { return generatorCurrent() * generatorVoltage(); }

    float loadCurrent(void) const { return batteryCurrent() - generatorCurrent(); }

    int16_t rectifierTemperature(void) const;

    // Pointer to the CAN interface used to communicate with the generator
    AP_PiccoloCAN* _can_iface;

    // CAN device type
    uint16_t _can_device_type;
    uint8_t _can_device_id = 0xFF;

    // Last telemetry reading from the generator
    uint32_t last_reading_ms;

    // Connection state, used to detect changes
    bool connected;

    bool send_message(AP_HAL::CANFrame &frame);
};

#endif // AP_GENERATOR_CORTEX_ENABLED
