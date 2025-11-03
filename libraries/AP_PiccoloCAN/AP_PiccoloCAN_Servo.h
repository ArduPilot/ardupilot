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

#include <AP_CANManager/AP_CANManager.h>

#include "AP_PiccoloCAN_config.h"
#include "AP_PiccoloCAN_Device.h"
#include "piccolo_protocol/ServoPackets.h"

#if HAL_PICCOLO_CAN_ENABLE

#define PICCOLO_CAN_MAX_NUM_SERVO 16
#define PICCOLO_CAN_MAX_GROUP_SERVO (PICCOLO_CAN_MAX_NUM_SERVO / 4)


/*
 * Class representing an individual PiccoloCAN servo
 */
class AP_PiccoloCAN_Servo : public AP_PiccoloCAN_Device
{
public:

    virtual bool handle_can_frame(AP_HAL::CANFrame &frame) override;

    virtual bool is_enabled(void) const override { return status.statusA.status.enabled; }

    // Helper functions for accessing servo status data
    float position() const { return (float) status.statusA.position; }
    float commandedPosition() const { return (float) status.statusA.command; }
    float current() const { return (float) status.statusB.current * 0.01f; }
    float voltage() const { return (float) status.statusB.voltage * 0.01f; }
    float speed() const { return (float) status.statusB.speed; }
    uint8_t dutyCycle() const { return abs(status.statusB.dutyCycle); }
    float temperature() const { return (float) status.statusB.temperature; }

    int16_t command = 0;
    bool newCommand = false;
    bool newTelemetry = false;

    // Status / telemetry data
    struct Status_t {
        Servo_StatusA_t statusA;
        Servo_StatusB_t statusB;
    } status;

    // Settings information
    struct Settings_t {
        Servo_Firmware_t firmware;
        Servo_Address_t address;
        Servo_SettingsInfo_t settings;
    } settings;
};

#endif // HAL_PICCOLO_CAN_ENABLE
