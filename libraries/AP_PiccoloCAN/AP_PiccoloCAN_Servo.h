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
 * Author: Oliver Walters
 */

#pragma once

#include <AP_Logger/AP_Logger.h>

#include "AP_PiccoloCAN_Device.h"

#if HAL_PICCOLO_CAN_ENABLE

// Protocol files for the CBS servo
#include <AP_PiccoloCAN/piccolo_protocol/ServoProtocol.h>
#include <AP_PiccoloCAN/piccolo_protocol/ServoPackets.h>

// Maximum number of servo allowed on CAN bus simultaneously
#define PICCOLO_CAN_MAX_NUM_SERVO 16
#define PICCOLO_CAN_MAX_GROUP_SERVO (PICCOLO_CAN_MAX_NUM_SERVO / 4)


/**
 * Class definition for the Currawong CBS servo
 */
class AP_PiccoloCAN_Servo : public AP_PiccoloCAN_Device
{
public:
    AP_PiccoloCAN_Servo();

    bool decode_servo_packet(AP_HAL::CANFrame &frame, uint64_t timestamp);

    void set_output_command(uint16_t cmd);

    bool is_enabled(void) const;

    bool add_log_data(AP_Logger *logger, uint64_t timestamp, uint8_t id);

    // Servo status data
    Servo_StatusA_t statusA;
    Servo_StatusB_t statusB;

    // Servo configuration data
    Servo_Firmware_t firmware;
    Servo_Address_t address;
    Servo_SettingsInfo_t settings;
    Servo_SystemInfo_t systemInfo;
    Servo_TelemetryConfig_t telemetry;

    // Servo command data
    int16_t command = 0;        //! Raw command to send to each servo
    bool newCommand = false;    //! Is the command "new"?
};

#endif // HAL_PICCOLO_CAN_ENABLE
