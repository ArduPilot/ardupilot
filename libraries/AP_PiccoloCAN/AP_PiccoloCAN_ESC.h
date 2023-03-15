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

#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#include "AP_PiccoloCAN_Device.h"

// Protocol files for the Velocity ESC
#include <AP_PiccoloCAN/piccolo_protocol/ESCVelocityProtocol.h>
#include <AP_PiccoloCAN/piccolo_protocol/ESCPackets.h>


// Maximum number of ESC allowed on CAN bus simultaneously
#define PICCOLO_CAN_MAX_NUM_ESC 16
#define PICCOLO_CAN_MAX_GROUP_ESC (PICCOLO_CAN_MAX_NUM_ESC / 4)


/**
 * Class definition for the Currawong ESC Velocity
 */
class AP_PiccoloCAN_ESC : public AP_PiccoloCAN_Device, public AP_ESC_Telem_Backend
{
public:
    AP_PiccoloCAN_ESC();

    // Decode an incoming ESC packet
    bool decode_esc_packet(AP_HAL::CANFrame &frame, uint8_t index, uint64_t timestamp);

    void set_output_command(uint16_t command);

    // Accessor functions for ESC operational data
    virtual int16_t rpm(void) const;
    virtual float voltage(void) const;
    virtual float current(void) const;
    virtual float temperature(void) const;
    virtual float motor_temperature(void) const;

    virtual bool is_enabled(void) const;

    virtual bool is_hardware_enabled(void) const;
    virtual bool is_software_enabled(void) const;

    // Status information
    ESC_StatusA_t statusA;
    ESC_StatusB_t statusB;
    ESC_StatusC_t statusC;

    ESC_MotorStatusFlags_t motorStatusFlags;

    ESC_WarningBits_t warningBits;
    ESC_ErrorBits_t errorBits;

    ESC_TelltaleValues_t telltale;

    // Settings information
    ESC_Firmware_t firmware;
    ESC_Address_t address;
    ESC_EEPROMSettings_t eeprom;

    // Output information
    int16_t command = 0;            //! Raw command to send to each ESC
    bool newCommand = false;        //! Is the command "new"?

    // Telemetry Packets Available
    bool telemAvailableStatusA;
    bool telemAvailableStatusB;
    bool telemAvailableStatusC;
    bool telemAvailableWarningErrorStatus;
    bool telemAvailableTelltale;
    bool telemAvailableMotorStatusFlags;
};
