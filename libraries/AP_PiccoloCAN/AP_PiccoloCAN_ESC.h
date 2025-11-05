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

#include <AP_Math/AP_Math.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#include "AP_PiccoloCAN_config.h"
#include "AP_PiccoloCAN_Device.h"
#include "piccolo_protocol/ESCPackets.h"

#if HAL_PICCOLO_CAN_ENABLE

#define PICCOLO_CAN_MAX_NUM_ESC 16
#define PICCOLO_CAN_MAX_GROUP_ESC (PICCOLO_CAN_MAX_NUM_ESC / 4)


/*
 * Class representing an individual PiccoloCAN ESC
 */
class AP_PiccoloCAN_ESC : public AP_PiccoloCAN_Device, public AP_ESC_Telem_Backend
{
public:

    virtual bool handle_can_frame(AP_HAL::CANFrame &frame) override;

    bool is_sw_inhibited(void) const { return status.statusA.status.swInhibit; }
    bool is_hw_inhibited(void) const { return status.statusA.status.hwInhibit; }

    virtual bool is_enabled(void) const override { return !is_sw_inhibited() && !is_hw_inhibited(); }

    float voltage() { return (float) status.statusB.voltage * 0.01f; }      // Convert to V
    float current() { return (float) status.statusB.current * 0.01f; }      // Convert to A
    uint16_t rpm() { return status.statusA.rpm; }
    float temperature() { return MAX(status.statusB.escTemperature, status.statusC.fetTemperature); }
    float motorTemperature() { return status.statusB.motorTemperature; }

    int16_t command;    //! Raw command to send to each ESC
    bool newCommand;    //! Is the command "new"?
    bool newTelemetry;  //! Is there new telemetry data available?

    // Status / telemetry data
    struct Status_t {
        ESC_StatusA_t statusA;
        ESC_StatusB_t statusB;
        ESC_StatusC_t statusC;

        ESC_WarningBits_t warnings;
        ESC_ErrorBits_t errors;
    } status;

    // Settings information
    struct Settings_t {
        ESC_Firmware_t firmware;
        ESC_Address_t address;
        ESC_EEPROMSettings_t eeprom;
    } settings;
};

#endif // HAL_PICCOLO_CAN_ENABLE
