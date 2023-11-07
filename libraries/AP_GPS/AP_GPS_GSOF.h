/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//  Trimble GPS driver for ArduPilot.
//	Code by Michael Oborne
//  https://receiverhelp.trimble.com/oem-gnss/index.html#Welcome.html?TocPath=_____1

#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#if AP_GPS_GSOF_ENABLED
class AP_GPS_GSOF : public AP_GPS_Backend
{
public:
    AP_GPS_GSOF(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    AP_GPS::GPS_Status highest_supported_status(void) override WARN_IF_UNUSED {
        return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
    }

    // Methods
    bool read() override WARN_IF_UNUSED;

    const char *name() const override { return "GSOF"; }

private:

    // A subset of the port identifiers in the GSOF protocol that are used for serial.
    // Ethernet, USB, etc are not supported by the GPS driver at this time so they are omitted.
    // These values are not documented in the API.
    enum class HW_Port {
        COM1 = 0, // RS232 serial
        COM2 = 1, // TTL serial
    };

    // A subset of the data frequencies in the GSOF protocol that are used for serial.
    // These values are not documented in the API.
    enum class Output_Rate {
        FREQ_10_HZ = 1,
        FREQ_50_HZ = 15,
        FREQ_100_HZ = 16,
    };

    bool parse(const uint8_t temp) WARN_IF_UNUSED;
    bool process_message() WARN_IF_UNUSED;

    // Send a request to the GPS to set the baud rate on the specified port.
    // Note - these request functions currently ignore the ACK from the device.
    // If the device is already sending serial traffic, there is no mechanism to prevent conflict.
    // According to the manufacturer, the best approach is to switch to ethernet.
    void requestBaud(const HW_Port portIndex);
    // Send a request to the GPS to enable a message type on the port at the specified rate.
    void requestGSOF(const uint8_t messageType, const HW_Port portIndex, const Output_Rate rateHz);

    double SwapDouble(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;
    float SwapFloat(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;
    uint32_t SwapUint32(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;
    uint16_t SwapUint16(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;

    bool validate_baud(const uint8_t baud) const WARN_IF_UNUSED;
    bool validate_com_port(const uint8_t com_port) const WARN_IF_UNUSED;

    struct Msg_Parser
    {

        enum class State
        {
            STARTTX = 0,
            STATUS,
            PACKETTYPE,
            LENGTH,
            DATA,
            CHECKSUM,
            ENDTX
        };

        State state;

        uint8_t status;
        uint8_t packettype;
        uint8_t length;
        uint8_t data[256];
        uint8_t checksum;
        uint8_t endtx;

        uint16_t read;
        uint8_t checksumcalc;
    } msg;

    static const uint8_t STX = 0x02;
    static const uint8_t ETX = 0x03;

    uint8_t packetcount;
    uint32_t gsofmsg_time;
    uint8_t gsofmsgreq_index;
    const uint8_t gsofmsgreq[5] = {1,2,8,9,12};
};
#endif
