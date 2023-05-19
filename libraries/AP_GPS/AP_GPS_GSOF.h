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

    bool parse(const uint8_t temp) WARN_IF_UNUSED;
    bool process_message() WARN_IF_UNUSED;
    void requestBaud(const uint8_t portindex);
    void requestGSOF(const uint8_t messagetype, const uint8_t portindex);
    double SwapDouble(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;
    float SwapFloat(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;
    uint32_t SwapUint32(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;
    uint16_t SwapUint16(const uint8_t* src, const uint32_t pos) const WARN_IF_UNUSED;

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
