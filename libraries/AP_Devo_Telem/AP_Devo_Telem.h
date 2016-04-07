// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#ifndef __AP_DEVO_TELEM_H__
#define __AP_DEVO_TELEM_H__

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_SerialManager/AP_SerialManager.h>

typedef uint8_t byte;

#define DEVOM_SYNC_BYTE        0xAA

#pragma pack(push, 1)
typedef struct {
        uint8_t header;                                ///< 0xAA for a valid packet
        int32_t lon;
        int32_t lat;
        int32_t alt;
        int16_t speed;
        int16_t temp;
        int16_t volt;
        uint8_t cs8;   ///< checksum
} DevoMPacket;
#pragma pack(pop)

class AP_DEVO_Telem
{
public:
    //constructor
    AP_DEVO_Telem(AP_AHRS &ahrs, AP_BattMonitor &battery);

    // init - perform require initialisation including detecting which protocol to use
    void init(const AP_SerialManager& serial_manager);

    // send_frames - sends updates down telemetry link for both DPORT and SPORT protocols
    //  should be called by main program at 50hz to allow poll for serial bytes
    //  coming from the receiver for the SPort protocol
    void send_frames(uint8_t control_mode);

private:
    long gpsDdToDmsFormat(float ddm);


    static DevoMPacket devoPacket;

    AP_AHRS &_ahrs;                         // reference to attitude estimate
    AP_BattMonitor &_battery;               // reference to battery monitor object
    AP_HAL::UARTDriver *_port;              // UART used to send data to receiver
    bool _initialised_uart;                 // true when we have detected the protocol and UART has been initialised
    uint32_t _last_frame_ms;

};
#endif
