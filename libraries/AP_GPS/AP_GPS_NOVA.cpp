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

//  Novatel/Tersus/ComNav GPS driver for ArduPilot.
//  Code by Michael Oborne
//  Derived from http://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf

#include "AP_GPS.h"
#include "AP_GPS_NOVA.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define NOVA_DEBUGGING 0

#if NOVA_DEBUGGING
#include <cstdio>
 # define Debug(fmt, args ...)                  \
do {                                            \
    printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_NOVA::AP_GPS_NOVA(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    AP_GPS_BinaryCRCMessageParser({0xaa,0x44,0x12},4,sizeof(nova_msg.header),sizeof(nova_msg.data))
{
    const char *init_str = _initialisation_blob[0];
    const char *init_str1 = _initialisation_blob[1];
    
    port->write((const uint8_t*)init_str, strlen(init_str));
    port->write((const uint8_t*)init_str1, strlen(init_str1));
}

const char* const AP_GPS_NOVA::_initialisation_blob[6] {
    "\r\n\r\nunlogall\r\n", // cleanup enviroment
    "log bestposb ontime 0.2 0 nohold\r\n", // get bestpos
    "log bestvelb ontime 0.2 0 nohold\r\n", // get bestvel
    "log psrdopb onchanged\r\n", // tersus
    "log psrdopb ontime 0.2\r\n", // comnav
    "log psrdopb\r\n" // poll message, as dop only changes when a sat is dropped/added to the visible list
};

// Process all bytes available from the stream
//
bool
AP_GPS_NOVA::read(void)
{
    uint32_t now = AP_HAL::millis();

    if (_init_blob_index < (sizeof(_initialisation_blob) / sizeof(_initialisation_blob[0]))) {
        const char *init_str = _initialisation_blob[_init_blob_index];

        if (now > _init_blob_time) {
            port->write((const uint8_t*)init_str, strlen(init_str));
            _init_blob_time = now + 200;
            _init_blob_index++;
        }
    }

    bool ret = false;
    while (port->available() > 0) {
        uint8_t temp = port->read();
        ret |= parse(temp);
    }
    
    return ret;
}

bool
AP_GPS_NOVA::process_message(void)
{
    uint16_t messageid = nova_msg.header.nova_headeru.messageid;

    Debug("NOVA process_message messid=%u\n",messageid);

    check_new_itow(nova_msg.header.nova_headeru.tow, nova_msg.header.nova_headeru.messagelength + nova_msg.header.nova_headeru.headerlength);
    
    if (messageid == 42) // bestpos
    {
        const bestpos &bestposu = nova_msg.data.bestposu;

        state.time_week = nova_msg.header.nova_headeru.week;
        state.time_week_ms = (uint32_t) nova_msg.header.nova_headeru.tow;
        state.last_gps_time_ms = AP_HAL::millis();

        state.location.lat = (int32_t) (bestposu.lat * (double)1e7);
        state.location.lng = (int32_t) (bestposu.lng * (double)1e7);
        state.location.alt = (int32_t) (bestposu.hgt * 100);

        state.num_sats = bestposu.svsused;

        state.horizontal_accuracy = (float) ((bestposu.latsdev + bestposu.lngsdev)/2);
        state.vertical_accuracy = (float) bestposu.hgtsdev;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        state.rtk_age_ms = bestposu.diffage * 1000;
        state.rtk_num_sats = bestposu.svsused;

        if (bestposu.solstat == 0) // have a solution
        {
            switch (bestposu.postype)
            {
                case 16:
                    state.status = AP_GPS::GPS_OK_FIX_3D;
                    break;
                case 17: // psrdiff
                case 18: // waas
                case 20: // omnistar
                case 68: // ppp_converg
                case 69: // ppp
                    state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                    break;
                case 32: // l1 float
                case 33: // iono float
                case 34: // narrow float
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                    break;
                case 48: // l1 int
                case 50: // narrow int
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                    break;
                case 0: // NONE
                case 1: // FIXEDPOS
                case 2: // FIXEDHEIGHT
                default:
                    state.status = AP_GPS::NO_FIX;
                    break;
            }
        }
        else
        {
            state.status = AP_GPS::NO_FIX;
        }
        
        _new_position = true;
    }

    if (messageid == 99) // bestvel
    {
        const bestvel &bestvelu = nova_msg.data.bestvelu;

        state.ground_speed = (float) bestvelu.horspd;
        state.ground_course = (float) bestvelu.trkgnd;
        fill_3d_velocity();
        state.velocity.z = -(float) bestvelu.vertspd;
        state.have_vertical_velocity = true;
        
        _last_vel_time = (uint32_t) nova_msg.header.nova_headeru.tow;
        _new_speed = true;
    }

    if (messageid == 174) // psrdop
    {
        const psrdop &psrdopu = nova_msg.data.psrdopu;

        state.hdop = (uint16_t) (psrdopu.hdop*100);
        state.vdop = (uint16_t) (psrdopu.htdop*100);
        return false;
    }

    // ensure out position and velocity stay insync
    if (_new_position && _new_speed && _last_vel_time == state.time_week_ms) {
        _new_speed = _new_position = false;
        
        return true;
    }
    
    return false;
}