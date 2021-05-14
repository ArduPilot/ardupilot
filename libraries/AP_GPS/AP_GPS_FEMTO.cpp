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

//  Femtomes GPS driver for ArduPilot.
//  Code by Rui Zheng <ruizheng@femtomes.com>


#include "AP_GPS.h"
#include "AP_GPS_FEMTO.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define FEMTO_DEBUGGING 0

#define FEMTO_MSG_ID_UAVGPS 		8001

#if FEMTO_DEBUGGING
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

const char* const AP_GPS_FEMTO::_initialisation_blob[2] {
    "\r\n\r\nunlogall\r\n", // cleanup enviroment
    "log uavgpsb ontime 0.2\r\n", // get bestpos
};

AP_GPS_FEMTO::AP_GPS_FEMTO(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    femto_msg.femto_state = femto_msg_parser::PREAMBLE1;

    const char *init_str = _initialisation_blob[0];
    const char *init_str1 = _initialisation_blob[1];
    
    port->write((const uint8_t*)init_str, strlen(init_str));
    port->write((const uint8_t*)init_str1, strlen(init_str1));
}

// Process all bytes available from the stream
//
bool
AP_GPS_FEMTO::read(void)
{
    uint32_t now = AP_HAL::millis();

    bool ret = false;
    while (port->available() > 0) {
        uint8_t temp = port->read();
        ret |= parse(temp);
    }
    
    return ret;
}

bool
AP_GPS_FEMTO::parse(uint8_t temp)
{
    switch (femto_msg.femto_state)
    {
        default:
        case femto_msg_parser::PREAMBLE1:
            if (temp == FEMTO_PREAMBLE1)
                femto_msg.femto_state = femto_msg_parser::PREAMBLE2;
            femto_msg.read = 0;
            break;
        case femto_msg_parser::PREAMBLE2:
            if (temp == FEMTO_PREAMBLE2)
            {
                femto_msg.femto_state = femto_msg_parser::PREAMBLE3;
            }
            else
            {
                femto_msg.femto_state = femto_msg_parser::PREAMBLE1;
            }
            break;
        case femto_msg_parser::PREAMBLE3:
            if (temp == FEMTO_PREAMBLE3)
            {
                femto_msg.femto_state = femto_msg_parser::HEADERLENGTH;
            }
            else
            {
                femto_msg.femto_state = femto_msg_parser::PREAMBLE1;
            }
            break;
        case femto_msg_parser::HEADERLENGTH:
            Debug("FEMTO HEADERLENGTH\n");
            femto_msg.header.data[0] = FEMTO_PREAMBLE1;
            femto_msg.header.data[1] = FEMTO_PREAMBLE2;
            femto_msg.header.data[2] = FEMTO_PREAMBLE3;
            femto_msg.header.data[3] = temp;
            femto_msg.header.femto_header.headerlength = temp;
            femto_msg.femto_state = femto_msg_parser::HEADERDATA;
            femto_msg.read = 4;
            break;
        case femto_msg_parser::HEADERDATA:
            if (femto_msg.read >= sizeof(femto_msg.header.data)) {
                Debug("parse header overflow length=%u\n", (unsigned)femto_msg.read);
                femto_msg.femto_state = femto_msg_parser::PREAMBLE1;
                break;
            }
            femto_msg.header.data[femto_msg.read] = temp;
            femto_msg.read++;
            if (femto_msg.read >= femto_msg.header.femto_header.headerlength)
            {
                femto_msg.femto_state = femto_msg_parser::DATA;
            }
            break;
        case femto_msg_parser::DATA:
            if (femto_msg.read >= sizeof(femto_msg.data)) {
                Debug("parse data overflow length=%u msglength=%u\n", (unsigned)femto_msg.read,femto_msg.header.femto_header.messagelength);
                femto_msg.femto_state = femto_msg_parser::PREAMBLE1;
                break;
            }
            femto_msg.data.bytes[femto_msg.read - femto_msg.header.femto_header.headerlength] = temp;
            femto_msg.read++;
            if (femto_msg.read >= (femto_msg.header.femto_header.messagelength + femto_msg.header.femto_header.headerlength))
            {
                Debug("FEMTO DATA exit\n");
                femto_msg.femto_state = femto_msg_parser::CRC1;
            }
            break;
        case femto_msg_parser::CRC1:
            femto_msg.crc = (uint32_t) (temp << 0);
            femto_msg.femto_state = femto_msg_parser::CRC2;
            break;
        case femto_msg_parser::CRC2:
            femto_msg.crc += (uint32_t) (temp << 8);
            femto_msg.femto_state = femto_msg_parser::CRC3;
            break;
        case femto_msg_parser::CRC3:
            femto_msg.crc += (uint32_t) (temp << 16);
            femto_msg.femto_state = femto_msg_parser::CRC4;
            break;
        case femto_msg_parser::CRC4:
            femto_msg.crc += (uint32_t) (temp << 24);
            femto_msg.femto_state = femto_msg_parser::PREAMBLE1;

            uint32_t crc = CalculateBlockCRC32((uint32_t)femto_msg.header.femto_header.headerlength, (uint8_t *)&femto_msg.header.data, (uint32_t)0);
            crc = CalculateBlockCRC32((uint32_t)femto_msg.header.femto_header.messagelength, (uint8_t *)&femto_msg.data, crc);

            if (femto_msg.crc == crc)
            {
                return process_message();
            }
            else
            {
                Debug("crc failed");
                crc_error_counter++;
            }
            break;
    }

    return false;
}

bool
AP_GPS_FEMTO::process_message(void)
{
    uint16_t messageid = femto_msg.header.femto_header.messageid;

    Debug("FEMTO process_message messid=%u\n",messageid);

    check_new_itow(femto_msg.header.femto_header.tow, femto_msg.header.femto_header.messagelength + femto_msg.header.femto_header.headerlength);
    
    if (messageid == FEMTO_MSG_ID_UAVGPS) // bestpos
    {
        const femto_uav_gps_t &uav_gps = femto_msg.data.uav_gps;

        state.time_week = femto_msg.header.femto_header.week;
        state.time_week_ms = (uint32_t) femto_msg.header.femto_header.tow;
        state.last_gps_time_ms = AP_HAL::millis();

        state.location.lat = uav_gps.lat;
        state.location.lng = uav_gps.lon;
        state.location.alt = uav_gps.alt;

        state.num_sats = uav_gps.satellites_used;

        state.horizontal_accuracy = uav_gps.eph;
        state.vertical_accuracy = uav_gps.epv;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
//@TODO       state.rtk_age_ms = bestposu.diffage * 1000;
//@TODO       state.rtk_num_sats = bestposu.svsused;

        switch (uav_gps.fix_type)
        {
            case 0:
            case 1:
                state.status = AP_GPS::NO_FIX;
                break;
            case 2:
                state.status = AP_GPS::GPS_FIX_TYPE_2D_FIX;
                break;                
            case 3:
                state.status = AP_GPS::GPS_FIX_TYPE_3D_FIX;
                break;  
            case 4:
                state.status = AP_GPS::GPS_FIX_TYPE_DGPS;
                break;              
            case 5:
                state.status = AP_GPS::GPS_FIX_TYPE_RTK_FLOAT;
                break;                
            case 6:
                state.status = AP_GPS::GPS_FIX_TYPE_RTK_FIXED;
                break;             
            default:
                state.status = AP_GPS::NO_FIX;
                break;
        }

        state.ground_speed = uav_gps.vel_m_s;
        state.ground_course = uav_gps.cog_rad * RAD_TO_DEG;
        fill_3d_velocity();
        state.velocity.z = uav_gps.vel_d_m_s;
        state.have_vertical_velocity = true;

        state.hdop = (uint16_t) (uav_gps.hdop*100);
        state.vdop = (uint16_t) (uav_gps.vdop*100);

        return true;
    }

    return false;
}

void
AP_GPS_FEMTO::inject_data(const uint8_t *data, uint16_t len)
{
    if (port->txspace() > len) {
        last_injected_data_ms = AP_HAL::millis();
        port->write(data, len);
    } else {
        Debug("FEMTO: Not enough TXSPACE");
    }
}

#define CRC32_POLYNOMIAL 0xEDB88320L
uint32_t AP_GPS_FEMTO::CRC32Value(uint32_t icrc)
{
    int i;
    uint32_t crc = icrc;
    for ( i = 8 ; i > 0; i-- )
    {
        if ( crc & 1 )
            crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            crc >>= 1;
    }
    return crc;
}

uint32_t AP_GPS_FEMTO::CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc)
{
    while ( length-- != 0 )
    {
        crc = ((crc >> 8) & 0x00FFFFFFL) ^ (CRC32Value(((uint32_t) crc ^ *buffer++) & 0xff));
    }
    return( crc );
}
