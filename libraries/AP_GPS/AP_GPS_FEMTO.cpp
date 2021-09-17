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


#define FEMTO_DEBUG 0

#if FEMTO_DEBUG
#include <cstdio>
 # define Debug(fmt, args ...)                  \
do {                                            \
    printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
} while(0)
#else
 # define Debug(fmt, args ...)
#endif

const char* const AP_GPS_FEMTO::_initialisation_blob[] {
    "\r\n\r\nUNLOGALL THISPORT\r\n",         /**< cleanup enviroment */
    "LOG UAVGPSB ontime 0.2\r\n",   /**< get uavgps */
    "LOG UAVSTATUSB ontime 0.2\r\n", /**< get uavstatus */
};

AP_GPS_FEMTO::AP_GPS_FEMTO(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    AP_GPS_BinaryCRCMessageParser({FEMTO_PREAMBLE1, FEMTO_PREAMBLE2, FEMTO_PREAMBLE3},4,sizeof(_femto_msg.header),sizeof(_femto_msg.body)),
    _init_blob_index(0),
    _init_blob_time(0),
    _new_uavstatus(false),
    _last_uav_status_time(0),
    _new_uavgps(false),
    _last_uav_gps_time(0)
{
    const char *init_str = _initialisation_blob[0];
    port->write((const uint8_t*)init_str, strlen(init_str));
}

/** Process all bytes available from the stream */
bool AP_GPS_FEMTO::read(void)
{
    uint8_t data;
    int16_t numc;
    bool parsed = false;
    uint32_t now = AP_HAL::millis();

    /** send initialisation commands to device */
    if (_init_blob_index < (sizeof(_initialisation_blob) / sizeof(_initialisation_blob[0]))) {
        const char *init_str = _initialisation_blob[_init_blob_index];

        if (now > _init_blob_time) {
            port->write((const uint8_t*)init_str, strlen(init_str));
            _init_blob_time = now + 200;
            _init_blob_index++;
        }
    }

    /** receive and parse msg */
    numc = port->available();
    for (int16_t i = 0; i < numc; i++) {
        data = port->read();
        parsed |= parse(data);
    }
    
    return parsed;
}

bool AP_GPS_FEMTO::process_message(void)
{
    uint16_t messageid = _femto_msg.header.msg_header.messageid;

    Debug("FEMTO message messid=%u\n",messageid);

    check_new_itow(_femto_msg.header.msg_header.tow, _femto_msg.header.msg_header.messagelength + _femto_msg.header.msg_header.headerlength);
    
    if (messageid == FEMTO_MSG_ID_UAVGPS) {  /**< uavgps */
        const femto_uav_gps_t &uav_gps = _femto_msg.body.uav_gps;

        state.time_week = _femto_msg.header.msg_header.week;
        state.time_week_ms = (uint32_t) _femto_msg.header.msg_header.tow;

        _last_uav_gps_time = state.time_week_ms;

        state.last_gps_time_ms = AP_HAL::millis();

        state.location.lat = uav_gps.lat;
        state.location.lng = uav_gps.lon;
        state.location.alt = uav_gps.alt/10;

        state.num_sats = uav_gps.satellites_used;

        state.horizontal_accuracy = uav_gps.eph;
        state.vertical_accuracy = uav_gps.epv;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        state.rtk_num_sats = state.num_sats;

        switch (uav_gps.fix_type) {
            case 0:
            case 1:
                state.status = AP_GPS::NO_FIX;
                break;
            case 2:
                state.status = AP_GPS::GPS_OK_FIX_2D;
                break;                
            case 3:
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;  
            case 4:
                state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                break;              
            case 5:
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                break;                
            case 6:
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
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

        /** about heading */
        state.gps_yaw = uav_gps.heading;
        state.gps_yaw_time_ms = state.last_gps_time_ms;
        state.gps_yaw_accuracy = uav_gps.c_variance_rad * RAD_TO_DEG;
        bool headingFixed = uav_gps.heading_type == 6;
        state.gps_yaw_configured = headingFixed;
        state.have_gps_yaw = headingFixed;
        state.have_gps_yaw_accuracy = headingFixed;


        _new_uavgps = true;
    }

    if (messageid == FEMTO_MSG_ID_UAVSTATUS) { /**< uavstatus */
        const femto_uav_status_t &uavstatus = _femto_msg.body.uav_status;

        state.rtk_age_ms = uavstatus.diff_age * 1000;

        _last_uav_status_time = (uint32_t) _femto_msg.header.msg_header.tow;

        _new_uavstatus = true;
    }

    /** ensure uavstatus and uavgps stay insync */
    if (_new_uavstatus && _new_uavgps && _last_uav_status_time == _last_uav_gps_time) {
        _new_uavstatus = _new_uavgps = false;

        return true;
    }

    return false;
}
