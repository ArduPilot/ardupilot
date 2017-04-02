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

/*
  GPS driver backend class
 */
#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_GPS.h"

class AP_GPS_Backend
{
public:
	AP_GPS_Backend(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    // we declare a virtual destructor so that GPS drivers can
    // override with a custom destructor if need be.
    virtual ~AP_GPS_Backend(void) {}

    // The read() method is the only one needed in each driver. It
    // should return true when the backend has successfully received a
    // valid packet from the GPS.
    virtual bool read() = 0;

    // Highest status supported by this GPS. 
    // Allows external system to identify type of receiver connected.
    virtual AP_GPS::GPS_Status highest_supported_status(void) { return AP_GPS::GPS_OK_FIX_3D; }

    virtual bool is_configured(void) { return true; }

    virtual void inject_data(const uint8_t *data, uint16_t len) { return; }

    //MAVLink methods
    virtual void send_mavlink_gps_rtk(mavlink_channel_t chan) { return ; }

    virtual void send_mavlink_gps2_rtk(mavlink_channel_t chan) { return ; }

    virtual void broadcast_configuration_failure_reason(void) const { return ; }

    virtual void handle_msg(const mavlink_message_t *msg) { return ; }
    virtual void handle_gnss_msg(const AP_GPS::GPS_State &msg) { return ; }

    // driver specific lag
    virtual float get_lag(void) const { return 0.2f; }

protected:
    AP_HAL::UARTDriver *port;           ///< UART we are attached to
    AP_GPS &gps;                        ///< access to frontend (for parameters)
    AP_GPS::GPS_State &state;           ///< public state for this instance

    // common utility functions
    int32_t swap_int32(int32_t v) const;
    int16_t swap_int16(int16_t v) const;

    /*
      fill in 3D velocity from 2D components
     */
    void fill_3d_velocity(void);

    /*
       fill in time_week_ms and time_week from BCD date and time components
       assumes MTK19 millisecond form of bcd_time
    */
    void make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds);
};
