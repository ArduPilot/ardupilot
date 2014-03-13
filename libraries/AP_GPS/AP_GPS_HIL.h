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

//
//  Hardware in the loop gps class.
//  Code by James Goppert
//
//
#ifndef __AP_GPS_HIL_H__
#define __AP_GPS_HIL_H__

#include <AP_HAL.h>
#include "GPS.h"

class AP_GPS_HIL : public GPS {
public:
    AP_GPS_HIL() : 
		GPS(),
		_updated(false)
		{}

    virtual void        init(AP_HAL::UARTDriver *s, enum GPS_Engine_Setting nav_setting);
    virtual bool        read(void);

    /**
     * Hardware in the loop set function
     * @param latitude  - latitude in deggrees
     * @param longitude - longitude in degrees
     * @param altitude - altitude in degrees
     * @param ground_speed - ground speed in meters/second
     * @param ground_course - ground course in degrees
     * @param speed_3d - ground speed in meters/second
     * @param altitude - altitude in meters
     */
    virtual void        setHIL(Fix_Status fix_status,
                               uint64_t time_epoch_ms, float latitude, float longitude, float altitude,
                               float ground_speed, float ground_course, float speed_3d, uint8_t num_sats);

private:
    bool        _updated;
};

#endif  // __AP_GPS_HIL_H__
