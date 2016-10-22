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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>


#define PROXIMITY_MAX_INSTANCES             1   // Maximum number of proximity sensor instances available on this platform
#define PROXIMITY_YAW_CORRECTION_DEFAULT    22  // default correction for sensor error in yaw

class AP_Proximity_Backend;

class AP_Proximity
{
public:
    friend class AP_Proximity_Backend;

    AP_Proximity(AP_SerialManager &_serial_manager);

    // Proximity driver types
    enum Proximity_Type {
        Proximity_Type_None  = 0,
        Proximity_Type_SF40C = 1,
    };

    enum Proximity_Status {
        Proximity_NotConnected = 0,
        Proximity_NoData,
        Proximity_Good
    };

    // detect and initialise any available rangefinders
    void init(void);

    // update state of all rangefinders. Should be called at high rate from main loop
    void update(void);

    // return sensor orientation and yaw correction
    uint8_t get_orientation(uint8_t instance) const;
    int16_t get_yaw_correction(uint8_t instance) const;

    // return sensor health
    Proximity_Status get_status(uint8_t instance) const;
    Proximity_Status get_status() const;

    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // get distance in meters in a particular direction in degrees (0 is forward, clockwise)
    // returns true on successful read and places distance in distance
    bool get_horizontal_distance(uint8_t instance, float angle_deg, float &distance) const;
    bool get_horizontal_distance(float angle_deg, float &distance) const;

    // The Proximity_State structure is filled in by the backend driver
    struct Proximity_State {
        uint8_t                 instance;   // the instance number of this proximity sensor
        enum Proximity_Status   status;     // sensor status
    };

    // parameter list
    static const struct AP_Param::GroupInfo var_info[];

private:
    Proximity_State state[PROXIMITY_MAX_INSTANCES];
    AP_Proximity_Backend *drivers[PROXIMITY_MAX_INSTANCES];
    uint8_t primary_instance:3;
    uint8_t num_instances:3;
    AP_SerialManager &serial_manager;

    // parameters for all instances
    AP_Int8  _type[PROXIMITY_MAX_INSTANCES];
    AP_Int8  _orientation[PROXIMITY_MAX_INSTANCES];
    AP_Int16 _yaw_correction[PROXIMITY_MAX_INSTANCES];

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  
};
