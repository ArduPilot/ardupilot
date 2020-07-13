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
#include <GCS_MAVLink/GCS_MAVLink.h>

#define PROXIMITY_MAX_INSTANCES             1   // Maximum number of proximity sensor instances available on this platform
#define PROXIMITY_MAX_IGNORE                6   // up to six areas can be ignored
#define PROXIMITY_MAX_DIRECTION 8
#define PROXIMITY_SENSOR_ID_START 10

class AP_Proximity_Backend;

class AP_Proximity
{
public:
    friend class AP_Proximity_Backend;

    AP_Proximity();

    AP_Proximity(const AP_Proximity &other) = delete;
    AP_Proximity &operator=(const AP_Proximity) = delete;

    // Proximity driver types
    enum class Type {
        None    = 0,
        SF40C_v09 = 1,
        MAV     = 2,
        TRTOWER = 3,
        RangeFinder = 4,
        RPLidarA2 = 5,
        TRTOWEREVO = 6,
        SF40C = 7,
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        SITL    = 10,
        MorseSITL = 11,
        AirSimSITL = 12,
#endif
    };

    enum class Status {
        NotConnected = 0,
        NoData,
        Good
    };

    // structure holding distances in PROXIMITY_MAX_DIRECTION directions. used for sending distances to ground station
    struct Proximity_Distance_Array {
        uint8_t orientation[PROXIMITY_MAX_DIRECTION]; // orientation (i.e. rough direction) of the distance (see MAV_SENSOR_ORIENTATION)
        float distance[PROXIMITY_MAX_DIRECTION];      // distance in meters
    };

    // detect and initialise any available proximity sensors
    void init(void);

    // update state of all proximity sensors. Should be called at high rate from main loop
    void update(void);

    // return sensor orientation and yaw correction
    uint8_t get_orientation(uint8_t instance) const;
    int16_t get_yaw_correction(uint8_t instance) const;

    // return sensor health
    Status get_status(uint8_t instance) const;
    Status get_status() const;

    // Return the number of proximity sensors
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // get distances in PROXIMITY_MAX_DIRECTION directions. used for sending distances to ground station
    bool get_horizontal_distances(Proximity_Distance_Array &prx_dist_array) const;

    // get boundary points around vehicle for use by avoidance
    //   returns nullptr and sets num_points to zero if no boundary can be returned
    const Vector2f* get_boundary_points(uint8_t instance, uint16_t& num_points) const;
    const Vector2f* get_boundary_points(uint16_t& num_points) const;

    // get distance and angle to closest object (used for pre-arm check)
    //   returns true on success, false if no valid readings
    bool get_closest_object(float& angle_deg, float &distance) const;

    // get number of objects, angle and distance - used for non-GPS avoidance
    uint8_t get_object_count() const;
    bool get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const;

    // get maximum and minimum distances (in meters) of primary sensor
    float distance_max() const;
    float distance_min() const;

    // handle mavlink DISTANCE_SENSOR messages
    void handle_msg(const mavlink_message_t &msg);

    // The Proximity_State structure is filled in by the backend driver
    struct Proximity_State {
        uint8_t                 instance;   // the instance number of this proximity sensor
        Status   status;     // sensor status
    };

    //
    // support for upward facing sensors
    //

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(uint8_t instance, float &distance) const;
    bool get_upward_distance(float &distance) const;

    Type get_type(uint8_t instance) const;

    // parameter list
    static const struct AP_Param::GroupInfo var_info[];

    static AP_Proximity *get_singleton(void) { return _singleton; };

    // methods for mavlink SYS_STATUS message (send_sys_status)
    // these methods cover only the primary instance
    bool sensor_present() const;
    bool sensor_enabled() const;
    bool sensor_failed() const;

private:
    static AP_Proximity *_singleton;
    Proximity_State state[PROXIMITY_MAX_INSTANCES];
    AP_Proximity_Backend *drivers[PROXIMITY_MAX_INSTANCES];
    uint8_t primary_instance;
    uint8_t num_instances;

    bool valid_instance(uint8_t i) const {
        if (drivers[i] == nullptr) {
            return false;
        }
        return (Type)_type[i].get() != Type::None;
    }

    // parameters for all instances
    AP_Int8  _type[PROXIMITY_MAX_INSTANCES];
    AP_Int8  _orientation[PROXIMITY_MAX_INSTANCES];
    AP_Int16 _yaw_correction[PROXIMITY_MAX_INSTANCES];
    AP_Int16 _ignore_angle_deg[PROXIMITY_MAX_IGNORE];   // angle (in degrees) of area that should be ignored by sensor (i.e. leg shows up)
    AP_Int8 _ignore_width_deg[PROXIMITY_MAX_IGNORE];    // width of beam (in degrees) that should be ignored

    void detect_instance(uint8_t instance);
};

namespace AP {
    AP_Proximity *proximity();
};
