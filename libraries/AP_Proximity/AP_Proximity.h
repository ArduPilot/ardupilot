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

#include "AP_Proximity_config.h"

#if HAL_PROXIMITY_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_Proximity_Params.h"

#define PROXIMITY_MAX_INSTANCES             2   // Maximum number of proximity sensor instances available on this platform
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
        // 1 was SF40C_v09
        MAV     = 2,
        TRTOWER = 3,
        RangeFinder = 4,
        RPLidarA2 = 5,
        TRTOWEREVO = 6,
        SF40C = 7,
        SF45B = 8,
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        SITL    = 10,
        AirSimSITL = 12,
#endif
        CYGBOT_D1 = 13,
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
        bool valid(uint8_t offset) const {
            // returns true if the distance stored at offset is valid
            return (offset < 8 && (offset_valid & (1U<<offset)));
        };

        uint8_t offset_valid; // bitmask
    };

    // detect and initialise any available proximity sensors
    void init(void);

    // update state of all proximity sensors. Should be called at high rate from main loop
    void update(void);

    // return sensor orientation and yaw correction
    uint8_t get_orientation(uint8_t instance) const;
    int16_t get_yaw_correction(uint8_t instance) const;
    float get_filter_freq() const { return _filt_freq; }

    // return sensor health
    Status get_status(uint8_t instance) const;
    Status get_status() const;

    // Return the number of proximity sensors
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // get distances in PROXIMITY_MAX_DIRECTION directions. used for sending distances to ground station
    bool get_horizontal_distances(Proximity_Distance_Array &prx_dist_array) const;

    // get raw and filtered distances in 8 directions per layer. used for logging
    bool get_active_layer_distances(uint8_t layer, AP_Proximity::Proximity_Distance_Array &prx_dist_array, AP_Proximity::Proximity_Distance_Array &prx_filt_dist_array) const;

    // get total number of obstacles, used in GPS based Simple Avoidance
    uint8_t get_obstacle_count() const;

    // get vector to obstacle based on obstacle_num passed, used in GPS based Simple Avoidance
    bool get_obstacle(uint8_t obstacle_num, Vector3f& vec_to_obstacle) const;

    // returns shortest distance to "obstacle_num" obstacle, from a line segment formed between "seg_start" and "seg_end"
    // returns FLT_MAX if it's an invalid instance.
    bool closest_point_from_segment_to_obstacle(uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const;

    // get distance and angle to closest object (used for pre-arm check)
    //   returns true on success, false if no valid readings
    bool get_closest_object(float& angle_deg, float &distance) const;

    // get number of objects, angle and distance - used for non-GPS avoidance
    uint8_t get_object_count() const;
    bool get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const;

    // get number of layers
    uint8_t get_num_layers() const;

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

    // set alt as read from downward facing rangefinder. Tilt is already adjusted for
    void set_rangefinder_alt(bool use, bool healthy, float alt_cm);

    // method called by vehicle to have AP_Proximity write onboard log
    // messages:
    void log();

protected:

    // parameters for backends
    AP_Proximity_Params params[PROXIMITY_MAX_INSTANCES];

private:
    static AP_Proximity *_singleton;
    Proximity_State state[PROXIMITY_MAX_INSTANCES];
    AP_Proximity_Backend *drivers[PROXIMITY_MAX_INSTANCES];
    uint8_t primary_instance;
    uint8_t num_instances;

    // return true if the given instance exists
    bool valid_instance(uint8_t i) const;

    void detect_instance(uint8_t instance);

    // parameters for all instances
    AP_Int8 _raw_log_enable;                            // enable logging raw distances
    AP_Int8 _ign_gnd_enable;                           // true if land detection should be enabled
    AP_Float _filt_freq;                               // cutoff frequency for low pass filter
};

namespace AP {
    AP_Proximity *proximity();
};

#endif // HAL_PROXIMITY_ENABLED
