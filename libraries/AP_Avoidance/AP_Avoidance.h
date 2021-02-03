#pragma once

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
  Situational awareness for ArduPilot

 - record a series of moving points in space which should be avoided
 - produce messages for GCS if a collision risk is detected

  Peter Barker, May 2016

  based on AP_ADSB,  Tom Pittenger, November 2015
*/

#include <AP_ADSB/AP_ADSB.h>

#if HAL_ADSB_ENABLED

#define AP_AVOIDANCE_STATE_RECOVERY_TIME_MS                 2000    // we will not downgrade state any faster than this (2 seconds)

#define AP_AVOIDANCE_ESCAPE_TIME_SEC                        2       // vehicle runs from thread for 2 seconds

class AP_Avoidance {
public:

    // constructor
    AP_Avoidance(class AP_ADSB &adsb);

    /* Do not allow copies */
    AP_Avoidance(const AP_Avoidance &other) = delete;
    AP_Avoidance &operator=(const AP_Avoidance&) = delete;

    // get singleton instance
    static AP_Avoidance *get_singleton() {
        return _singleton;
    }

    // F_RCVRY possible parameter values:
    enum class RecoveryAction {
        REMAIN_IN_AVOID_ADSB       = 0,
        RESUME_PREVIOUS_FLIGHTMODE = 1,
        RTL                        = 2,
        RESUME_IF_AUTO_ELSE_LOITER = 3,
    };

    // obstacle class to hold latest information for a known obstacles
    class Obstacle {
    public:
        MAV_COLLISION_SRC src;
        uint32_t src_id;
        uint32_t timestamp_ms;

        Location _location;
        Vector3f _velocity;

        // fields relating to this being a threat.  These would be the reason to have a separate list of threats:
        MAV_COLLISION_THREAT_LEVEL threat_level;
        float closest_approach_xy; // metres
        float closest_approach_z; // metres
        float time_to_closest_approach; // seconds, 3D approach
        float distance_to_closest_approach; // metres, 3D
        uint32_t last_gcs_report_time; // millis
    };


    // add obstacle to the list of known obstacles
    void add_obstacle(uint32_t obstacle_timestamp_ms,
                      const MAV_COLLISION_SRC src,
                      uint32_t src_id,
                      const Location &loc,
                      const Vector3f &vel_ned);

    void add_obstacle(uint32_t obstacle_timestamp_ms,
                      const MAV_COLLISION_SRC src,
                      uint32_t src_id,
                      const Location &loc,
                      float cog,
                      float hspeed,
                      float vspeed);

    // update should be called at 10hz or higher
    void update();

    // enable or disable avoidance
    void enable() { _enabled = true; };
    void disable() { _enabled = false; };

    // current overall threat level
    MAV_COLLISION_THREAT_LEVEL current_threat_level() const;

    // add obstacles into the Avoidance system from MAVLink messages
    void handle_msg(const mavlink_message_t &msg);

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // top level avoidance handler.  This calls the vehicle specific handle_avoidance with requested action
    void handle_avoidance_local(AP_Avoidance::Obstacle *threat);

    // avoid the most significant threat.  child classes must override this method
    // function returns the action that it is actually taking
    virtual MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) = 0;

    // recover after all threats have cleared.  child classes must override this method
    // recovery_action is from F_RCVRY parameter
    virtual void handle_recovery(RecoveryAction recovery_action) = 0;

    uint32_t _last_state_change_ms = 0;
    MAV_COLLISION_THREAT_LEVEL _threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;

    // gcs notification
    // specifies how long we should continue sending messages about a threat after it has cleared
    static const uint8_t _gcs_cleared_messages_duration = 5; // seconds
    uint32_t _gcs_cleared_messages_first_sent;

    void handle_threat_gcs_notify(AP_Avoidance::Obstacle *threat);

    AP_Avoidance::Obstacle *most_serious_threat();

    // returns an entry from the MAV_COLLISION_ACTION representative
    // of what the current avoidance handler is up to.
    MAV_COLLISION_ACTION mav_avoidance_action() { return _latest_action; }

    // get target destination that best gets vehicle away from the nearest obstacle
    bool get_destination_perpendicular(const AP_Avoidance::Obstacle *obstacle, Vector3f &newdest_neu, const float wp_speed_xy, const float wp_speed_z, const uint8_t _minimum_avoid_height);

    // get unit vector away from the nearest obstacle
    bool get_vector_perpendicular(const AP_Avoidance::Obstacle *obstacle, Vector3f &vec_neu) const;

    // helper functions to calculate destination to get us away from obstacle
    // Note: v1 is NED
    static Vector3f perpendicular_xyz(const Location &p1, const Vector3f &v1, const Location &p2);
    static Vector2f perpendicular_xy(const Location &p1, const Vector3f &v1, const Location &p2);

private:

    void send_collision_all(const AP_Avoidance::Obstacle &threat, MAV_COLLISION_ACTION behaviour) const;

    // constants
    const uint32_t MAX_OBSTACLE_AGE_MS = 5000;      // obstacles that have not been heard from for 5 seconds are removed from the list
    const static uint8_t _gcs_notify_interval = 1; // seconds

    // speed below which we will fly directly away from a threat
    // rather than perpendicular to its velocity:
    const uint8_t _low_velocity_threshold = 1; // meters/second

    // check to see if we are initialised (and possibly do initialisation)
    bool check_startup();

    // initialize _obstacle_list
    void init();

    // free _obstacle_list
    void deinit();

    // get unique id for adsb
    uint32_t src_id_for_adsb_vehicle(const AP_ADSB::adsb_vehicle_t &vehicle) const;

    void check_for_threats();
    void update_threat_level(const Location &my_loc,
                             const Vector3f &my_vel,
                             AP_Avoidance::Obstacle &obstacle);

    // calls into the AP_ADSB library to retrieve vehicle data
    void get_adsb_samples();

    // returns true if the obstacle should be considered more of a
    // threat than the current most serious threat
    bool obstacle_is_more_serious_threat(const AP_Avoidance::Obstacle &obstacle) const;

    // internal variables
    AP_Avoidance::Obstacle *_obstacles;
    uint8_t _obstacles_allocated;
    uint8_t _obstacle_count;
    int8_t _current_most_serious_threat;
    MAV_COLLISION_ACTION _latest_action = MAV_COLLISION_ACTION_NONE;

    // external references
    class AP_ADSB &_adsb;

    // parameters
    AP_Int8     _enabled;
    AP_Int8     _obstacles_max;

    AP_Int8     _fail_action;
    AP_Int8     _fail_recovery;
    AP_Int8     _fail_time_horizon;
    AP_Int16    _fail_distance_xy;
    AP_Int16    _fail_distance_z;
    AP_Int16    _fail_altitude_minimum;

    AP_Int8     _warn_action;
    AP_Int8     _warn_time_horizon;
    AP_Float    _warn_distance_xy;
    AP_Float    _warn_distance_z;

    // multi-thread support for avoidance
    HAL_Semaphore _rsem;

    static AP_Avoidance *_singleton;
};

float closest_approach_xy(const Location &my_loc,
                          const Vector3f &my_vel,
                          const Location &obstacle_loc,
                          const Vector3f &obstacle_vel,
                          uint8_t time_horizon);

float closest_approach_z(const Location &my_loc,
                         const Vector3f &my_vel,
                         const Location &obstacle_loc,
                         const Vector3f &obstacle_vel,
                         uint8_t time_horizon);


namespace AP {
    AP_Avoidance *ap_avoidance();
};

#endif

