/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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

#include <AP_AHRS/AP_AHRS.h>
#include <AP_ADSB/AP_ADSB.h>

class AP_Avoidance {

public:

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

    AP_Avoidance(AP_AHRS &ahrs, class AP_ADSB &adsb);

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

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

    void update();

    static AP_Avoidance *instance(void) {
        return _instance;
    }

    // add obstacles into the Avoidance system from MAVLink messages
    void MAVLink_packetReceived(const mavlink_message_t &msg);

    AP_Int8     _enabled;
    AP_Int8     _obstacles_max;

    AP_Int8     _fail_action;
    AP_Int8     _fail_recovery;
    AP_Int8     _fail_time_horizon;
    AP_Int16    _fail_distance_xy;
    AP_Int16    _fail_distance_z;

    AP_Int8     _warn_action;
    AP_Int8     _warn_recovery;
    AP_Int8     _warn_time_horizon;
    AP_Float    _warn_distance_xy;
    AP_Float    _warn_distance_z;

    MAV_COLLISION_THREAT_LEVEL current_threat_level() const;

protected:

    enum state_t {
        STATE_CLEAR = 0,
        STATE_WARN = 1,
        STATE_FAIL = 2,
    };
    // contains English names corresponding to state_t entries
    static const char *_state_names[];

    uint32_t _last_state_change_ms;
    // we will not recover from a state any faster than this
    static const uint8_t _state_recovery_hysteresis = 20; // seconds
    state_t _old_state = STATE_CLEAR;

    enum avoidance_recovery_f_t {
        AVOIDANCE_RECOVERY_F_CONTINUE_FAIL,
        AVOIDANCE_RECOVERY_F_MOVE_TO_WARN,
    };
    enum avoidance_recovery_w_t {
        AVOIDANCE_RECOVERY_W_RESUME,
        AVOIDANCE_RECOVERY_W_CONTINUE_ACTION
    };

    uint32_t _gcs_cleared_messages_first_sent;
    // specifies how long we should continue sending messages about a threat after it has cleared
    static const uint8_t _gcs_cleared_messages_duration = 5; // seconds


    void handle_threat_gcs_notify(AP_Avoidance::Obstacle *threat);

    AP_Avoidance::Obstacle *most_serious_threat();

    // reference to AHRS, so we can ask for our position, heading and
    // speed
    const AP_AHRS &_ahrs;

    void internal_error();

    // Deal with the most significant threat
    virtual void handle_avoidance(AP_Avoidance::Obstacle *threat);

    // returns an entry from the MAV_COLLISION_ACTION representative
    // of what the curent avoidance handler is up to.
    MAV_COLLISION_ACTION mav_avoidance_action();

    // returns an object which is responsible for avoiding the most significant threat
    virtual class AvoidanceHandler &handler_for_action(MAV_COLLISION_ACTION action) = 0;

private:

    class AP_ADSB &_adsb;

    static AP_Avoidance *_instance;

    uint8_t _obstacles_allocated;
    uint8_t _obstacle_count;
    AP_Avoidance::Obstacle *_obstacles;

    // check to see if we are initialised (and possibly do initialisation)
    bool check_startup();

    // initialize _obstacle_list
    void init();

    // free _obstacle_list
    void deinit();

    const uint32_t MAX_OBSTACLE_AGE_MS = 60000;
//    const uint32_t MAX_OBSTACLE_AGE_MS = 1000; 

    uint32_t src_id_for_adsb_vehicle(AP_ADSB::adsb_vehicle_t vehicle) const;

    void check_for_threats();
    void update_threat_level(const Location &my_loc,
                             const Vector3f &my_vel,
                             AP_Avoidance::Obstacle &obstacle);

    // calls into the AP_ADSB library to retrieve vehicle data
    void get_adsb_samples();

    const static uint8_t _gcs_notify_interval = 1; // seconds

    // in SITL, at least, the threat level never remains higher for
    // more than a fraction of a second.  This could potentially lead
    // to an aircraft switching rapidly between its avoidance
    // action and its recovery action.
    const static uint8_t _minimum_handler_duration = 1; // seconds

    int8_t _current_most_serious_threat;

    class AvoidanceHandler *_current_avoidance_handler = nullptr;

    // returns true if the obstacle should be considered more of a
    // threat than the current most serious threat
    bool obstacle_is_more_serious_threat(const AP_Avoidance::Obstacle &obstacle) const;
};

float closest_distance_between_radial_and_point(const Vector2f &w,
                                                const Vector2f &p);
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
