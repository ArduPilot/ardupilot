#include "AP_Avoidance.h"

extern const AP_HAL::HAL& hal;

#include <limits>
#include <GCS_MAVLink/GCS.h>

#define AVOIDANCE_DEBUGGING 0

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    #define AP_AVOIDANCE_WARN_TIME_DEFAULT              30
    #define AP_AVOIDANCE_FAIL_TIME_DEFAULT              30
    #define AP_AVOIDANCE_WARN_DISTANCE_XY_DEFAULT       1000
    #define AP_AVOIDANCE_WARN_DISTANCE_Z_DEFAULT        300
    #define AP_AVOIDANCE_FAIL_DISTANCE_XY_DEFAULT       300
    #define AP_AVOIDANCE_FAIL_DISTANCE_Z_DEFAULT        100
    #define AP_AVOIDANCE_RECOVERY_DEFAULT               AP_AVOIDANCE_RECOVERY_RESUME_IF_AUTO_ELSE_LOITER
    #define AP_AVOIDANCE_FAIL_ACTION_DEFAULT            MAV_COLLISION_ACTION_REPORT
#else // APM_BUILD_TYPE(APM_BUILD_ArduCopter), Rover, Boat
    #define AP_AVOIDANCE_WARN_TIME_DEFAULT              30
    #define AP_AVOIDANCE_FAIL_TIME_DEFAULT              30
    #define AP_AVOIDANCE_WARN_DISTANCE_XY_DEFAULT       300
    #define AP_AVOIDANCE_WARN_DISTANCE_Z_DEFAULT        300
    #define AP_AVOIDANCE_FAIL_DISTANCE_XY_DEFAULT       100
    #define AP_AVOIDANCE_FAIL_DISTANCE_Z_DEFAULT        100
    #define AP_AVOIDANCE_RECOVERY_DEFAULT               AP_AVOIDANCE_RECOVERY_RTL
    #define AP_AVOIDANCE_FAIL_ACTION_DEFAULT            MAV_COLLISION_ACTION_REPORT
#endif

#if AVOIDANCE_DEBUGGING
#include <stdio.h>
#define debug(fmt, args ...)  do {::fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Avoidance::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Avoidance using ADSB
    // @Description: Enable Avoidance using ADSB
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Avoidance, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: F_ACTION
    // @DisplayName: Collision Avoidance Behavior
    // @Description: Specifies aircraft behaviour when a collision is imminent
    // The following values should come from the mavlink COLLISION_ACTION enum
    // @Values: 0:None,1:Report,2:Climb Or Descend,3:Move Horizontally,4:Move Perpendicularly in 3D,5:RTL,6:Hover
    // @User: Advanced
    AP_GROUPINFO("F_ACTION",    2, AP_Avoidance, _fail_action, AP_AVOIDANCE_FAIL_ACTION_DEFAULT),

    // @Param: W_ACTION
    // @DisplayName: Collision Avoidance Behavior - Warn
    // @Description: Specifies aircraft behaviour when a collision may occur
    // The following values should come from the mavlink COLLISION_ACTION enum
    // @Values: 0:None,1:Report
    // @User: Advanced
    AP_GROUPINFO("W_ACTION",    3, AP_Avoidance, _warn_action, MAV_COLLISION_ACTION_REPORT),

    // @Param: F_RCVRY
    // @DisplayName: Recovery behaviour after a fail event
    // @Description: Determines what the aircraft will do after a fail event is resolved
    // @Values: 0:Remain in AVOID_ADSB,1:Resume previous flight mode,2:RTL,3:Resume if AUTO else Loiter
    // @User: Advanced
    AP_GROUPINFO("F_RCVRY",     4, AP_Avoidance, _fail_recovery, AP_AVOIDANCE_RECOVERY_DEFAULT),

    // @Param: OBS_MAX
    // @DisplayName: Maximum number of obstacles to track
    // @Description: Maximum number of obstacles to track
    // @User: Advanced
    AP_GROUPINFO("OBS_MAX",     5, AP_Avoidance, _obstacles_max, 20),

    // @Param: W_TIME
    // @DisplayName: Time Horizon Warn
    // @Description: Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than W_DIST_XY or W_DIST_Z then W_ACTION is undertaken (assuming F_ACTION is not undertaken)
    // @User: Advanced
    AP_GROUPINFO("W_TIME",      6, AP_Avoidance, _warn_time_horizon, AP_AVOIDANCE_WARN_TIME_DEFAULT),

    // @Param: F_TIME
    // @DisplayName: Time Horizon Fail
    // @Description: Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than F_DIST_XY or F_DIST_Z then F_ACTION is undertaken
    // @User: Advanced
    AP_GROUPINFO("F_TIME",      7, AP_Avoidance, _fail_time_horizon, AP_AVOIDANCE_FAIL_TIME_DEFAULT),

    // @Param: W_DIST_XY
    // @DisplayName: Distance Warn XY
    // @Description: Closest allowed projected distance before W_ACTION is undertaken
    // @User: Advanced
    AP_GROUPINFO("W_DIST_XY",   8, AP_Avoidance, _warn_distance_xy, AP_AVOIDANCE_WARN_DISTANCE_XY_DEFAULT),

    // @Param: F_DIST_XY
    // @DisplayName: Distance Fail XY
    // @Description: Closest allowed projected distance before F_ACTION is undertaken
    // @User: Advanced
    AP_GROUPINFO("F_DIST_XY",   9, AP_Avoidance, _fail_distance_xy, AP_AVOIDANCE_FAIL_DISTANCE_XY_DEFAULT),

    // @Param: W_DIST_Z
    // @DisplayName: Distance Warn Z
    // @Description: Closest allowed projected distance before BEHAVIOUR_W is undertaken
    // @User: Advanced
    AP_GROUPINFO("W_DIST_Z",    10, AP_Avoidance, _warn_distance_z, AP_AVOIDANCE_WARN_DISTANCE_Z_DEFAULT),

    // @Param: F_DIST_Z
    // @DisplayName: Distance Fail Z
    // @Description: Closest allowed projected distance before BEHAVIOUR_F is undertaken
    // @User: Advanced
    AP_GROUPINFO("F_DIST_Z",    11, AP_Avoidance, _fail_distance_z, AP_AVOIDANCE_FAIL_DISTANCE_Z_DEFAULT),

    AP_GROUPEND
};

AP_Avoidance::AP_Avoidance(AP_AHRS &ahrs, AP_ADSB &adsb) :
    _ahrs(ahrs),
    _adsb(adsb)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * Initialize variables and allocate memory for array
 */
void AP_Avoidance::init(void)
{
    debug("ADSB initialisation: %d obstacles", _obstacles_max.get());
    if (_obstacles == NULL) {
        _obstacles = new AP_Avoidance::Obstacle[_obstacles_max];

        if (_obstacles == NULL) {
            // dynamic RAM allocation of _obstacles[] failed, disable gracefully
            hal.console->printf("Unable to initialize Avoidance obstacle list\n");
            // disable ourselves to avoid repeated allocation attempts
            _enabled.set(0);
            return;
        }
        _obstacles_allocated = _obstacles_max;
    }
    _obstacle_count = 0;
    _last_state_change_ms = 0;
    _threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;
    _gcs_cleared_messages_first_sent = std::numeric_limits<uint32_t>::max();
    _current_most_serious_threat = -1;
}

/*
 * de-initialize and free up some memory
 */
void AP_Avoidance::deinit(void)
{
    if (_obstacles != nullptr) {
        delete [] _obstacles;
        _obstacles = nullptr;
        _obstacles_allocated = 0;
        handle_recovery(AP_AVOIDANCE_RECOVERY_RTL);
    }
    _obstacle_count = 0;
}

bool AP_Avoidance::check_startup()
{
    if (!_enabled) {
        if (_obstacles != nullptr) {
            deinit();
        }
        // nothing to do
        return false;
    }
    if (_obstacles == nullptr)  {
        init();
    }
    return _obstacles != nullptr;
}

// vel is north/east/down!
void AP_Avoidance::add_obstacle(const uint32_t obstacle_timestamp_ms,
                                const MAV_COLLISION_SRC src,
                                const uint32_t src_id,
                                const Location &loc,
                                const Vector3f &vel_ned)
{
    if (! check_startup()) {
        return;
    }
    uint32_t oldest_timestamp = std::numeric_limits<uint32_t>::max();
    uint8_t oldest_index = 255; // avoid compiler warning with initialisation
    int16_t index = -1;
    uint8_t i;
    for (i=0; i<_obstacle_count; i++) {
        if (_obstacles[i].src_id == src_id &&
            _obstacles[i].src == src) {
            // pre-existing obstacle found; we will update its information
            index = i;
            break;
        }
        if (_obstacles[i].timestamp_ms < oldest_timestamp) {
            oldest_timestamp = _obstacles[i].timestamp_ms;
            oldest_index = i;
        }
    }
    if (index == -1) {
        // existing obstacle not found.  See if we can store it anyway:
        if (i <_obstacles_allocated) {
            // have room to store more vehicles...
            index = _obstacle_count++;
        } else if (oldest_timestamp < obstacle_timestamp_ms) {
            // replace this very old entry with this new data
            index = oldest_index;
        }
        _obstacles[index].src = src;
        _obstacles[index].src_id = src_id;
    }

    if (index == -1) {
        // no room for this (old?!) data
        return;
    }
    _obstacles[index]._location = loc;
    _obstacles[index]._velocity = vel_ned;
    _obstacles[index].timestamp_ms = obstacle_timestamp_ms;
}

void AP_Avoidance::add_obstacle(const uint32_t obstacle_timestamp_ms,
                                const MAV_COLLISION_SRC src,
                                const uint32_t src_id,
                                const Location &loc,
                                const float cog,
                                const float hspeed,
                                const float vspeed)
{
    Vector3f vel;
    vel[0] = hspeed * cosf(radians(cog));
    vel[1] = hspeed * sinf(radians(cog));
    vel[2] = vspeed;
    // debug("cog=%f hspeed=%f veln=%f vele=%f", cog, hspeed, vel[0], vel[1]);
    return add_obstacle(obstacle_timestamp_ms, src, src_id, loc, vel);
}

uint32_t AP_Avoidance::src_id_for_adsb_vehicle(AP_ADSB::adsb_vehicle_t vehicle) const
{
    // TODO: need to include squawk code and callsign
    return vehicle.info.ICAO_address;
}

void AP_Avoidance::get_adsb_samples()
{
    AP_ADSB::adsb_vehicle_t vehicle;
    while (_adsb.next_sample(vehicle)) {
        uint32_t src_id = src_id_for_adsb_vehicle(vehicle);
        Location loc = _adsb.get_location(vehicle);
        add_obstacle(vehicle.last_update_ms,
                   MAV_COLLISION_SRC_ADSB,
                   src_id,
                   loc,
                   vehicle.info.heading/100.0f,
                   vehicle.info.hor_velocity/100.0f,
                   -vehicle.info.ver_velocity/1000.0f); // convert mm-up to m-down
    }
}

float closest_approach_xy(const Location &my_loc,
                          const Vector3f &my_vel,
                          const Location &obstacle_loc,
                          const Vector3f &obstacle_vel,
                          const uint8_t time_horizon)
{

    Vector2f delta_vel_ne = Vector2f(obstacle_vel[0] - my_vel[0], obstacle_vel[1] - my_vel[1]);
    Vector2f delta_pos_ne = location_diff(obstacle_loc, my_loc);

    Vector2f line_segment_ne = delta_vel_ne * time_horizon;

    float ret = Vector2<float>::closest_distance_between_radial_and_point
        (line_segment_ne,
         delta_pos_ne);

    debug("   time_horizon: (%d)", time_horizon);
    debug("   delta pos: (y=%f,x=%f)", delta_pos_ne[0], delta_pos_ne[1]);
    debug("   delta vel: (y=%f,x=%f)", delta_vel_ne[0], delta_vel_ne[1]);
    debug("   line segment: (y=%f,x=%f)", line_segment_ne[0], line_segment_ne[1]);
    debug("   closest: (%f)", ret);

    return ret;
}

// returns the closest these objects will get in the body z axis (in metres)
float closest_approach_z(const Location &my_loc,
                         const Vector3f &my_vel,
                         const Location &obstacle_loc,
                         const Vector3f &obstacle_vel,
                         const uint8_t time_horizon)
{

    float delta_vel_d = obstacle_vel[2] - my_vel[2];
    float delta_pos_d = obstacle_loc.alt - my_loc.alt;

    float ret;
    if (delta_pos_d >= 0 && delta_vel_d >= 0) {
        ret = delta_pos_d;
    } else if (delta_pos_d <= 0 && delta_vel_d <= 0) {
        ret = fabs(delta_pos_d);
    } else {
        ret = fabs(delta_pos_d - delta_vel_d * time_horizon);
    }

    debug("   time_horizon: (%d)", time_horizon);
    debug("   delta pos: (%f) metres", delta_pos_d/100.0f);
    debug("   delta vel: (%f) m/s", delta_vel_d);
    debug("   closest: (%f) metres", ret/100.0f);

    return ret/100.0f;
}

void AP_Avoidance::update_threat_level(const Location &my_loc,
                                       const Vector3f &my_vel,
                                       AP_Avoidance::Obstacle &obstacle)
{

    Location &obstacle_loc = obstacle._location;
    Vector3f &obstacle_vel = obstacle._velocity;

    obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;

    const uint32_t obstacle_age = AP_HAL::millis() - obstacle.timestamp_ms;
    float closest_xy = closest_approach_xy(my_loc, my_vel, obstacle_loc, obstacle_vel, _fail_time_horizon + obstacle_age/1000);
    if (closest_xy < _fail_distance_xy) {
        obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_HIGH;
    } else {
        closest_xy = closest_approach_xy(my_loc, my_vel, obstacle_loc, obstacle_vel, _warn_time_horizon + obstacle_age/1000);
        if (closest_xy < _warn_distance_xy) {
            obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_LOW;
        }
    }

    // check for vertical separation; our threat level is the minimum
    // of vertical and horizontal threat levels
    float closest_z = closest_approach_z(my_loc, my_vel, obstacle_loc, obstacle_vel, _warn_time_horizon + obstacle_age/1000);
    if (obstacle.threat_level != MAV_COLLISION_THREAT_LEVEL_NONE) {
        if (closest_z > _warn_distance_z) {
            obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;
        } else {
            closest_z = closest_approach_z(my_loc, my_vel, obstacle_loc, obstacle_vel, _fail_time_horizon + obstacle_age/1000);
            if (closest_z > _fail_distance_z) {
                obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_LOW;
            }
        }
    }

    // If we haven't heard from a vehicle then assume it is no threat
    if (obstacle_age > MAX_OBSTACLE_AGE_MS) {
        obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;
    }

    // could optimise this to not calculate a lot of this if threat
    // level is none - but only *once the GCS has been informed*!
    obstacle.closest_approach_xy = closest_xy;
    obstacle.closest_approach_z = closest_z;
    float current_distance = get_distance(my_loc, obstacle_loc);
    obstacle.distance_to_closest_approach = current_distance - closest_xy;
    Vector2f net_velocity_ne = Vector2f(my_vel[0] - obstacle_vel[0], my_vel[1] - obstacle_vel[1]);
    obstacle.time_to_closest_approach = 0.0f;
    if (!is_zero(obstacle.distance_to_closest_approach) &&
        ! is_zero(net_velocity_ne.length())) {
        obstacle.time_to_closest_approach = obstacle.distance_to_closest_approach / net_velocity_ne.length();
    }
}

MAV_COLLISION_THREAT_LEVEL AP_Avoidance::current_threat_level() const {
    if (_obstacles == nullptr) {
        return MAV_COLLISION_THREAT_LEVEL_NONE;
    }
    if (_current_most_serious_threat == -1) {
        return MAV_COLLISION_THREAT_LEVEL_NONE;
    }
    return _obstacles[_current_most_serious_threat].threat_level;
}

void AP_Avoidance::handle_threat_gcs_notify(AP_Avoidance::Obstacle *threat)
{
    if (threat == nullptr) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    if (threat->threat_level == MAV_COLLISION_THREAT_LEVEL_NONE) {
        // only send cleared messages for a few seconds:
        if (_gcs_cleared_messages_first_sent == 0) {
            _gcs_cleared_messages_first_sent = now;
        }
        if (now - _gcs_cleared_messages_first_sent > _gcs_cleared_messages_duration * 1000) {
            return;
        }
    } else {
        _gcs_cleared_messages_first_sent = 0;
    }
    if (now - threat->last_gcs_report_time > _gcs_notify_interval * 1000) {
        GCS_MAVLINK::send_collision_all(*threat, mav_avoidance_action());
        threat->last_gcs_report_time = now;
    }

}

bool AP_Avoidance::obstacle_is_more_serious_threat(const AP_Avoidance::Obstacle &obstacle) const
{
    if (_current_most_serious_threat == -1) {
        // any threat is more of a threat than no threat
        return true;
    }
    const AP_Avoidance::Obstacle &current = _obstacles[_current_most_serious_threat];
    if (obstacle.threat_level > current.threat_level) {
        // threat_level is updated by update_threat_level
        return true;
    }
    if (obstacle.threat_level == current.threat_level &&
        obstacle.time_to_closest_approach < current.time_to_closest_approach) {
        return true;
    }
    return false;
}

void AP_Avoidance::check_for_threats()
{
    Location my_loc;
    if (!_ahrs.get_position(my_loc)) {
        // if we don't know our own location we can't determine any threat level
        return;
    }

    Vector3f my_vel;
    if (!_ahrs.get_velocity_NED(my_vel)) {
        // assuming our own velocity to be zero here may cause us to
        // fly into something.  Better not to attempt to avoid in this
        // case.
        return;
    }

    // we always check all obstacles to see if they are threats since it
    // is most likely our own position and/or velocity have changed
    // determine the current most-serious-threat
    _current_most_serious_threat = -1;
    for (uint8_t i=0; i<_obstacle_count; i++) {

        AP_Avoidance::Obstacle &obstacle = _obstacles[i];
        const uint32_t obstacle_age = AP_HAL::millis() - obstacle.timestamp_ms;
        debug("i=%d src_id=%d timestamp=%u age=%d", i, obstacle.src_id, obstacle.timestamp_ms, obstacle_age);

        update_threat_level(my_loc, my_vel, obstacle);
        debug("   threat-level=%d", obstacle.threat_level);

        // ignore any really old data:
        if (obstacle_age > MAX_OBSTACLE_AGE_MS) {
            // shrink list if this is the last entry:
            if (i == _obstacle_count-1) {
                _obstacle_count -= 1;
            }
            continue;
        }

        if (obstacle_is_more_serious_threat(obstacle)) {
            _current_most_serious_threat = i;
        }
    }
    if (_current_most_serious_threat != -1) {
        debug("Current most serious threat: %d level=%d", _current_most_serious_threat, _obstacles[_current_most_serious_threat].threat_level);
    }
}


AP_Avoidance::Obstacle *AP_Avoidance::most_serious_threat()
{
    if (_current_most_serious_threat < 0) {
        // we *really_ should not have been called!
        return nullptr;
    }
    return &_obstacles[_current_most_serious_threat];
}


void AP_Avoidance::update()
{
    if (!check_startup()) {
        return;
    }

    if (_adsb.enabled()) {
        get_adsb_samples();
    }

    check_for_threats();

    // notify GCS of most serious thread
    handle_threat_gcs_notify(most_serious_threat());

    // avoid object (if necessary)
    handle_avoidance_local(most_serious_threat());
}

void AP_Avoidance::handle_avoidance_local(AP_Avoidance::Obstacle *threat)
{
    MAV_COLLISION_THREAT_LEVEL new_threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;
    MAV_COLLISION_ACTION action = MAV_COLLISION_ACTION_NONE;

    if (threat != nullptr) {
        new_threat_level = threat->threat_level;
        if (new_threat_level == MAV_COLLISION_THREAT_LEVEL_HIGH) {
            action = (MAV_COLLISION_ACTION)_fail_action.get();
        }
    }

    uint32_t now = AP_HAL::millis();

    if (new_threat_level != _threat_level) {
        // transition to higher states immediately, recovery to lower states more slowly
        if (((now - _last_state_change_ms) > AP_AVOIDANCE_STATE_RECOVERY_TIME_MS) || (new_threat_level > _threat_level)) {
            // handle recovery from high threat level
            if (_threat_level == MAV_COLLISION_THREAT_LEVEL_HIGH) {
                handle_recovery(_fail_recovery);
                _latest_action = MAV_COLLISION_ACTION_NONE;
            }

            // update state
            _last_state_change_ms = now;
            _threat_level = new_threat_level;
        }
    }

    // handle ongoing threat by calling vehicle specific handler
    if ((threat != nullptr) && (_threat_level == MAV_COLLISION_THREAT_LEVEL_HIGH) && (action > MAV_COLLISION_ACTION_REPORT)) {
        _latest_action = handle_avoidance(threat, action);
    }
}


void AP_Avoidance::handle_msg(const mavlink_message_t &msg)
{
    if (!check_startup()) {
        // avoidance is not active / allocated
        return;
    }

    if (msg.msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        // we only take position from GLOBAL_POSITION_INT
        return;
    }

    if (msg.sysid == mavlink_system.sysid) {
        // we do not obstruct ourselves....
        return;
    }

    // inform AP_Avoidance we have a new player
    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&msg, &packet);
    Location loc;
    loc.lat = packet.lat;
    loc.lng = packet.lon;
    loc.alt = packet.alt / 10; // mm -> cm
    loc.flags.relative_alt = false;
    Vector3f vel = Vector3f(packet.vx/100.0f, // cm to m
                            packet.vy/100.0f,
                            packet.vz/100.0f);
    add_obstacle(AP_HAL::millis(),
                 MAV_COLLISION_SRC_ADSB,
                 msg.sysid,
                 loc,
                 vel);
}

// wp_speeds in cm/s
bool AP_Avoidance::get_destination_perpendicular(const AP_Avoidance::Obstacle *obstacle, Vector3f &newdest_neu, const float wp_speed_xy, const float wp_speed_z, const uint8_t _minimum_avoid_height)
{
    if (obstacle == nullptr) {
        // why where we called?!
        return false;
    }

    Location my_abs_pos;
    if (! _ahrs.get_position(my_abs_pos)) {
        // we should not get to here!  If we don't know our position
        // we can't know if there are any threats, for starters!
        return false;
    }

    Vector3f my_pos_ned;
    if (! _ahrs.get_relative_position_NED(my_pos_ned)) {
        // we should not get to here!  If we don't know our position
        // we know if there are any threats, for starters!
        return false;
    }

    // if their velocity is moving around close to zero then flying
    // perpendicular to that velocity may mean we do weird things.
    // Instead, we will fly directly away from them:
    if (obstacle->_velocity.length() < _low_velocity_threshold) {
        const Vector2f delta_pos_xy =  location_diff(obstacle->_location, my_abs_pos);
        const float delta_pos_z = my_abs_pos.alt - obstacle->_location.alt;
        Vector3f delta_pos_xyz = Vector3f(delta_pos_xy[0],delta_pos_xy[1],delta_pos_z);
        // avoid divide by zero
        if (delta_pos_xyz.is_zero()) {
            return false;
        }
        delta_pos_xyz.normalize();
        newdest_neu[0] = my_pos_ned[0]*100 + delta_pos_xyz[0] * wp_speed_xy * AP_AVOIDANCE_ESCAPE_TIME_SEC;
        newdest_neu[1] = my_pos_ned[1]*100 + delta_pos_xyz[1] * wp_speed_xy * AP_AVOIDANCE_ESCAPE_TIME_SEC;
        newdest_neu[2] = -my_pos_ned[2]*100 + delta_pos_xyz[2] * wp_speed_z * AP_AVOIDANCE_ESCAPE_TIME_SEC;
        if(newdest_neu[2] < _minimum_avoid_height*100) {
            newdest_neu[0] = my_pos_ned[0]*100 + delta_pos_xy[0] * wp_speed_xy * AP_AVOIDANCE_ESCAPE_TIME_SEC;
            newdest_neu[1] = my_pos_ned[1]*100 + delta_pos_xy[1] * wp_speed_xy * AP_AVOIDANCE_ESCAPE_TIME_SEC;
            newdest_neu[2] = -my_pos_ned[2]*100;
        }
        return true;
    }

    {
        Vector3f perp_xyz = perpendicular_xyz(obstacle->_location, obstacle->_velocity, my_abs_pos);
        perp_xyz.normalize();
        newdest_neu[0] = my_pos_ned[0]*100 + perp_xyz[0] * wp_speed_xy * AP_AVOIDANCE_ESCAPE_TIME_SEC;
        newdest_neu[1] = my_pos_ned[1]*100 + perp_xyz[1] * wp_speed_xy * AP_AVOIDANCE_ESCAPE_TIME_SEC;
        newdest_neu[2] =  -my_pos_ned[2]*100 + perp_xyz[2] * wp_speed_z * AP_AVOIDANCE_ESCAPE_TIME_SEC;
    }

    if (newdest_neu[2] < _minimum_avoid_height*100) {
        // too close to the ground to do 3D avoidance
        // GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "AVOID: PERPENDICULAR: 2D");
        Vector2f perp_xy = perpendicular_xy(obstacle->_location, obstacle->_velocity, my_abs_pos);
        perp_xy.normalize();
        newdest_neu[0] = my_pos_ned[0]*100 + perp_xy[0] * wp_speed_xy * AP_AVOIDANCE_ESCAPE_TIME_SEC;
        newdest_neu[1] = my_pos_ned[1]*100 + perp_xy[1] * wp_speed_xy * AP_AVOIDANCE_ESCAPE_TIME_SEC;
        newdest_neu[2] = -my_pos_ned[2]*100;
    }

    return true;
}

// get unit vector away from the nearest obstacle
bool AP_Avoidance::get_vector_perpendicular(const AP_Avoidance::Obstacle *obstacle, Vector3f &vec_neu)
{
    if (obstacle == nullptr) {
        // why where we called?!
        return false;
    }

    Location my_abs_pos;
    if (!_ahrs.get_position(my_abs_pos)) {
        // we should not get to here!  If we don't know our position
        // we can't know if there are any threats, for starters!
        return false;
    }

    // if their velocity is moving around close to zero then flying
    // perpendicular to that velocity may mean we do weird things.
    // Instead, we will fly directly away from them
    if (obstacle->_velocity.length() < _low_velocity_threshold) {
        const Vector2f delta_pos_xy =  location_diff(obstacle->_location, my_abs_pos);
        const float delta_pos_z = my_abs_pos.alt - obstacle->_location.alt;
        Vector3f delta_pos_xyz = Vector3f(delta_pos_xy.x, delta_pos_xy.y, delta_pos_z);
        // avoid div by zero
        if (delta_pos_xyz.is_zero()) {
            return false;
        }
        delta_pos_xyz.normalize();
        vec_neu = delta_pos_xyz;
        return true;
    } else {
        vec_neu = perpendicular_xyz(obstacle->_location, obstacle->_velocity, my_abs_pos);
        // avoid div by zero
        if (vec_neu.is_zero()) {
            return false;
        }
        vec_neu.normalize();
        return true;
    }
}

// helper functions to calculate 3D destination to get us away from obstacle
// v1 is NED
Vector3f AP_Avoidance::perpendicular_xyz(const Location &p1, const Vector3f &v1, const Location &p2)
{
    Vector2f delta_p_2d = location_diff(p1, p2);
    Vector3f delta_p_xyz = Vector3f(delta_p_2d[0],delta_p_2d[1],(p2.alt-p1.alt)/100.0f); //check this line
    Vector3f v1_xyz = Vector3f(v1[0], v1[1], -v1[2]);
    Vector3f ret = Vector3f::perpendicular(delta_p_xyz, v1_xyz);
    return ret;
}

// helper functions to calculate horizontal destination to get us away from obstacle
// v1 is NED
Vector2f AP_Avoidance::perpendicular_xy(const Location &p1, const Vector3f &v1, const Location &p2)
{
    Vector2f delta_p = location_diff(p1, p2);
    Vector2f delta_p_n = Vector2f(delta_p[0],delta_p[1]);
    Vector2f v1n(v1[0],v1[1]);
    Vector2f ret_xy = Vector2f::perpendicular(delta_p_n, v1n);
    return ret_xy;
}
