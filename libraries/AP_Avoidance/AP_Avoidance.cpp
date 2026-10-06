#include "AP_Avoidance_config.h"

#if AP_ADSB_AVOIDANCE_ENABLED

#include "AP_Avoidance.h"

extern const AP_HAL::HAL& hal;

#include <limits>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#define AVOIDANCE_DEBUGGING 0

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    #define AP_AVOIDANCE_WARN_TIME_DEFAULT              30
    #define AP_AVOIDANCE_FAIL_TIME_DEFAULT              30
    #define AP_AVOIDANCE_WARN_DISTANCE_XY_DEFAULT       1000
    #define AP_AVOIDANCE_WARN_DISTANCE_Z_DEFAULT        300
    #define AP_AVOIDANCE_FAIL_DISTANCE_XY_DEFAULT       300
    #define AP_AVOIDANCE_FAIL_DISTANCE_Z_DEFAULT        100
    #define AP_AVOIDANCE_RECOVERY_DEFAULT               RecoveryAction::RESUME_IF_AUTO_ELSE_LOITER
    #define AP_AVOIDANCE_FAIL_ACTION_DEFAULT            MAV_COLLISION_ACTION_REPORT
#else // APM_BUILD_TYPE(APM_BUILD_ArduCopter),Heli, Rover, Boat
    #define AP_AVOIDANCE_WARN_TIME_DEFAULT              30
    #define AP_AVOIDANCE_FAIL_TIME_DEFAULT              30
    #define AP_AVOIDANCE_WARN_DISTANCE_XY_DEFAULT       300
    #define AP_AVOIDANCE_WARN_DISTANCE_Z_DEFAULT        300
    #define AP_AVOIDANCE_FAIL_DISTANCE_XY_DEFAULT       100
    #define AP_AVOIDANCE_FAIL_DISTANCE_Z_DEFAULT        100
    #define AP_AVOIDANCE_RECOVERY_DEFAULT               RecoveryAction::RTL
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
    // @Values: 0:None,1:Report,2:Climb Or Descend,3:Move Horizontally,4:Move Perpendicularly in 3D,5:RTL,6:Hover
    // @User: Advanced
    AP_GROUPINFO("F_ACTION",    2, AP_Avoidance, _fail_action, AP_AVOIDANCE_FAIL_ACTION_DEFAULT),

    // @Param: W_ACTION
    // @DisplayName: Collision Avoidance Behavior - Warn
    // @Description: Specifies aircraft behaviour when a collision may occur
    // @Values: 0:None,1:Report
    // @User: Advanced
    AP_GROUPINFO("W_ACTION",    3, AP_Avoidance, _warn_action, MAV_COLLISION_ACTION_REPORT),

    // @Param: F_RCVRY
    // @DisplayName: Recovery behaviour after a fail event
    // @Description: Determines what the aircraft will do after a fail event is resolved
    // @Values: 0:Remain in AVOID_ADSB,1:Resume previous flight mode,2:RTL,3:Resume if AUTO else Loiter
    // @User: Advanced
    AP_GROUPINFO("F_RCVRY",     4, AP_Avoidance, _fail_recovery, uint8_t(AP_AVOIDANCE_RECOVERY_DEFAULT)),

    // @Param: OBS_MAX
    // @DisplayName: Maximum number of obstacles to track
    // @Description: Maximum number of obstacles to track
    // @User: Advanced
    AP_GROUPINFO("OBS_MAX",     5, AP_Avoidance, _obstacles_max, 20),

    // @Param: W_TIME
    // @DisplayName: Time Horizon Warn
    // @Description: Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than W_DIST_XY or W_DIST_Z then W_ACTION is undertaken (assuming F_ACTION is not undertaken)
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("W_TIME",      6, AP_Avoidance, _warn_time_horizon_s, AP_AVOIDANCE_WARN_TIME_DEFAULT),

    // @Param: F_TIME
    // @DisplayName: Time Horizon Fail
    // @Description: Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than F_DIST_XY or F_DIST_Z then F_ACTION is undertaken
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("F_TIME",      7, AP_Avoidance, _fail_time_horizon_s, AP_AVOIDANCE_FAIL_TIME_DEFAULT),

    // @Param: W_DIST_XY
    // @DisplayName: Distance Warn XY
    // @Description: Closest allowed projected distance before W_ACTION is undertaken
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("W_DIST_XY",   8, AP_Avoidance, _warn_distance_ne_m, AP_AVOIDANCE_WARN_DISTANCE_XY_DEFAULT),

    // @Param: F_DIST_XY
    // @DisplayName: Distance Fail XY
    // @Description: Closest allowed projected distance before F_ACTION is undertaken
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("F_DIST_XY",   9, AP_Avoidance, _fail_distance_ne_m, AP_AVOIDANCE_FAIL_DISTANCE_XY_DEFAULT),

    // @Param: W_DIST_Z
    // @DisplayName: Distance Warn Z
    // @Description: Closest allowed projected distance before BEHAVIOUR_W is undertaken
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("W_DIST_Z",    10, AP_Avoidance, _warn_distance_d_m, AP_AVOIDANCE_WARN_DISTANCE_Z_DEFAULT),

    // @Param: F_DIST_Z
    // @DisplayName: Distance Fail Z
    // @Description: Closest allowed projected distance before BEHAVIOUR_F is undertaken
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("F_DIST_Z",    11, AP_Avoidance, _fail_distance_d_m, AP_AVOIDANCE_FAIL_DISTANCE_Z_DEFAULT),
    
    // @Param: F_ALT_MIN
    // @DisplayName: ADS-B avoidance minimum altitude
    // @Description: Minimum AMSL (above mean sea level) altitude for ADS-B avoidance. If the vehicle is below this altitude, no avoidance action will take place. Useful to prevent ADS-B avoidance from activating while below the tree line or around structures. Default of 0 is no minimum.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("F_ALT_MIN",    12, AP_Avoidance, _fail_altitude_min_m, 0),

    AP_GROUPEND
};

AP_Avoidance::AP_Avoidance(AP_ADSB &adsb) :
    _adsb(adsb)
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Avoidance must be singleton");
    }
    _singleton = this;
}

/*
 * Initialize variables and allocate memory for array
 */
void AP_Avoidance::init(void)
{
    debug("ADSB initialisation: %d obstacles", _obstacles_max.get());
    if (_obstacles == nullptr) {
        _obstacles = NEW_NOTHROW AP_Avoidance::Obstacle[_obstacles_max];

        if (_obstacles == nullptr) {
            // dynamic RAM allocation of _obstacles[] failed, disable gracefully
            DEV_PRINTF("Unable to initialize Avoidance obstacle list\n");
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
        handle_recovery(RecoveryAction::RTL);
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

// vel_ned_ms is north/east/down
void AP_Avoidance::add_obstacle(const uint32_t obstacle_timestamp_ms,
                                const MAV_COLLISION_SRC src,
                                const uint32_t src_id,
                                const Location &loc,
                                const Vector3f &vel_ned_ms)
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
    WITH_SEMAPHORE(_rsem);
    
    if (index == -1) {
        // existing obstacle not found.  See if we can store it anyway:
        if (i <_obstacles_allocated) {
            // have room to store more vehicles...
            index = _obstacle_count++;
        } else if (oldest_timestamp < obstacle_timestamp_ms) {
            // replace this very old entry with this new data
            index = oldest_index;
        } else {
            // no room for this (old?!) data
            return;
        }

        _obstacles[index].src = src;
        _obstacles[index].src_id = src_id;
    }

    _obstacles[index]._location = loc;
    _obstacles[index]._velocity_ned_ms = vel_ned_ms;
    _obstacles[index].timestamp_ms = obstacle_timestamp_ms;
}

void AP_Avoidance::add_obstacle(const uint32_t obstacle_timestamp_ms,
                                const MAV_COLLISION_SRC src,
                                const uint32_t src_id,
                                const Location &loc,
                                const float cog,
                                const float speed_ne_ms,
                                const float speed_d_ms)
{
    Vector3f vel_ned_ms;
    vel_ned_ms[0] = speed_ne_ms * cosf(radians(cog));
    vel_ned_ms[1] = speed_ne_ms * sinf(radians(cog));
    vel_ned_ms[2] = speed_d_ms;
    // debug("cog=%f speed_ne_ms=%f veln=%f vele=%f", cog, speed_ne_ms, vel_ned_ms[0], vel[1]);
    return add_obstacle(obstacle_timestamp_ms, src, src_id, loc, vel_ned_ms);
}

uint32_t AP_Avoidance::src_id_for_adsb_vehicle(const AP_ADSB::adsb_vehicle_t &vehicle) const
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
                   vehicle.info.heading * 0.01,
                   vehicle.info.hor_velocity * 0.01,
                   -vehicle.info.ver_velocity * 0.01); // convert cm-up to m-down
    }
}

float closest_approach_NE_m(const Location &loc,
                          const Vector3f &vel_ned_ms,
                          const Location &obstacle_loc,
                          const Vector3f &obstacle_vel_ned_ms,
                          const uint8_t time_horizon_s)
{

    Vector2f delta_vel_ne_ms = Vector2f(obstacle_vel_ned_ms[0] - vel_ned_ms[0], obstacle_vel_ned_ms[1] - vel_ned_ms[1]);
    const Vector2f delta_pos_ne_m = obstacle_loc.get_distance_NE(loc);

    Vector2f line_segment_ne_m = delta_vel_ne_ms * time_horizon_s;

    float dist_ne_m = Vector2<float>::closest_distance_between_radial_and_point
        (line_segment_ne_m,
         delta_pos_ne_m);

    debug("   time_horizon: (%d)", time_horizon_s);
    debug("   delta pos: (y=%f,x=%f)", delta_pos_ne_m[0], delta_pos_ne_m[1]);
    debug("   delta vel: (y=%f,x=%f)", delta_vel_ne_ms[0], delta_vel_ne_ms[1]);
    debug("   line segment: (y=%f,x=%f)", line_segment_ne_m[0], line_segment_ne_m[1]);
    debug("   closest: (%f)", dist_ne_m);

    return dist_ne_m;
}

// returns the closest these objects will get in the body z axis (in metres)
float closest_approach_D_m(const Location &loc,
                         const Vector3f &vel_ned_ms,
                         const Location &obstacle_loc,
                         const Vector3f &obstacle_vel_ned_ms,
                         const uint8_t time_horizon_s)
{

    float delta_vel_d_ms = obstacle_vel_ned_ms[2] - vel_ned_ms[2];
    float delta_pos_d_cm = obstacle_loc.alt - loc.alt;

    float dist_d_cm;
    if (delta_pos_d_cm >= 0 && delta_vel_d_ms >= 0) {
        dist_d_cm = delta_pos_d_cm;
    } else if (delta_pos_d_cm <= 0 && delta_vel_d_ms <= 0) {
        dist_d_cm = fabsf(delta_pos_d_cm);
    } else {
        dist_d_cm = fabsf(delta_pos_d_cm - delta_vel_d_ms * time_horizon_s * 100.0);
    }

    debug("   time_horizon: (%d)", time_horizon_s);
    debug("   delta pos: (%f) metres", delta_pos_d_cm*0.01f);
    debug("   delta vel: (%f) m/s", delta_vel_d_ms);
    debug("   closest: (%f) metres", dist_d_cm*0.01f);

    return dist_d_cm * 0.01f;
}

void AP_Avoidance::update_threat_level(const Location &loc,
                                       const Vector3f &vel_ned_ms,
                                       AP_Avoidance::Obstacle &obstacle)
{

    Location &obstacle_loc = obstacle._location;
    Vector3f &obstacle_vel_ned_ms = obstacle._velocity_ned_ms;

    obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;

    const uint32_t obstacle_age_ms = AP_HAL::millis() - obstacle.timestamp_ms;
    float closest_ne_m = closest_approach_NE_m(loc, vel_ned_ms, obstacle_loc, obstacle_vel_ned_ms, _fail_time_horizon_s + obstacle_age_ms/1000);
    if (closest_ne_m < _fail_distance_ne_m) {
        obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_HIGH;
    } else {
        closest_ne_m = closest_approach_NE_m(loc, vel_ned_ms, obstacle_loc, obstacle_vel_ned_ms, _warn_time_horizon_s + obstacle_age_ms/1000);
        if (closest_ne_m < _warn_distance_ne_m) {
            obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_LOW;
        }
    }

    // check for vertical separation; our threat level is the minimum
    // of vertical and horizontal threat levels
    float closest_d_m = closest_approach_D_m(loc, vel_ned_ms, obstacle_loc, obstacle_vel_ned_ms, _warn_time_horizon_s + obstacle_age_ms/1000);
    if (obstacle.threat_level != MAV_COLLISION_THREAT_LEVEL_NONE) {
        if (closest_d_m > _warn_distance_d_m) {
            obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;
        } else {
            closest_d_m = closest_approach_D_m(loc, vel_ned_ms, obstacle_loc, obstacle_vel_ned_ms, _fail_time_horizon_s + obstacle_age_ms/1000);
            if (closest_d_m > _fail_distance_d_m) {
                obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_LOW;
            }
        }
    }

    // If we haven't heard from a vehicle then assume it is no threat
    if (obstacle_age_ms > MAX_OBSTACLE_AGE_MS) {
        obstacle.threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;
    }

    // could optimise this to not calculate a lot of this if threat
    // level is none - but only *once the GCS has been informed*!
    obstacle.closest_approach_ne_m = closest_ne_m;
    obstacle.closest_approach_d_m = closest_d_m;
    float current_distance_ne_m = loc.get_distance(obstacle_loc);
    obstacle.distance_to_closest_approach_ned_m = current_distance_ne_m - closest_ne_m;
    Vector2f net_velocity_ne_ms = Vector2f(vel_ned_ms[0] - obstacle_vel_ned_ms[0], vel_ned_ms[1] - obstacle_vel_ned_ms[1]);
    obstacle.time_to_closest_approach_s = 0.0f;
    if (!is_zero(obstacle.distance_to_closest_approach_ned_m) &&
        ! is_zero(net_velocity_ne_ms.length())) {
        obstacle.time_to_closest_approach_s = obstacle.distance_to_closest_approach_ned_m / net_velocity_ne_ms.length();
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

#if HAL_GCS_ENABLED
void AP_Avoidance::send_collision_all(const AP_Avoidance::Obstacle &threat, MAV_COLLISION_ACTION behaviour) const
{
    const mavlink_collision_t packet{
        id: threat.src_id,
        time_to_minimum_delta: threat.time_to_closest_approach_s,
        altitude_minimum_delta: threat.closest_approach_d_m,
        horizontal_minimum_delta: threat.closest_approach_ne_m,
        src: MAV_COLLISION_SRC_ADSB,
        action: (uint8_t)behaviour,
        threat_level: (uint8_t)threat.threat_level,
    };
    gcs().send_to_active_channels(MAVLINK_MSG_ID_COLLISION, (const char *)&packet);
}
#endif

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
        send_collision_all(*threat, mav_avoidance_action());
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
        obstacle.time_to_closest_approach_s < current.time_to_closest_approach_s) {
        return true;
    }
    return false;
}

void AP_Avoidance::check_for_threats()
{
    const AP_AHRS &_ahrs = AP::ahrs();

    Location loc;
    if (!_ahrs.get_location(loc)) {
        // if we don't know our own location we can't determine any threat level
        return;
    }

    Vector3f vel_ned_ms;
    if (!_ahrs.get_velocity_NED(vel_ned_ms)) {
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
        const uint32_t obstacle_age_ms = AP_HAL::millis() - obstacle.timestamp_ms;
        debug("i=%d src_id=%d timestamp=%u age=%d", i, obstacle.src_id, obstacle.timestamp_ms, obstacle_age_ms);

        update_threat_level(loc, vel_ned_ms, obstacle);
        debug("   threat-level=%d", obstacle.threat_level);

        // ignore any really old data:
        if (obstacle_age_ms > MAX_OBSTACLE_AGE_MS) {
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

    // avoid object (if necessary)
    handle_avoidance_local(most_serious_threat());

    // notify GCS of most serious thread
    handle_threat_gcs_notify(most_serious_threat());
}

void AP_Avoidance::handle_avoidance_local(AP_Avoidance::Obstacle *threat)
{
    MAV_COLLISION_THREAT_LEVEL new_threat_level = MAV_COLLISION_THREAT_LEVEL_NONE;
    MAV_COLLISION_ACTION action = MAV_COLLISION_ACTION_NONE;

    if (threat != nullptr) {
        new_threat_level = threat->threat_level;
        if (new_threat_level == MAV_COLLISION_THREAT_LEVEL_HIGH) {
            action = (MAV_COLLISION_ACTION)_fail_action.get();
            Location loc;
            if (action != MAV_COLLISION_ACTION_NONE && _fail_altitude_min_m > 0 &&
                AP::ahrs().get_location(loc) && ((loc.alt * 0.01f) < _fail_altitude_min_m)) {
                // disable avoidance when close to ground, report only
                action = MAV_COLLISION_ACTION_REPORT;
			}
		}
    }

    uint32_t now = AP_HAL::millis();

    if (new_threat_level != _threat_level) {
        // transition to higher states immediately, recovery to lower states more slowly
        if (((now - _last_state_change_ms) > AP_AVOIDANCE_STATE_RECOVERY_TIME_MS) || (new_threat_level > _threat_level)) {
            // handle recovery from high threat level
            if (_threat_level == MAV_COLLISION_THREAT_LEVEL_HIGH) {
                handle_recovery(RecoveryAction(_fail_recovery.get()));
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
    const Location loc {
        packet.lat,
        packet.lon,
        int32_t(packet.alt * 0.1),  // mm -> cm
        Location::AltFrame::ABSOLUTE
    };
    const Vector3f vel_ned_ms {
        packet.vx * 0.01f, // cm to m
        packet.vy * 0.01f,
        packet.vz * 0.01f
    };
    add_obstacle(AP_HAL::millis(),
                 MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT,
                 msg.sysid,
                 loc,
                 vel_ned_ms);
}

// get unit vector away from the nearest obstacle
bool AP_Avoidance::get_vector_perpendicular(const AP_Avoidance::Obstacle *obstacle, Vector3f &vec_neu_unit) const
{
    if (obstacle == nullptr) {
        // why where we called?!
        return false;
    }

    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        // we should not get to here!  If we don't know our position
        // we can't know if there are any threats, for starters!
        return false;
    }

    // if their velocity is moving around close to zero then flying
    // perpendicular to that velocity may mean we do weird things.
    // Instead, we will fly directly away from them
    if (obstacle->_velocity_ned_ms.length() < _low_velocity_threshold) {
        const Vector2f delta_pos_ne_m =  obstacle->_location.get_distance_NE(current_loc);
        const float delta_pos_u_cm = current_loc.alt - obstacle->_location.alt;
        Vector3f delta_pos_neu_m = Vector3f{delta_pos_ne_m.x, delta_pos_ne_m.y, delta_pos_u_cm * 0.01};
        // avoid div by zero
        if (delta_pos_neu_m.is_zero()) {
            return false;
        }
        delta_pos_neu_m.normalize();
        vec_neu_unit = delta_pos_neu_m;
        return true;
    } else {
        vec_neu_unit = perpendicular_neu_m(obstacle->_location, obstacle->_velocity_ned_ms, current_loc);
        // avoid div by zero
        if (vec_neu_unit.is_zero()) {
            return false;
        }
        vec_neu_unit.normalize();
        return true;
    }
}

// helper functions to calculate 3D destination to get us away from obstacle
// v1_ned is NED
Vector3f AP_Avoidance::perpendicular_neu_m(const Location &p1, const Vector3f &v1_ned, const Location &p2)
{
    const Vector2f delta_p_ne_m = p1.get_distance_NE(p2);
    Vector3f delta_p_neu_m = Vector3f(delta_p_ne_m[0], delta_p_ne_m[1], (p2.alt - p1.alt) * 0.01f); //check this line
    Vector3f v1_neu = Vector3f(v1_ned[0], v1_ned[1], -v1_ned[2]);
    Vector3f ret_neu_m = Vector3f::perpendicular(delta_p_neu_m, v1_neu);
    return ret_neu_m;
}

// singleton instance
AP_Avoidance *AP_Avoidance::_singleton;

namespace AP {

AP_Avoidance *ap_avoidance()
{
    return AP_Avoidance::get_singleton();
}

}

#endif // AP_ADSB_AVOIDANCE_ENABLED
