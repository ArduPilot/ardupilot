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
    // @Description: Closest allowed projected distance before W_ACTION is undertaken
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("W_DIST_Z",    10, AP_Avoidance, _warn_distance_d_m, AP_AVOIDANCE_WARN_DISTANCE_Z_DEFAULT),

    // @Param: F_DIST_Z
    // @DisplayName: Distance Fail Z
    // @Description: Closest allowed projected distance before F_ACTION is undertaken
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("F_DIST_Z",    11, AP_Avoidance, _fail_distance_d_m, AP_AVOIDANCE_FAIL_DISTANCE_Z_DEFAULT),
    
    // @Param: F_ALT_MIN
    // @DisplayName: ADS-B avoidance minimum altitude
    // @Description: Minimum AMSL (above mean sea level) altitude for ADS-B avoidance. If the vehicle is below this altitude, no avoidance action will take place. Useful to prevent ADS-B avoidance from activating while below the tree line or around structures. Default of 0 is no minimum.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("F_ALT_MIN",    12, AP_Avoidance, _fail_altitude_min_m, 0),

// The APM_BUILD_TYPE term is redundant - AP_OA_SCRIPTING_ENABLED already includes it -
// but it must appear in this .cpp's text, because that is how waf decides to compile
// this source per-vehicle; it does not follow macros through headers.
#if AP_OA_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)   // DAA standoff params, consumed only by AP_OAScripting
    // @Param: WCLR_XY
    // @DisplayName: Well Clear horizontal
    // @Description: Horizontal "Well Clear" separation kept from crewed aircraft during ADS-B avoidance (metres). The ASTM F3442M-23 standard specifies 2000 ft (= 609.6 m).
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("WCLR_XY",    13, AP_Avoidance, _well_clear_xy, 609.6),

    // @Param: WCLR_Z
    // @DisplayName: Well Clear vertical
    // @Description: Vertical "Well Clear" separation kept from crewed aircraft during ADS-B avoidance (metres). The ASTM F3442M-23 standard specifies 250 ft (= 76.2 m).
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("WCLR_Z",    14, AP_Avoidance, _well_clear_z, 76.2),

    // @Param: NMAC_XY
    // @DisplayName: Near Miss Horizontal
    // @Description: Horizontal Near Mid-Air Collision (NMAC) separation from crewed aircraft; closer than this counts as a near miss (metres, 0 disables). The FAA figure is 500 ft (= 152.4 m).
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("NMAC_XY",    15, AP_Avoidance, _near_miss_xy, 152.4),

    // @Param: NMAC_Z
    // @DisplayName: Near Miss Vertical
    // @Description: Vertical Near Mid-Air Collision (NMAC) separation from crewed aircraft; within this counts as a near miss (metres, 0 disables). The RTCA DO-396 (TCAS MOPS) figure is 100 ft (= 30.48 m).
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("NMAC_Z",    16, AP_Avoidance, _near_miss_z, 30.48),

    // @Param: UAV_XY
    // @DisplayName: UAV horizontal avoidance radius
    // @Description: Horizontal keep-out radius used for ADS-B drones/UAVs (emitter type UAV). This is the drone equivalent of the crewed-aircraft Well Clear AVD_WCLR_XY, and is normally smaller since drone-to-drone separation needs are lower.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("UAV_XY",    17, AP_Avoidance, _uav_xy, 150),

    // @Param: UAV_Z
    // @DisplayName: UAV vertical avoidance gate
    // @Description: Vertical separation gate used for ADS-B drones/UAVs (emitter type UAV). Obstacles more than this far above or below are ignored. This is the drone equivalent of the crewed-aircraft Well Clear AVD_WCLR_Z, and is normally small because drones are vertically thin.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("UAV_Z",    18, AP_Avoidance, _uav_z, 25),


#endif

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
    // the scripting-thread readers walk _obstacles[]; publish the allocation under
    // the same semaphore they take
    WITH_SEMAPHORE(_rsem);
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
    bool was_allocated = false;
    {
        // exclude the scripting-thread readers: they walk _obstacles[] up to
        // _obstacle_count, so the count must reach zero before the array is freed
        WITH_SEMAPHORE(_rsem);
        _obstacle_count = 0;
        if (_obstacles != nullptr) {
            delete [] _obstacles;
            _obstacles = nullptr;
            _obstacles_allocated = 0;
            was_allocated = true;
        }
    }
    if (was_allocated) {
        // outside the semaphore: this can change flight mode and must not be
        // holding a lock the rest of the vehicle may want
        handle_recovery(RecoveryAction::RTL);
    }
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
                                const Vector3f &vel_ned_ms,
                                const uint8_t  emitter_type)
{
    if (! check_startup()) {
        return;
    }
    // take the lock before the scan below, not after it: the loop reads _obstacle_count
    // and _obstacles[] to pick the slot, and check_for_threats() can be shrinking the
    // list at the same time.  check_startup() deliberately stays outside - it can call
    // deinit(), which takes this same semaphore.
    WITH_SEMAPHORE(_rsem);

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
        } else {
            // no room for this (old?!) data
            return;
        }

        _obstacles[index].src = src;
        _obstacles[index].src_id = src_id;
    }

    _obstacles[index].emitter_type = emitter_type;
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
                                const float speed_d_ms,
                                const uint8_t emitter_type)
{
    Vector3f vel_ned_ms;
    vel_ned_ms[0] = speed_ne_ms * cosf(radians(cog));
    vel_ned_ms[1] = speed_ne_ms * sinf(radians(cog));
    vel_ned_ms[2] = speed_d_ms;
    // debug("cog=%f speed_ne_ms=%f veln=%f vele=%f", cog, speed_ne_ms, vel_ned_ms[0], vel[1]);
    return add_obstacle(obstacle_timestamp_ms, src, src_id, loc, vel_ned_ms, emitter_type);
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
                   vehicle.info.heading * 0.01,         // convert cm-up to m-down
                   vehicle.info.hor_velocity * 0.01,
                   -vehicle.info.ver_velocity * 0.01,
                   vehicle.info.emitter_type);
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
    // the loop prunes stale entries, so hold off the scripting-thread readers while
    // _obstacle_count moves.  Scoped to the loop: the mode-changing avoidance handlers
    // run later, in update(), and must not be called holding this.
    WITH_SEMAPHORE(_rsem);
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
    if (_current_most_serious_threat < 0 || _obstacles == nullptr) {
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
                 vel_ned_ms,
                 static_cast<uint8_t>(ADSB_EMITTER_TYPE_UAV));
}

#if AP_OA_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)   // see above: APM_BUILD_TYPE repeated for waf's per-vehicle detection
// get the avoidance radius in meters of a given obstacle type
// the definition of "Well Clear" (2000ft = 609.6m) is from ASTM F3442M-23
float AP_Avoidance::get_obstacle_radius_m(uint8_t emitter_type) const
{
    switch (static_cast<ADSB_EMITTER_TYPE>(emitter_type))
    {
    case ADSB_EMITTER_TYPE_NO_INFO:
    case ADSB_EMITTER_TYPE_LIGHT:
    case ADSB_EMITTER_TYPE_SMALL:
    case ADSB_EMITTER_TYPE_LARGE:
    case ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE:
    case ADSB_EMITTER_TYPE_HEAVY:
    case ADSB_EMITTER_TYPE_HIGHLY_MANUV:
        return _well_clear_xy;                           // crewed aircraft (AVD_WCLR_XY)
    case ADSB_EMITTER_TYPE_ROTOCRAFT:
        return _well_clear_xy;                           // helicopters (AVD_WCLR_XY)
    // 8 Unassigned
    case ADSB_EMITTER_TYPE_GLIDER:
    case ADSB_EMITTER_TYPE_LIGHTER_AIR:
    case ADSB_EMITTER_TYPE_PARACHUTE:
    case ADSB_EMITTER_TYPE_ULTRA_LIGHT:
        return _well_clear_xy;                           // also use well clear for these
    // 13 Unassigned
    case ADSB_EMITTER_TYPE_UAV:                          // drone/UAV horizontal radius (AVD_UAV_XY)
        return _uav_xy;
    case ADSB_EMITTER_TYPE_SPACE:
        return 9600;                                     // lets give rockets a wide berth, 5nm
    // Surface types
    case ADSB_EMITTER_TYPE_EMERGENCY_SURFACE:
    case ADSB_EMITTER_TYPE_SERVICE_SURFACE:
        return 150;
    // Obstacle types
    case ADSB_EMITTER_TYPE_POINT_OBSTACLE:
        return 50.0;
    default:
        return 100;
    }
}

// get the avoidance height in meters of a given obstacle type
// the definition of "Well Clear" (2000ft = 609.6m) is from ASTM F3442M-23
float AP_Avoidance::get_obstacle_height_m(uint8_t emitter_type) const
{
    switch (static_cast<ADSB_EMITTER_TYPE>(emitter_type))
    {
    case ADSB_EMITTER_TYPE_NO_INFO:
    case ADSB_EMITTER_TYPE_LIGHT:
    case ADSB_EMITTER_TYPE_SMALL:
    case ADSB_EMITTER_TYPE_LARGE:
    case ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE:
    case ADSB_EMITTER_TYPE_HEAVY:
    case ADSB_EMITTER_TYPE_HIGHLY_MANUV:
        return _well_clear_z;                           // crewed aircraft (AVD_WCLR_Z)
    case ADSB_EMITTER_TYPE_ROTOCRAFT:
        return _well_clear_z;                           // helicopters (AVD_WCLR_Z)
    // 8 Unassigned
    case ADSB_EMITTER_TYPE_GLIDER:
    case ADSB_EMITTER_TYPE_LIGHTER_AIR:
    case ADSB_EMITTER_TYPE_PARACHUTE:
    case ADSB_EMITTER_TYPE_ULTRA_LIGHT:
        return _well_clear_z;                           // also use well clear for these
    // 13 Unassigned
    case ADSB_EMITTER_TYPE_UAV:                          // drone/UAV vertical gate (AVD_UAV_Z)
        return _uav_z;
    case ADSB_EMITTER_TYPE_SPACE:
        return 9600;                                     // lets give rockets a wide berth, 5nm
    // Surface types - lets make this unlimited
    case ADSB_EMITTER_TYPE_EMERGENCY_SURFACE:
    case ADSB_EMITTER_TYPE_SERVICE_SURFACE:
        return FLT_MAX;
    // Obstacle types - also unlimited
    case ADSB_EMITTER_TYPE_POINT_OBSTACLE:
        return FLT_MAX;
    default:        // Default to infinite height if we don't have a specific height
        return FLT_MAX;
    }
}

bool AP_Avoidance::is_adsb_uav(uint8_t emitter_type)
{
    switch (static_cast<ADSB_EMITTER_TYPE>(emitter_type) )
    {
    case ADSB_EMITTER_TYPE_UAV:         // Drones
        return true;
    default:
        return false;
    }
    return false;
}

// ADS-B surface (ground) vehicle categories. We deliberately do not avoid these:
// an airborne vehicle has no requirement to manoeuvre around a vehicle on the ground.
bool AP_Avoidance::is_ground_vehicle(uint8_t emitter_type)
{
    switch (static_cast<ADSB_EMITTER_TYPE>(emitter_type))
    {
    case ADSB_EMITTER_TYPE_EMERGENCY_SURFACE:
    case ADSB_EMITTER_TYPE_SERVICE_SURFACE:
        return true;
    default:
        return false;
    }
}

bool AP_Avoidance::is_adsb_aircraft(uint8_t emitter_type)
{
    switch (static_cast<ADSB_EMITTER_TYPE>(emitter_type) )
    {
    case ADSB_EMITTER_TYPE_LIGHT:
    case ADSB_EMITTER_TYPE_SMALL:
    case ADSB_EMITTER_TYPE_LARGE:
    case ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE:
    case ADSB_EMITTER_TYPE_HEAVY:
    case ADSB_EMITTER_TYPE_HIGHLY_MANUV:
    case ADSB_EMITTER_TYPE_ROTOCRAFT:   // Helicopter
    // 8 Unassigned
    case ADSB_EMITTER_TYPE_GLIDER:
    case ADSB_EMITTER_TYPE_LIGHTER_AIR:
    case ADSB_EMITTER_TYPE_ULTRA_LIGHT:
    // 13 Unassigned
    case ADSB_EMITTER_TYPE_SPACE:       // Call this aircraft for now
    // 16 Unassigned
        return true;

    case ADSB_EMITTER_TYPE_NO_INFO:
    case ADSB_EMITTER_TYPE_PARACHUTE:
    case ADSB_EMITTER_TYPE_UAV:         // Drones

    // Surface types
    case ADSB_EMITTER_TYPE_EMERGENCY_SURFACE:
    case ADSB_EMITTER_TYPE_SERVICE_SURFACE:

    // Stationary Obstacle types
    case ADSB_EMITTER_TYPE_POINT_OBSTACLE:
        return false;

    default:
        return false;
    }

    return false;
}

// For AP_AOScripting to check for obstacles and return the closest one.
// Crewed aircraft are found separately, by distance_to_aircraft(): that applies the
// caller's vertical gate, whereas this uses the per-emitter table.
float AP_Avoidance::distance_to_obstacle(const Vector3f &start_NED_m, const Vector3f &end_NED_m,
                                            // return values
                                            Obstacle &avoid_obstacle
                                        ) const
{
    // guard the obstacle database against concurrent updates from the MAVLink thread
    WITH_SEMAPHORE(_rsem);

    float distance_new_m = FLT_MAX;

    const uint32_t now_ms = AP_HAL::millis();
    for(uint8_t i = 0; i < _obstacle_count; i++) {
        const Obstacle obstacle         = _obstacles[i];
        // a contact that stopped transmitting is not a threat at its last known position;
        // check_for_threats() only prunes a stale entry when it is last in the list, so
        // filter here rather than trusting the list to be current
        if (now_ms - obstacle.timestamp_ms > MAX_OBSTACLE_AGE_MS) {
            continue;
        }
        // deliberately ignore ground vehicles: an airborne vehicle does not avoid them
        if (is_ground_vehicle(obstacle.emitter_type)) {
            continue;
        }
        const Location obstacle_loc     = _obstacles[i]._location;
        Vector3f obstacle_NED_m;

        Vector2f start_NE_m(start_NED_m.x, start_NED_m.y);
        Vector2f end_NE_m(end_NED_m.x, end_NED_m.y);
        Vector2f obstacle_NE_m;
        if (obstacle_loc.get_vector_xy_from_origin_NE_m(obstacle_NE_m)
                && obstacle_loc.get_vector_from_origin_NEU_m(obstacle_NED_m)) {

            // until we get the new NED functions
            obstacle_NED_m.z = -obstacle_NED_m.z;

            // effective distance = horizontal clearance from the path segment to the obstacle,
            // minus the obstacle's radius. Sample the segment at its horizontal closest point (t)
            // and reuse that same point for the vertical check, so altitude is evaluated where the
            // path actually passes the obstacle (matters on climbing/descending legs, and on the
            // small drone vertical band).
            const Vector2f seg_NE_m = end_NE_m - start_NE_m;
            const float seg_len_sq_m = seg_NE_m.length_squared();
            float t = 0.0f;
            if (seg_len_sq_m > 1.0e-6f) {
                t = constrain_float((obstacle_NE_m - start_NE_m) * seg_NE_m / seg_len_sq_m, 0.0f, 1.0f);
            }
            const Vector2f closest_NE_m = start_NE_m + seg_NE_m * t;
            float distance_m = (obstacle_NE_m - closest_NE_m).length() - get_obstacle_radius_m(obstacle.emitter_type);

            // height difference between the obstacle and the path at that same closest point.
            // This is a static-position check by design: the closing/receding motion of ADS-B
            // traffic is handled a layer up, in the Lua assess_obstacle_motion() CPA logic.
            const float path_z_at_closest_m = start_NED_m.z + t * (end_NED_m.z - start_NED_m.z);
            float height_difference_m = fabsf(path_z_at_closest_m - obstacle_NED_m.z);

            if (distance_m < distance_new_m && height_difference_m < get_obstacle_height_m(obstacle.emitter_type)) {
                // we are within the horizontal distance - next check the vertical distance
                distance_new_m  = distance_m;
                avoid_obstacle  = obstacle;
            }
        }
    }

    return distance_new_m;
}

// For AP_AOScripting to check for crewed aircraft and return the closest one
float AP_Avoidance::distance_to_aircraft(const Vector3f &vehicle_NED_m, const float lookahead_m, const float vertical_lookahead_m,
                                            // return values
                                            Obstacle &avoid_obstacle
                                        ) const
{
    // guard the obstacle database against concurrent updates from the MAVLink thread
    WITH_SEMAPHORE(_rsem);

    float distance_new_msq  = lookahead_m * lookahead_m;

    const uint32_t now_ms = AP_HAL::millis();
    for(uint8_t i = 0; i < _obstacle_count; i++) {
        const Obstacle obstacle         = _obstacles[i];
        // skip contacts that have gone quiet - see distance_to_obstacle()
        if (now_ms - obstacle.timestamp_ms > MAX_OBSTACLE_AGE_MS) {
            continue;
        }
        const Location obstacle_loc     = _obstacles[i]._location;
        Vector3f obstacle_NED_m;

        Vector2f vehicle_NE_m(vehicle_NED_m.x, vehicle_NED_m.y);
        Vector2f obstacle_NE_m;

        // this needs to account for the moving obstacle as done in closest_approach_ne_m

        if(is_adsb_aircraft(obstacle.emitter_type)
                && obstacle_loc.get_vector_xy_from_origin_NE_m(obstacle_NE_m)
                && obstacle_loc.get_vector_from_origin_NEU_m(obstacle_NED_m)) {

            // until we get the new NED functions
            obstacle_NED_m.z = -obstacle_NED_m.z;

            float distance_msq = (vehicle_NE_m - obstacle_NE_m).length_squared();

            // height difference is the difference in the height between the vehicle and the obstacle
            float height_difference_m = fabsf(vehicle_NED_m.z - obstacle_NED_m.z);

            // this finds the nearest aircraft iff it is within the caller-supplied vertical
            // gate (metres). The caller passes the full vertical separation (e.g. AVD_WCLR_Z +
            // margin from the scripting layer), mirroring the full horizontal lookahead, so the
            // gate policy lives with the caller rather than the per-emitter get_obstacle_height_m() table.
            if (distance_msq < distance_new_msq && height_difference_m < vertical_lookahead_m) {
                distance_new_msq    = distance_msq;
                avoid_obstacle      = obstacle;
            }
        }
    }

    // we need to do one square root here at the end. But by using squared above we avoid lots of them
    return safe_sqrt(distance_new_msq);
}
#endif // AP_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)


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
