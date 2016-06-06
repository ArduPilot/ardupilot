#include "AP_Avoidance.h"

extern const AP_HAL::HAL& hal;

#include <limits>
#include <GCS_MAVLink/GCS.h>

AP_Avoidance *AP_Avoidance::_instance;

#include "AvoidanceHandler.h"

#define AVOIDANCE_DEBUGGING 0

#if AVOIDANCE_DEBUGGING
#include <stdio.h>
#define debug(fmt, args ...)  do {::fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Avoidance::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable AVOIDANCE
    // @Description: Enable Avoidance
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE",     0, AP_Avoidance, _enabled,    0),

    // @Param: F_ACTION
    // @DisplayName: Collision Avoidance Behavior
    // @Description: Specifies aircraft behaviour when a collision is imminent
    // The following values should come from the mavlink COLLISION_ACTION enum
    // @Values: 0:None,1:Report,4:TCAS,5:Perpendicular,6:RTL,7:Hover
    // @User: Advanced
    AP_GROUPINFO("F_ACTION",   1, AP_Avoidance, _fail_action, MAV_COLLISION_ACTION_REPORT),

    // @Param: W_ACTION
    // @DisplayName: Collision Avoidance Behavior - Warn
    // @Description: Specifies aircraft behaviour when a collision may occur
    // The following values should come from the mavlink COLLISION_ACTION enum
    // @Values: 0:None,1:Report,4:TCAS,5:Perpendicular,6:RTL,7:Hover
    // @User: Advanced
    AP_GROUPINFO("W_ACTION",   2, AP_Avoidance, _warn_action, MAV_COLLISION_ACTION_REPORT),

    // @Param: W_RCVRY
    // @DisplayName: Recovery behaviour after a warn event
    // @Description: Determines what the aircraft will do after a warn event (which has not progressed to a fail event) has been resolved
    // @Values: 0:Resume previous flight mode,1:Mode Loiter,2:Continue collision avoidance action
    // @User: Advanced
    AP_GROUPINFO("W_RCVRY",   3, AP_Avoidance, _warn_recovery, AVOIDANCE_RECOVERY_W_RESUME),

    // @Param: F_RCVRY
    // @DisplayName: Recovery behaviour after a fail event
    // @Description: Determines what the aircraft will do after a fail event is resolved
    // @Values: 0:Continue collision avoidance ACTION_F,1:Move to collision avoidance W_ACTION
    // @User: Advanced
    AP_GROUPINFO("F_RCVRY",   4, AP_Avoidance, _fail_recovery, AVOIDANCE_RECOVERY_F_MOVE_TO_WARN),

    // @Param: OBS_MAX
    // @DisplayName: Maximum number of obstacles to track
    // @Description: Maximum number of obstacles to track
    // @User: Advanced
    AP_GROUPINFO("OBS_MAX",   5, AP_Avoidance, _obstacles_max, 20),

    // @Param: W_TIME
    // @DisplayName: Time Horizon Warn
    // @Description: Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than W_DIST_XY or W_DIST_Z then W_ACTION is undertaken (assuming F_ACTION is not undertaken)
    // @User: Advanced
    AP_GROUPINFO("W_TIME",     6, AP_Avoidance, _warn_time_horizon,    30),

    // @Param: F_TIME
    // @DisplayName: Time Horizon Fail
    // @Description: Aircraft velocity vectors are multiplied by this time to determine closest approach.  If this results in an approach closer than F_DIST_XY or F_DIST_Z then F_ACTION is undertaken
    // @User: Advanced
    AP_GROUPINFO("F_TIME",     7, AP_Avoidance, _fail_time_horizon,    30),

    // @Param: W_DIST_XY
    // @DisplayName: Distance Warn XY
    // @Description: Closest allowed projected distance before W_ACTION is undertaken
    // @User: Advanced
    AP_GROUPINFO("W_DIST_XY",     8, AP_Avoidance, _warn_distance_xy,    300),

    // @Param: F_DIST_XY
    // @DisplayName: Distance Fail XY
    // @Description: Closest allowed projected distance before F_ACTION is undertaken
    // @User: Advanced
    AP_GROUPINFO("F_DIST_XY",     9, AP_Avoidance, _fail_distance_xy,    100),


    // @Param: W_DIST_Z
    // @DisplayName: Distance Warn Z
    // @Description: Closest allowed projected distance before BEHAVIOUR_W is undertaken
    // @User: Advanced
    AP_GROUPINFO("W_DIST_Z",     10, AP_Avoidance, _warn_distance_z,    300),

    // @Param: F_DIST_Z
    // @DisplayName: Distance Fail Z
    // @Description: Closest allowed projected distance before BEHAVIOUR_F is undertaken
    // @User: Advanced
    AP_GROUPINFO("F_DIST_Z",     11, AP_Avoidance, _fail_distance_z,    100),

    AP_GROUPEND
};

const char *AP_Avoidance::_state_names[] = { "CLEAR", "WARN", "FAIL" };

AP_Avoidance::AP_Avoidance(AP_AHRS &ahrs, AP_ADSB &adsb) :
    _ahrs(ahrs),
    _adsb(adsb)
{
    AP_Param::setup_object_defaults(this, var_info);
    _instance = this;
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
    _old_state = STATE_CLEAR;
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
        Location loc = vehicle.get_location();
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
    if (delta_pos_d >= 0 &&
        delta_vel_d >= 0) {
        ret = delta_pos_d;
    } else if (delta_pos_d <= 0 &&
        delta_vel_d <= 0) {
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

void AP_Avoidance::internal_error()
{
    // internal_errors++;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    abort();
#endif
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

    handle_avoidance(most_serious_threat());
}

// returns an entry from the MAV_COLLISION_ACTION representative
// of what the curent avoidance handler is up to.
MAV_COLLISION_ACTION AP_Avoidance::mav_avoidance_action() {
    if (_current_avoidance_handler == nullptr) {
        return MAV_COLLISION_ACTION_NONE;
    }
    return _current_avoidance_handler->mav_avoidance_action();
}

void AP_Avoidance::handle_avoidance(AP_Avoidance::Obstacle *threat)
{
    handle_threat_gcs_notify(threat);

    if (threat == nullptr) {
        if (_current_avoidance_handler != nullptr) {
            _current_avoidance_handler->exit();
            _current_avoidance_handler = nullptr;
        }
        return;
    }

    int32_t action;
    state_t new_state;
    switch (threat->threat_level) {
    case MAV_COLLISION_THREAT_LEVEL_NONE:
        new_state = STATE_CLEAR;
        action = MAV_COLLISION_ACTION_NONE;
        break;
    case MAV_COLLISION_THREAT_LEVEL_LOW:
        new_state = STATE_WARN;
        action = _warn_action;
        break;
    case MAV_COLLISION_THREAT_LEVEL_HIGH:
        new_state = STATE_FAIL;
        action = _fail_action;
        break;
    default:
        internal_error();
        new_state = STATE_CLEAR;
        action = MAV_COLLISION_ACTION_NONE;
        break;
    }

    uint32_t now = AP_HAL::millis();
    bool should_change_state = false;
    do {
        if (new_state == _old_state) {
            break;
        }
        // do not transition back from a fail or warn state too quickly:
        if (now - _last_state_change_ms < _state_recovery_hysteresis*1000) {
            if (new_state == STATE_CLEAR ||
                (new_state == STATE_WARN && _old_state == STATE_FAIL)) {
                break;
            }
        }
        should_change_state = true;
    } while(0);

    if (should_change_state) {

        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "AVOID: State change: (%s)=>(%s)", _state_names[_old_state], _state_names[new_state]);

        _last_state_change_ms = now;

        // we can change state but not change handler - for example,
        // the user has specified the recovery handler for Fail->Warn
        // as "continue fail behaviour".

        // Note that it is entirely possible to transition from
        // Fail->Clear; in this case we honour both _recovery_f and
        // _recovery_w

        bool change_handler = true;

        if (_old_state == STATE_FAIL) {
            // transitioning from fail -> warn
            switch(_fail_recovery) {
            case AVOIDANCE_RECOVERY_F_CONTINUE_FAIL:
                change_handler = false;
                break;
            case AVOIDANCE_RECOVERY_F_MOVE_TO_WARN:
                change_handler = true;
                break;
            }
        }
        if (new_state == STATE_CLEAR) {
            // transitioning from (fail or warn) to clear
            switch(_warn_recovery) {
            case AVOIDANCE_RECOVERY_W_CONTINUE_ACTION:
                change_handler = true;
                break;
            case AVOIDANCE_RECOVERY_W_RESUME:
                change_handler = true;
                break;
            }
        }

        if (change_handler) {
            AvoidanceHandler *new_avoidance_handler;
            if (new_state == STATE_CLEAR) {
                new_avoidance_handler = nullptr;
            } else {
                AvoidanceHandler &tmp = handler_for_action((MAV_COLLISION_ACTION)action);
                new_avoidance_handler = &tmp;
            }

            if (new_avoidance_handler != _current_avoidance_handler) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "AVOID: Using handler (%s)", ((new_avoidance_handler != nullptr) ? new_avoidance_handler->name() : "NULL"));
                if (new_avoidance_handler == nullptr) {
                    if (_current_avoidance_handler != nullptr) {
                        _current_avoidance_handler->exit();
                        _current_avoidance_handler = nullptr;
                    }
                } else {
                    if (new_avoidance_handler->enter(*threat, _current_avoidance_handler)) {
                        _current_avoidance_handler = new_avoidance_handler;
                    } else {
                        // whinge. This may not be an internal error -
                        // e.g. we may have lost GPS
                    }
                }
            }
        }
        _old_state = new_state;
    }

    if (_current_avoidance_handler != nullptr) {
        _current_avoidance_handler->update();
    }
}



void AP_Avoidance::MAVLink_packetReceived(const mavlink_message_t &msg)
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
