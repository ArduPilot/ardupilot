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

#include "AP_OAScripting.h"


// AP_OA_SCRIPTING_ENABLED folds in scripting/avoidance/ADS-B/ArduPlane; the
// explicit APM_BUILD_TYPE in active text is what makes waf compile this file
// per-vehicle (it scans the source text for the build-type macro).
#if AP_OA_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)

#if AP_FENCE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#endif // AP_FENCE_ENABLED

static constexpr float OA_MARGIN_MAX_DEFAULT = 609.6;

const AP_Param::GroupInfo AP_OAScripting::var_info[] = {

    // @Param: MARGIN_GA
    // @DisplayName: Margin for crude aircraft
    // @Description: Object Avoidance will ignore aircraft more than this many meters from vehicle
    // @Units: m
    // @Range: 0.1 10000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("MARGIN_GA", 1, AP_OAScripting, _margin_aircraft, OA_MARGIN_MAX_DEFAULT),

    AP_GROUPEND
};

/// Constructor
AP_OAScripting::AP_OAScripting()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one AP_OAScripting");
    }
    _singleton = this;

    //AP_Param::setup_object_defaults(this, var_info);
}

// singleton instance
AP_OAScripting *AP_OAScripting::_singleton;

bool AP_OAScripting::distance_obstacle_test(const Location &start_loc, const Location &end_loc, const float &lookahead_m, float &distance_m, Location &location_out) const
{
    distance_m = lookahead_m;
    return true;
}

bool AP_OAScripting::find_threats(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                            // Return values
                                            float       &distance_m,
                                            OAObstacle  &any_obstacle,
                                            OAObstacle  &aircraft_obstacle,
                                            OAObstacle  &proximity_obstacle,
                                            OAObstacle  &fence_obstacle
                                            ) const
{
    float distance_new_m = FLT_MAX;
    OAObstacle obstacle;

    distance_m = lookahead_m;

    // convert start and end to offsets from EKF origin
    Vector3f start_NED_m,end_NED_m;
    if (!start_loc.get_vector_from_origin_NEU_m(start_NED_m) ||
        !end_loc.get_vector_from_origin_NEU_m(end_NED_m)) {
        return false;
    }
    if (start_NED_m == end_NED_m) {
        return false;
    }
    // until we get the new NED functions
    start_NED_m.z   = -start_NED_m.z;
    end_NED_m.z     = -end_NED_m.z;

    // "obstacles" are stored in AP_Avoidance - the are typically populated by MAVLink (ADSB, GLOBAL_POSITION, FOLLOW_TARGET)
    // These have priority over all other obstacles, especially if they are ADSB messages representing crude aircraft
    OAObstacle obstacle_found;
    OAObstacle aircraft_found;
    distance_new_m = _distance_to_avoidance(start_NED_m, end_NED_m, obstacle_found, aircraft_found);
    if (distance_new_m < distance_m) {
        obstacle            = obstacle_found;
        aircraft_obstacle   = aircraft_found;
        distance_m          = distance_new_m;
    }

    // "objects" are stored in the AP_OADatabase - they are typically populated by proximity sensors
    distance_new_m = _distance_to_object(start_NED_m, end_NED_m, obstacle_found); 
    if (distance_new_m < distance_m) {
        obstacle            = obstacle_found;
        proximity_obstacle  = obstacle_found;
        distance_m          = distance_new_m;
    }

#ifdef AP_FENCE_ENABLED
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence != nullptr && fence->enabled()) {
        // fences use cm (for now), so do this once now so we can pass to all the fence methods
        const Vector2f start_NE_cm(start_NED_m.x * 100.0f, start_NED_m.y * 100.0f);
        const Vector2f end_NE_cm(end_NED_m.x * 100.0f, end_NED_m.y * 100.0f);

        // We do each type of fence one at a time, because 
        // a. they are stored in separate lists and 
        // b. we want to tell the user which fence type of fence it is
        distance_new_m = fence->distance_line_to_home_inclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_HOME);
            distance_m          = distance_new_m;
            obstacle.radius_m   = fence->get_radius_m();
            obstacle.margin_m   = fence->get_margin_ne_m();
        }
        distance_new_m = fence->distance_line_to_circle_inclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_CIRCLE_INCLUSION);
            fence_obstacle      = obstacle;
            distance_m          = distance_new_m;
            obstacle.radius_m   = fence->get_radius_m();
            obstacle.margin_m   = fence->get_margin_ne_m();
        }
        distance_new_m = fence->distance_line_to_circle_exclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_CIRCLE_EXCLUSION);
            fence_obstacle      = obstacle;
            distance_m          = distance_new_m;
            obstacle.radius_m   = fence->get_radius_m();
            obstacle.margin_m   = fence->get_margin_ne_m();
        }
        distance_new_m = fence->distance_line_to_polygon_inclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_POLYGON_INCLUSION);
            fence_obstacle  = obstacle;
            distance_m      = distance_new_m;
        }
        distance_new_m = fence->distance_line_to_polygon_exclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_POLYGON_EXCLUSION);
            fence_obstacle  = obstacle;
            distance_m      = distance_new_m;
        }
    }
#endif
    if (distance_m < lookahead_m) {
        any_obstacle    = obstacle;
        return true;
    }

    return false;
}

// Lua binding to find the nearest crude aircraft (GA/GENERAL_AVIATION) 
// this is needed because avoiding aircraft is often a higher priority than avoiding other obstacles
bool AP_OAScripting::find_aircraft(const Location &vehicle_loc, const float lookahead_m, const float vertical_lookahead_m,
                                            float       &distance_m,
                                            OAObstacle  &aircraft_obstacle
                                    ) const
{
    float distance_new_m = FLT_MAX;
    OAObstacle obstacle;

    distance_m = lookahead_m;

    // convert start and end to offsets from EKF origin (waiting for NEU/NED changes)
    Vector3f vehicle_NED_m;
    if (!vehicle_loc.get_vector_from_origin_NEU_m(vehicle_NED_m)) {
        return false;
    }
    // until we get the new NED functions
    vehicle_NED_m.z = -vehicle_NED_m.z;

    // "obstacles" are stored in AP_Avoidance - the are typically populated by MAVLink (ADSB, GLOBAL_POSITION, FOLLOW_TARGET)
    // These have priority over all other obstacles, especially if they are ADSB messages representing crude aircraft
    OAObstacle obstacle_found;
    distance_new_m = _distance_to_aircraft(vehicle_NED_m, distance_m, vertical_lookahead_m, obstacle_found);
    if (distance_new_m < distance_m) {
        obstacle = obstacle_found;
        distance_m  = distance_new_m;
    }

    // we only care if the obstacle is in the provided lookahead boundary
    if (distance_m < lookahead_m) {
        aircraft_obstacle = obstacle_found;
        return true;
    }

    return false;
}

// closest distance (metres) from a location to the nearest fence boundary edge — polygon or
// circle, inclusion or exclusion — as a real geometric distance to the physical boundary.
// Lets the AVOIDING message report a true distance for fence obstacles, which (unlike ADS-B
// point obstacles) carry no single usable "location".  Returns true and sets distance_m if
// any polygon/circle fence is loaded.
//
// The fence loader stores its points/centres as NE offsets in cm from the EKF origin; that cm
// frame is confined to this function - everything handed back to the caller (Lua) is in metres.
bool AP_OAScripting::fence_distance(const Location &loc, uint8_t fence_type, float &distance_m) const
{
#ifdef AP_FENCE_ENABLED
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    Vector3f loc_NEU_m;
    if (!loc.get_vector_from_origin_NEU_m(loc_NEU_m)) {
        return false;
    }
    const Vector2f point_NE_cm(loc_NEU_m.x * 100.0f, loc_NEU_m.y * 100.0f);
    const AC_PolyFence_loader &poly = fence->polyfence();

    // scope the search to the fence category the caller is avoiding, so the returned distance
    // belongs to the same kind of fence the AVOIDING message names (e.g. an "Excl. Circle" label
    // no longer reports the distance to a nearer inclusion polygon).  Any non-fence-category value
    // (0/GENERAL, FENCE_LUA, ...) falls back to searching every polygon/circle fence.
    typedef AP_OAScripting::ObstacleType OT;
    bool want_excl_poly = (fence_type == (uint8_t)OT::FENCE_POLYGON_EXCLUSION);
    bool want_incl_poly = (fence_type == (uint8_t)OT::FENCE_POLYGON_INCLUSION);
    bool want_excl_circ = (fence_type == (uint8_t)OT::FENCE_CIRCLE_EXCLUSION);
    bool want_incl_circ = (fence_type == (uint8_t)OT::FENCE_CIRCLE_INCLUSION
                           || fence_type == (uint8_t)OT::FENCE_HOME);
    if (!(want_excl_poly || want_incl_poly || want_excl_circ || want_incl_circ)) {
        want_excl_poly = want_incl_poly = want_excl_circ = want_incl_circ = true;
    }

    float closest_m = FLT_MAX;

    // polygon fences (inclusion + exclusion): true geometric point-to-edge distance
    Vector2f closest_vec_cm;
    for (uint8_t i = 0; want_excl_poly && i < poly.get_exclusion_polygon_count(); i++) {
        uint16_t num_points = 0;
        const Vector2f *points = poly.get_exclusion_polygon(i, num_points);
        if (points != nullptr && Polygon_closest_distance_point(points, num_points, point_NE_cm, closest_vec_cm)) {
            closest_m = MIN(closest_m, closest_vec_cm.length() * 0.01f);
        }
    }
    for (uint8_t i = 0; want_incl_poly && i < poly.get_inclusion_polygon_count(); i++) {
        uint16_t num_points = 0;
        const Vector2f *points = poly.get_inclusion_polygon(i, num_points);
        if (points != nullptr && Polygon_closest_distance_point(points, num_points, point_NE_cm, closest_vec_cm)) {
            closest_m = MIN(closest_m, closest_vec_cm.length() * 0.01f);
        }
    }

    // circle fences (inclusion + exclusion): distance to the ring is |range-to-centre - radius|
    Vector2f centre_cm;
    float radius_m = 0.0f;
    for (uint8_t i = 0; want_excl_circ && i < poly.get_exclusion_circle_count(); i++) {
        if (poly.get_exclusion_circle(i, centre_cm, radius_m)) {
            closest_m = MIN(closest_m, fabsf((point_NE_cm - centre_cm).length() * 0.01f - radius_m));
        }
    }
    for (uint8_t i = 0; want_incl_circ && i < poly.get_inclusion_circle_count(); i++) {
        if (poly.get_inclusion_circle(i, centre_cm, radius_m)) {
            closest_m = MIN(closest_m, fabsf((point_NE_cm - centre_cm).length() * 0.01f - radius_m));
        }
    }

    if (closest_m >= FLT_MAX) {
        return false;
    }
    distance_m = closest_m;
    return true;
#else
    (void)loc;
    (void)fence_type;
    (void)distance_m;
    return false;
#endif
}

// Lua binding to be used in Object Detection to find the neareast fence, ADS-B object, or proximity obstacle
//   the use of the word "obstacle" is intended to be generic, unfortunately one of the cases of ArduPilot is "Obstacles" stored in the AP_OAAvoidance library
// Note that the distance is the distance to any margin around the obstacles. AP_OAAvoidance obstacles have this as do fences. So the distance can be negative if you are too close.
bool AP_OAScripting::find_closest_obstacle(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                            float &distance_m, 
                                            uint16_t &type, 
                                            char *&label, 
                                            uint32_t &sysid,
                                            Location &location, 
                                            Vector3f &pos_NED_m,
                                            Vector3f &velocity_NED_ms
                                            ) const
{
    float distance_new_m = FLT_MAX;
    OAObstacle obstacle;

    distance_m = lookahead_m;

    // convert start and end to offsets from EKF origin (waiting for NEU/NED changes)
    Vector3f start_NED_m,end_NED_m;
    if (!start_loc.get_vector_from_origin_NEU_m(start_NED_m) ||
        !end_loc.get_vector_from_origin_NEU_m(end_NED_m)) {
        return false;
    }
    if (start_NED_m == end_NED_m) {
        return false;
    }
    // until we get the new NED functions
    start_NED_m.z   = -start_NED_m.z;
    end_NED_m.z     = -end_NED_m.z;

    // "obstacles" are stored in AP_Avoidance - the are typically populated by MAVLink (ADSB, GLOBAL_POSITION, FOLLOW_TARGET)
    // These have priority over all other obstacles, especially if they are ADSB messages representing crude aircraft
    OAObstacle obstacle_found;
    OAObstacle aircraft_found;
    distance_new_m = _distance_to_avoidance(start_NED_m, end_NED_m, obstacle_found, aircraft_found);
    if (distance_new_m < distance_m) {
        obstacle    = obstacle_found;
        distance_m  = distance_new_m;
    }

    // "objects" are stored in the AP_OADatabase - they are typically populated by proximity sensors
    distance_new_m = _distance_to_object(start_NED_m, end_NED_m, obstacle_found); 
    if (distance_new_m < distance_m) {
        obstacle    = obstacle_found;
        distance_m  = distance_new_m;
    }

#ifdef AP_FENCE_ENABLED
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence != nullptr) {
        // fences use cm (for now), so do this once now so we can pass to all the fence methods
        const Vector2f start_NE_cm(start_NED_m.x * 100.0f, start_NED_m.y * 100.0f);
        const Vector2f end_NE_cm(end_NED_m.x * 100.0f, end_NED_m.y * 100.0f);

        // We do each type of fence one at a time, because 
        // a. they are stored in separate lists and 
        // b. we want to tell the user which fence type of fence it is
        distance_new_m = fence->distance_line_to_home_inclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_HOME);
            distance_m      = distance_new_m;
        }
        distance_new_m = fence->distance_line_to_circle_inclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_CIRCLE_INCLUSION);
            distance_m      = distance_new_m;
        }
        distance_new_m = fence->distance_line_to_circle_exclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_CIRCLE_EXCLUSION);
            distance_m      = distance_new_m;
        }
        distance_new_m = fence->distance_line_to_polygon_inclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_POLYGON_INCLUSION);
            distance_m      = distance_new_m;
        }
        distance_new_m = fence->distance_line_to_polygon_exclusion(start_NE_cm, end_NE_cm);
        if (distance_new_m < distance_m) {
            _populate_fence_obstacle(obstacle, ObstacleType::FENCE_POLYGON_EXCLUSION);
            distance_m      = distance_new_m;
        }
    }
#endif
    if (distance_m < lookahead_m) {
        type            = (uint16_t)obstacle.obstacle_type;
        label           = obstacle.label;
        sysid           = obstacle.src_id;
        velocity_NED_ms = obstacle.velocity_NED_ms;

        if (obstacle.location.initialised()) {
            location        = obstacle.location;
            location.get_vector_from_origin_NEU_m(pos_NED_m);
            // until we get the new NED functions
            pos_NED_m.z = -pos_NED_m.z;
        }
        return true;
    }

    return false;
}

// Distance to objects in the AP_OADatabase
float AP_OAScripting::_distance_to_object(const Vector3f &start_NED_m, const Vector3f end_NED_m, OAObstacle &script_obstacle) const
{
    float distance_new_m = FLT_MAX;
    
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy() || oaDb->database_count() == 0) {
        return distance_new_m;
    }

    for (uint16_t i=0; i < oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        // result is distance between line segment and obstacle minus obstacle's radius
        const float distance_m = Vector3f::closest_distance_between_line_and_point(start_NED_m, end_NED_m, item.pos) - item.radius;
        if (distance_m < distance_new_m) {
            distance_new_m = distance_m;
            Vector3p item_pos_3p(item.pos.x, item.pos.y, item.pos.z);
            if (AP::ahrs().get_origin(script_obstacle.location)) {
                script_obstacle.location.offset(item_pos_3p);
            }
            switch(item.source) {
            case AP_OADatabase::OA_DbItem::Source::proximity:
                script_obstacle.label           = (char *)"proximity";
                script_obstacle.obstacle_type   = static_cast<uint8_t>(ObstacleType::PROXIMITY);
                break;
            case AP_OADatabase::OA_DbItem::Source::AIS:
                script_obstacle.label           = (char *)"AIS";
                script_obstacle.obstacle_type   = static_cast<uint8_t>(ObstacleType::AIS);
                break;
            }
            script_obstacle.src_id              = -1;
        }
    }

    return distance_new_m;
}

// translate an AP_Avoidance obstacle src_id into a string for display purposes
char* AP_OAScripting::_get_obstacle_label(uint8_t emitter_type, int32_t obstacle_id)
{
    if (AP_Avoidance::is_adsb_uav(emitter_type) || obstacle_id < 0x3FFF) {
        return (char *)"MAV";
    }
    else if (obstacle_id >= 0xB00000 and obstacle_id <= 0xB10000) {
        return (char *)"Weather";
    }
    else if (obstacle_id >= 0xB10000 and obstacle_id <= 0xB20000) {
        return (char *)"Migratory Bird";
    }
    else if (obstacle_id >= 0xB20000 and obstacle_id <= 0xB30000) {
        return (char *)"Bird of Prey";
    }
    else if(obstacle_id > 0xA00000) {
        return (char *)"ADS-B";
    }
    return (char *)"Unknown";
}

// translate an AP_Avoidance obstacle src_id into an enum for further processing in Lua
AP_OAScripting::ObstacleType AP_OAScripting::_get_obstacle_type(uint8_t emitter_type, int32_t icao_code)
{
    if (AP_Avoidance::is_adsb_uav(emitter_type) || icao_code <= 0x0BFFF) {
        return ObstacleType::MAV_SYSID;
    }
    else if (icao_code >= 0xB00000 and icao_code <= 0xB10000) {
        return ObstacleType::WEATHER;
    }
    else if (icao_code >= 0xB10000 and icao_code <= 0xB20000) {
        return ObstacleType::BIRD_MIGRATORY;
    }
    else if (icao_code >= 0xB20000 and icao_code <= 0xB30000) {
        return ObstacleType::BIRD_OF_PREY;
    }
    else if(AP_Avoidance::is_adsb_aircraft(emitter_type)) {
       return ObstacleType::GENERAL_AVIATION;
    }
    return ObstacleType::GENERAL;
}

// create a "Scripting Obstacle" to easily pass info about an obstacle to Lua for fences
void AP_OAScripting::_populate_fence_obstacle(OAObstacle &fence_obstacle, AP_OAScripting::ObstacleType obstacle_type)
{
    fence_obstacle.timestamp_ms    = AP_HAL::millis();
    fence_obstacle.obstacle_type   = static_cast<uint8_t>(obstacle_type);
    fence_obstacle.src_id          = 0;
    fence_obstacle.icao_code       = 0;
    fence_obstacle.emitter_type    = 0;
    fence_obstacle.is_aircraft     = false;
    fence_obstacle.is_drone        = false;
    switch (obstacle_type)
    {
    case AP_OAScripting::ObstacleType::FENCE_HOME:
        fence_obstacle.label    = (char *)"Tin Can";
        break;
    case AP_OAScripting::ObstacleType::FENCE_CIRCLE_INCLUSION:
        fence_obstacle.label    = (char *)"Incl. Circle";
        break;
    case AP_OAScripting::ObstacleType::FENCE_CIRCLE_EXCLUSION:
        fence_obstacle.label    = (char *)"Excl. Circle";
        break;
    case AP_OAScripting::ObstacleType::FENCE_POLYGON_INCLUSION:
        fence_obstacle.label    = (char *)"Incl. Polygon";
        break;
    case AP_OAScripting::ObstacleType::FENCE_POLYGON_EXCLUSION:
        fence_obstacle.label    = (char *)"Excl. Polygon";
        break;
    case AP_OAScripting::ObstacleType::FENCE_LUA:
        fence_obstacle.label    = (char *)"Lua Fence";
        break;
    case AP_OAScripting::ObstacleType::FENCE_ALT_MAX:
        fence_obstacle.label    = (char *)"Alt Max Fence";
        break;
    case AP_OAScripting::ObstacleType::FENCE_ALT_MIN:
        fence_obstacle.label    = (char *)"Alt Min Fence";
        break;
    default:
        fence_obstacle.label    = (char *)"Geo Fence";
        break;
    }
}

// create a "Scripting Obstacle" to easily pass info about an obstacle to Lua for avoidance obstacles
void AP_OAScripting::_populate_scripting_obstacle(OAObstacle &script_obstacle, const AP_Avoidance::Obstacle *avoid_obstacle)
{
    script_obstacle.timestamp_ms    = avoid_obstacle->timestamp_ms;

    script_obstacle.obstacle_type   = static_cast<uint8_t>(_get_obstacle_type(avoid_obstacle->emitter_type, script_obstacle.icao_code));
    script_obstacle.label           = _get_obstacle_label(avoid_obstacle->emitter_type, avoid_obstacle->src_id);
    script_obstacle.src_id          = avoid_obstacle->src_id;

    script_obstacle.icao_code       = avoid_obstacle->src_id & 0xFFFFFF;
    script_obstacle.emitter_type    = avoid_obstacle->emitter_type;

    script_obstacle.is_aircraft     = AP_Avoidance::is_adsb_aircraft(avoid_obstacle->emitter_type);
    script_obstacle.is_drone        = AP_Avoidance::is_adsb_uav(avoid_obstacle->emitter_type);

    script_obstacle.velocity_NED_ms = avoid_obstacle->_velocity_ned_ms;
    script_obstacle.location        = avoid_obstacle->_location;
    if(script_obstacle.location.initialised()) {
        script_obstacle.location.get_vector_from_origin_NEU_m(script_obstacle.position_NED_m);
        // until we get the new NED functions
        script_obstacle.position_NED_m.z = -script_obstacle.position_NED_m.z;
    }
}

//
// AP_Avoidance Overrides
// the AP_Avoidance data base contains ADSB objects and a model for mapping threats that works for aircraft
// but doesn't consider other (non aircraft) threats. This is an attempt to selectively use whats
// useful in AP_Avoidance (a lot), while overriding and parameterizing what is not.
//

// Distance to objects in the AP_Avoidance database from a line from START_NED_cm to  end_NED_cm
float AP_OAScripting::_distance_to_avoidance(const Vector3f &start_NED_cm, const Vector3f &end_NED_cm, 
                                                // return values
                                                OAObstacle &script_any_obstacle,
                                                OAObstacle &script_aircraft_obstacle
                                                ) const
{
    AP_Avoidance *avoid = AP_Avoidance::get_singleton();
    AP_Avoidance::Obstacle any_avoidance;
    AP_Avoidance::Obstacle aircraft_avoidance;

    float distance_m = avoid->distance_to_obstacle(start_NED_cm, end_NED_cm, any_avoidance, aircraft_avoidance);
    if (distance_m < FLT_MAX) {
        _populate_scripting_obstacle(script_any_obstacle, &any_avoidance);
        _populate_scripting_obstacle(script_aircraft_obstacle, &aircraft_avoidance);
    }
    return distance_m;
}

// Closest Distance to aircraft in the AP_Avoidance database from a single point
float AP_OAScripting::_distance_to_aircraft(const Vector3f &vehicle_NED_cm, const float lookahead_m, const float vertical_lookahead_m,
                                                // return values
                                                OAObstacle &script_obstacle
                                                ) const
{
    AP_Avoidance *avoid = AP_Avoidance::get_singleton();
    AP_Avoidance::Obstacle avoid_obstacle;

    float distance_m = avoid->distance_to_aircraft(vehicle_NED_cm, FLT_MAX, vertical_lookahead_m, avoid_obstacle);
    if (distance_m < lookahead_m) {
        _populate_scripting_obstacle(script_obstacle, &avoid_obstacle);
    }
    return distance_m;
}

#endif // AP_OA_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)
