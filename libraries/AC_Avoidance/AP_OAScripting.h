#pragma once

#include <AP_Scripting/AP_Scripting_config.h>
#include "AC_Avoidance_config.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Avoidance/AP_Avoidance_config.h>

// AP_OAScripting uses the AP_Avoidance (ADS-B) class, which exists only when
// AP_ADSB_AVOIDANCE_ENABLED (= HAL_ADSB_ENABLED, false on 1MB boards such as
// fmuv2). The matching applet (planedaa.lua) is Plane-only, so gate the feature
// to ArduPlane + scripting + avoidance + ADS-B avoidance. The macro is ALWAYS
// defined (to 0 or 1) so "#if AP_OA_SCRIPTING_ENABLED" is valid under
// -Werror=undef wherever this header is included. The build-type macro lives
// here (so the class is not declared on other vehicles), hence this header is
// whitelisted in Tools/ardupilotwaf/ap_library.py. The #ifndef allows an
// explicit override if ever needed.
#ifndef AP_OA_SCRIPTING_ENABLED
#define AP_OA_SCRIPTING_ENABLED (AP_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane) && AP_AVOIDANCE_ENABLED && AP_ADSB_AVOIDANCE_ENABLED)
#endif

#if AP_OA_SCRIPTING_ENABLED

#include "AP_OADatabase.h"
#include <AC_Fence/AC_Fence.h>
#include <AP_Avoidance/AP_Avoidance.h>

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include "AP_OAVisGraph.h"
#include <AP_Logger/AP_Logger_config.h>
#include <GCS_MAVLink/GCS.h>

// Forward declare the class and its nested struct cleanly
class AP_Avoidance; 

struct OAObstacle;

class AP_OAScripting {
public:

    AP_OAScripting();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OAScripting);

    static AP_OAScripting *get_singleton() {
        return _singleton;
    }

    // enum of obstacle types for passing back to Lua
    enum class ObstacleType : uint8_t {
        GENERAL                     = 0,
        MAV_SYSID                   = 1,
        GENERAL_AVIATION            = 2,
        WEATHER                     = 3,
        BIRD_MIGRATORY              = 4,
        BIRD_OF_PREY                = 5,
        FENCE_HOME                  = 6,
        FENCE_CIRCLE_INCLUSION      = 7,
        FENCE_CIRCLE_EXCLUSION      = 8,
        FENCE_POLYGON_INCLUSION     = 9,
        FENCE_POLYGON_EXCLUSION     = 10,
        FENCE_LUA                   = 11,
        PROXIMITY                   = 12,
        AIS                         = 13,
        FENCE_ALT_MAX               = 14,   // max altitude fence (AC_FENCE_TYPE_ALT_MAX, FENCE_TYPE bit 0)
        FENCE_ALT_MIN               = 15,   // min altitude fence (AC_FENCE_TYPE_ALT_MIN, FENCE_TYPE bit 3)
    };



    // For efficiency don't want to loop through the obstacles multiple times. so this 
    // returns both the closes aircraft (from ADS-B) and the closest obstacle in general which might be
    // ADS-B Objects from AP_Avoidance
    // Proximity objects from AP_OADatabase
    // Fences from AC_Fence
    bool find_threats(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                                // Return values
                                                float       &distance_m,
                                                OAObstacle  &any_obstacle,
                                                OAObstacle  &aircraft_obstacle,
                                                OAObstacle  &proximity_obstacle,
                                                OAObstacle  &fence_obstacle
                                                ) const;

    // This function is for Lua, so each Obstacle field gets returned as multiple parameters
    bool find_closest_obstacle(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                    // Return values
                                    float    &distance_m, 
                                    uint16_t &type, 
                                    char    *&label, 
                                    uint32_t &sys_id,
                                    Location &location, 
                                    Vector3f &pos_NED_m,
                                    Vector3f &velocity_NED_ms
                                ) const;
    
    bool find_aircraft(const Location &vehicle_loc, const float lookahead_m, const float vertical_lookahead_m,
                                    float       &distance_m,
                                    OAObstacle  &aircraft_obstacle
                            ) const;

    // closest distance (metres) from a location to the nearest fence boundary edge.  For Lua so
    // the AVOIDING message can report a real distance to a fence obstacle (which, unlike ADS-B
    // point obstacles, has no single usable "location").  fence_type scopes the search to the
    // matching fence category (an AP_OAScripting::ObstacleType FENCE_* value) so the reported
    // distance belongs to the same kind of fence the message names; pass 0 (GENERAL) or any
    // non-fence value to search all polygon/circle fences.  Returns true and sets distance_m if a
    // matching polygon/circle fence is found, false otherwise.
    bool fence_distance(const Location &loc, uint8_t fence_type, float &distance_m) const;
    // This function fines the closed crude aircraft by checking the ADS-B database
    /*bool find_closest_aircraft(const Location &vehicle_loc, float lookahead_m,
                                            float &distance_m, 
                                            uint16_t &type, 
                                            char *&label, 
                                            uint32_t &sysid,
                                            Location &location, 
                                            Vector3f &pos_NED_m,
                                            Vector3f &velocity_NED_ms
                                            ) const;*/

    // This function is for Lua, so each Obstacle field gets returned as multiple parameters
    bool distance_obstacle_test(const Location &start_loc, const Location &end_loc, const float &lookahead_m, float &distance_min_m, Location &location_out) const;
    //bool distance_obstacle_test2(const float &lookahead_m, float &distance_min_m) const;

    // Setup for parameters under "AVD" so "AVD_SCR"
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_OAScripting *_singleton;

    float _distance_to_avoidance(const Vector3f &start_NED_cm, const Vector3f &end_NED_cm, OAObstacle &script_obstacle, OAObstacle &aircraft_obstacle) const;
    float _distance_to_object(const Vector3f &start_NED_m, const Vector3f end_NED_m, OAObstacle &script_obstacle) const;
    float _distance_to_aircraft(const Vector3f &vehicle_NED_cm, const float lookahead_m, const float vertical_lookahead_m, OAObstacle &script_obstacle) const;

    // create a "Scripting Obstacle" to easily pass info about an obstacle to Lua
    static void _populate_scripting_obstacle(OAObstacle &scripting_obstacle, const AP_Avoidance::Obstacle *avoidance_obstacle);
    static void _populate_fence_obstacle(OAObstacle &fence_obstacle, AP_OAScripting::ObstacleType obstacle_type);

    static ObstacleType _get_obstacle_type(uint8_t emitter_type, int32_t obstacle_id);
    static char* _get_obstacle_label(uint8_t emitter_type, int32_t obstacle_id);

    // Properties to work around Lua binding problem of the binding generator not being able
    // to pass in Locations and return a number of other values
    Location    _lua_start_loc;
    Location    _lua_end_loc;

    // parameters
    AP_Float _margin_aircraft;     // what is the margin around aircraft we need to avoid?
};

struct OAObstacle {
    public:
    uint32_t                    timestamp_ms;
    uint32_t                    src_id;         // The AP_Avoid src_id
    uint32_t                    icao_code;      // The ICAO code (if relevant) from ADSB
    uint32_t                    emitter_type;   // The ADSB_EMITTER of the obstacle (if relevant)
    bool                        is_aircraft;    // Whether or not this is an aircraft
    bool                        is_drone;       // Whether or not this is a drone/UAV
    uint8_t                     obstacle_type;
    char                        *label;

    Vector3f                    velocity_NED_ms;
    Location                    location;
    Vector3f                    position_NED_m;

    float                       margin_m;       // only for fences
    float                       radius_m;       // only for circular fences
};

#endif // AP_OA_SCRIPTING_ENABLED
