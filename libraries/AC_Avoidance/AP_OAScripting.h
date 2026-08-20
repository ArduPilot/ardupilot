#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include "AC_Avoidance_config.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Avoidance/AP_Avoidance_config.h>

// AP_OA_SCRIPTING_ENABLED is defined in AP_Avoidance_config.h, included above, so
// that this library and the AP_Avoidance queries and AVD_ parameters it depends on
// compile in and out as one unit.  It is ALWAYS defined (to 0 or 1), so
// "#if AP_OA_SCRIPTING_ENABLED" is valid under -Werror=undef wherever this header
// is included.

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
        // Named AIRCRAFT, not "ADSB", on purpose: the source being ADS-B does not
        // fix the type. An ADS-B contact with a UAV emitter is a drone (MAV_SYSID above),
        // so "ADSB" would span two ObstacleTypes. This one is specifically crewed aircraft.
        CREWED_AIRCRAFT             = 2,
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



    // Returns the single closest obstacle to the line, whichever source it came from:
    // ADS-B objects from AP_Avoidance
    // Proximity objects from AP_OADatabase
    // Fences from AC_Fence
    // Crewed aircraft are found separately by find_aircraft(), which applies the
    // caller's vertical gate rather than the per-emitter table.
    bool find_threats(const Location &start_loc, const Location &end_loc, float lookahead_m,
                                                // Return values
                                                float       &distance_m,
                                                OAObstacle  &any_obstacle
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


private:

    static AP_OAScripting *_singleton;

    float _distance_to_avoidance(const Vector3f &start_NED_cm, const Vector3f &end_NED_cm, OAObstacle &script_obstacle) const;
    float _distance_to_object(const Vector3f &start_NED_m, const Vector3f end_NED_m, OAObstacle &script_obstacle) const;
    float _distance_to_aircraft(const Vector3f &vehicle_NED_cm, const float lookahead_m, const float vertical_lookahead_m, OAObstacle &script_obstacle) const;

    // create a "Scripting Obstacle" to easily pass info about an obstacle to Lua
    static void _populate_scripting_obstacle(OAObstacle &scripting_obstacle, const AP_Avoidance::Obstacle *avoidance_obstacle);
    static void _populate_fence_obstacle(OAObstacle &fence_obstacle, AP_OAScripting::ObstacleType obstacle_type);

    static ObstacleType _get_obstacle_type(uint8_t emitter_type, int32_t obstacle_id);

    // Properties to work around Lua binding problem of the binding generator not being able
    // to pass in Locations and return a number of other values
    Location    _lua_start_loc;
    Location    _lua_end_loc;

    // parameters
};

struct OAObstacle {
    public:
    uint32_t                    timestamp_ms;
    uint32_t                    src_id;         // The AP_Avoid src_id
    uint32_t                    icao_code;      // The ICAO code (if relevant) from ADSB
    uint8_t                     emitter_type;   // The ADSB_EMITTER of the obstacle (if relevant)
    uint8_t                     obstacle_type;

    Vector3f                    velocity_NED_ms;
    Location                    location;
    Vector3f                    position_NED_m;
};

#endif // AP_OA_SCRIPTING_ENABLED
