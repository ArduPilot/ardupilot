#pragma once

#include "AC_Avoidance_config.h"

#if AP_OAPATHPLANNER_BENDYRULER_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger_config.h>

/*
 * BendyRuler avoidance algorithm for avoiding the polygon and circular fence and dynamic objects detected by the proximity sensor
 */
class AP_OABendyRuler {
public:
    AP_OABendyRuler();

    CLASS_NO_COPY(AP_OABendyRuler);  /* Do not allow copies */

    // send configuration info stored in front end parameters
    void set_config(float margin_max) { _margin_max = MAX(margin_max, 0.0f); }

    enum class OABendyType {
        OA_BENDY_DISABLED   = 0,
        OA_BENDY_HORIZONTAL = 1,
        OA_BENDY_VERTICAL   = 2,
    };

    // run background task to find best path
    // returns true and updates origin_new and destination_new if a best path has been found.  returns false if OA is not required
    // bendy_type is set to the type of BendyRuler used
    bool update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, OABendyType &bendy_type, bool proximity_only);

    static const struct AP_Param::GroupInfo var_info[];

private:

    // return type of BendyRuler in use
    OABendyType get_type() const;

    // search for path in XY direction
    bool search_xy_path(const Location& current_loc, const Location& destination, float ground_course_deg, Location &destination_new, float lookahead_step_1_dist, float lookahead_step_2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only);

    // search for path in the Vertical directions
    bool search_vertical_path(const Location &current_loc, const Location &destination, Location &destination_new, float lookahead_step1_dist, float lookahead_step2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only);

    // calculate minimum distance between a path and any obstacle
    float calc_avoidance_margin(const Location &start, const Location &end, bool proximity_only) const;

    // determine if BendyRuler should accept the new bearing or try and resist it. Returns true if bearing is not changed  
    bool resist_bearing_change(const Location &destination, const Location &current_loc, bool active, float bearing_test, float lookahead_step1_dist, float margin, Location &prev_dest, float &prev_bearing, float &final_bearing, float &final_margin, bool proximity_only) const;    

    // calculate minimum distance between a path and the circular fence (centered on home)
    // on success returns true and updates margin
    bool calc_margin_from_circular_fence(const Location &start, const Location &end, float &margin) const;

    // calculate minimum distance between a path and the altitude fence
    // on success returns true and updates margin
    bool calc_margin_from_alt_fence(const Location &start, const Location &end, float &margin) const;

    // calculate minimum distance between a path and all inclusion and exclusion polygons,
    // except the exclusion polygon with index skip_exclusion_index (if any)
    // on success returns true and updates margin
    bool calc_margin_from_inclusion_and_exclusion_polygons(const Location &start, const Location &end, float &margin, int16_t skip_exclusion_index = -1) const;

    // calculate minimum distance between a path and all inclusion and exclusion circles,
    // except the exclusion circle with index skip_exclusion_index (if any)
    // on success returns true and updates margin
    bool calc_margin_from_inclusion_and_exclusion_circles(const Location &start, const Location &end, float &margin, int16_t skip_exclusion_index = -1) const;

    // if current_loc is inside an exclusion polygon or circle, find the closest point outside it,
    // placed the fence margin plus OA_MARGIN_MAX beyond its edge, that clears every fence and whose
    // route from current_loc crosses no fence or obstacle other than the zone being left
    // returns true and updates exit_loc if such a point was found
    bool calc_exclusion_zone_exit(const Location &current_loc, Location &exit_loc) const;

    // returns true if the exit at candidate_NE, an offset in cm from the EKF origin, clears every fence,
    // and the route to it from current_loc crosses no fence or obstacle other than the exclusion polygon
    // or circle being left, given by its index (-1 for neither)
    // candidate_loc is set to the exit's location
    bool exclusion_zone_exit_clear(const Location &current_loc, const Vector2f &current_NE, const Vector2f &candidate_NE, int16_t polygon_index, int16_t circle_index, Location &candidate_loc) const;

    // calculate minimum distance between a path and proximity sensor obstacles
    // on success returns true and updates margin
    bool calc_margin_from_object_database(const Location &start, const Location &end, float &margin) const;

    // Logging function
#if HAL_LOGGING_ENABLED
    void Write_OABendyRuler(const uint8_t type, const bool active, const float target_yaw, const float target_pitch, const bool resist_chg, const float margin, const Location &final_dest, const Location &oa_dest) const;
#else
    void Write_OABendyRuler(const uint8_t type, const bool active, const float target_yaw, const float target_pitch, const bool resist_chg, const float margin, const Location &final_dest, const Location &oa_dest) const {}
#endif

    // OA common parameters
    float _margin_max;              // minimum distance in meters to keep from obstacles and fences
    
    // BendyRuler parameters
    AP_Float _lookahead;            // object avoidance will look this many meters ahead of vehicle
    AP_Float _bendy_ratio;          // object avoidance will avoid major directional change if change in margin ratio is less than this param
    AP_Int16 _bendy_angle;          // object avoidance will try avoiding change in direction over this much angle
    AP_Int8  _bendy_type;           // Type of BendyRuler to run
    
    // internal variables used by background thread
    float _current_lookahead;       // distance (in meters) ahead of the vehicle we are looking for obstacles
    float _bearing_prev;            // stored bearing in degrees 
    Location _destination_prev;     // previous destination, to check if there has been a change in destination
};

#endif  // AP_OAPATHPLANNER_BENDYRULER_ENABLED
