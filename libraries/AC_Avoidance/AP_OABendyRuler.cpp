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

#include "AP_OABendyRuler.h"
#include <AC_Avoidance/AP_OADatabase.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

// parameter defaults
const float OA_BENDYRULER_LOOKAHEAD_DEFAULT = 15.0f;
const float OA_BENDYRULER_RATIO_DEFAULT = 1.5f;
const int16_t OA_BENDYRULER_ANGLE_DEFAULT = 75;
const int16_t OA_BENDYRULER_TYPE_DEFAULT = 1;

const int16_t OA_BENDYRULER_BEARING_INC_XY = 5;            // check every 5 degrees around vehicle
const int16_t OA_BENDYRULER_BEARING_INC_VERTICAL = 90;
const float OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO = 1.0f; // step2's lookahead length as a ratio of step1's lookahead length
const float OA_BENDYRULER_LOOKAHEAD_STEP2_MIN = 2.0f;   // step2 checks at least this many meters past step1's location
const float OA_BENDYRULER_LOOKAHEAD_PAST_DEST = 2.0f;   // lookahead length will be at least this many meters past the destination
const float OA_BENDYRULER_LOW_SPEED_SQUARED = (0.2f * 0.2f);    // when ground course is below this speed squared, vehicle's heading will be used

#define VERTICAL_ENABLED APM_BUILD_TYPE(APM_BUILD_ArduCopter)

const AP_Param::GroupInfo AP_OABendyRuler::var_info[] = {

    // @Param: LOOKAHEAD
    // @DisplayName: Object Avoidance look ahead distance maximum
    // @Description: Object Avoidance will look this many meters ahead of vehicle
    // @Units: m
    // @Range: 1 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOOKAHEAD", 1, AP_OABendyRuler, _lookahead, OA_BENDYRULER_LOOKAHEAD_DEFAULT),

    // @Param: CONT_RATIO
    // @DisplayName: Obstacle Avoidance margin ratio for BendyRuler to change bearing significantly 
    // @Description:  BendyRuler will avoid changing bearing unless ratio of previous margin from obstacle (or fence) to present calculated margin is atleast this much.
    // @Range: 1.1 2
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CONT_RATIO", 2, AP_OABendyRuler, _bendy_ratio, OA_BENDYRULER_RATIO_DEFAULT),

    // @Param: CONT_ANGLE
    // @DisplayName: BendyRuler's bearing change resistance threshold angle   
    // @Description:  BendyRuler will resist changing current bearing if the change in bearing is over this angle
    // @Range: 20 180
    // @Increment: 5
    // @User: Standard
    AP_GROUPINFO("CONT_ANGLE", 3, AP_OABendyRuler, _bendy_angle, OA_BENDYRULER_ANGLE_DEFAULT),

    // @Param{Copter}: TYPE
    // @DisplayName: Type of BendyRuler
    // @Description: BendyRuler will search for clear path along the direction defined by this parameter
    // @Values: 1:Horizontal search, 2:Vertical search
    // @User: Standard
    AP_GROUPINFO_FRAME("TYPE", 4, AP_OABendyRuler, _bendy_type, OA_BENDYRULER_TYPE_DEFAULT, AP_PARAM_FRAME_COPTER | AP_PARAM_FRAME_HELI | AP_PARAM_FRAME_TRICOPTER),

    AP_GROUPEND
};

AP_OABendyRuler::AP_OABendyRuler() 
{ 
    AP_Param::setup_object_defaults(this, var_info); 
    _bearing_prev = FLT_MAX;
}

// run background task to find best path and update avoidance_results
// returns true and updates origin_new and destination_new if a best path has been found
// bendy_type is set to the type of BendyRuler used
bool AP_OABendyRuler::update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new, OABendyType &bendy_type, bool proximity_only)
{
    // bendy ruler always sets origin to current_loc
    origin_new = current_loc;

    // init bendy_type returned
    bendy_type = OABendyType::OA_BENDY_DISABLED;

    // calculate bearing and distance to final destination
    const float bearing_to_dest = current_loc.get_bearing_to(destination) * 0.01f;
    const float distance_to_dest = current_loc.get_distance(destination);

    // make sure user has set a meaningful value for _lookahead
    _lookahead = MAX(_lookahead,1.0f);

    // lookahead distance is adjusted dynamically based on avoidance results
    _current_lookahead = constrain_float(_current_lookahead, _lookahead * 0.5f, _lookahead);

    // calculate lookahead dist and time for step1.  distance can be slightly longer than
    // the distance to the destination to allow room to dodge after reaching the destination
    const float lookahead_step1_dist = MIN(_current_lookahead, distance_to_dest + OA_BENDYRULER_LOOKAHEAD_PAST_DEST);

    // calculate lookahead dist for step2
    const float lookahead_step2_dist = _current_lookahead * OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO;

    // get ground course
    float ground_course_deg;
    if (ground_speed_vec.length_squared() < OA_BENDYRULER_LOW_SPEED_SQUARED) {
        // with zero ground speed use vehicle's heading
        ground_course_deg = AP::ahrs().yaw_sensor * 0.01f;
    } else {
        ground_course_deg = degrees(ground_speed_vec.angle());
    }

    bool ret;
    switch (get_type()) {
        case OABendyType::OA_BENDY_VERTICAL:
        #if VERTICAL_ENABLED 
            ret = search_vertical_path(current_loc, destination, destination_new, lookahead_step1_dist, lookahead_step2_dist, bearing_to_dest, distance_to_dest, proximity_only);
            bendy_type = OABendyType::OA_BENDY_VERTICAL;
            break;
        #endif

        case OABendyType::OA_BENDY_HORIZONTAL:
        default:
            ret = search_xy_path(current_loc, destination, ground_course_deg, destination_new, lookahead_step1_dist, lookahead_step2_dist, bearing_to_dest, distance_to_dest, proximity_only);
            bendy_type = OABendyType::OA_BENDY_HORIZONTAL;
    }

    return ret;
}

// Search for path in the horizontal directions
bool AP_OABendyRuler::search_xy_path(const Location& current_loc, const Location& destination, float ground_course_deg, Location &destination_new, float lookahead_step1_dist, float lookahead_step2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only) 
{
    // check OA_BEARING_INC definition allows checking in all directions
    static_assert(360 % OA_BENDYRULER_BEARING_INC_XY == 0, "check 360 is a multiple of OA_BEARING_INC");

    // search in OA_BENDYRULER_BEARING_INC degree increments around the vehicle alternating left
    // and right. For each direction check if vehicle would avoid all obstacles
    float best_bearing = bearing_to_dest;
    float best_bearing_margin = -FLT_MAX;
    bool have_best_bearing = false;
    float best_margin = -FLT_MAX;
    float best_margin_bearing = best_bearing;

    for (uint8_t i = 0; i <= (170 / OA_BENDYRULER_BEARING_INC_XY); i++) {
        for (uint8_t bdir = 0; bdir <= 1; bdir++) {
            // skip duplicate check of bearing straight towards destination
            if ((i==0) && (bdir > 0)) {
                continue;
            }
            // bearing that we are probing
            const float bearing_delta = i * OA_BENDYRULER_BEARING_INC_XY * (bdir == 0 ? -1.0f : 1.0f);
            const float bearing_test = wrap_180(bearing_to_dest + bearing_delta);

            // ToDo: add effective groundspeed calculations using airspeed
            // ToDo: add prediction of vehicle's position change as part of turn to desired heading

            // test location is projected from current location at test bearing
            Location test_loc = current_loc;
            test_loc.offset_bearing(bearing_test, lookahead_step1_dist);

            // calculate margin from obstacles for this scenario
            float margin = calc_avoidance_margin(current_loc, test_loc, proximity_only);
            if (margin > best_margin) {
                best_margin_bearing = bearing_test;
                best_margin = margin;
            }
            if (margin > _margin_max) {
                // this bearing avoids obstacles out to the lookahead_step1_dist
                // now check in there is a clear path in three directions towards the destination
                if (!have_best_bearing) {
                    best_bearing = bearing_test;
                    best_bearing_margin = margin;
                    have_best_bearing = true;
                } else if (fabsf(wrap_180(ground_course_deg - bearing_test)) <
                           fabsf(wrap_180(ground_course_deg - best_bearing))) {
                    // replace bearing with one that is closer to our current ground course
                    best_bearing = bearing_test;
                    best_bearing_margin = margin;
                }

                // perform second stage test in three directions looking for obstacles
                const float test_bearings[] { 0.0f, 45.0f, -45.0f };
                const float bearing_to_dest2 = test_loc.get_bearing_to(destination) * 0.01f;
                float distance2 = constrain_float(lookahead_step2_dist, OA_BENDYRULER_LOOKAHEAD_STEP2_MIN, test_loc.get_distance(destination));
                for (uint8_t j = 0; j < ARRAY_SIZE(test_bearings); j++) {
                    float bearing_test2 = wrap_180(bearing_to_dest2 + test_bearings[j]);
                    Location test_loc2 = test_loc;
                    test_loc2.offset_bearing(bearing_test2, distance2);

                    // calculate minimum margin to fence and obstacles for this scenario
                    float margin2 = calc_avoidance_margin(test_loc, test_loc2, proximity_only);
                    if (margin2 > _margin_max) {
                        // if the chosen direction is directly towards the destination avoidance can be turned off
                        // i == 0 && j == 0 implies no deviation from bearing to destination 
                        const bool active = (i != 0 || j != 0);
                        float final_bearing = bearing_test;
                        float final_margin = margin;
                        // check if we need ignore test_bearing and continue on previous bearing
                        const bool ignore_bearing_change = resist_bearing_change(destination, current_loc, active, bearing_test, lookahead_step1_dist, margin, _destination_prev,_bearing_prev, final_bearing, final_margin, proximity_only);

                        // all good, now project in the chosen direction by the full distance
                        destination_new = current_loc;
                        destination_new.offset_bearing(final_bearing, MIN(distance_to_dest, lookahead_step1_dist));
                        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.1f);
                        Write_OABendyRuler((uint8_t)OABendyType::OA_BENDY_HORIZONTAL, active, bearing_to_dest, 0.0f, ignore_bearing_change, final_margin, destination, destination_new);
                        return active;
                    }
                }
            }
        }
    }

    float chosen_bearing;
    float chosen_distance;
    if (have_best_bearing) {
        // none of the directions tested were OK for 2-step checks. Choose the direction
        // that was best for the first step
        chosen_bearing = best_bearing;
        chosen_distance = MAX(lookahead_step1_dist + MIN(best_bearing_margin, 0), 0);
        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.05f);
    } else {
        // none of the possible paths had a positive margin. Choose
        // the one with the highest margin
        chosen_bearing = best_margin_bearing;
        chosen_distance = MAX(lookahead_step1_dist + MIN(best_margin, 0), 0);
        _current_lookahead = MAX(_lookahead * 0.5f, _current_lookahead * 0.9f);
    }

    // calculate new target based on best effort
    destination_new = current_loc;
    destination_new.offset_bearing(chosen_bearing, chosen_distance);

    // log results
    Write_OABendyRuler((uint8_t)OABendyType::OA_BENDY_HORIZONTAL, true, chosen_bearing, 0.0f, false, best_margin, destination, destination_new);

    return true;
}

// Search for path in the vertical directions
bool AP_OABendyRuler::search_vertical_path(const Location &current_loc, const Location &destination, Location &destination_new, float lookahead_step1_dist, float lookahead_step2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only)
{
    // check OA_BEARING_INC_VERTICAL definition allows checking in all directions
    static_assert(360 % OA_BENDYRULER_BEARING_INC_VERTICAL == 0, "check 360 is a multiple of OA_BEARING_INC_VERTICAL");
    float best_pitch = 0.0f;
    bool  have_best_pitch = false;
    float best_margin = -FLT_MAX;
    float best_margin_pitch = best_pitch;
    const uint8_t angular_limit = 180 / OA_BENDYRULER_BEARING_INC_VERTICAL;

    for (uint8_t i = 0; i <= angular_limit; i++) {
        for (uint8_t bdir = 0; bdir <= 1; bdir++) {
            // skip duplicate check of bearing straight towards destination or 180 degrees behind
            if (((i==0) && (bdir > 0)) || ((i == angular_limit) && (bdir > 0))) {
                continue;
            }

            // bearing that we are probing
            const float pitch_delta = i * OA_BENDYRULER_BEARING_INC_VERTICAL * (bdir == 0 ? 1.0f : -1.0f);

            Location test_loc = current_loc;
            test_loc.offset_bearing_and_pitch(bearing_to_dest, pitch_delta, lookahead_step1_dist);

            // calculate margin from obstacles for this scenario
            float margin = calc_avoidance_margin(current_loc, test_loc, proximity_only);

            if (margin > best_margin) {
                best_margin_pitch = pitch_delta;
                best_margin = margin;
            }

            if (margin > _margin_max) {
                // this path avoids the obstacles with the required margin, now check for the path ahead
                if (!have_best_pitch) {
                    best_pitch = pitch_delta;
                    have_best_pitch = true;
                }
                const float test_pitch_step2[] { 0.0f, 90.0f, -90.0f, 180.0f};
                float bearing_to_dest2;
                if (is_equal(fabsf(pitch_delta), 90.0f)) {
                    bearing_to_dest2 = bearing_to_dest; 
                } else { 
                    bearing_to_dest2 = test_loc.get_bearing_to(destination) * 0.01f;
                }
                float distance2 = constrain_float(lookahead_step2_dist, OA_BENDYRULER_LOOKAHEAD_STEP2_MIN, test_loc.get_distance(destination));

                for (uint8_t j = 0; j < ARRAY_SIZE(test_pitch_step2); j++) {
                    float bearing_test2 = wrap_180(test_pitch_step2[j]);
                    Location test_loc2 = test_loc;
                    test_loc2.offset_bearing_and_pitch(bearing_to_dest2, bearing_test2, distance2);

                    // calculate minimum margin to fence and obstacles for this scenario
                    float margin2 = calc_avoidance_margin(test_loc, test_loc2, proximity_only);
                    if (margin2 > _margin_max) {
                        // if the chosen direction is directly towards the destination we might turn off avoidance
                        // i == 0 && j == 0 implies no deviation from bearing to destination 
                        bool active = (i != 0 || j != 0);
                        if (!active) {
                            // do a sub test for proximity obstacles to confirm if we should really turn of BendyRuler
                            const float sub_test_pitch_step2[] {-90.0f, 90.0f};
                            for (uint8_t k = 0; k < ARRAY_SIZE(sub_test_pitch_step2); k++) {
                                Location test_loc_sub_test = test_loc;
                                test_loc_sub_test.offset_bearing_and_pitch(bearing_to_dest2, sub_test_pitch_step2[k], _margin_max);
                                float margin_sub_test = calc_avoidance_margin(test_loc, test_loc_sub_test, true);
                                if (margin_sub_test < _margin_max) {
                                    // BendyRuler will remain active
                                    active = true;
                                    break;
                                }
                            }
                        }
                        // project in the chosen direction by the full distance
                        destination_new = current_loc;
                        destination_new.offset_bearing_and_pitch(bearing_to_dest, pitch_delta, distance_to_dest);
                        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.1f);
                    
                        Write_OABendyRuler((uint8_t)OABendyType::OA_BENDY_VERTICAL, active, bearing_to_dest, pitch_delta, false, margin, destination, destination_new);
                        return active;
                    }
                }
            }
        }        
    }   

    float chosen_pitch;
    if (have_best_pitch) {
        // none of the directions tested were OK for 2-step checks. Choose the direction
        // that was best for the first step
        chosen_pitch = best_pitch;
        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.05f);
    } else {
        // none of the possible paths had a positive margin. Choose
        // the one with the highest margin
        chosen_pitch = best_margin_pitch;
        _current_lookahead = MAX(_lookahead * 0.5f, _current_lookahead * 0.9f);
    }

    // calculate new target based on best effort
    destination_new = current_loc;
    destination_new.offset_bearing_and_pitch(bearing_to_dest, chosen_pitch, distance_to_dest);

    // log results
    Write_OABendyRuler((uint8_t)OABendyType::OA_BENDY_VERTICAL, true, bearing_to_dest, chosen_pitch,false, best_margin, destination, destination_new);

    return true;
}

AP_OABendyRuler::OABendyType AP_OABendyRuler::get_type() const
{
    switch (_bendy_type) {
        case (uint8_t)OABendyType::OA_BENDY_VERTICAL:
        #if VERTICAL_ENABLED 
            return OABendyType::OA_BENDY_VERTICAL;
        #endif

        case (uint8_t)OABendyType::OA_BENDY_HORIZONTAL:
        default:
            return OABendyType::OA_BENDY_HORIZONTAL;
    }
    // should never reach here
    return OABendyType::OA_BENDY_HORIZONTAL;
}

/*
This function is called when BendyRuler has found a bearing which is obstacles free at atleast lookahead_step1_dist and  then lookahead_step2_dist from the present location
In many situations, this new bearing can be either left or right of the obstacle, and BendyRuler can have a tough time deciding between the two.
It has the tendency to move the vehicle back and forth, if the margin obtained is even slightly better in the newer iteration.
Therefore, this method attempts to avoid changing direction of the vehicle by more than _bendy_angle degrees, 
unless the new margin is atleast _bendy_ratio times better than the margin with previously calculated bearing.
We return true if we have resisted the change and will follow the last calculated bearing. 
*/
bool AP_OABendyRuler::resist_bearing_change(const Location &destination, const Location &current_loc, bool active, float bearing_test, float lookahead_step1_dist, float margin, Location &prev_dest, float &prev_bearing, float &final_bearing, float &final_margin, bool proximity_only) const
{      
    bool resisted_change = false;
    // see if there was a change in destination, if so, do not resist changing bearing 
    bool dest_change = false;
    if (!destination.same_latlon_as(prev_dest)) {
        dest_change = true;
        prev_dest = destination;
    }
                        
    // check if we need to resist the change in direction of the vehicle. If we have a clear path to destination, go there any how  
    if (active && !dest_change && is_positive(_bendy_ratio)) { 
        // check the change in bearing between freshly calculated and previous stored BendyRuler bearing
        if ((fabsf(wrap_180(prev_bearing-bearing_test)) > _bendy_angle) && (!is_equal(prev_bearing,FLT_MAX))) {
            // check margin in last bearing's direction
            Location test_loc_previous_bearing = current_loc;
            test_loc_previous_bearing.offset_bearing(wrap_180(prev_bearing), lookahead_step1_dist);
            float previous_bearing_margin = calc_avoidance_margin(current_loc,test_loc_previous_bearing, proximity_only);

            if (margin < (_bendy_ratio * previous_bearing_margin)) {
                // don't change direction abruptly. If margin difference is not significant, follow the last direction
                final_bearing = prev_bearing;
                final_margin  = previous_bearing_margin;
                resisted_change = true;
            } 
        } 
    } else {
        // reset stored bearing if BendyRuler is not active or if WP has changed for unnecessary resistance to path change
        prev_bearing = FLT_MAX;
    }
    if (!resisted_change) {
        // we are not resisting the change, hence store BendyRuler's presently calculated bearing for future iterations
        prev_bearing = bearing_test;
    }

    return resisted_change;
}

// calculate minimum distance between a segment and any obstacle
float AP_OABendyRuler::calc_avoidance_margin(const Location &start, const Location &end, bool proximity_only) const
{
    float margin_min = FLT_MAX;

    float latest_margin;
    
    if (calc_margin_from_object_database(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }
    
    if (proximity_only) {
        // only need margin from proximity data
        return margin_min;
    }
    
    if (calc_margin_from_circular_fence(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }
    
    #if VERTICAL_ENABLED 
    // alt fence only is only needed in vertical avoidance
    if (get_type() == OABendyType::OA_BENDY_VERTICAL) {
        if (calc_margin_from_alt_fence(start, end, latest_margin)) {
            margin_min = MIN(margin_min, latest_margin);
        }
    }
    #endif

    if (calc_margin_from_inclusion_and_exclusion_polygons(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }

    if (calc_margin_from_inclusion_and_exclusion_circles(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }

    // return smallest margin from any obstacle
    return margin_min;
}

// calculate minimum distance between a path and the circular fence (centered on home)
// on success returns true and updates margin
bool AP_OABendyRuler::calc_margin_from_circular_fence(const Location &start, const Location &end, float &margin) const
{
    // exit immediately if polygon fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) == 0) {
        return false;
    }

    // calculate start and end point's distance from home
    const Location &ahrs_home = AP::ahrs().get_home();
    const float start_dist_sq = ahrs_home.get_distance_NE(start).length_squared();
    const float end_dist_sq = ahrs_home.get_distance_NE(end).length_squared();

    // get circular fence radius + margin
    const float fence_radius_plus_margin = fence->get_radius() - fence->get_margin();

    // margin is fence radius minus the longer of start or end distance
    margin = fence_radius_plus_margin - sqrtf(MAX(start_dist_sq, end_dist_sq));
    return true;
}

// calculate minimum distance between a path and the altitude fence
// on success returns true and updates margin
bool AP_OABendyRuler::calc_margin_from_alt_fence(const Location &start, const Location &end, float &margin) const
{   
    // exit immediately if polygon fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) == 0) {
        return false;
    }

    int32_t alt_above_home_cm_start, alt_above_home_cm_end;    
    if (!start.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_above_home_cm_start)) {
        return false;
    }
    if (!end.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_above_home_cm_end )) {
        return false;
    }

    // safe max alt = fence alt - fence margin
    const float max_fence_alt = fence->get_safe_alt_max();
    const float margin_start =  max_fence_alt - alt_above_home_cm_start * 0.01f;
    const float margin_end =  max_fence_alt - alt_above_home_cm_end * 0.01f;

    // margin is minimum distance to fence from either start or end location
    margin = MIN(margin_start,margin_end);

    return true;
}

// calculate minimum distance between a path and all inclusion and exclusion polygons
// on success returns true and updates margin
bool AP_OABendyRuler::calc_margin_from_inclusion_and_exclusion_polygons(const Location &start, const Location &end, float &margin) const
{
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // exclusion polygons enabled along with polygon fences
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return false;
    }

    // return immediately if no inclusion nor exclusion polygons
    const uint8_t num_inclusion_polygons = fence->polyfence().get_inclusion_polygon_count();
    const uint8_t num_exclusion_polygons = fence->polyfence().get_exclusion_polygon_count();
    if ((num_inclusion_polygons == 0) && (num_exclusion_polygons == 0)) {
        return false;
    }

    // convert start and end to offsets from EKF origin
    Vector2f start_NE, end_NE;
    if (!start.get_vector_xy_from_origin_NE(start_NE) || !end.get_vector_xy_from_origin_NE(end_NE)) {
        return false;
    }

    // get fence margin
    const float fence_margin = fence->get_margin();

    // iterate through inclusion polygons and calculate minimum margin
    bool margin_updated = false;
    for (uint8_t i = 0; i < num_inclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_inclusion_polygon(i, num_points);
     
        // if outside the fence margin is the closest distance but with negative sign
        const float sign = Polygon_outside(start_NE, boundary, num_points) ? -1.0f : 1.0f;

        // calculate min distance (in meters) from line to polygon
        float margin_new = (sign * Polygon_closest_distance_line(boundary, num_points, start_NE, end_NE) * 0.01f) - fence_margin;
        if (!margin_updated || (margin_new < margin)) {
            margin_updated = true;
            margin = margin_new;
        }
    }

    // iterate through exclusion polygons and calculate minimum margin
    for (uint8_t i = 0; i < num_exclusion_polygons; i++) {
        uint16_t num_points;
        const Vector2f* boundary = fence->polyfence().get_exclusion_polygon(i, num_points);
   
        // if start is inside the polygon the margin's sign is reversed
        const float sign = Polygon_outside(start_NE, boundary, num_points) ? 1.0f : -1.0f;

        // calculate min distance (in meters) from line to polygon
        float margin_new = (sign * Polygon_closest_distance_line(boundary, num_points, start_NE, end_NE) * 0.01f) - fence_margin;
        if (!margin_updated || (margin_new < margin)) {
            margin_updated = true;
            margin = margin_new;
        }
    }

    return margin_updated;
}

// calculate minimum distance between a path and all inclusion and exclusion circles
// on success returns true and updates margin
bool AP_OABendyRuler::calc_margin_from_inclusion_and_exclusion_circles(const Location &start, const Location &end, float &margin) const
{
    // exit immediately if fence is not enabled
    const AC_Fence *fence = AC_Fence::get_singleton();
    if (fence == nullptr) {
        return false;
    }

    // inclusion/exclusion circles enabled along with polygon fences
    if ((fence->get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return false;
    }

    // return immediately if no inclusion nor exclusion circles
    const uint8_t num_inclusion_circles = fence->polyfence().get_inclusion_circle_count();
    const uint8_t num_exclusion_circles = fence->polyfence().get_exclusion_circle_count();
    if ((num_inclusion_circles == 0) && (num_exclusion_circles == 0)) {
        return false;
    }

    // convert start and end to offsets from EKF origin
    Vector2f start_NE, end_NE;
    if (!start.get_vector_xy_from_origin_NE(start_NE) || !end.get_vector_xy_from_origin_NE(end_NE)) {
        return false;
    }

    // get fence margin
    const float fence_margin = fence->get_margin();

    // iterate through inclusion circles and calculate minimum margin
    bool margin_updated = false;
    for (uint8_t i = 0; i < num_inclusion_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_inclusion_circle(i, center_pos_cm, radius)) {

            // calculate start and ends distance from the center of the circle
            const float start_dist_sq = (start_NE - center_pos_cm).length_squared();
            const float end_dist_sq = (end_NE - center_pos_cm).length_squared();

            // margin is fence radius minus the longer of start or end distance
            const float margin_new = (radius + fence_margin) - (sqrtf(MAX(start_dist_sq, end_dist_sq)) * 0.01f);

            // update margin with lowest value so far
            if (!margin_updated || (margin_new < margin)) {
                margin_updated = true;
                margin = margin_new;
            }
        }
    }

    // iterate through exclusion circles and calculate minimum margin
    for (uint8_t i = 0; i < num_exclusion_circles; i++) {
        Vector2f center_pos_cm;
        float radius;
        if (fence->polyfence().get_exclusion_circle(i, center_pos_cm, radius)) {

            // first calculate distance between circle's center and segment
            const float dist_cm = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, center_pos_cm);

            // margin is distance to the center minus the radius
            const float margin_new = (dist_cm * 0.01f) - (radius + fence_margin);

            // update margin with lowest value so far
            if (!margin_updated || (margin_new < margin)) {
                margin_updated = true;
                margin = margin_new;
            }
        }
    }

    return margin_updated;
}

// calculate minimum distance between a path and proximity sensor obstacles
// on success returns true and updates margin
bool AP_OABendyRuler::calc_margin_from_object_database(const Location &start, const Location &end, float &margin) const
{
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    // convert start and end to offsets (in cm) from EKF origin
    Vector3f start_NEU,end_NEU;
    if (!start.get_vector_from_origin_NEU(start_NEU) || !end.get_vector_from_origin_NEU(end_NEU)) {
        return false;
    }
    if (start_NEU == end_NEU) {
        return false;
    }

    // check each obstacle's distance from segment
    float smallest_margin = FLT_MAX;
    for (uint16_t i=0; i<oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        const Vector3f point_cm = item.pos * 100.0f;
        // margin is distance between line segment and obstacle minus obstacle's radius
        const float m = Vector3f::closest_distance_between_line_and_point(start_NEU, end_NEU, point_cm) * 0.01f - item.radius;
        if (m < smallest_margin) {
            smallest_margin = m;
        }
    }

    // return smallest margin
    if (smallest_margin < FLT_MAX) {
        margin = smallest_margin;
        return true;
    }

    return false;
}
