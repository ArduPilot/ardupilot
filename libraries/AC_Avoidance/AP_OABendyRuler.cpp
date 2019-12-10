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

const int16_t OA_BENDYRULER_BEARING_INC = 5;            // check every 5 degrees around vehicle
const float OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO = 1.0f; // step2's lookahead length as a ratio of step1's lookahead length
const float OA_BENDYRULER_LOOKAHEAD_STEP2_MIN = 2.0f;   // step2 checks at least this many meters past step1's location
const float OA_BENDYRULER_LOOKAHEAD_PAST_DEST = 2.0f;   // lookahead length will be at least this many meters past the destination
const float OA_BENDYRULER_LOW_SPEED_SQUARED = (0.2f * 0.2f);    // when ground course is below this speed squared, vehicle's heading will be used

// run background task to find best path and update avoidance_results
// returns true and updates origin_new and destination_new if a best path has been found
bool AP_OABendyRuler::update(const Location& current_loc, const Location& destination, const Vector2f &ground_speed_vec, Location &origin_new, Location &destination_new)
{
    // bendy ruler always sets origin to current_loc
    origin_new = current_loc;

    // calculate bearing and distance to final destination
    const float bearing_to_dest = current_loc.get_bearing_to(destination) * 0.01f;
    const float distance_to_dest = current_loc.get_distance(destination);

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

    // check OA_BEARING_INC definition allows checking in all directions
    static_assert(360 % OA_BENDYRULER_BEARING_INC == 0, "check 360 is a multiple of OA_BEARING_INC");

    // search in OA_BENDYRULER_BEARING_INC degree increments around the vehicle alternating left
    // and right. For each direction check if vehicle would avoid all obstacles
    float best_bearing = bearing_to_dest;
    bool have_best_bearing = false;
    float best_margin = -FLT_MAX;
    float best_margin_bearing = best_bearing;

    for (uint8_t i = 0; i <= (170 / OA_BENDYRULER_BEARING_INC); i++) {
        for (uint8_t bdir = 0; bdir <= 1; bdir++) {
            // skip duplicate check of bearing straight towards destination
            if ((i==0) && (bdir > 0)) {
                continue;
            }
            // bearing that we are probing
            const float bearing_delta = i * OA_BENDYRULER_BEARING_INC * (bdir == 0 ? -1.0f : 1.0f);
            const float bearing_test = wrap_180(bearing_to_dest + bearing_delta);

            // ToDo: add effective groundspeed calculations using airspeed
            // ToDo: add prediction of vehicle's position change as part of turn to desired heading

            // test location is projected from current location at test bearing
            Location test_loc = current_loc;
            test_loc.offset_bearing(bearing_test, lookahead_step1_dist);

            // calculate margin from fence for this scenario
            float margin = calc_avoidance_margin(current_loc, test_loc);
            if (margin > best_margin) {
                best_margin_bearing = bearing_test;
                best_margin = margin;
            }
            if (margin > _margin_max) {
                // this bearing avoids obstacles out to the lookahead_step1_dist
                // now check in there is a clear path in three directions towards the destination
                if (!have_best_bearing) {
                    best_bearing = bearing_test;
                    have_best_bearing = true;
                } else if (fabsf(wrap_180(ground_course_deg - bearing_test)) <
                           fabsf(wrap_180(ground_course_deg - best_bearing))) {
                    // replace bearing with one that is closer to our current ground course
                    best_bearing = bearing_test;
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
                    float margin2 = calc_avoidance_margin(test_loc, test_loc2);
                    if (margin2 > _margin_max) {
                        // all good, now project in the chosen direction by the full distance
                        destination_new = current_loc;
                        destination_new.offset_bearing(bearing_test, distance_to_dest);
                        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.1f);
                        // if the chosen direction is directly towards the destination turn off avoidance
                        const bool active = (i != 0 || j != 0);
                        AP::logger().Write_OABendyRuler(active, bearing_to_dest, margin, destination, destination_new);
                        return active;
                    }
                }
            }
        }
    }

    float chosen_bearing;
    if (have_best_bearing) {
        // none of the directions tested were OK for 2-step checks. Choose the direction
        // that was best for the first step
        chosen_bearing = best_bearing;
        _current_lookahead = MIN(_lookahead, _current_lookahead * 1.05f);
    } else {
        // none of the possible paths had a positive margin. Choose
        // the one with the highest margin
        chosen_bearing = best_margin_bearing;
        _current_lookahead = MAX(_lookahead * 0.5f, _current_lookahead * 0.9f);
    }

    // calculate new target based on best effort
    destination_new = current_loc;
    destination_new.offset_bearing(chosen_bearing, distance_to_dest);

    // log results
    AP::logger().Write_OABendyRuler(true, chosen_bearing, best_margin, destination, destination_new);

    return true;
}

// calculate minimum distance between a segment and any obstacle
float AP_OABendyRuler::calc_avoidance_margin(const Location &start, const Location &end)
{
    float margin_min = FLT_MAX;

    float latest_margin;
    if (calc_margin_from_circular_fence(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }

    if (calc_margin_from_object_database(start, end, latest_margin)) {
        margin_min = MIN(margin_min, latest_margin);
    }

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
bool AP_OABendyRuler::calc_margin_from_circular_fence(const Location &start, const Location &end, float &margin)
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

// calculate minimum distance between a path and all inclusion and exclusion polygons
// on success returns true and updates margin
bool AP_OABendyRuler::calc_margin_from_inclusion_and_exclusion_polygons(const Location &start, const Location &end, float &margin)
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
        if (num_points < 3) {
            // ignore exclusion polygons with less than 3 points
            continue;
        }

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
        if (num_points < 3) {
            // ignore exclusion polygons with less than 3 points
            continue;
        }

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
bool AP_OABendyRuler::calc_margin_from_inclusion_and_exclusion_circles(const Location &start, const Location &end, float &margin)
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
bool AP_OABendyRuler::calc_margin_from_object_database(const Location &start, const Location &end, float &margin)
{
    // exit immediately if db is empty
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    // convert start and end to offsets (in cm) from EKF origin
    Vector2f start_NE, end_NE;
    if (!start.get_vector_xy_from_origin_NE(start_NE) || !end.get_vector_xy_from_origin_NE(end_NE)) {
        return false;
    }

    // check each obstacle's distance from segment
    float smallest_margin = FLT_MAX;
    for (uint16_t i=0; i<oaDb->database_count(); i++) {
        const AP_OADatabase::OA_DbItem& item = oaDb->get_item(i);
        const Vector2f point_cm = item.pos * 100.0f;
        // margin is distance between line segment and obstacle minus obstacle's radius
        const float m = Vector2f::closest_distance_between_line_and_point(start_NE, end_NE, point_cm) * 0.01f - item.radius;
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
