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
/*
  Base class for serial proximity sensors
*/

#include "SIM_SerialProximitySensor.h"

#include <AP_Math/AP_Math.h>

#include <stdio.h>

using namespace SITL;

void SerialProximitySensor::update(const Location &location)
{
    // just send a chunk of data at 5Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < reading_interval_ms()) {
        return;
    }
    last_sent_ms = now;

    uint8_t data[255];
    const uint32_t packetlen = packet_for_location(location,
                                                   data,
                                                   ARRAY_SIZE(data));

    write_to_autopilot((char*)data, packetlen);
}

float SerialProximitySensor::measure_distance_at_angle(const Location &location, float angle) const
{
    // const SITL *sitl = AP::sitl();

    Vector2f pos1_cm;
    if (!location.get_vector_xy_from_origin_NE(pos1_cm)) {
        // should probably use SITL variables...
        return 0.0f;
    }
    static uint64_t count = 0;
    count++;

    // the 1000 here is so the files don't grow unbounded
    const bool write_debug_files = true && count < 1000;

    FILE *rayfile = nullptr;
    if (write_debug_files) {
        rayfile = fopen("/tmp/rayfile.scr", "a");
    }
    // cast a ray from location out 200m...
    Location location2 = location;
    location2.offset_bearing(wrap_180(angle), 200);
    Vector2f pos2_cm;
    if (!location2.get_vector_xy_from_origin_NE(pos2_cm)) {
        // should probably use SITL variables...
        return 0.0f;
    }
    if (rayfile != nullptr) {
        ::fprintf(rayfile, "map icon %f %f barrell\n", location2.lat*1e-7, location2.lng*1e-7);
        fclose(rayfile);
    }

    // setup a grid of posts
    FILE *postfile = nullptr;
    FILE *intersectionsfile = nullptr;
    if (write_debug_files) {
        postfile = fopen("/tmp/post-locations.scr", "w");
        intersectionsfile = fopen("/tmp/intersections.scr", "a");
    }
    float min_dist_cm = 1000000.0;
    const uint8_t num_post_offset = 10;
    for (int8_t x=-num_post_offset; x<num_post_offset; x++) {
        for (int8_t y=-num_post_offset; y<num_post_offset; y++) {
            Location post_location = post_origin;
            post_location.offset(x*10+0.5, y*10+0.5);
            if (postfile != nullptr) {
                ::fprintf(postfile, "map icon %f %f hoop\n", post_location.lat*1e-7, post_location.lng*1e-7);
            }
            Vector2f post_position_cm;
            if (!post_location.get_vector_xy_from_origin_NE(post_position_cm)) {
                // should probably use SITL variables...
                return 0.0f;
            }
            const float radius_cm = 10.0f;
            Vector2f intersection_point_cm;
            if (Vector2f::circle_segment_intersection(pos1_cm, pos2_cm, post_position_cm, radius_cm, intersection_point_cm)) {
                float dist_cm = (intersection_point_cm-pos1_cm).length();
                if (intersectionsfile != nullptr) {
                    Location intersection_point = location;
                    intersection_point.offset(intersection_point_cm.x/100.0f,
                                              intersection_point_cm.y/100.0f);
                    ::fprintf(intersectionsfile,
                              "map icon %f %f barrell\n",
                              intersection_point.lat*1e-7,
                              intersection_point.lng*1e-7);
                }
                if (dist_cm < min_dist_cm) {
                    min_dist_cm = dist_cm;
                }
            }
        }
    }
    if (postfile != nullptr) {
        fclose(postfile);
    }
    if (intersectionsfile != nullptr) {
        fclose(intersectionsfile);
    }

    // ::fprintf(stderr, "Distance @%f = %fm\n", angle, min_dist_cm/100.0f);
    return min_dist_cm / 100.0f;
}
