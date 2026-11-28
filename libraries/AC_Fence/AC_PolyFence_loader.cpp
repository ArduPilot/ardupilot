#include "AC_PolyFence_loader.h"

#if AP_FENCE_ENABLED

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Avoidance/AP_Avoidance_config.h>

#ifndef AC_FENCE_DUMMY_METHODS_ENABLED
#define AC_FENCE_DUMMY_METHODS_ENABLED  (!(APM_BUILD_TYPE(APM_BUILD_Rover) | APM_BUILD_COPTER_OR_HELI | APM_BUILD_TYPE(APM_BUILD_ArduPlane) | APM_BUILD_TYPE(APM_BUILD_ArduSub) | (AP_FENCE_ENABLED == 1)))
#endif

#if !AC_FENCE_DUMMY_METHODS_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AC_Fence/AC_Fence.h>

#include <stdio.h>

#define POLYFENCE_LOADER_DEBUGGING 0

#if POLYFENCE_LOADER_DEBUGGING
#define Debug(fmt, args ...)  do { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#else
#define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

static StorageAccess fence_storage(StorageManager::StorageFence);

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define AC_FENCE_SDCARD_FILENAME "APM/fence.stg"
#else
#define AC_FENCE_SDCARD_FILENAME "fence.stg"
#endif

void AC_PolyFence_loader::init()
{
#if AP_SDCARD_STORAGE_ENABLED
    // check for extra storage on microsd
    const auto *bc = AP::boardConfig();
    if (bc != nullptr) {
        const auto size_kb = bc->get_sdcard_fence_kb();
        if (size_kb > 0) {
            _failed_sdcard_storage = !fence_storage.attach_file(AC_FENCE_SDCARD_FILENAME, size_kb);
            if (_failed_sdcard_storage) {
                // wipe fence if storage not available, but don't
                // save. This allows sdcard error to be fixed and
                // reboot
                _total.set(0);
            }
        }
    }
#endif
    if (!check_indexed()) {
        // tell the user, perhaps?
    }
    _old_total = _total;
}

bool AC_PolyFence_loader::find_index_for_seq(const uint16_t seq, const FenceIndex *&entry, uint16_t &i) const
{
    if (_index == nullptr) {
        return false;
    }

    if (seq > _eeprom_item_count) {
        return false;
    }

    i = 0;
    for (uint16_t j=0; j<_num_fences; j++) {
        entry = &_index[j];
        if (seq < i + entry->count) {
            return true;
        }
        i += entry->count;
    }
    return false;
}

bool AC_PolyFence_loader::find_storage_offset_for_seq(const uint16_t seq, uint16_t &offset, AC_PolyFenceType &type, uint16_t &vertex_count_offset) const
{
    if (_index == nullptr) {
        return false;
    }

    uint16_t i = 0;
    const FenceIndex *entry = nullptr;
    if (!find_index_for_seq(seq, entry, i)) {
        return false;
    }

    if (entry == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }

    const uint16_t delta = seq - i;

    offset = entry->storage_offset;
    type = entry->type;
    offset++; // skip over type
    switch (type) {
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
    case AC_PolyFenceType::HOME_CIRCLE_INCLUSION:
        if (delta != 0) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
        }
        break;
    case AC_PolyFenceType::POLYGON_INCLUSION:
    case AC_PolyFenceType::POLYGON_EXCLUSION:
        vertex_count_offset = offset;
        offset += 1; // the count of points in the fence
        offset += (delta * 8);
        break;
    case AC_PolyFenceType::RETURN_POINT:
        if (delta != 0) {
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
        }
        break;
    case AC_PolyFenceType::END_OF_STORAGE:
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    return true;
}

bool AC_PolyFence_loader::get_item(const uint16_t seq, AC_PolyFenceItem &item)
{
    if (!check_indexed()) {
        return false;
    }

    uint16_t vertex_count_offset = 0; // initialised to make compiler happy
    uint16_t offset;
    AC_PolyFenceType type;
    if (!find_storage_offset_for_seq(seq, offset, type, vertex_count_offset)) {
        return false;
    }

    item.type = type;

    switch (type) {
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        item.radius = fence_storage.read_float(offset);
        break;
    case AC_PolyFenceType::HOME_CIRCLE_INCLUSION: {
        item.radius = fence_storage.read_float(offset);
        break;
    }
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        item.radius = fence_storage.read_uint32(offset);
        // magically change int item into a float item:
        if (type == AC_PolyFenceType::CIRCLE_INCLUSION_INT) {
            item.type = AC_PolyFenceType::CIRCLE_INCLUSION;
        } else {
            item.type = AC_PolyFenceType::CIRCLE_EXCLUSION;
        }
        break;
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    case AC_PolyFenceType::POLYGON_INCLUSION:
    case AC_PolyFenceType::POLYGON_EXCLUSION:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        item.vertex_count = fence_storage.read_uint8(vertex_count_offset);
        break;
    case AC_PolyFenceType::RETURN_POINT:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        break;
    case AC_PolyFenceType::END_OF_STORAGE:
        // read end-of-storage when I should never do so
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    return true;
}

bool AC_PolyFence_loader::write_type_to_storage(uint16_t &offset, const AC_PolyFenceType type)
{
    fence_storage.write_uint8(offset, (uint8_t)type);
    offset++;
    return true;
}

bool AC_PolyFence_loader::write_latlon_to_storage(uint16_t &offset, const Vector2l &latlon)
{
    fence_storage.write_uint32(offset, latlon.x);
    offset += 4;
    fence_storage.write_uint32(offset, latlon.y);
    offset += 4;
    return true;
}

bool AC_PolyFence_loader::read_latlon_from_storage(uint16_t &read_offset, Vector2l &ret) const
{
    ret.x = fence_storage.read_uint32(read_offset);
    read_offset += 4;
    ret.y = fence_storage.read_uint32(read_offset);
    read_offset += 4;
    return true;
}

bool AC_PolyFence_loader::breached() const
{
    AbsAltLocation loc;
    if (!AP::ahrs().get_location(loc)) {
        return false;
    }

    return breached(loc);
}

// check if a position (expressed as lat/lng) is within the boundary
//   returns true if location is outside the boundary
bool AC_PolyFence_loader::breached(const Location& loc, float& distance_outside_fence, Vector2f& fence_direction) const
{
    if (!loaded() || total_fence_count() == 0) {
        return false;
    }

    Vector2f scaled_pos;
    Vector2l pos { loc.lat, loc.lng };
    if (!scale_latlon_from_origin(loaded_origin, pos, scaled_pos)) {
        return false;
    }

    const uint16_t num_inclusion = _num_loaded_circle_inclusion_boundaries + _num_loaded_inclusion_boundaries;
    uint16_t num_inclusion_outside = 0;
    distance_outside_fence = -FLT_MAX;

    // check we are inside each inclusion zone:
    for (uint8_t i=0; i<_num_loaded_inclusion_boundaries; i++) {
        const InclusionBoundary &boundary = _loaded_inclusion_boundary[i];
        bool valid_distance = Polygon_closest_distance_point(boundary.points, boundary.count, scaled_pos, fence_direction);
        float distance = fence_direction.length() * 0.01f; // convert back to meters
        if (Polygon_outside(pos, boundary.points_lla, boundary.count)) {
            num_inclusion_outside++;
            if (valid_distance) {
                if (is_positive(distance_outside_fence)) {
                    distance_outside_fence = MIN(distance_outside_fence, distance);
                } else {
                    distance_outside_fence = distance;
                }
            }
        } else if (valid_distance) {
            distance_outside_fence = MAX(distance_outside_fence, -distance);
        }
    }

    // check we are outside each exclusion zone:
    for (uint8_t i=0; i<_num_loaded_exclusion_boundaries; i++) {
        const ExclusionBoundary &boundary = _loaded_exclusion_boundary[i];
        bool valid_distance = Polygon_closest_distance_point(boundary.points, boundary.count, scaled_pos, fence_direction);
        float distance = fence_direction.length() * 0.01f; // convert back to meters
        if (!Polygon_outside(pos, boundary.points_lla, boundary.count)) {
            if (valid_distance) {
                distance_outside_fence = distance;
            } else {
                distance_outside_fence = 0.0f;
            }
            return true;
        } else if (valid_distance) {
            distance_outside_fence = MAX(distance_outside_fence, -distance);
        }
    }

    for (uint8_t i=0; i<_num_loaded_circle_exclusion_boundaries; i++) {
        const ExclusionCircle &circle = _loaded_circle_exclusion_boundary[i];
        Location circle_center;
        circle_center.lat = circle.point.x;
        circle_center.lng = circle.point.y;
        const float diff_cm = loc.get_distance(circle_center)*100.0f;
        distance_outside_fence = MAX(distance_outside_fence, circle.radius - diff_cm*0.01f);
        if (diff_cm < circle.radius * 100.0f) {
            return true;
        }
    }

    for (uint8_t i=0; i<_num_loaded_circle_inclusion_boundaries; i++) {
        const InclusionCircle &circle = _loaded_circle_inclusion_boundary[i];
        Location circle_center;
        if (circle.is_home_centered()) {
            circle_center = AP::ahrs().get_home();
        } else {
            circle_center.lat = circle.point.x;
            circle_center.lng = circle.point.y;
        }

        const float diff_cm = loc.get_distance(circle_center)*100.0f;
        distance_outside_fence = MAX(distance_outside_fence, diff_cm*0.01f - circle.radius);
        if (diff_cm > circle.radius * 100.0f) {
            num_inclusion_outside++;
        }
    }

    if (AC_Fence::option_enabled(AC_Fence::OPTIONS::INCLUSION_UNION, _options)) {
        // using union of inclusion areas, we are outside the fence if
        // there is at least one inclusion areas and we are outside
        // all of them
        if (num_inclusion > 0 && num_inclusion == num_inclusion_outside) {
            return true;
        }
    } else {
        // using intersection of inclusion areas. We are outside if we
        // are outside any of them
        if (num_inclusion_outside > 0) {
            return true;
        }
    }

    // if no value was found, reset to 0
    if (is_equal(distance_outside_fence, -FLT_MAX)) {
        distance_outside_fence = 0.0f;
    }

    // no fence breached
    return false;
}

#if AP_OA_SCRIPTING_ENABLED
// returns the closest distance in metres from (start_NE_cm, end_NE_cm) to the inclusion fences,
// positive while inside the inclusion and negative once it has been breached, or FLT_MAX when no
// inclusion fence is loaded.  fence_type reports which kind of inclusion area the distance belongs
// to, so a caller can name the fence it is avoiding.
//
// Circles and polygons are considered together because FENCE_OPTIONS INCLUSION_UNION spans both:
// by default the inclusion areas intersect, so every one of them must be satisfied and the worst
// clearance governs; with INCLUSION_UNION being inside any single area is legal, so the best
// clearance governs instead.  Taking the minimum under union semantics would report a disjoint
// area the vehicle is not required to be inside as a blocking fence, and because AC_Fence itself
// correctly reports no breach, nothing downstream would correct it.
float AC_PolyFence_loader::distance_line_to_inclusion(const Vector2f& start_NE_cm, const Vector2f &end_NE_cm,
                                                      AC_PolyFenceType &fence_type) const
{
    const bool use_union = AC_Fence::option_enabled(AC_Fence::OPTIONS::INCLUSION_UNION, _options);
    float distance_new_m = FLT_MAX;
    bool found = false;

    for (uint8_t i=0; i<_num_loaded_circle_inclusion_boundaries; i++) {
        const InclusionCircle &circle = _loaded_circle_inclusion_boundary[i];
        Vector2f centre_cm = circle.pos_cm;
        if (circle.is_home_centered()) {
            // Express home as an offset from EKF origin in the NE frame, matching
            // start_NE_cm/end_NE_cm - see the identical pattern in get_inclusion_circle().
            Location origin;
            if (!AP::ahrs().get_origin(origin)) {
                continue;   // cannot place this circle yet
            }
            centre_cm = origin.get_distance_NE(AP::ahrs().get_home()) * 100.0f;
        }
        // the segment is closest to exiting the circle at whichever endpoint
        // lies farthest from the centre
        const float far_cm = MAX((start_NE_cm - centre_cm).length(), (end_NE_cm - centre_cm).length());
        const float distance_m = circle.radius - far_cm * 0.01f;
        if (!found || (use_union ? (distance_m > distance_new_m) : (distance_m < distance_new_m))) {
            distance_new_m = distance_m;
            fence_type = AC_PolyFenceType::CIRCLE_INCLUSION;
            found = true;
        }
    }

    for (uint8_t i=0; i<_num_loaded_inclusion_boundaries; i++) {
        const InclusionBoundary &boundary = _loaded_inclusion_boundary[i];
        // Polygon_closest_distance_line() is signed only when the segment actually
        // crosses the boundary (negative); a segment wholly outside (or wholly inside)
        // returns a positive nearest-edge distance either way, and only that case needs
        // the side sign applied - re-signing an already-negative crossing result would
        // double-negate it. Establish the sign from the start point's own inside/outside
        // test first, matching the pattern already used in AP_OABendyRuler.cpp's
        // inclusion-polygon margin calculation.
        const float sign = Polygon_outside(start_NE_cm, boundary.points, boundary.count) ? -1.0f : 1.0f;
        const float raw_cm = Polygon_closest_distance_line(boundary.points, boundary.count, start_NE_cm, end_NE_cm);
        const float distance_m = (raw_cm < 0) ? raw_cm * 0.01f : sign * raw_cm * 0.01f;
        if (!found || (use_union ? (distance_m > distance_new_m) : (distance_m < distance_new_m))) {
            distance_new_m = distance_m;
            fence_type = AC_PolyFenceType::POLYGON_INCLUSION;
            found = true;
        }
    }

    // no inclusion fence loaded is no constraint at all
    return found ? distance_new_m : FLT_MAX;
}

// returns distance_new_m the closest distance from (start_NE_cm, end_NE_cm) and any circle exclusion fence.
// distance_new_m will be positive if we are outside the exclusion, or -ve if we have already breached
// result is true if a fence is found or false if no fence is found
float AC_PolyFence_loader::distance_line_to_circle_exclusion(const Vector2f& start_NE_cm, const Vector2f &end_NE_cm) const
{
    float distance_new_m = FLT_MAX;

    for (uint8_t i=0; i<_num_loaded_circle_exclusion_boundaries; i++) {
        const ExclusionCircle &circle = _loaded_circle_exclusion_boundary[i];
        float distance_m = Vector2f::closest_distance_between_line_and_point(start_NE_cm, end_NE_cm, circle.pos_cm) * 0.01f - circle.radius;
        distance_new_m = (distance_m < distance_new_m) ? distance_m : distance_new_m;
    }
    return distance_new_m;
}

// returns distance_new_m the closest distance from (start_NE_cm, end_NE_cm) and any polygon exclusion fence.
// distance_new_m will be positive if we are outside the exclusion, or -ve if we have already breached
// result is true if a fence is found or false if no fence is found
float AC_PolyFence_loader::distance_line_to_polygon_exclusion(const Vector2f& start_NE_cm, const Vector2f &end_NE_cm) const
{
    float distance_new_m = FLT_MAX;

    // check how far we are outside any polygon exclusion zone: Return the minimum distance;
    for (uint8_t i=0; i<_num_loaded_exclusion_boundaries; i++) {
        const ExclusionBoundary &boundary = _loaded_exclusion_boundary[i];
        // Same signing rule as distance_line_to_inclusion() above: only an unsigned
        // nearest-edge distance needs the side sign - a crossing is already negative
        // from Polygon_closest_distance_line() itself.
        const float sign = Polygon_outside(start_NE_cm, boundary.points, boundary.count) ? 1.0f : -1.0f;
        const float raw_cm = Polygon_closest_distance_line(boundary.points, boundary.count, start_NE_cm, end_NE_cm);
        const float distance_m = (raw_cm < 0) ? raw_cm * 0.01f : sign * raw_cm * 0.01f;
        distance_new_m = (distance_m < distance_new_m) ? distance_m : distance_new_m;
    }

    return distance_new_m;
}
#else
float AC_PolyFence_loader::distance_line_to_inclusion(const Vector2f& start_NE_cm, const Vector2f &end_NE_cm, AC_PolyFenceType &fence_type) const { return FLT_MAX; }
float AC_PolyFence_loader::distance_line_to_circle_exclusion(const Vector2f& start_NE_cm, const Vector2f &end_NE_cm) const { return FLT_MAX; }
float AC_PolyFence_loader::distance_line_to_polygon_exclusion(const Vector2f& start_NE_cm, const Vector2f &end_NE_cm) const { return FLT_MAX; }
#endif  // AP_OA_SCRIPTING_ENABLED

// finds the two points where circles cross.  returns false if they do not cross
static bool circle_crossings(const Vector2f &center1, float radius1, const Vector2f &center2, float radius2, Vector2f &crossing1, Vector2f &crossing2)
{
    const Vector2f center1_to_2 = center2 - center1;
    const float dist = center1_to_2.length();
    if (is_zero(dist) || (dist > radius1 + radius2) || (dist < fabsf(radius1 - radius2))) {
        return false;
    }

    // the crossings lie either side of the line between the centers, on a chord perpendicular to it
    const Vector2f unit = center1_to_2 / dist;
    const float chord_dist = (sq(dist) + sq(radius1) - sq(radius2)) / (2.0f * dist);
    const float half_chord = safe_sqrt(sq(radius1) - sq(chord_dist));
    const Vector2f chord_center = center1 + unit * chord_dist;
    const Vector2f perpendicular {-unit.y, unit.x};
    crossing1 = chord_center + perpendicular * half_chord;
    crossing2 = chord_center - perpendicular * half_chord;
    return true;
}

// get_closest_loc_within_fence - finds the closest point to loc that is inside the fence and,
// where possible, at least margin_cm from the edge of every loaded polygon and circle.
// if loc does not breach the fence then loc_closest is simply set to loc.
// returns false if no such point could be found (e.g. fence not loaded)
bool AC_PolyFence_loader::get_closest_loc_within_fence(const Location& loc, float margin_cm, Location& loc_closest) const
{
    if (!loaded() || total_fence_count() == 0) {
        return false;
    }

    // if loc does not breach the fence then no adjustment is required
    if (!breached(loc)) {
        loc_closest = loc;
        return true;
    }

    // calculate loc as an offset (in cm) from the fence origin
    Vector2f pos_cm;
    Vector2l latlon { loc.lat, loc.lng };
    if (!scale_latlon_from_origin(loaded_origin, latlon, pos_cm)) {
        return false;
    }

    // if the fence is too small for every candidate to clear margin_cm then all of them lie outside it,
    // so the margin is halved until a point is found.  below 1cm clears_margin() accepts any point inside the fence
    Vector2f closest_pos_cm;
    for (float search_margin_cm = margin_cm; !find_closest_pos_within_fence(pos_cm, search_margin_cm, closest_pos_cm); search_margin_cm *= 0.5f) {
        if (search_margin_cm < 1.0f) {
            return false;
        }
    }

    // return the caller's location moved horizontally, leaving its altitude and frame
    // untouched so that the result matches the no adjustment case above
    Location closest_loc = loaded_origin;
    closest_loc.offset(closest_pos_cm.x * 0.01f, closest_pos_cm.y * 0.01f);
    loc_closest = loc;
    loc_closest.lat = closest_loc.lat;
    loc_closest.lng = closest_loc.lng;
    return true;
}

// helper for get_closest_loc_within_fence: finds the closest point to pos_cm that is inside the fence and,
// where possible, at least margin_cm from the edge of every loaded polygon and circle
bool AC_PolyFence_loader::find_closest_pos_within_fence(const Vector2f &pos_cm, float margin_cm, Vector2f &closest_pos_cm) const
{
    // each boundary is moved margin_cm into the fence.  the closest point is either the closest point
    // on one of these moved boundaries or a point where two of them cross
    ClosestCandidate best_clear {};
    ClosestCandidate best_inside {};

    // polygon edges: the closest point on each edge, and where it crosses every later edge and every circle
    for (uint8_t p1 = 0; p1 < offset_polygon_count(); p1++) {
        const Vector2f *points1;
        uint16_t count1;
        float inset1_cm;
        if (!get_offset_polygon(p1, margin_cm, points1, count1, inset1_cm)) {
            continue;
        }
        Vector2f start1_cm = inset_vertex(points1, count1, count1 - 1, inset1_cm);
        for (uint16_t e1 = 0; e1 < count1; e1++) {
            const Vector2f end1_cm = inset_vertex(points1, count1, e1, inset1_cm);
            consider_candidate(Vector2f::closest_point(pos_cm, start1_cm, end1_cm), pos_cm, margin_cm, best_clear, best_inside);

            for (uint8_t p2 = p1; p2 < offset_polygon_count(); p2++) {
                const Vector2f *points2;
                uint16_t count2;
                float inset2_cm;
                if (!get_offset_polygon(p2, margin_cm, points2, count2, inset2_cm)) {
                    continue;
                }
                // within the same polygon only the edges after this one are tested
                const uint16_t first_e2 = (p2 == p1) ? e1 + 1 : 0;
                if (first_e2 >= count2) {
                    continue;
                }
                Vector2f start2_cm = inset_vertex(points2, count2, (first_e2 + count2 - 1) % count2, inset2_cm);
                for (uint16_t e2 = first_e2; e2 < count2; e2++) {
                    const Vector2f end2_cm = inset_vertex(points2, count2, e2, inset2_cm);
                    Vector2f crossing_cm;
                    if (Vector2f::segment_intersection(start1_cm, end1_cm, start2_cm, end2_cm, crossing_cm)) {
                        consider_candidate(crossing_cm, pos_cm, margin_cm, best_clear, best_inside);
                    }
                    start2_cm = end2_cm;
                }
            }

            for (uint8_t c = 0; c < offset_circle_count(); c++) {
                Vector2f center_cm;
                float radius_cm;
                if (!get_offset_circle(c, margin_cm, center_cm, radius_cm)) {
                    continue;
                }
                // an edge can cross a circle twice and each call finds the crossing nearest its first argument
                Vector2f crossing_cm;
                if (Vector2f::circle_segment_intersection(start1_cm, end1_cm, center_cm, radius_cm, crossing_cm)) {
                    consider_candidate(crossing_cm, pos_cm, margin_cm, best_clear, best_inside);
                }
                if (Vector2f::circle_segment_intersection(end1_cm, start1_cm, center_cm, radius_cm, crossing_cm)) {
                    consider_candidate(crossing_cm, pos_cm, margin_cm, best_clear, best_inside);
                }
            }
            start1_cm = end1_cm;
        }
    }

    // circles: the closest point on each circle, and where it crosses every later circle
    for (uint8_t c1 = 0; c1 < offset_circle_count(); c1++) {
        Vector2f center1_cm;
        float radius1_cm;
        if (!get_offset_circle(c1, margin_cm, center1_cm, radius1_cm)) {
            continue;
        }
        // a point at the centre has no closest direction, so north is used
        const Vector2f center_to_pos_cm = pos_cm - center1_cm;
        const Vector2f direction = center_to_pos_cm.is_zero() ? Vector2f{1.0f, 0.0f} : center_to_pos_cm.normalized();
        consider_candidate(center1_cm + direction * radius1_cm, pos_cm, margin_cm, best_clear, best_inside);
        for (uint8_t c2 = c1 + 1; c2 < offset_circle_count(); c2++) {
            Vector2f center2_cm;
            float radius2_cm;
            if (!get_offset_circle(c2, margin_cm, center2_cm, radius2_cm)) {
                continue;
            }
            Vector2f crossing1_cm, crossing2_cm;
            if (circle_crossings(center1_cm, radius1_cm, center2_cm, radius2_cm, crossing1_cm, crossing2_cm)) {
                consider_candidate(crossing1_cm, pos_cm, margin_cm, best_clear, best_inside);
                consider_candidate(crossing2_cm, pos_cm, margin_cm, best_clear, best_inside);
            }
        }
    }

    // a point inside the fence that is closer than margin_cm to an edge is only used if no point clears every edge
    const ClosestCandidate &best = best_clear.found ? best_clear : best_inside;
    if (!best.found) {
        return false;
    }
    closest_pos_cm = best.pos_cm;
    return true;
}

// helper for get_closest_loc_within_fence: tests candidate_pos_cm and keeps it as the best point that
// clears every boundary by margin_cm, and as the best point that is merely inside the fence, if it is
// closer to pos_cm than the best of each found so far
void AC_PolyFence_loader::consider_candidate(const Vector2f &candidate_pos_cm, const Vector2f &pos_cm, float margin_cm,
                                             ClosestCandidate &best_clear, ClosestCandidate &best_inside) const
{
    const float dist_cm_sq = (candidate_pos_cm - pos_cm).length_squared();
    const bool closer_than_clear = !best_clear.found || (dist_cm_sq < best_clear.dist_cm_sq);
    const bool closer_than_inside = !best_inside.found || (dist_cm_sq < best_inside.dist_cm_sq);
    if (!closer_than_clear && !closer_than_inside) {
        return;
    }

    Location candidate_loc = loaded_origin;
    candidate_loc.offset(candidate_pos_cm.x * 0.01f, candidate_pos_cm.y * 0.01f);
    if (breached(candidate_loc)) {
        return;
    }

    if (closer_than_inside) {
        best_inside = {true, dist_cm_sq, candidate_pos_cm};
    }
    if (closer_than_clear && clears_margin(candidate_pos_cm, margin_cm)) {
        best_clear = {true, dist_cm_sq, candidate_pos_cm};
    }
}

// helper for get_closest_loc_within_fence: returns true if pos_cm (an offset in cm from the fence
// origin) is at least margin_cm from the edge of every loaded polygon and circle
bool AC_PolyFence_loader::clears_margin(const Vector2f &pos_cm, float margin_cm) const
{
    // 1cm allows for float error in points placed exactly on a moved boundary
    const float min_dist_cm = margin_cm - 1.0f;

    for (uint8_t i = 0; i < _num_loaded_inclusion_boundaries; i++) {
        const InclusionBoundary &boundary = _loaded_inclusion_boundary[i];
        Vector2f closest_vec_cm;
        if (Polygon_closest_distance_point(boundary.points, boundary.count, pos_cm, closest_vec_cm) &&
            (closest_vec_cm.length() < min_dist_cm)) {
            return false;
        }
    }

    for (uint8_t i = 0; i < _num_loaded_exclusion_boundaries; i++) {
        const ExclusionBoundary &boundary = _loaded_exclusion_boundary[i];
        Vector2f closest_vec_cm;
        if (Polygon_closest_distance_point(boundary.points, boundary.count, pos_cm, closest_vec_cm) &&
            (closest_vec_cm.length() < min_dist_cm)) {
            return false;
        }
    }

    for (uint8_t i = 0; i < get_inclusion_circle_count(); i++) {
        Vector2f center_pos_cm;
        float radius_m;
        if (get_inclusion_circle(i, center_pos_cm, radius_m) &&
            (fabsf((pos_cm - center_pos_cm).length() - (radius_m * 100.0f)) < min_dist_cm)) {
            return false;
        }
    }

    for (uint8_t i = 0; i < get_exclusion_circle_count(); i++) {
        Vector2f center_pos_cm;
        float radius_m;
        if (get_exclusion_circle(i, center_pos_cm, radius_m) &&
            (fabsf((pos_cm - center_pos_cm).length() - (radius_m * 100.0f)) < min_dist_cm)) {
            return false;
        }
    }

    return true;
}

// helper for get_closest_loc_within_fence: returns the number of loaded inclusion and exclusion polygons
uint8_t AC_PolyFence_loader::offset_polygon_count() const
{
    return _num_loaded_inclusion_boundaries + _num_loaded_exclusion_boundaries;
}

// helper for get_closest_loc_within_fence: gets polygon idx (inclusion polygons first, then exclusion
// polygons) and the distance each of its edges must move to its left to move margin_cm into the fence
bool AC_PolyFence_loader::get_offset_polygon(uint8_t idx, float margin_cm, const Vector2f *&points, uint16_t &count, float &inset_cm) const
{
    // inclusion edges move inwards and exclusion edges move outwards
    float move_cm;
    if (idx < _num_loaded_inclusion_boundaries) {
        points = _loaded_inclusion_boundary[idx].points;
        count = _loaded_inclusion_boundary[idx].count;
        move_cm = margin_cm;
    } else {
        idx -= _num_loaded_inclusion_boundaries;
        if (idx >= _num_loaded_exclusion_boundaries) {
            return false;
        }
        points = _loaded_exclusion_boundary[idx].points;
        count = _loaded_exclusion_boundary[idx].count;
        move_cm = -margin_cm;
    }

    if (Polygon_complete(points, count)) {
        count--;
    }
    if (count < 3) {
        return false;
    }

    // the sign of the polygon's area gives its winding, which decides which side of each edge is inside
    float area_x2 = 0.0f;
    for (uint16_t i = 0; i < count; i++) {
        area_x2 += points[i] % points[(i + 1) % count];
    }
    inset_cm = is_negative(area_x2) ? -move_cm : move_cm;
    return true;
}

// helper for get_closest_loc_within_fence: returns the number of loaded inclusion and exclusion circles
uint8_t AC_PolyFence_loader::offset_circle_count() const
{
    return get_inclusion_circle_count() + get_exclusion_circle_count();
}

// helper for get_closest_loc_within_fence: gets circle idx (inclusion circles first, then exclusion
// circles) with its radius changed so that its edge is margin_cm into the fence
bool AC_PolyFence_loader::get_offset_circle(uint8_t idx, float margin_cm, Vector2f &center_pos_cm, float &radius_cm) const
{
    float radius_m;
    if (idx < get_inclusion_circle_count()) {
        if (!get_inclusion_circle(idx, center_pos_cm, radius_m)) {
            return false;
        }
        radius_cm = (radius_m * 100.0f) - margin_cm;
        return is_positive(radius_cm);
    }
    idx -= get_inclusion_circle_count();
    if (!get_exclusion_circle(idx, center_pos_cm, radius_m)) {
        return false;
    }
    radius_cm = (radius_m * 100.0f) + margin_cm;
    return true;
}

// helper for get_closest_loc_within_fence: returns vertex i of the polygon moved to where its two
// neighbouring edges meet once each is moved inset_cm to its left.  At a near hairpin corner the
// move is limited, so that corner is less than inset_cm from its edges
//   points, count: polygon vertices (unclosed), as offsets in cm from the fence origin
//   i: index of the vertex to move
//   inset_cm: distance to move each edge to its left (the inside of a polygon with positive area)
Vector2f AC_PolyFence_loader::inset_vertex(const Vector2f *points, uint16_t count, uint16_t i, float inset_cm) const
{
    const Vector2f &vertex = points[i];
    Vector2f edge_in = vertex - points[(i + count - 1) % count];
    Vector2f edge_out = points[(i + 1) % count] - vertex;
    if (edge_in.is_zero() || edge_out.is_zero()) {
        return vertex;
    }
    edge_in.normalize();
    edge_out.normalize();

    // left-hand normal of each edge
    const Vector2f normal_in {-edge_in.y, edge_in.x};
    const Vector2f normal_out {-edge_out.y, edge_out.x};

    // the moved edges meet at vertex + (normal_in + normal_out) * inset_cm / (1 + cos(turn angle)).
    // limit the divisor so a near hairpin corner does not throw the point far away
    const float divisor = MAX(1.0f + (normal_in * normal_out), 0.1f);
    return vertex + (normal_in + normal_out) * (inset_cm / divisor);
}

bool AC_PolyFence_loader::formatted() const
{
    return (fence_storage.read_uint8(0) == new_fence_storage_magic &&
            fence_storage.read_uint8(1) == 0 &&
            fence_storage.read_uint8(2) == 0 &&
            fence_storage.read_uint8(3) == 0);
}

uint16_t AC_PolyFence_loader::max_items() const
{
    // this is 84 items on PixHawk
    return fence_storage.size() / sizeof(Vector2l);
}

bool AC_PolyFence_loader::format()
{
    uint16_t offset = 0;
    fence_storage.write_uint32(offset, 0);
    fence_storage.write_uint8(offset, new_fence_storage_magic);
    offset += 4;
    void_index();
    _eeprom_fence_count = 0;
    _eeprom_item_count = 0;
    return write_eos_to_storage(offset);
}

bool AC_PolyFence_loader::scale_latlon_from_origin(const Location &origin, const Vector2l &point, Vector2f &pos_cm) const
{
    Location tmp_loc;
    tmp_loc.lat = point.x;
    tmp_loc.lng = point.y;
    pos_cm = origin.get_distance_NE(tmp_loc) * 100.0f;
    return true;
}

bool AC_PolyFence_loader::read_polygon_from_storage(const Location &origin, uint16_t &read_offset, const uint8_t vertex_count, Vector2f *&next_storage_point, Vector2l *&next_storage_point_lla)
{
    for (uint8_t i=0; i<vertex_count; i++) {
        // read from storage to lat/lon
        if (!read_latlon_from_storage(read_offset, *next_storage_point_lla)) {
            return false;
        }
        // convert lat/lon to position in cm from origin
        if (!scale_latlon_from_origin(origin, *next_storage_point_lla, *next_storage_point)) {
            return false;
        }
        
        next_storage_point_lla++;
        next_storage_point++;
    }
    return true;
}

bool AC_PolyFence_loader::scan_eeprom(scan_fn_t scan_fn)
{
    uint16_t read_offset = 0; // skipping reserved first 4 bytes
    if (!formatted()) {
        return false;
    }
    read_offset += 4;
    bool all_done = false;
    while (!all_done) {
        if (read_offset > fence_storage.size()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("did not find end-of-storage-marker before running out of space");
#endif
            return false;
        }
        const AC_PolyFenceType type = (AC_PolyFenceType)fence_storage.read_uint8(read_offset);
        // validate what we've just pulled back from storage:
        switch (type) {
        case AC_PolyFenceType::END_OF_STORAGE:
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
        case AC_PolyFenceType::HOME_CIRCLE_INCLUSION:
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::RETURN_POINT:
            break;
        default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Fence corrupt (offset=%u)", read_offset);
#endif
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Fence corrupt");
            return false;
        }

        scan_fn(type, read_offset);
        read_offset++;
        switch (type) {
        case AC_PolyFenceType::END_OF_STORAGE:
            _eos_offset = read_offset-1;
            all_done = true;
            break;
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION: {
            const uint8_t vertex_count = fence_storage.read_uint8(read_offset);
            read_offset += 1; // for the count we just read
            read_offset += vertex_count*8;
            break;
        }
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION: {
            read_offset += 8; // for latlon
            read_offset += 4; // for radius
            break;
        }
        case AC_PolyFenceType::HOME_CIRCLE_INCLUSION:
            // Home circle only has a radius
            read_offset += 4; // for radius
            break;
        case AC_PolyFenceType::RETURN_POINT:
            read_offset += 8; // for latlon
            break;
        }
    }
    return true;
}

// note read_offset here isn't const and ALSO is not a reference
void AC_PolyFence_loader::scan_eeprom_count_fences(const AC_PolyFenceType type, uint16_t read_offset)
{
    if (type == AC_PolyFenceType::END_OF_STORAGE) {
        return;
    }
    _eeprom_fence_count++;
    switch (type) {
    case AC_PolyFenceType::END_OF_STORAGE:
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    case AC_PolyFenceType::POLYGON_EXCLUSION:
    case AC_PolyFenceType::POLYGON_INCLUSION: {
        const uint8_t vertex_count = fence_storage.read_uint8(read_offset+1); // skip type
        _eeprom_item_count += vertex_count;
        break;
    }
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
    case AC_PolyFenceType::HOME_CIRCLE_INCLUSION:
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    case AC_PolyFenceType::RETURN_POINT:
        _eeprom_item_count++;
        break;
    }
}

bool AC_PolyFence_loader::count_eeprom_fences()
{
    _eeprom_fence_count = 0;
    _eeprom_item_count = 0;
    const bool ret = scan_eeprom(FUNCTOR_BIND_MEMBER(&AC_PolyFence_loader::scan_eeprom_count_fences, void, const AC_PolyFenceType, uint16_t));
    return ret;
}

void AC_PolyFence_loader::scan_eeprom_index_fences(const AC_PolyFenceType type, uint16_t read_offset)
{
    if (_index == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }
    if (type == AC_PolyFenceType::END_OF_STORAGE) {
        return;
    }
    FenceIndex &index = _index[_num_fences++];
    index.type = type;
    index.storage_offset = read_offset;
    switch (type) {
    case AC_PolyFenceType::END_OF_STORAGE:
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    case AC_PolyFenceType::POLYGON_EXCLUSION:
    case AC_PolyFenceType::POLYGON_INCLUSION: {
        const uint8_t vertex_count = fence_storage.read_uint8(read_offset+1);
        index.count = vertex_count;
        break;
    }
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
    case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
    case AC_PolyFenceType::HOME_CIRCLE_INCLUSION:
    case AC_PolyFenceType::RETURN_POINT:
        index.count = 1;
        break;
    }
}

bool AC_PolyFence_loader::index_eeprom()
{
    if (!formatted()) {
        if (!format()) {
            return false;
        }
    }

    if (!count_eeprom_fences()) {
        return false;
    }

    void_index();

    if (_eeprom_fence_count == 0) {
        _num_fences = 0;
        _load_attempted = false;
        return true;
    }

    Debug("Fence: Allocating %u bytes for index",
          (unsigned)(_eeprom_fence_count*sizeof(FenceIndex)));
    _index = NEW_NOTHROW FenceIndex[_eeprom_fence_count];
    if (_index == nullptr) {
        return false;
    }

    _num_fences = 0;
    if (!scan_eeprom(FUNCTOR_BIND_MEMBER(&AC_PolyFence_loader::scan_eeprom_index_fences, void, const AC_PolyFenceType, uint16_t))) {
        void_index();
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_num_fences != _eeprom_fence_count) {
        AP_HAL::panic("indexed fences not equal to eeprom fences");
    }
#endif

    _load_attempted = false;

    return true;
}

bool AC_PolyFence_loader::check_indexed()
{
    if (!_index_attempted) {
        _indexed = index_eeprom();
        _index_attempted = true;
    }
    return _indexed;
}

void AC_PolyFence_loader::unload()
{
    delete[] _loaded_offsets_from_origin;
    _loaded_offsets_from_origin = nullptr;

    delete[] _loaded_points_lla;
    _loaded_points_lla = nullptr;

    delete[] _loaded_inclusion_boundary;
    _loaded_inclusion_boundary = nullptr;
    _num_loaded_inclusion_boundaries = 0;

    delete[] _loaded_exclusion_boundary;
    _loaded_exclusion_boundary = nullptr;
    _num_loaded_exclusion_boundaries = 0;

    delete[] _loaded_circle_inclusion_boundary;
    _loaded_circle_inclusion_boundary = nullptr;
    _num_loaded_circle_inclusion_boundaries = 0;

    delete[] _loaded_circle_exclusion_boundary;
    _loaded_circle_exclusion_boundary = nullptr;
    _num_loaded_circle_exclusion_boundaries = 0;

    _loaded_return_point = nullptr;
    _loaded_return_point_lla = nullptr;
    _load_time_ms = 0;
}

// return the number of fences of type type in the index:
uint16_t AC_PolyFence_loader::index_fence_count(const AC_PolyFenceType type)
{
    uint16_t ret = 0;
    for (uint8_t i=0; i<_eeprom_fence_count; i++) {
        const FenceIndex &index = _index[i];
        if (index.type == type) {
            ret++;
        }
    }
    return ret;
}

uint16_t AC_PolyFence_loader::sum_of_polygon_point_counts_and_returnpoint()
{
    uint16_t ret = 0;
    for (uint8_t i=0; i<_eeprom_fence_count; i++) {
        const FenceIndex &index = _index[i];
        switch (index.type) {
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
        case AC_PolyFenceType::HOME_CIRCLE_INCLUSION:
            break;
        case AC_PolyFenceType::RETURN_POINT:
            ret += 1;
            break;
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            ret += index.count;
            break;
        case AC_PolyFenceType::END_OF_STORAGE:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
        }
    }
    return ret;
}

bool AC_PolyFence_loader::load_from_storage()
{
    if (!check_indexed()) {
        return false;
    }

    if (_load_attempted) {
        return _load_time_ms != 0;
    }

    if (!AP::ahrs().get_origin(loaded_origin)) {
//        Debug("fence load requires origin");
        return false;
    }

    // find indexes of each fence:
    if (!get_loaded_fence_semaphore().take_nonblocking()) {
        return false;
    }

    _load_attempted = true;

    unload();

    if (_eeprom_item_count == 0) {
        get_loaded_fence_semaphore().give();
        _load_time_ms = AP_HAL::millis();
        return true;
    }

    { // allocate array to hold offsets-from-origin
        const uint16_t count = sum_of_polygon_point_counts_and_returnpoint();
        Debug("Fence: Allocating %u bytes for points",
              (unsigned)(count * sizeof(Vector2f)));
        _loaded_offsets_from_origin = NEW_NOTHROW Vector2f[count];
        _loaded_points_lla = NEW_NOTHROW Vector2l[count];
        if (_loaded_offsets_from_origin == nullptr || _loaded_points_lla == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    // FIXME: find some way of factoring out all of these allocation routines.

    { // allocate storage for inclusion polyfences:
        const auto count = index_fence_count(AC_PolyFenceType::POLYGON_INCLUSION);
        Debug("Fence: Allocating %u bytes for inc. fences",
              (unsigned)(count * sizeof(InclusionBoundary)));
        _loaded_inclusion_boundary = NEW_NOTHROW InclusionBoundary[count];
        if (_loaded_inclusion_boundary == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    { // allocate storage for exclusion polyfences:
        const auto count = index_fence_count(AC_PolyFenceType::POLYGON_EXCLUSION);
        Debug("Fence: Allocating %u bytes for exc. fences",
              (unsigned)(count * sizeof(ExclusionBoundary)));
        _loaded_exclusion_boundary = NEW_NOTHROW ExclusionBoundary[count];
        if (_loaded_exclusion_boundary == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    { // allocate storage for circular inclusion fences:
        uint32_t count = index_fence_count(AC_PolyFenceType::CIRCLE_INCLUSION);
        count += index_fence_count(AC_PolyFenceType::HOME_CIRCLE_INCLUSION);
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        count += index_fence_count(AC_PolyFenceType::CIRCLE_INCLUSION_INT);
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        Debug("Fence: Allocating %u bytes for circ. inc. fences",
              (unsigned)(count * sizeof(InclusionCircle)));
        _loaded_circle_inclusion_boundary = NEW_NOTHROW InclusionCircle[count];
        if (_loaded_circle_inclusion_boundary == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    { // allocate storage for circular exclusion fences:
        uint32_t count = index_fence_count(AC_PolyFenceType::CIRCLE_EXCLUSION);
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        count += index_fence_count(AC_PolyFenceType::CIRCLE_EXCLUSION_INT);
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        Debug("Fence: Allocating %u bytes for circ. exc. fences",
              (unsigned)(count * sizeof(ExclusionCircle)));
        _loaded_circle_exclusion_boundary = NEW_NOTHROW ExclusionCircle[count];
        if (_loaded_circle_exclusion_boundary == nullptr) {
            unload();
            get_loaded_fence_semaphore().give();
            return false;
        }
    }

    Vector2f *next_storage_point = _loaded_offsets_from_origin;
    Vector2l *next_storage_point_lla = _loaded_points_lla;

    // use index to load fences from eeprom
    bool storage_valid = true;
    for (uint8_t i=0; i<_eeprom_fence_count; i++) {
        if (!storage_valid) {
            break;
        }
        const FenceIndex &index = _index[i];
        uint16_t storage_offset = index.storage_offset;
        storage_offset += 1; // skip type
        switch (index.type) {
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("indexed end of storage found");
#endif
            storage_valid = false;
            break;
        case AC_PolyFenceType::POLYGON_INCLUSION: {
            // FIXME: consider factoring this with the EXCLUSION case
            InclusionBoundary &boundary = _loaded_inclusion_boundary[_num_loaded_inclusion_boundaries];
            boundary.points = next_storage_point;
            boundary.points_lla = next_storage_point_lla;
            boundary.count = index.count;
            if (index.count < 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: invalid polygon vertex count %u", index.count);
                storage_valid = false;
                break;
            }
            storage_offset += 1; // skip vertex count
            if (!read_polygon_from_storage(loaded_origin, storage_offset, index.count, next_storage_point, next_storage_point_lla)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: polygon read failed");
                storage_valid = false;
                break;
            }
            _num_loaded_inclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::POLYGON_EXCLUSION: {
            ExclusionBoundary &boundary = _loaded_exclusion_boundary[_num_loaded_exclusion_boundaries];
            boundary.points = next_storage_point;
            boundary.points_lla = next_storage_point_lla;
            boundary.count = index.count;
            if (index.count < 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: invalid polygon vertex count %u", index.count);
                storage_valid = false;
                break;
            }
            storage_offset += 1; // skip vertex count
            if (!read_polygon_from_storage(loaded_origin, storage_offset, index.count, next_storage_point, next_storage_point_lla)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: polygon read failed");
                storage_valid = false;
                break;
            }
            _num_loaded_exclusion_boundaries++;
            break;
        }
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_EXCLUSION: {
            ExclusionCircle &circle = _loaded_circle_exclusion_boundary[_num_loaded_circle_exclusion_boundaries];
            if (!read_latlon_from_storage(storage_offset, circle.point)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            if (!scale_latlon_from_origin(loaded_origin, circle.point, circle.pos_cm)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            // now read the radius
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
            if (index.type == AC_PolyFenceType::CIRCLE_EXCLUSION_INT) {
                circle.radius = fence_storage.read_uint32(storage_offset);
            } else
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
            {
                circle.radius = fence_storage.read_float(storage_offset);
            }
            if (!is_positive(circle.radius)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: non-positive circle radius");
                storage_valid = false;
                break;
            }
            _num_loaded_circle_exclusion_boundaries++;
            break;
        }
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION: {
            InclusionCircle &circle = _loaded_circle_inclusion_boundary[_num_loaded_circle_inclusion_boundaries];
            if (!read_latlon_from_storage(storage_offset, circle.point)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            if (!scale_latlon_from_origin(loaded_origin, circle.point, circle.pos_cm)){
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            // now read the radius
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
            if (index.type == AC_PolyFenceType::CIRCLE_INCLUSION_INT) {
                circle.radius = fence_storage.read_uint32(storage_offset);
            } else
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
            {
                circle.radius = fence_storage.read_float(storage_offset);
            }
            if (!is_positive(circle.radius)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: non-positive circle radius");
                storage_valid = false;
                break;
            }
            _num_loaded_circle_inclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::HOME_CIRCLE_INCLUSION: {
            if (!AP::ahrs().home_is_set()) {
                // technically we could look at this when we go to use
                // the fence, but that's a lot of places to get
                // correct!
                return false;
            }
            InclusionCircle &circle = _loaded_circle_inclusion_boundary[_num_loaded_circle_inclusion_boundaries];
            circle.set_home_centered();
            // now read the radius
            circle.radius = fence_storage.read_float(storage_offset);
            if (!is_positive(circle.radius)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AC_Fence: non-positive circle radius");
                storage_valid = false;
                break;
            }
            _num_loaded_circle_inclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::RETURN_POINT:
            if (_loaded_return_point != nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PolyFence: Multiple return points found");
                storage_valid = false;
                break;
            }
            _loaded_return_point = next_storage_point;
            if (_loaded_return_point_lla != nullptr) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PolyFence: Multiple return points found");
                storage_valid = false;
                break;
            }
            _loaded_return_point_lla = next_storage_point_lla;
            // Read the point from storage
            if (!read_latlon_from_storage(storage_offset, *next_storage_point_lla)) {
                storage_valid = false;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PolyFence: latlon read failed");
                break;
            }
            if (!scale_latlon_from_origin(loaded_origin, *next_storage_point_lla, *next_storage_point)) {
                storage_valid = false;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PolyFence: latlon read failed");
                break;
            }
            next_storage_point++;
            next_storage_point_lla++;
            break;
        }
    }

    if (!storage_valid) {
        unload();
        get_loaded_fence_semaphore().give();
        return false;
    }

    _load_time_ms = AP_HAL::millis();

    get_loaded_fence_semaphore().give();
    return true;
}

/// returns pointer to array of exclusion polygon points and num_points is filled in with the number of points in the polygon
/// points are offsets in cm from EKF origin in NE frame
Vector2f* AC_PolyFence_loader::get_exclusion_polygon(uint16_t index, uint16_t &num_points) const
{
    if (index >= _num_loaded_exclusion_boundaries) {
        num_points = 0;
        return nullptr;
    }
    const ExclusionBoundary &boundary = _loaded_exclusion_boundary[index];
    num_points = boundary.count;

    return boundary.points;
}

/// returns pointer to array of inclusion polygon points and num_points is filled in with the number of points in the polygon
/// points are offsets in cm from EKF origin in NE frame
Vector2f* AC_PolyFence_loader::get_inclusion_polygon(uint16_t index, uint16_t &num_points) const
{
    if (index >= _num_loaded_inclusion_boundaries) {
        num_points = 0;
        return nullptr;
    }
    const InclusionBoundary &boundary = _loaded_inclusion_boundary[index];
    num_points = boundary.count;

    return boundary.points;
}

/// returns the specified exclusion circle
/// circle center offsets in cm from EKF origin in NE frame, radius is in meters
bool AC_PolyFence_loader::get_exclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const
{
    if (index >= _num_loaded_circle_exclusion_boundaries) {
        return false;
    }
    center_pos_cm = _loaded_circle_exclusion_boundary[index].pos_cm;
    radius =  _loaded_circle_exclusion_boundary[index].radius;
    return true;
}

/// returns the specified inclusion circle
/// circle centre offsets in cm from EKF origin in NE frame, radius is in meters
bool AC_PolyFence_loader::get_inclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const
{
    if (index >= _num_loaded_circle_inclusion_boundaries) {
        return false;
    }
    const auto &circle = _loaded_circle_inclusion_boundary[index];
    if (circle.is_home_centered()) {
        // Express home as an offset from EKF origin in the NE frame, matching
        // the coordinate space callers (AC_Avoid, OA) expect.  Queried live so
        // that avoidance tracks home if it is relocated during flight.
        Location origin;
        if (!AP::ahrs().get_origin(origin)) {
            return false;
        }
        center_pos_cm = origin.get_distance_NE(AP::ahrs().get_home()) * 100.0f;
    } else {
        center_pos_cm = circle.pos_cm;
    }
    radius =  circle.radius;
    return true;
}

bool AC_PolyFence_loader::check_inclusion_circle_margin(float margin) const
{
    // check circular includes
    for (uint8_t i=0; i<_num_loaded_circle_inclusion_boundaries; i++) {
        const InclusionCircle &circle = _loaded_circle_inclusion_boundary[i];
        if (circle.radius < margin) {
            // circle radius should never be less than margin
            return false;
        } 
    }
    return true;
}

bool AC_PolyFence_loader::validate_fence(const AC_PolyFenceItem *new_items, uint16_t count) const
{
    // validate the fence items...
    AC_PolyFenceType expecting_type = AC_PolyFenceType::END_OF_STORAGE;
    uint16_t expected_type_count = 0;
    uint16_t orig_expected_type_count = 0;
    bool seen_return_point = false;

    for (uint16_t i=0; i<count; i++) {
        bool validate_latlon = false;

        switch (new_items[i].type) {
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("passed in an END_OF_STORAGE");
#endif
            return false;

        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            if (new_items[i].vertex_count < 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Invalid vertex count (%u)", new_items[i].vertex_count);
                return false;
            }
            if (expected_type_count == 0) {
                expected_type_count = new_items[i].vertex_count;
                orig_expected_type_count = expected_type_count;
                expecting_type = new_items[i].type;
            } else {
                if (new_items[i].type != expecting_type) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Received incorrect vertex type (want=%u got=%u)", (unsigned)expecting_type, (unsigned)new_items[i].type);
                    return false;
                } else if (new_items[i].vertex_count != orig_expected_type_count) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Unexpected vertex count want=%u got=%u", orig_expected_type_count, new_items[i].vertex_count);
                    return false;
                }
            }
            expected_type_count--;
            validate_latlon = true;
            break;

#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
            // should never have AC_PolyFenceItems of these types
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
            if (expected_type_count) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Received incorrect type (want=%u got=%u)", (unsigned)expecting_type, (unsigned)new_items[i].type);
                return false;
            }
            if (!is_positive(new_items[i].radius)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Non-positive circle radius");
                return false;
            }
            validate_latlon = true;
            break;
        case AC_PolyFenceType::HOME_CIRCLE_INCLUSION:
            if (expected_type_count) {
               GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Received incorrect type (want=%u got=%u)", (unsigned)expecting_type, (unsigned)new_items[i].type);
               return false;
            }
            if (!is_positive(new_items[i].radius)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Non-positive circle radius");
                return false;
            }
            // Rely on the AHRS home checking.
            break;
        case AC_PolyFenceType::RETURN_POINT:
            if (expected_type_count) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Received incorrect type (want=%u got=%u)", (unsigned)expecting_type, (unsigned)new_items[i].type);
                return false;
            }

            // spec says only one return point allowed
            if (seen_return_point) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Multiple return points");
                return false;
            }
            seen_return_point = true;
            validate_latlon = true;
            // TODO: ensure return point is within all fences and
            // outside all exclusion zones
            break;
        }

        if (validate_latlon) {
            if (!check_latlng(new_items[i].loc[0], new_items[i].loc[1])) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Bad lat or lon");
                return false;
            }
        }
    }

    if (expected_type_count) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Incorrect item count");
        return false;
    }

    return true;
}

uint16_t AC_PolyFence_loader::fence_storage_space_required(const AC_PolyFenceItem *new_items, uint16_t count)
{
    uint16_t ret = 4; // for the format header
    uint16_t i = 0;
    while (i < count) {
        ret += 1; // one byte for type
        switch (new_items[i].type) {
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            ret += 1 + 8 * new_items[i].vertex_count; // 1 count, 4 lat, 4 lon for each point
            i += new_items[i].vertex_count - 1; // i is incremented down below
            break;
        case AC_PolyFenceType::END_OF_STORAGE:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
            // should never have AC_PolyFenceItems of these types
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
            ret += 12; // 4 radius, 4 lat, 4 lon
            break;
        case AC_PolyFenceType::HOME_CIRCLE_INCLUSION:
            ret += 4; // radius
            break;
        case AC_PolyFenceType::RETURN_POINT:
            ret += 8; // 4 lat, 4 lon
            break;
        }
        i++;
    }
    return ret;
}

bool AC_PolyFence_loader::write_fence(const AC_PolyFenceItem *new_items, uint16_t count)
{
    if (!validate_fence(new_items, count)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Fence validation failed");
        return false;
    }

    if (fence_storage_space_required(new_items, count) > fence_storage.size()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Fence exceeds storage size");
        return false;
    }

    if (!format()) {
        return false;
    }

    uint16_t total_vertex_count = 0;
    uint16_t offset = 4; // skipping magic
    uint8_t vertex_count = 0;
    for (uint16_t i=0; i<count; i++) {
        const AC_PolyFenceItem new_item = new_items[i];
        switch (new_item.type) {
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            if (vertex_count == 0) {
                // write out new polygon count
                vertex_count = new_item.vertex_count;
                total_vertex_count += vertex_count;
                if (!write_type_to_storage(offset, new_item.type)) {
                    return false;
                }
                fence_storage.write_uint8(offset, vertex_count);
                offset++;
            }
            vertex_count--;
            if (!write_latlon_to_storage(offset, new_item.loc)) {
                return false;
            }
            break;
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("asked to store end-of-storage marker");
#endif
            return false;
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION_INT:
        case AC_PolyFenceType::CIRCLE_EXCLUSION_INT:
            // should never have AC_PolyFenceItems of these types
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return false;
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION: {
            total_vertex_count++; // useful to make number of lines in QGC file match FENCE_TOTAL
            if (!write_type_to_storage(offset, new_item.type)) {
                return false;
            }
            if (!write_latlon_to_storage(offset, new_item.loc)) {
                return false;
            }
            fence_storage.write_float(offset, new_item.radius);
            offset += 4; // radius is 4 bytes.
            break;
        }
        case AC_PolyFenceType::HOME_CIRCLE_INCLUSION: {
            total_vertex_count++; // useful to make number of lines in QGC file match FENCE_TOTAL
            if (!write_type_to_storage(offset, new_item.type)) {
                return false;
            }
            fence_storage.write_float(offset, new_item.radius);
            offset += 4; // radius is 4 bytes.
            break;
        }
        case AC_PolyFenceType::RETURN_POINT:
            if (!write_type_to_storage(offset, new_item.type)) {
                return false;
            }
            if (!write_latlon_to_storage(offset, new_item.loc)) {
                return false;
            }
            break;
        }
    }
    if (!write_eos_to_storage(offset)) {
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // sanity-check the EEPROM in SITL to make sure we can read what
    // we've just written.
    if (!index_eeprom()) {
        AP_HAL::panic("Failed to index eeprom");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Fence Indexed OK");
#endif

#if HAL_LOGGING_ENABLED
    // start logger logging new fence
    AP::logger().Write_Fence();
#endif

    void_index();

    // this may be completely bogus total.  If we are storing an
    // advanced fence then the old protocol which relies on this value
    // will error off if the GCS tries to fetch points.  This number
    // should be correct for a "compatible" fence, however.
    uint16_t new_total = 0;
    if (total_vertex_count >= 3) {
        new_total = total_vertex_count+2;
    }
    _total.set_and_save(new_total);

    return true;
}


bool AC_PolyFence_loader::get_return_point(Vector2l &ret)
{
    if (!check_indexed()) {
        return false;
    }

    const FenceIndex *rp = find_first_fence(AC_PolyFenceType::RETURN_POINT);
    if (rp != nullptr) {
        uint16_t read_offset = rp->storage_offset + 1;
        return read_latlon_from_storage(read_offset, ret);
    }

    const FenceIndex *inc = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
    if (inc == nullptr) {
        return false;
    }

    // we found an inclusion fence but not a return point.  Calculate
    // and return the centroid.  Note that this may not actually be
    // inside all inclusion fences...
    uint16_t offset = inc->storage_offset;
    if ((AC_PolyFenceType)fence_storage.read_uint8(offset) != AC_PolyFenceType::POLYGON_INCLUSION) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("wrong type at offset");
#endif
        return false;
    }
    offset++;
    const uint8_t count = fence_storage.read_uint8(offset);
    if (count < 3) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("invalid count found");
#endif
        return false;
    }
    offset++;
    Vector2l min_loc;
    if (!read_latlon_from_storage(offset, min_loc)) {
        return false;
    }
    if (min_loc.is_zero()) {
        return false;
    }
    Vector2l max_loc = min_loc;
    for (uint8_t i=1; i<count; i++) {
        Vector2l new_loc;
        if (!read_latlon_from_storage(offset, new_loc)) {
            return false;
        }
        if (new_loc.is_zero()) {
            return false;
        }
        if (new_loc.x < min_loc.x) {
            min_loc.x = new_loc.x;
        }
        if (new_loc.y < min_loc.y) {
            min_loc.y = new_loc.y;
        }
        if (new_loc.x > max_loc.x) {
            max_loc.x = new_loc.x;
        }
        if (new_loc.y > max_loc.y) {
            max_loc.y = new_loc.y;
        }
    }

    // Potential for int32_t overflow when longitudes are beyond [-107, 107].
    // As a result, the calculated return point's longitude is calculated using overflowed figure.
    // Dividing first before adding avoids the potential overflow.
    ret.x = (min_loc.x / 2) + (max_loc.x / 2);
    ret.y = (min_loc.y / 2) + (max_loc.y / 2);

    return true;
}

AC_PolyFence_loader::FenceIndex *AC_PolyFence_loader::find_first_fence(const AC_PolyFenceType type) const
{
    if (_index == nullptr) {
        return nullptr;
    }
    for (uint8_t i=0; i<_num_fences; i++) {
        if (_index[i].type == type) {
            return &_index[i];
        }
    }
    return nullptr;
}

bool AC_PolyFence_loader::write_eos_to_storage(uint16_t &offset)
{
    if (!write_type_to_storage(offset, AC_PolyFenceType::END_OF_STORAGE)) {
        return false;
    }
    _eos_offset = offset - 1; // should point to the marker
    return true;
}

void AC_PolyFence_loader::update()
{
    if (!load_from_storage()) {
        return;
    }
}

#else  // build type is not appropriate; provide a dummy implementation:

void AC_PolyFence_loader::init() {};

bool AC_PolyFence_loader::get_item(const uint16_t seq, AC_PolyFenceItem &item) { return false; }

Vector2f* AC_PolyFence_loader::get_exclusion_polygon(uint16_t index, uint16_t &num_points) const { return nullptr; }
Vector2f* AC_PolyFence_loader::get_inclusion_polygon(uint16_t index, uint16_t &num_points) const { return nullptr; }

bool AC_PolyFence_loader::get_exclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const { return false; }
bool AC_PolyFence_loader::get_inclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const { return false; }

bool AC_PolyFence_loader::breached() const { return false; }
bool AC_PolyFence_loader::breached(const Location& loc, float& distance_outside_fence, Vector2f& fence_direction) const { return false; }
bool AC_PolyFence_loader::get_closest_loc_within_fence(const Location& loc, float margin_cm, Location& loc_closest) const { return false; }

uint16_t AC_PolyFence_loader::max_items() const { return 0; }

bool AC_PolyFence_loader::write_fence(const AC_PolyFenceItem *new_items, uint16_t count) { return false; }

void AC_PolyFence_loader::update() {};

#endif // #if AC_FENCE_DUMMY_METHODS_ENABLED
#endif // AP_FENCE_ENABLED
