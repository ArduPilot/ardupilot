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

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#if APM_BUILD_COPTER_OR_HELI
#include <AP_Logger/AP_Logger.h>
#endif
#include "SCurve.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

extern const AP_HAL::HAL &hal;

#define SEG_INIT                0
#define SEG_ACCEL_MAX           4
#define SEG_ACCEL_END           7
#define SEG_SPEED_CHANGE_END    14
#define SEG_CONST               15
#define SEG_DECEL_START         15
#define SEG_DECEL_END           22

// constructor
SCurve::SCurve()
{
    init();
}

// initialise and clear the path
void SCurve::init()
{
    snap_max = 0.0f;
    jerk_max = 0.0f;
    accel_max = 0.0f;
    vel_max = 0.0f;
    time = 0.0f;
    num_segs = SEG_INIT;
    add_segment(num_segs, 0.0f, SegmentType::CONSTANT_JERK, 0.0f, 0.0f, 0.0f, 0.0f);

    is_arc_segment = false;
    seg_delta.zero();
    seg_length = 0.0;
    arc = {};
}

// generate a 3D trigonometric track defined by origin, destination, and arc angle in radians (0 = straight)
// includes speed, acceleration, and jerk limits for horizontal and vertical motion
void SCurve::calculate_track(const Vector3p &origin, const Vector3p &destination, float arc_ang_rad,
                             float speed_xy, float speed_up, float speed_down,
                             float accel_xy, float accel_z, float accel_c,
                             float snap_maximum, float jerk_maximum)
{
    init();

    // ensure arguments are positive
    speed_xy = fabsf(speed_xy);
    speed_up = fabsf(speed_up);
    speed_down = fabsf(speed_down);
    accel_xy = fabsf(accel_xy);
    accel_z = fabsf(accel_z);

    // leave track as zero length if origin and destination are equal or if the new track length squared is zero
    seg_delta = (destination - origin).tofloat();
    if (seg_delta.is_zero() || is_zero(seg_delta.length_squared())) {
        seg_delta.zero();
        return;
    }

    const Vector2f chord = seg_delta.xy();
    const float chord_length = seg_delta.xy().length();
    if (!is_positive(chord_length) || fabsf(wrap_PI(arc_ang_rad)) < radians(1.0)) {
        // straight segment
        is_arc_segment = false;
        arc.angle_rad = 0.0f;
        arc.length_ne = chord_length;
        arc.radius_ne = 0.0f;
        arc.center_ne = Vector2f();
        seg_length = seg_delta.length();
    } else {
        is_arc_segment = true;
        arc.angle_rad = arc_ang_rad;
        arc.radius_ne = fabsf(chord_length / (2.0f * fabsf(sinf(arc.angle_rad * 0.5f))));
        const float center_offset = safe_sqrt(sq(arc.radius_ne) - sq(chord_length * 0.5f)); // perpendicular offset from chord to circle center
        const float turn_dir = is_negative(arc.angle_rad) ? -1.0f : 1.0f; // -1 for CCW, 1 for CW 
        const float center_side = (is_positive(wrap_PI(fabsf(arc.angle_rad)))) ? 1.0f : -1.0f; // -1 for CCW, 1 for CW
        if (!is_zero(arc.radius_ne) && !is_zero(chord_length)) {
            arc.center_ne = chord * 0.5f + Vector2f(-chord.y, chord.x) * (center_side * turn_dir * center_offset / chord_length);
            arc.length_ne = arc.radius_ne * fabsf(arc.angle_rad);
            seg_length = safe_sqrt(sq(seg_delta.z) + sq(arc.length_ne));
            accel_c = is_positive(accel_c) ? accel_c : accel_xy;
            speed_xy = MIN(speed_xy, safe_sqrt(accel_c * arc.radius_ne));
        } else {
            // straight segment
            is_arc_segment = false;
            arc.angle_rad = 0.0f;
            arc.length_ne = chord_length;
            arc.radius_ne = 0.0f;
            arc.center_ne = Vector2f();
            seg_length = seg_delta.length();
        }
    }
    if (is_zero(seg_length)) {
        seg_delta.zero();
        return;
    }

    // set snap_max and jerk max
    snap_max = snap_maximum;
    jerk_max = jerk_maximum;

    // update speed and acceleration limits along path
    set_kinematic_limits(origin, destination,
                         speed_xy, speed_up, speed_down,
                         accel_xy, accel_z);

    // avoid divide-by zeros. Path will be left as a zero length path
    if (!is_positive(snap_max) || !is_positive(jerk_max) || !is_positive(accel_max) || !is_positive(vel_max)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        ::printf("SCurve::calculate_track created zero length path\n");
#endif
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    add_segments(seg_length);

    // catch calculation errors
    if (!valid()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        ::printf("SCurve::calculate_track invalid path\n");
        debug();
#endif
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        init();
    }
}

// set maximum velocity and re-calculate the path using these limits
void SCurve::set_speed_max(float speed_xy, float speed_up, float speed_down)
{
    // ensure arguments are positive
    speed_xy = fabsf(speed_xy);
    speed_up = fabsf(speed_up);
    speed_down = fabsf(speed_down);

    // return immediately if zero length path
    if (num_segs != segments_max) {
        return;
    }

    // segment accelerations can not be changed after segment creation.
    const float track_speed_max = kinematic_limit(arc.length_ne, seg_delta.z, speed_xy, speed_up, speed_down);

    if (is_equal(vel_max, track_speed_max)) {
        // new speed is equal to current speed maximum so no need to change anything
        return;
    }

    if (is_zero(track_speed_max)) {
        // new speed is zero which is not supported
        return;
    }
    vel_max = track_speed_max;

    if (time >= segment[SEG_CONST].end_time) {
        return;
    }

    // re-calculate the s-curve path based on update speeds

    const float Pend = segment[SEG_DECEL_END].end_pos;
    float Vend = MIN(vel_max, segment[SEG_DECEL_END].end_vel);

    if (is_zero(time)) {
        // path has not started so we can recompute the path
        const float Vstart = MIN(vel_max, segment[SEG_INIT].end_vel);
        num_segs = SEG_INIT;
        add_segment(num_segs, 0.0f, SegmentType::CONSTANT_JERK, 0.0f, 0.0f, 0.0f, 0.0f);
        add_segments(Pend);
        set_origin_speed_max(Vstart);
        set_destination_speed_max(Vend);
        return;
    }

    if ((time >= segment[SEG_ACCEL_END].end_time) && (time <= segment[SEG_SPEED_CHANGE_END].end_time)) {
        // in the speed change phase
        // move speed change phase to acceleration phase to provide room for further speed adjustments

        // set initial segment to last acceleration segment
        segment[SEG_INIT].seg_type = SegmentType::CONSTANT_JERK;
        segment[SEG_INIT].jerk_ref = 0.0f;
        segment[SEG_INIT].end_time = segment[SEG_ACCEL_END].end_time;
        segment[SEG_INIT].end_accel = segment[SEG_ACCEL_END].end_accel;
        segment[SEG_INIT].end_vel = segment[SEG_ACCEL_END].end_vel;
        segment[SEG_INIT].end_pos = segment[SEG_ACCEL_END].end_pos;

        // move speed change segments to acceleration segments
        for (uint8_t i = SEG_INIT+1; i <= SEG_ACCEL_END; i++) {
            segment[i] = segment[i+7];
        }

        // set change segments to last acceleration speed
        for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_SPEED_CHANGE_END; i++) {
            segment[i].seg_type = SegmentType::CONSTANT_JERK;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = segment[SEG_ACCEL_END].end_time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
            segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
        }

    } else if ((time > segment[SEG_SPEED_CHANGE_END].end_time) && (time <= segment[SEG_CONST].end_time)) {
        // in the constant speed phase
        // overwrite the acceleration and speed change phases with the current position and velocity

        // set initial segment to last acceleration segment
        segment[SEG_INIT].seg_type = SegmentType::CONSTANT_JERK;
        segment[SEG_INIT].jerk_ref = 0.0f;
        segment[SEG_INIT].end_time = segment[SEG_SPEED_CHANGE_END].end_time;
        segment[SEG_INIT].end_accel = 0.0f;
        segment[SEG_INIT].end_vel = segment[SEG_SPEED_CHANGE_END].end_vel;
        segment[SEG_INIT].end_pos = segment[SEG_SPEED_CHANGE_END].end_pos;

        // set acceleration and change segments to current constant speed
        float Jt_out, At_out, Vt_out, Pt_out;
        get_jerk_accel_vel_pos_at_time(time, Jt_out, At_out, Vt_out, Pt_out);
        for (uint8_t i = SEG_INIT+1; i <= SEG_SPEED_CHANGE_END; i++) {
            segment[i].seg_type = SegmentType::CONSTANT_JERK;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = Vt_out;
            segment[i].end_pos = Pt_out;
        }
    }

    // adjust the INIT and ACCEL segments for new speed
    if ((time <= segment[SEG_ACCEL_MAX].end_time) && is_positive(segment[SEG_ACCEL_MAX].end_time - segment[SEG_ACCEL_MAX-1].end_time) && (vel_max < segment[SEG_ACCEL_END].end_vel) && is_positive(segment[SEG_ACCEL_MAX].end_accel) ) {
        // path has not finished constant positive acceleration segment
        // reduce velocity as close to target velocity as possible

        const float Vstart = segment[SEG_INIT].end_vel;

        // minimum velocity that can be obtained by shortening SEG_ACCEL_MAX
        const float Vmin = segment[SEG_ACCEL_END].end_vel - segment[SEG_ACCEL_MAX].end_accel * (segment[SEG_ACCEL_MAX].end_time - MAX(time, segment[SEG_ACCEL_MAX-1].end_time));

        float Jm, tj, t2, t4, t6;
        calculate_path(snap_max, jerk_max, Vstart, accel_max, MAX(Vmin, vel_max), Pend * 0.5f, Jm, tj, t2, t4, t6);

        uint8_t seg = SEG_INIT+1;
        add_segments_jerk(seg, tj, Jm, t2);
        add_segment_const_jerk(seg, t4, 0.0f);
        add_segments_jerk(seg, tj, -Jm, t6);

        // remove numerical errors
        segment[SEG_ACCEL_END].end_accel = 0.0f;

        // add empty speed adjust segments
        for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CONST; i++) {
            segment[i].seg_type = SegmentType::CONSTANT_JERK;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = segment[SEG_ACCEL_END].end_time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
            segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
        }

        calculate_path(snap_max, jerk_max, 0.0f, accel_max, MAX(Vmin, vel_max), Pend * 0.5f, Jm, tj, t2, t4, t6);

        seg = SEG_CONST + 1;
        add_segments_jerk(seg, tj, -Jm, t6);
        add_segment_const_jerk(seg, t4, 0.0f);
        add_segments_jerk(seg, tj, Jm, t2);

        // remove numerical errors
        segment[SEG_DECEL_END].end_accel = 0.0f;
        segment[SEG_DECEL_END].end_vel = MAX(0.0f, segment[SEG_DECEL_END].end_vel);

        // add to constant velocity segment to end at the correct position
        const float dP = MAX(0.0f, Pend - segment[SEG_DECEL_END].end_pos);
        const float t15 = dP / segment[SEG_CONST].end_vel;
        for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
            segment[i].end_time += t15;
            segment[i].end_pos += dP;
        }
    }

    // adjust the speed change segments (8 to 14) for new speed
    // start with empty speed adjust segments
    for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_SPEED_CHANGE_END; i++) {
        segment[i].seg_type = SegmentType::CONSTANT_JERK;
        segment[i].jerk_ref = 0.0f;
        segment[i].end_time = segment[SEG_ACCEL_END].end_time;
        segment[i].end_accel = 0.0f;
        segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
        segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
    }
    if (!is_equal(vel_max, segment[SEG_ACCEL_END].end_vel)) {
        // add velocity adjustment
        // check there is enough time to make velocity change
        // we use the approximation that the time will be distance/max_vel and 8 jerk segments
        const float L = segment[SEG_CONST].end_pos - segment[SEG_ACCEL_END].end_pos;
        float Jm = 0;
        float tj = 0;
        float t2 = 0;
        float t4 = 0;
        float t6 = 0;
        float jerk_time = MIN(powf((fabsf(vel_max - segment[SEG_ACCEL_END].end_vel) * M_PI) / (4 * snap_max), 1/3), jerk_max * M_PI / (2 * snap_max));
        if ((vel_max < segment[SEG_ACCEL_END].end_vel) && (jerk_time*12.0f < L/segment[SEG_ACCEL_END].end_vel)) {
            // we have a problem here with small segments.
            calculate_path(snap_max, jerk_max, vel_max, accel_max, segment[SEG_ACCEL_END].end_vel, L * 0.5f, Jm, tj, t6, t4, t2);
            Jm = -Jm;

        } else if ((vel_max > segment[SEG_ACCEL_END].end_vel) && (L/(jerk_time*12.0f) > segment[SEG_ACCEL_END].end_vel)) {
            float Vm = MIN(vel_max, L/(jerk_time*12.0f));
            calculate_path(snap_max, jerk_max, segment[SEG_ACCEL_END].end_vel, accel_max, Vm, L * 0.5f, Jm, tj, t2, t4, t6);
        }

        uint8_t seg = SEG_ACCEL_END + 1;
        if (!is_zero(Jm) && !is_negative(t2) && !is_negative(t4) && !is_negative(t6)) {
            add_segments_jerk(seg, tj, Jm, t2);
            add_segment_const_jerk(seg, t4, 0.0f);
            add_segments_jerk(seg, tj, -Jm, t6);

            // remove numerical errors
            segment[SEG_SPEED_CHANGE_END].end_accel = 0.0f;
        }
    }

    // add deceleration segments
    // earlier check should ensure that we should always have sufficient time to stop
    uint8_t seg = SEG_CONST;
    Vend = MIN(Vend, segment[SEG_SPEED_CHANGE_END].end_vel);
    add_segment_const_jerk(seg, 0.0f, 0.0f);
    if (Vend < segment[SEG_SPEED_CHANGE_END].end_vel) {
        float Jm, tj, t2, t4, t6;
        calculate_path(snap_max, jerk_max, Vend, accel_max, segment[SEG_CONST].end_vel, Pend - segment[SEG_CONST].end_pos, Jm, tj, t2, t4, t6);
        add_segments_jerk(seg, tj, -Jm, t6);
        add_segment_const_jerk(seg, t4, 0.0f);
        add_segments_jerk(seg, tj, Jm, t2);
    } else {
        // No deceleration is required
        for (uint8_t i = SEG_CONST+1; i <= SEG_DECEL_END; i++) {
            segment[i].seg_type = SegmentType::CONSTANT_JERK;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = segment[SEG_CONST].end_time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = segment[SEG_CONST].end_vel;
            segment[i].end_pos = segment[SEG_CONST].end_pos;
        }
    }

    // remove numerical errors
    segment[SEG_DECEL_END].end_accel = 0.0f;
    segment[SEG_DECEL_END].end_vel = MAX(0.0f, segment[SEG_DECEL_END].end_vel);

    // add to constant velocity segment to end at the correct position
    const float dP = MAX(0.0f, Pend - segment[SEG_DECEL_END].end_pos);
    const float t15 = dP / segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        segment[i].end_time += t15;
        segment[i].end_pos += dP;
    }

    // catch calculation errors
    if (!valid()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        ::printf("SCurve::set_speed_max invalid path\n");
        debug();
#endif
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        init();
    }
}

// set the maximum vehicle speed at the origin
// returns the expected speed at the origin which will always be equal or lower than speed
float SCurve::set_origin_speed_max(float speed)
{
    // if path is zero length then start speed must be zero
    if (num_segs != segments_max) {
        return 0.0f;
    }

    // ensure speed is positive
    speed = fabsf(speed);

    // avoid re-calculating if unnecessary
    if (is_equal(segment[SEG_INIT].end_vel, speed)) {
        return speed;
    }

    const float Vm = segment[SEG_ACCEL_END].end_vel;
    speed = MIN(speed, Vm);

    float Jm, tj, t2, t4, t6;
    calculate_path(snap_max, jerk_max, speed, accel_max, Vm, seg_length * 0.5f, Jm, tj, t2, t4, t6);

    uint8_t seg = SEG_INIT;
    add_segment(seg, 0.0f, SegmentType::CONSTANT_JERK, 0.0f, 0.0f, speed, 0.0f);
    add_segments_jerk(seg, tj, Jm, t2);
    add_segment_const_jerk(seg, t4, 0.0f);
    add_segments_jerk(seg, tj, -Jm, t6);

    // remove numerical errors
    segment[SEG_ACCEL_END].end_accel = 0.0f;

    // offset acceleration segment if we can't fit it all into half the original length
    const float dPstart = MIN(0.0f, seg_length * 0.5f - segment[SEG_ACCEL_END].end_pos);
    const float dt = dPstart / segment[SEG_ACCEL_END].end_vel;
    for (uint8_t i = SEG_INIT; i <= SEG_ACCEL_END; i++) {
        segment[i].end_time += dt;
        segment[i].end_pos += dPstart;
    }

    // add empty speed change segments and constant speed segment
    for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_SPEED_CHANGE_END; i++) {
        segment[i].seg_type = SegmentType::CONSTANT_JERK;
        segment[i].jerk_ref = 0.0f;
        segment[i].end_time = segment[SEG_ACCEL_END].end_time;
        segment[i].end_accel = 0.0f;
        segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
        segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
    }

    seg = SEG_CONST;
    add_segment_const_jerk(seg, 0.0f, 0.0f);

    calculate_path(snap_max, jerk_max, 0.0f, accel_max, segment[SEG_CONST].end_vel, seg_length * 0.5f, Jm, tj, t2, t4, t6);

    add_segments_jerk(seg, tj, -Jm, t6);
    add_segment_const_jerk(seg, t4, 0.0f);
    add_segments_jerk(seg, tj, Jm, t2);

    // remove numerical errors
    segment[SEG_DECEL_END].end_accel = 0.0f;
    segment[SEG_DECEL_END].end_vel = MAX(0.0f, segment[SEG_DECEL_END].end_vel);

    // add to constant velocity segment to end at the correct position
    const float dP = MAX(0.0f, seg_length - segment[SEG_DECEL_END].end_pos);
    const float t15 = dP / segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        segment[i].end_time += t15;
        segment[i].end_pos += dP;
    }

    // catch calculation errors
    if (!valid()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        ::printf("SCurve::set_origin_speed_max invalid path\n");
        debug();
#endif
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        init();
        return 0.0f;
    }

    return speed;
}

// set the maximum vehicle speed at the destination
void SCurve::set_destination_speed_max(float speed)
{
    // if path is zero length then all speeds must be zero
    if (num_segs != segments_max) {
        return;
    }
    
    // ensure speed is positive
    speed = fabsf(speed);

    // avoid re-calculating if unnecessary
    if (is_equal(segment[segments_max-1].end_vel, speed)) {
        return;
    }

    const float Vm = segment[SEG_CONST].end_vel;
    speed = MIN(speed, Vm);

    float Jm, tj, t2, t4, t6;
    calculate_path(snap_max, jerk_max, speed, accel_max, Vm, seg_length * 0.5f, Jm, tj, t2, t4, t6);

    uint8_t seg = SEG_CONST;
    add_segment_const_jerk(seg, 0.0f, 0.0f);

    add_segments_jerk(seg, tj, -Jm, t6);
    add_segment_const_jerk(seg, t4, 0.0f);
    add_segments_jerk(seg, tj, Jm, t2);

    // remove numerical errors
    segment[SEG_DECEL_END].end_accel = 0.0f;
    segment[SEG_DECEL_END].end_vel = MAX(0.0f, segment[SEG_DECEL_END].end_vel);

    // add to constant velocity segment to end at the correct position
    const float dP = MAX(0.0f, seg_length - segment[SEG_DECEL_END].end_pos);
    const float t15 = dP / segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        segment[i].end_time += t15;
        segment[i].end_pos += dP;
    }

    // catch calculation errors
    if (!valid()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        ::printf("SCurve::set_destination_speed_max invalid path\n");
        debug();
#endif
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        init();
    }
}

// move target location along path from origin to destination
// prev_leg and next_leg are the paths before and after this path
// wp_radius is max distance from the waypoint at the apex of the turn
// fast_waypoint should be true if vehicle will not stop at end of this leg
// dt is the time increment the vehicle will move along the path
// target_pos should be set to this segment's origin and it will be updated to the current position target
// target_vel and target_accel are updated with new targets
// returns true if vehicle has passed the apex of the corner
bool SCurve::advance_target_along_track(SCurve &prev_leg, SCurve &next_leg, float wp_radius, float accel_corner, bool fast_waypoint, float dt, Vector3p &target_pos, Vector3f &target_vel, Vector3f &target_accel)
{
    prev_leg.move_to_pos_vel_accel(dt, target_pos, target_vel, target_accel);
    move_from_pos_vel_accel(dt, target_pos, target_vel, target_accel);
    bool s_finished = finished();

    // check for change of leg on fast waypoint
    const float time_to_destination = get_time_remaining();
    if (fast_waypoint 
        && is_zero(next_leg.get_time_elapsed()) // The next leg has not started
        && (get_time_elapsed() >= time_decel_start()) // The current leg has started the deceleration phase
        && (get_time_remaining() <= next_leg.time_accel_end()) // The current leg will finish before completion of the acceleration phase of the next leg
        ) {

        // Calculate the position, velocity and acceleration at the turn mid point
        Vector3p turn_pos = -get_track().topostype();
        Vector3f turn_vel, turn_accel;
        move_from_time_pos_vel_accel(get_time_elapsed() + time_to_destination * 0.5f, turn_pos, turn_vel, turn_accel);
        next_leg.move_from_time_pos_vel_accel(time_to_destination * 0.5f, turn_pos, turn_vel, turn_accel);
        const float speed_min = MIN(get_speed_along_track(), next_leg.get_speed_along_track());
        const float accel_z_lim = MIN(get_accel_z_max(), next_leg.get_accel_z_max());
        if ((turn_pos.length() < wp_radius) // The turn mid point is within the waypoint radius
            && (turn_vel.length() < speed_min) // The speed at the turn mid point is less than the minimum speed
            && (Vector2f{turn_accel.x, turn_accel.y}.length() < accel_corner) // The acceleration at the turn mid point is less than the corner acceleration
            && (fabsf(turn_accel.z) < accel_z_lim) // The vertical acceleration at the turn mid point is less than the maximum vertical acceleration
            ) {
            next_leg.move_from_pos_vel_accel(dt, target_pos, target_vel, target_accel);
        }
    } else if (!is_zero(next_leg.get_time_elapsed())) {
        next_leg.move_from_pos_vel_accel(dt, target_pos, target_vel, target_accel);
        if (next_leg.get_time_elapsed() >= get_time_remaining()) {
            // consider the current leg finished when we have passed half way through the turn between legs
            s_finished = true;
        }
    }

    return s_finished;
}

// time has reached the end of the sequence
bool SCurve::finished() const
{
    return time >= time_end();
}

// increment time pointer and return the position, velocity and acceleration vectors relative to the origin
void SCurve::move_from_pos_vel_accel(float dt, Vector3p &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(dt);
    float scurve_P1 = 0.0f;
    float scurve_V1, scurve_A1, scurve_J1;
    get_jerk_accel_vel_pos_at_time(time, scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    project_scurve_onto_track(scurve_A1, scurve_V1, scurve_P1, pos, vel, accel);
}

// increment time pointer and return the position, velocity and acceleration vectors relative to the destination
void SCurve::move_to_pos_vel_accel(float dt, Vector3p &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(dt);
    float scurve_P1 = 0.0f;
    float scurve_V1, scurve_A1, scurve_J1;
    get_jerk_accel_vel_pos_at_time(time, scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    project_scurve_onto_track(scurve_A1, scurve_V1, scurve_P1, pos, vel, accel);
    pos -= seg_delta.topostype();
}

// return the position, velocity and acceleration vectors relative to the origin at a specified time along the path
void SCurve::move_from_time_pos_vel_accel(float time_now, Vector3p &pos, Vector3f &vel, Vector3f &accel)
{
    float scurve_P1 = 0.0f;
    float scurve_V1 = 0.0f, scurve_A1 = 0.0f, scurve_J1 = 0.0f;
    get_jerk_accel_vel_pos_at_time(time_now, scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    project_scurve_onto_track(scurve_A1, scurve_V1, scurve_P1, pos, vel, accel);
}

// project the straight-line S-curve motion profile onto the active track segment
// converts scalar S-curve kinematics (A1, V1, P1) into 3D position, velocity and acceleration
// along either a circular arc or straight segment
void SCurve::project_scurve_onto_track(float scurve_A1, float scurve_V1, float scurve_P1, Vector3p &pos, Vector3f &vel, Vector3f &accel)
{
    if (is_zero(seg_length)) {
        return;
    }

    if (is_arc_segment) {
        // Arc segment projection
        
        // protect against divide by zero
        if (!is_positive(arc.radius_ne)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            ::printf("SCurve:: Arc radius is negative or zero\n");
#endif
            INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
            init();
            return;
        }

        Vector2f center_to_pos_ne = -arc.center_ne;
        const float turn_dir = is_negative(arc.angle_rad) ? -1.0f : 1.0f; // -1 for CCW, 1 for CW 
        // rotate by progress along the arc
        center_to_pos_ne.rotate(turn_dir * fabsf(arc.angle_rad) * (scurve_P1 / seg_length));

        // position
        const float dz_ds = seg_delta.z / seg_length;
        Vector3f delta_pos(arc.center_ne + center_to_pos_ne, scurve_P1 * dz_ds);
        pos += delta_pos.topostype();

        // direction unit (tangent + vertical slope)
        Vector2f arc_tangent_ne = Vector2f(-center_to_pos_ne.y, center_to_pos_ne.x) * turn_dir;
        arc_tangent_ne /= arc.radius_ne;
        Vector3f path_unit(arc_tangent_ne.x, arc_tangent_ne.y, dz_ds);
        path_unit.normalize();

        // velocity & tangential accel
        vel += path_unit * scurve_V1;
        accel += path_unit * scurve_A1;

        // centripetal accel
        accel.xy() -= center_to_pos_ne * sq(scurve_V1 / arc.radius_ne);

        return;
    }

    // Straight segment projection
    const Vector3f path_unit = seg_delta.normalized();
    pos += path_unit.topostype() * (postype_t)scurve_P1;
    vel += path_unit * scurve_V1;
    accel += path_unit * scurve_A1;
}

// time at the end of the sequence
float SCurve::time_end() const
{
    if (num_segs != segments_max) {
        return 0.0;
    }
    return segment[SEG_DECEL_END].end_time;
}

// time left before sequence will complete
float SCurve::get_time_remaining() const
{
    if (num_segs != segments_max) {
        return 0.0;
    }
    return segment[SEG_DECEL_END].end_time - time;
}

// time when acceleration section of the sequence will complete
float SCurve::get_accel_finished_time() const
{
    if (num_segs != segments_max) {
        return 0.0;
    }
    return segment[SEG_ACCEL_END].end_time;
}

// return true if the sequence is braking to a stop
bool SCurve::braking() const
{
    if (num_segs != segments_max) {
        return true;
    }
    return time >= segment[SEG_CONST].end_time;
}

// return time offset for the start of the constant speed section and end of the acceleration section
// used to initiate the turn onto leg
float SCurve::time_accel_end() const
{
    if (num_segs != segments_max) {
        return 0.0;
    }
    return segment[SEG_ACCEL_END].end_time;
}

// return time offset for the end of the constant speed section and start of the deceleration section
// used to initiate the turn from leg
float SCurve::time_decel_start() const
{
    if (num_segs != segments_max) {
        return 0.0;
    }
    return segment[SEG_DECEL_START].end_time;
}

// increment the internal time
void SCurve::advance_time(float dt)
{
    time = MIN(time+dt, time_end());
}

// calculate the jerk, acceleration, velocity and position at the provided time
void SCurve::get_jerk_accel_vel_pos_at_time(float time_now, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out) const
{
    // start with zeros as function is void and we want to guarantee all outputs are initialised
    Jt_out = 0;
    At_out = 0;
    Vt_out = 0;
    Pt_out = 0;
    if (num_segs != segments_max) {
        return;
    }

    SegmentType Jtype;
    uint8_t pnt = num_segs;
    float Jm, tj, T0, A0, V0, P0;

    // find active segment at time_now
    for (uint8_t i = 0; i < num_segs; i++) {
        if (time_now < segment[num_segs - 1 - i].end_time) {
            pnt = num_segs - 1 - i;
        }
    }
    if (pnt == 0) {
        Jtype = SegmentType::CONSTANT_JERK;
        Jm = 0.0f;
        tj = 0.0f;
        T0 = segment[pnt].end_time;
        A0 = segment[pnt].end_accel;
        V0 = segment[pnt].end_vel;
        P0 = segment[pnt].end_pos;
    } else if (pnt == num_segs) {
        Jtype = SegmentType::CONSTANT_JERK;
        Jm = 0.0f;
        tj = 0.0f;
        T0 = segment[pnt - 1].end_time;
        A0 = segment[pnt - 1].end_accel;
        V0 = segment[pnt - 1].end_vel;
        P0 = segment[pnt - 1].end_pos;
    } else {
        Jtype = segment[pnt].seg_type;
        Jm = segment[pnt].jerk_ref;
        tj = segment[pnt].end_time - segment[pnt - 1].end_time;
        T0 = segment[pnt - 1].end_time;
        A0 = segment[pnt - 1].end_accel;
        V0 = segment[pnt - 1].end_vel;
        P0 = segment[pnt - 1].end_pos;
    }

    switch (Jtype) {
    case SegmentType::CONSTANT_JERK:
        calc_javp_for_segment_const_jerk(time_now - T0, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case SegmentType::POSITIVE_JERK:
        calc_javp_for_segment_incr_jerk(time_now - T0, tj, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case SegmentType::NEGATIVE_JERK:
        calc_javp_for_segment_decr_jerk(time_now - T0, tj, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    }
    Pt_out = MAX(0.0f, Pt_out);
}

// calculate the jerk, acceleration, velocity and position at time time_now when running the constant jerk time segment
void SCurve::calc_javp_for_segment_const_jerk(float time_now, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    Jt = J0;
    At = A0 + J0 * time_now;
    Vt = V0 + A0 * time_now + 0.5f * J0 * (time_now * time_now);
    Pt = P0 + V0 * time_now + 0.5f * A0 * (time_now * time_now) + (1.0f / 6.0f) * J0 * (time_now * time_now * time_now);
}

// Calculate the jerk, acceleration, velocity and position at time time_now when running the increasing jerk magnitude time segment based on a raised cosine profile
void SCurve::calc_javp_for_segment_incr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    if (!is_positive(tj)) {
        Jt = 0.0;
        At = A0;
        Vt = V0;
        Pt = P0;
        return;
    }
    const float Alpha = Jm * 0.5f;
    const float Beta = M_PI / tj;
    Jt = Alpha * (1.0f - cosf(Beta * time_now));
    At = A0 + Alpha * time_now - (Alpha / Beta) * sinf(Beta * time_now);
    Vt = V0 + A0 * time_now + (Alpha * 0.5f) * (time_now * time_now) + (Alpha / (Beta * Beta)) * cosf(Beta * time_now) - Alpha / (Beta * Beta);
    Pt = P0 + V0 * time_now + 0.5f * A0 * (time_now * time_now) + (-Alpha / (Beta * Beta)) * time_now + Alpha * (time_now * time_now * time_now) / 6.0f + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * time_now);
}

// Calculate the jerk, acceleration, velocity and position at time time_now when running the decreasing jerk magnitude time segment based on a raised cosine profile
void SCurve::calc_javp_for_segment_decr_jerk(float time_now, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    if (!is_positive(tj)) {
        Jt = 0.0;
        At = A0;
        Vt = V0;
        Pt = P0;
        return;
    }
    const float Alpha = Jm * 0.5f;
    const float Beta = M_PI / tj;
    const float AT = Alpha * tj;
    const float VT = Alpha * ((tj * tj) * 0.5f - 2.0f / (Beta * Beta));
    const float PT = Alpha * ((-1.0f / (Beta * Beta)) * tj + (1.0f / 6.0f) * (tj * tj * tj));
    Jt = Alpha * (1.0f - cosf(Beta * (time_now + tj)));
    At = (A0 - AT) + Alpha * (time_now + tj) - (Alpha / Beta) * sinf(Beta * (time_now + tj));
    Vt = (V0 - VT) + (A0 - AT) * time_now + 0.5f * Alpha * (time_now + tj) * (time_now + tj) + (Alpha / (Beta * Beta)) * cosf(Beta * (time_now + tj)) - Alpha / (Beta * Beta);
    Pt = (P0 - PT) + (V0 - VT) * time_now + 0.5f * (A0 - AT) * (time_now * time_now) + (-Alpha / (Beta * Beta)) * (time_now + tj) + (Alpha / 6.0f) * (time_now + tj) * (time_now + tj) * (time_now + tj) + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * (time_now + tj));
}

// generate the segments for a path of length L
// the path consists of 23 segments
// 1 initial segment
// 7 segments forming the acceleration S-Curve
// 7 segments forming the velocity change S-Curve
// 1 constant velocity S-Curve
// 7 segments forming the deceleration S-Curve
void SCurve::add_segments(float L)
{
    if (is_zero(L)) {
        return;
    }

    float Jm, tj, t2, t4, t6;
    calculate_path(snap_max, jerk_max, 0.0f, accel_max, vel_max, L * 0.5f, Jm, tj, t2, t4, t6);

    add_segments_jerk(num_segs, tj, Jm, t2);
    add_segment_const_jerk(num_segs, t4, 0.0f);
    add_segments_jerk(num_segs, tj, -Jm, t6);

    // remove numerical errors
    segment[SEG_ACCEL_END].end_accel = 0.0f;

    // add empty speed adjust segments
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);
    add_segment_const_jerk(num_segs, 0.0f, 0.0f);

    const float t15 = MAX(0.0f, (L - 2.0f * segment[SEG_SPEED_CHANGE_END].end_pos) / segment[SEG_SPEED_CHANGE_END].end_vel);
    add_segment_const_jerk(num_segs, t15, 0.0f);

    add_segments_jerk(num_segs, tj, -Jm, t6);
    add_segment_const_jerk(num_segs, t4, 0.0f);
    add_segments_jerk(num_segs, tj, Jm, t2);

    // remove numerical errors
    segment[SEG_DECEL_END].end_accel = 0.0f;
    segment[SEG_DECEL_END].end_vel = 0.0f;
}

// calculate the segment times for the trigonometric S-Curve path defined by:
// Sm - duration of the raised cosine jerk profile
// Jm - maximum value of the raised cosine jerk profile
// V0 - initial velocity magnitude
// Am - maximum constant acceleration
// Vm - maximum constant velocity
// L - Length of the path
// tj_out, t2_out, t4_out, t6_out are the segment durations needed to achieve the kinematic path specified by the input variables
void SCurve::calculate_path(float Sm, float Jm, float V0, float Am, float Vm, float L,float &Jm_out, float &tj_out,  float &t2_out, float &t4_out, float &t6_out)
{
    // init outputs
    Jm_out = 0.0f;
    tj_out = 0.0f;
    t2_out = 0.0f;
    t4_out = 0.0f;
    t6_out = 0.0f;

    // check for invalid arguments
    if (!is_positive(Sm) || !is_positive(Jm) || !is_positive(Am) || !is_positive(Vm) || !is_positive(L)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        ::printf("SCurve::calculate_path invalid inputs\n");
#endif
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return;
    }

    if (V0 >= Vm) {
        // no velocity change so all segments as zero length
        return;
    }

    float tj = Jm * M_PI / (2 * Sm);
    float At = MIN(MIN(Am, 
        (Vm - V0) / (2.0f * tj) ), 
        (L + 4.0f * V0 * tj) / (4.0f * sq(tj)) );
    if (fabsf(At) < Jm * tj) {
        if (is_zero(V0)) {
            // we do not have a solution for non-zero initial velocity
            tj = MIN( MIN( MIN( tj,
                powf((L * M_PI) / (8.0 * Sm), 1.0/4.0) ), 
                powf((Vm * M_PI) / (4.0 * Sm), 1.0/3.0) ), 
                safe_sqrt((Am * M_PI) / (2.0 * Sm)) );
            Jm = 2.0 * Sm * tj / M_PI;
            Am = Jm * tj;
        } else {
            // When doing speed change we use fixed tj and adjust Jm for small changes
            Am = At;
            Jm = Am / tj;
        }
        if ((Vm <= V0 + 2.0f * Am * tj) || (L <= 4.0f * V0 * tj + 4.0f * Am * sq(tj))) {
            // solution = 0 - t6 t4 t2 = 0 0 0
            t2_out = 0.0f;
            t4_out = 0.0f;
            t6_out = 0.0f;
        } else {
            // solution = 2 - t6 t4 t2 = 0 1 0
            t2_out = 0.0f;
            t4_out = MIN(-(V0 - Vm + Am * tj + (Am * Am) / Jm) / Am, MAX(((Am * Am) * (-3.0f / 2.0f) + safe_sqrt((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm), ((Am * Am) * (-3.0f / 2.0f) - safe_sqrt((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm)));
            t4_out = MAX(t4_out, 0.0);
            t6_out = 0.0f;
        }
    } else {
        if ((Vm < V0 + Am * tj + (Am * Am) / Jm) || (L < 1.0f / (Jm * Jm) * (Am * Am * Am + Am * Jm * (V0 * 2.0f + Am * tj * 2.0f)) + V0 * tj * 2.0f + Am * (tj * tj))) {
            // solution = 5 - t6 t4 t2 = 1 0 1
            Am = MIN(MIN(Am, MAX(Jm * (tj + safe_sqrt((V0 * -4.0f + Vm * 4.0f + Jm * (tj * tj)) / Jm)) * (-1.0f / 2.0f), Jm * (tj - safe_sqrt((V0 * -4.0f + Vm * 4.0f + Jm * (tj * tj)) / Jm)) * (-1.0f / 2.0f))), Jm * tj * (-2.0f / 3.0f) + ((Jm * Jm) * (tj * tj) * (1.0f / 9.0f) - Jm * V0 * (2.0f / 3.0f)) * 1.0f / powf(safe_sqrt(powf(- (Jm * Jm) * L * (1.0f / 2.0f) + (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) - Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) + (Jm * Jm) * V0 * tj, 2.0f) - powf((Jm * Jm) * (tj * tj) * (1.0f / 9.0f) - Jm * V0 * (2.0f / 3.0f), 3.0f)) + (Jm * Jm) * L * (1.0f / 2.0f) - (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) + Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) - (Jm * Jm) * V0 * tj, 1.0f / 3.0f) + powf(safe_sqrt(powf(- (Jm * Jm) * L * (1.0f / 2.0f) + (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) - Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) + (Jm * Jm) * V0 * tj, 2.0f) - powf((Jm * Jm) * (tj * tj) * (1.0f / 9.0f) - Jm * V0 * (2.0f / 3.0f), 3.0f)) + (Jm * Jm) * L * (1.0f / 2.0f) - (Jm * Jm * Jm) * (tj * tj * tj) * (8.0f / 2.7E1f) + Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0f) * (1.0f / 3.0f) - (Jm * Jm) * V0 * tj, 1.0f / 3.0f));
            t2_out = Am / Jm - tj;
            t4_out = 0.0f;
            t6_out = t2_out;
        } else {
            // solution = 7 - t6 t4 t2 = 1 1 1
            t2_out = Am / Jm - tj;
            t4_out = MIN(-(V0 - Vm + Am * tj + (Am * Am) / Jm) / Am, MAX(((Am * Am) * (-3.0f / 2.0f) + safe_sqrt((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm), ((Am * Am) * (-3.0f / 2.0f) - safe_sqrt((Am * Am * Am * Am) * (1.0f / 4.0f) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0f / 4.0f) + Am * (Jm * Jm) * L * 2.0f - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0f / 2.0f) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0f / 2.0f)) / (Am * Jm)));
            t4_out = MAX(t4_out, 0.0);
            t6_out = t2_out;
        }
    }
    tj_out = tj;
    Jm_out = Jm;

    // check outputs and reset back to zero if necessary
    if (!isfinite(Jm_out) || is_negative(Jm_out) ||
        !isfinite(tj_out) || is_negative(tj_out) ||
        !isfinite(t2_out) || is_negative(t2_out) ||
        !isfinite(t4_out) || is_negative(t4_out) ||
        !isfinite(t6_out) || is_negative(t6_out)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        ::printf("SCurve::calculate_path invalid outputs\n");
#endif
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);

#if APM_BUILD_COPTER_OR_HELI
        // @LoggerMessage: SCVE
        // @Description: Debug message for SCurve internal error
        // @Field: TimeUS: Time since system startup
        // @Field: Sm: duration of the raised cosine jerk profile
        // @Field: Jm: maximum value of the raised cosine jerk profile
        // @Field: V0: initial velocity magnitude
        // @Field: Am: maximum constant acceleration
        // @Field: Vm: maximum constant velocity
        // @Field: L: Length of the path
        // @Field: Jm_out: maximum value of the raised cosine jerk profile
        // @Field: tj_out: segment duration
        // @Field: t2_out: segment duration
        // @Field: t4_out: segment duration
        // @Field: t6_out: segment duration

#if HAL_LOGGING_ENABLED
        static bool logged_scve; // only log once
        if (!logged_scve) {
            logged_scve = true;
            AP::logger().Write(
                "SCVE",
                "TimeUS,Sm,Jm,V0,Am,Vm,L,Jm_out,tj_out,t2_out,t4_out,t6_out",
                "s-----------",
                "F-----------",
                "Qfffffffffff",
                AP_HAL::micros64(),
                (double)Sm,
                (double)Jm,
                (double)V0,
                (double)Am,
                (double)Vm,
                (double)L,
                (double)Jm_out,
                (double)tj_out,
                (double)t2_out,
                (double)t4_out,
                (double)t6_out
                );
        }
#endif  // HAL_LOGGING_ENABLED

#endif  // APM_BUILD_COPTER_OR_HELI

        Jm_out = 0.0f;
        t2_out = 0.0f;
        t4_out = 0.0f;
        t6_out = 0.0f;
    }
}

// generate three consecutive segments forming a jerk profile
// the index variable is the position within the path array that this jerk profile should be added
// the index is incremented to reference the next segment in the array after the jerk profile
void SCurve::add_segments_jerk(uint8_t &index, float tj, float Jm, float Tcj)
{
    add_segment_incr_jerk(index, tj, Jm);
    add_segment_const_jerk(index, Tcj, Jm);
    add_segment_decr_jerk(index, tj, Jm);
}

// generate constant jerk time segment
// calculate the information needed to populate the constant jerk segment from the segment duration tj and jerk J0
// the index variable is the position of this segment in the path array and is incremented to reference the next segment in the array
void SCurve::add_segment_const_jerk(uint8_t &index, float tj, float J0)
{
    // if no time increase copy previous segment
    if (!is_positive(tj)) {
        add_segment(index, segment[index - 1].end_time,
                    SegmentType::CONSTANT_JERK,
                    J0,
                    segment[index - 1].end_accel,
                    segment[index - 1].end_vel,
                    segment[index - 1].end_pos);
        return;
    }

    const float J = J0;
    const float T = segment[index - 1].end_time + tj;
    const float A = segment[index - 1].end_accel + J0 * tj;
    const float V = segment[index - 1].end_vel + segment[index - 1].end_accel * tj + 0.5f * J0 * sq(tj);
    const float P = segment[index - 1].end_pos + segment[index - 1].end_vel * tj + 0.5f * segment[index - 1].end_accel * sq(tj) + (1.0f / 6.0f) * J0 * powf(tj, 3.0f);
    add_segment(index, T, SegmentType::CONSTANT_JERK, J, A, V, P);
}

// generate increasing jerk magnitude time segment based on a raised cosine profile
// calculate the information needed to populate the increasing jerk magnitude segment from the segment duration tj and jerk magnitude Jm
// the index variable is the position of this segment in the path array and is incremented to reference the next segment in the array
void SCurve::add_segment_incr_jerk(uint8_t &index, float tj, float Jm)
{
    // if no time increase copy previous segment
    if (!is_positive(tj)) {
        add_segment(index, segment[index - 1].end_time,
                    SegmentType::CONSTANT_JERK,
                    0.0,
                    segment[index - 1].end_accel,
                    segment[index - 1].end_vel,
                    segment[index - 1].end_pos);
        return;
    }
    const float Beta = M_PI / tj;
    const float Alpha = Jm * 0.5f;
    const float AT = Alpha * tj;
    const float VT = Alpha * (sq(tj) * 0.5f - 2.0f / sq(Beta));
    const float PT = Alpha * ((-1.0f / sq(Beta)) * tj + (1.0f / 6.0f) * powf(tj, 3.0f));

    const float J = Jm;
    const float T = segment[index - 1].end_time + tj;
    const float A = segment[index - 1].end_accel + AT;
    const float V = segment[index - 1].end_vel + segment[index - 1].end_accel * tj + VT;
    const float P = segment[index - 1].end_pos + segment[index - 1].end_vel * tj + 0.5f * segment[index - 1].end_accel * sq(tj) + PT;
    add_segment(index, T, SegmentType::POSITIVE_JERK, J, A, V, P);
}

// generate decreasing jerk magnitude time segment based on a raised cosine profile
// calculate the information needed to populate the decreasing jerk magnitude segment from the segment duration tj and jerk magnitude Jm
// the index variable is the position of this segment in the path and is incremented to reference the next segment in the array
void SCurve::add_segment_decr_jerk(uint8_t &index, float tj, float Jm)
{
    // if no time increase copy previous segment
    if (!is_positive(tj)) {
        add_segment(index, segment[index - 1].end_time,
                    SegmentType::CONSTANT_JERK,
                    0.0,
                    segment[index - 1].end_accel,
                    segment[index - 1].end_vel,
                    segment[index - 1].end_pos);
        return;
    }
    const float Beta = M_PI / tj;
    const float Alpha = Jm * 0.5f;
    const float AT = Alpha * tj;
    const float VT = Alpha * (sq(tj) * 0.5f - 2.0f / sq(Beta));
    const float PT = Alpha * ((-1.0f / sq(Beta)) * tj + (1.0f / 6.0f) * powf(tj, 3.0f));
    const float A2T = Jm * tj;
    const float V2T = Jm * sq(tj);
    const float P2T = Alpha * ((-1.0f / sq(Beta)) * 2.0f * tj + (4.0f / 3.0f) * powf(tj, 3.0f));

    const float J = Jm;
    const float T = segment[index - 1].end_time + tj;
    const float A = (segment[index - 1].end_accel - AT) + A2T;
    const float V = (segment[index - 1].end_vel - VT) + (segment[index - 1].end_accel - AT) * tj + V2T;
    const float P = (segment[index - 1].end_pos - PT) + (segment[index - 1].end_vel - VT) * tj + 0.5f * (segment[index - 1].end_accel - AT) * sq(tj) + P2T;
    add_segment(index, T, SegmentType::NEGATIVE_JERK, J, A, V, P);
}

// add single S-Curve segment
// populate the information for the segment specified in the path by the index variable.
// the index variable is incremented to reference the next segment in the array
void SCurve::add_segment(uint8_t &index, float end_time, SegmentType seg_type, float jerk_ref, float end_accel, float end_vel, float end_pos)
{
    segment[index].end_time = end_time;
    segment[index].seg_type = seg_type;
    segment[index].jerk_ref = jerk_ref;
    segment[index].end_accel = end_accel;
    segment[index].end_vel = end_vel;
    segment[index].end_pos = end_pos;
    index++;
}

// set speed and acceleration limits for the path
// origin and destination are offsets from EKF origin
// speed and acceleration parameters are given in horizontal, up and down.
void SCurve::set_kinematic_limits(const Vector3p &origin, const Vector3p &destination,
                                  float speed_xy, float speed_up, float speed_down,
                                  float accel_xy, float accel_z)
{
    Vector3f direction = (destination - origin).tofloat();
    const float track_speed_max = kinematic_limit(direction, speed_xy, speed_up, speed_down);
    const float track_accel_max = kinematic_limit(direction, accel_xy, accel_z, accel_z);

    vel_max = track_speed_max;
    accel_max = track_accel_max;
    accel_z_max = accel_z;
}

// return true if the curve is valid.  Used to identify and protect against code errors
bool SCurve::valid() const
{
    // check number of segments
    if (num_segs != segments_max) {
        return false;
    }

    for (uint8_t i = 0; i < num_segs; i++) {
        // jerk_ref should be finite (i.e. not NaN or infinity)
        // time, accel, vel and pos should finite and not negative
        if (!isfinite(segment[i].jerk_ref) ||
            !isfinite(segment[i].end_time) ||
            !isfinite(segment[i].end_accel) ||
            !isfinite(segment[i].end_vel) || is_negative(segment[i].end_vel) ||
            !isfinite(segment[i].end_pos)) {
            return false;
        }

        // time and pos should be increasing
        if (i >= 1) {
            if (is_negative(segment[i].end_time - segment[i-1].end_time) ||
                is_negative(segment[i].end_pos - segment[i-1].end_pos)) {
                return false;
            }
        }
    }

    // last segment should have zero acceleration
    if (!is_zero(segment[num_segs-1].end_accel)) {
        return false;
    }

    // if we get this far then the curve must be valid
    return true;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// debugging messages
void SCurve::debug() const
{
    ::printf("num_segs:%u, time:%4.2f, snap_max:%4.2f, jerk_max:%4.2f, accel_max:%4.2f, vel_max:%4.2f\n",
                        (unsigned)num_segs, (double)time, (double)snap_max, (double)jerk_max, (double)accel_max, (double)vel_max);
    ::printf("T, Jt, J, A, V, P \n");
    for (uint8_t i = 0; i < num_segs; i++) {
        ::printf("i:%u, T:%4.2f, Jtype:%4.2f, J:%4.2f, A:%4.2f, V: %4.2f, P: %4.2f\n",
                            (unsigned)i, (double)segment[i].end_time, (double)segment[i].seg_type, (double)segment[i].jerk_ref,
                            (double)segment[i].end_accel, (double)segment[i].end_vel, (double)segment[i].end_pos);
    }
    ::printf("track x:%4.2f, y:%4.2f, z:%4.2f\n", (double)seg_delta.x, (double)seg_delta.y, (double)seg_delta.z);
    ::printf("arc_center_ne x:%4.2f, y:%4.2f\n", (double)arc.center_ne.x, (double)arc.center_ne.y);
}
#endif
