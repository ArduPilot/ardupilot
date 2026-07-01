#include "mode.h"
#include "Plane.h"

#include <AP_Follow/AP_Follow_config.h>

#define FOLLOW_TARGET_WARNING_INTERVAL_MS 5000

namespace {
constexpr float FT_TARGET_FILTER_TAU_S = 0.5f;
constexpr float FT_DIRECT_MIN_ROVER_SPEED_MS = 10.0f;
constexpr float FT_DIRECT_CAPTURE_MIN_ROVER_SPEED_MS = 12.0f;
constexpr float FT_DIRECT_MIN_AIRSPEED_MS = 18.5f;
constexpr float FT_DIRECT_AIRSPEED_RELEASE_MS = 20.0f;
constexpr float FT_DIRECT_LOOKAHEAD_MIN_M = 60.0f;
constexpr float FT_DIRECT_LOOKAHEAD_MAX_M = 200.0f;
constexpr float FT_DIRECT_MAX_TIME_S = 12.0f;
constexpr float FT_DIRECT_TURN_SPEED_MS = 24.0f;
constexpr float FT_DIRECT_TURN_HEADING_ERR_DEG = 45.0f;
constexpr float FT_DIRECT_ENTRY_DIST_M = 410.0f;
constexpr float FT_DIRECT_ENTRY_RATE_MS = 0.8f;
constexpr uint32_t FT_DIRECT_ENTRY_STABLE_MS = 2000U;
constexpr uint32_t FT_DIRECT_MIN_HOLD_MS = 8000U;
constexpr uint32_t FT_DIRECT_EXIT_STABLE_MS = 4000U;
constexpr float FT_DIRECT_EXIT_RATE_MS = 0.5f;
constexpr float FT_DIRECT_LOOKAHEAD_TURN_FACTOR = 1.0f;
constexpr float FT_DIRECT_CAPTURE_HEADING_ERR_DEG = 95.0f;
constexpr float FT_DIRECT_CAPTURE_EXIT_HEADING_ERR_DEG = 45.0f;
constexpr uint32_t FT_DIRECT_CAPTURE_MIN_HOLD_MS = 8000U;
constexpr uint32_t FT_DIRECT_CAPTURE_EXIT_STABLE_MS = 3000U;
constexpr float FT_DIRECT_CAPTURE_MIN_DIST_M = 240.0f;
constexpr float FT_DIRECT_CAPTURE_ANGLE_DEG = 60.0f;
constexpr float FT_COMMAND_BEARING_RATE_LIMIT_DEG_S = 30.0f;
constexpr float FT_PRE_DIRECT_RATE_MS = 0.8f;
constexpr float FT_STOP_RECOVERY_DISTANCE_M = 240.0f;
constexpr float FT_STOP_RECOVERY_RATE_MS = 2.0f;
constexpr float FT_STOP_RECOVERY_AIRSPEED_MS = 24.0f;
constexpr float FT_LOOKAHEAD_MIN_M = 120.0f;
constexpr float FT_LOOKAHEAD_MAX_M = 280.0f;
constexpr float FT_LOOKAHEAD_TURN_THRESHOLD_DEG_S = 12.0f;
constexpr float FT_LOOKAHEAD_MAX_TIME_S = 12.0f;
constexpr float FT_LOOKAHEAD_TURN_FACTOR = 0.6f;
constexpr float FT_ROLL_LIMIT_DEG = 24.0f;
}

bool ModeFollowTarget::_enter()
{
    follow_state = FollowState::ORBIT;
    guidance_mode = GuidanceMode::ORBIT;
    direct_strategy = DirectStrategy::DIRECT;
    active_direct_navigation = false;
    airspeed_guard_active = false;
    last_target_warning_ms = 0;
    last_update_ms = 0;
    stop_candidate_ms = 0;
    resume_candidate_ms = 0;
    last_log_ms = 0;
    active_radius_m = constrain_float(plane.g2.follow_target_radius.get(), 1.0f, 1000.0f);
    active_airspeed_m_s = constrain_float(plane.g2.follow_target_speed_cruise.get(),
                                          plane.g2.follow_target_speed_min.get(),
                                          plane.g2.follow_target_speed_max.get());
    active_command_radius_m = active_radius_m;
    command_heading_error_deg = 0.0f;
    command_bearing_rate_deg_s = 0.0f;
    command_lead_m = 0.0f;
    command_predict_s = 0.0f;
    previous_target_heading_rad = 0.0f;
    active_predict_s = 0.0f;
    have_previous_predict = false;
    plane_to_target_m = 0.0f;
    previous_plane_to_target_m = 0.0f;
    distance_rate_m_s = 0.0f;
    raw_distance_rate_m_s = 0.0f;
    command_move_rate_m_s = 0.0f;
    target_speed_m_s = 0.0f;
    have_previous_distance = false;
    have_previous_center = false;
    have_previous_command = false;
    have_previous_target_heading = false;
    stop_orbit_active = false;
    stop_orbit_recovery_active = false;
    reset_direct_state();
    set_hold_target_to_current_location();

#if AP_FOLLOW_ENABLED
    if (!plane.g2.follow.enabled()) {
        plane.gcs().send_text(MAV_SEVERITY_ERROR, "FollowTarget: set FOLL_ENABLE=1");
        return false;
    }

    const uint8_t target_sysid = (uint8_t)constrain_int16(plane.g2.follow_target_sysid.get(), 0, 255);
    plane.g2.follow.set_target_sysid(target_sysid);
    if (target_sysid > 0) {
        plane.gcs().send_text(MAV_SEVERITY_INFO, "FollowTarget: target sysid %u", (unsigned)target_sysid);
    }

    if (!update_follow_target()) {
        plane.gcs().send_text(MAV_SEVERITY_WARNING, "FollowTarget: waiting for target");
    }
    return true;
#else
    plane.gcs().send_text(MAV_SEVERITY_ERROR, "FollowTarget: AP_FOLLOW disabled");
    return false;
#endif
}

void ModeFollowTarget::_exit()
{
    plane.new_airspeed_cm = -1;
#if AP_FOLLOW_ENABLED
    plane.g2.follow.clear_offsets_if_required();
#endif
}

void ModeFollowTarget::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();
}

void ModeFollowTarget::navigate()
{
    if (!update_follow_target()) {
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_target_warning_ms > FOLLOW_TARGET_WARNING_INTERVAL_MS) {
            last_target_warning_ms = now_ms;
            plane.gcs().send_text(MAV_SEVERITY_WARNING, "FollowTarget: target lost");
        }
    }

    plane.update_loiter((uint16_t)(active_direct_navigation ? 0.0f : active_radius_m));
}

void ModeFollowTarget::update_target_altitude()
{
    plane.set_target_altitude_location(plane.next_WP_loc);
}

void ModeFollowTarget::reset_direct_state()
{
    guidance_mode = GuidanceMode::ORBIT;
    direct_strategy = DirectStrategy::DIRECT;
    direct_entry_candidate_ms = 0;
    direct_exit_candidate_ms = 0;
    direct_hold_until_ms = 0;
    direct_strategy_exit_candidate_ms = 0;
    direct_strategy_hold_until_ms = 0;
}

void ModeFollowTarget::set_hold_target_to_current_location()
{
    Location hold_loc{plane.current_loc};
    const float alt_m = constrain_float(plane.g2.follow_target_alt.get(), 1.0f, 10000.0f);
    hold_loc.set_alt_cm((int32_t)(alt_m * 100.0f), Location::AltFrame::ABOVE_HOME);

    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = hold_loc;
    plane.fix_terrain_WP(plane.next_WP_loc, __LINE__);
    if (!plane.next_WP_loc.terrain_alt) {
        plane.next_WP_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    plane.loiter.direction = (plane.g2.follow_target_direction.get() == 0) ? 1 : -1;
    plane.auto_state.crosstrack = false;
    plane.loiter.start_time_ms = 0;
    plane.auto_state.vtol_loiter = false;
    plane.loiter_angle_reset();
    plane.set_target_altitude_location(plane.next_WP_loc);
    plane.new_airspeed_cm = (int32_t)(active_airspeed_m_s * 100.0f);
}

void ModeFollowTarget::set_follow_state(FollowState new_state)
{
    if (new_state == follow_state) {
        return;
    }

    follow_state = new_state;
    plane.gcs().send_text(MAV_SEVERITY_INFO, "FollowTarget: %s", follow_state_name(follow_state));
}

const char *ModeFollowTarget::follow_state_name(FollowState state)
{
    switch (state) {
    case FollowState::ORBIT:
        return "orbit";
    case FollowState::PRE_CATCH:
        return "pre-catch";
    case FollowState::CATCH_UP:
        return "catch-up";
    case FollowState::STOP_ORBIT:
        return "stop-orbit";
    }
    return "unknown";
}

void ModeFollowTarget::update_stop_orbit_state(float target_speed_ms, uint32_t now_ms)
{
    const float stop_speed_ms = constrain_float(plane.g2.follow_target_stop_speed.get(), 0.0f, 20.0f);
    const uint32_t stop_time_ms = (uint32_t)(constrain_float(plane.g2.follow_target_stop_time.get(), 0.0f, 30.0f) * 1000.0f);
    const float resume_speed_ms = constrain_float(plane.g2.follow_target_resume_speed.get(), stop_speed_ms, 40.0f);
    const uint32_t resume_time_ms = (uint32_t)(constrain_float(plane.g2.follow_target_resume_time.get(), 0.0f, 30.0f) * 1000.0f);

    if (stop_orbit_active) {
        if (target_speed_ms >= resume_speed_ms) {
            if (resume_candidate_ms == 0) {
                resume_candidate_ms = now_ms;
            } else if (now_ms - resume_candidate_ms >= resume_time_ms) {
                stop_orbit_active = false;
                resume_candidate_ms = 0;
                stop_candidate_ms = 0;
                set_follow_state(FollowState::ORBIT);
            }
        } else {
            resume_candidate_ms = 0;
        }
        return;
    }

    if (target_speed_ms <= stop_speed_ms) {
        if (stop_candidate_ms == 0) {
            stop_candidate_ms = now_ms;
        } else if (now_ms - stop_candidate_ms >= stop_time_ms) {
            stop_orbit_active = true;
            stop_candidate_ms = 0;
            resume_candidate_ms = 0;
            set_follow_state(FollowState::STOP_ORBIT);
        }
    } else {
        stop_candidate_ms = 0;
    }
}

bool ModeFollowTarget::update_follow_target()
{
#if !AP_FOLLOW_ENABLED
    return false;
#else
    Location target_loc;
    Vector3f target_vel_ned;
    if (!plane.g2.follow.get_target_location_and_velocity(target_loc, target_vel_ned)) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const float dt_s = (last_update_ms == 0) ? 0.1f : constrain_float((now_ms - last_update_ms) * 0.001f, 0.02f, 1.0f);
    last_update_ms = now_ms;

    target_speed_m_s = safe_sqrt(sq(target_vel_ned.x) + sq(target_vel_ned.y));
    const float target_heading_rad = (target_speed_m_s > 0.2f) ? atan2f(target_vel_ned.y, target_vel_ned.x) : previous_target_heading_rad;
    plane_to_target_m = plane.current_loc.get_distance(target_loc);
    update_distance_rate(plane_to_target_m, dt_s);

    const float base_radius_m = constrain_float(plane.g2.follow_target_radius.get(), 1.0f, 1000.0f);
    const float max_dist_m = MAX(constrain_float(plane.g2.follow_target_max_dist.get(), base_radius_m + 1.0f, 5000.0f),
                                 base_radius_m + 1.0f);
    const float exit_dist_m = constrain_float(plane.g2.follow_target_catchup_exit_dist.get(),
                                             base_radius_m,
                                             max_dist_m);
    const float precatch_dist_m = select_precatch_distance(exit_dist_m, max_dist_m);

    update_stop_orbit_state(target_speed_m_s, now_ms);

    if (stop_orbit_active) {
        set_follow_state(FollowState::STOP_ORBIT);
    } else if (follow_state == FollowState::CATCH_UP) {
        if (plane_to_target_m <= exit_dist_m) {
            set_follow_state(FollowState::ORBIT);
        }
    } else if (follow_state == FollowState::PRE_CATCH) {
        if (plane_to_target_m > max_dist_m) {
            set_follow_state(FollowState::CATCH_UP);
        } else if (plane_to_target_m <= exit_dist_m) {
            set_follow_state(FollowState::ORBIT);
        }
    } else {
        const float precatch_rate_ms = constrain_float(plane.g2.follow_target_precatch_rate.get(), 0.0f, 30.0f);
        const bool precatch_by_distance = plane_to_target_m >= precatch_dist_m;
        const bool precatch_by_rate = (plane_to_target_m > base_radius_m * 1.3f) && (distance_rate_m_s >= precatch_rate_ms);
        if (plane_to_target_m > max_dist_m) {
            set_follow_state(FollowState::CATCH_UP);
        } else if (precatch_by_distance || precatch_by_rate) {
            set_follow_state(FollowState::PRE_CATCH);
        }
    }

    const float raw_predict_s = select_predict_time(plane_to_target_m, exit_dist_m, max_dist_m);
    active_predict_s = dynamic_predict_time(raw_predict_s, max_dist_m, exit_dist_m, dt_s, target_heading_rad);
    have_previous_predict = true;

    Location center_loc{target_loc};
    center_loc.offset(target_vel_ned.x * active_predict_s, target_vel_ned.y * active_predict_s);
    const float alt_m = constrain_float(plane.g2.follow_target_alt.get(), 1.0f, 10000.0f);
    center_loc.set_alt_cm((int32_t)(alt_m * 100.0f), Location::AltFrame::ABOVE_HOME);

    const float center_tau_s = FT_TARGET_FILTER_TAU_S * ((follow_state == FollowState::CATCH_UP) ? 0.5f : 1.0f);
    if (have_previous_center) {
        center_loc = smooth_center_step(previous_center_loc, center_loc, center_tau_s, dt_s);
    }
    previous_center_loc = center_loc;
    have_previous_center = true;

    stop_orbit_recovery_active = false;
    active_radius_m = select_orbit_radius(target_speed_m_s, plane_to_target_m);
    if (follow_state == FollowState::STOP_ORBIT) {
        const Vector2f gs = AP::ahrs().groundspeed_vector();
        const bool low_stop_airspeed = gs.length() < FT_DIRECT_AIRSPEED_RELEASE_MS;
        stop_orbit_recovery_active = (plane_to_target_m >= FT_STOP_RECOVERY_DISTANCE_M) ||
                                     (distance_rate_m_s >= FT_STOP_RECOVERY_RATE_MS) ||
                                     low_stop_airspeed;
        if (stop_orbit_recovery_active) {
            const float stop_radius_max_m = MAX(constrain_float(plane.g2.follow_target_stop_radius.get(), base_radius_m, 2000.0f),
                                                base_radius_m);
            active_radius_m = constrain_float(MAX(base_radius_m, MIN(plane_to_target_m, stop_radius_max_m)),
                                              base_radius_m,
                                              stop_radius_max_m);
        } else {
            active_radius_m = base_radius_m;
        }
    }

    const bool direct_requested = update_guidance_mode(now_ms, max_dist_m, exit_dist_m, precatch_dist_m);
    Location command_loc{center_loc};
    command_predict_s = active_predict_s;
    command_lead_m = target_speed_m_s * active_predict_s;
    active_direct_navigation = false;

    if (guidance_mode == GuidanceMode::DIRECT) {
        float direct_predict_s = 0.0f;
        float direct_lead_m = 0.0f;
        Location direct_loc = compute_direct_target(target_loc, target_vel_ned, direct_predict_s, direct_lead_m);
        const float raw_direct_heading_error_deg = heading_error_to_target_deg(direct_loc);

        const float direct_exit_distance_m = exit_dist_m;
        const bool lead_saturated = direct_lead_m >= FT_DIRECT_LOOKAHEAD_MAX_M * 0.95f;
        const bool tangent_available = target_speed_m_s >= FT_DIRECT_CAPTURE_MIN_ROVER_SPEED_MS &&
                                       plane_to_target_m >= FT_DIRECT_CAPTURE_MIN_DIST_M;
        const bool tangent_entry_ready = tangent_available &&
                                         ((raw_direct_heading_error_deg >= FT_DIRECT_CAPTURE_HEADING_ERR_DEG) ||
                                          (lead_saturated && raw_direct_heading_error_deg >= FT_DIRECT_TURN_HEADING_ERR_DEG));

        if (!tangent_available) {
            direct_strategy = DirectStrategy::DIRECT;
            direct_strategy_hold_until_ms = 0;
            direct_strategy_exit_candidate_ms = 0;
        } else if (tangent_entry_ready) {
            direct_strategy = DirectStrategy::TANGENT;
            direct_strategy_hold_until_ms = MAX(direct_strategy_hold_until_ms, now_ms + FT_DIRECT_CAPTURE_MIN_HOLD_MS);
            direct_strategy_exit_candidate_ms = 0;
        } else if (direct_strategy == DirectStrategy::TANGENT) {
            const bool tangent_exit_ready = (raw_direct_heading_error_deg <= FT_DIRECT_CAPTURE_EXIT_HEADING_ERR_DEG) ||
                                           (plane_to_target_m <= direct_exit_distance_m);
            if (tangent_exit_ready) {
                if (direct_strategy_exit_candidate_ms == 0) {
                    direct_strategy_exit_candidate_ms = now_ms;
                } else if (now_ms >= direct_strategy_hold_until_ms &&
                           now_ms - direct_strategy_exit_candidate_ms >= FT_DIRECT_CAPTURE_EXIT_STABLE_MS) {
                    direct_strategy = DirectStrategy::DIRECT;
                    direct_strategy_hold_until_ms = 0;
                    direct_strategy_exit_candidate_ms = 0;
                }
            } else {
                direct_strategy_exit_candidate_ms = 0;
            }
        }

        if (direct_strategy == DirectStrategy::TANGENT) {
            command_loc = compute_tangent_target(center_loc, active_radius_m);
            command_predict_s = active_predict_s;
            command_lead_m = target_speed_m_s * active_predict_s;
        } else {
            command_loc = direct_loc;
            command_predict_s = direct_predict_s;
            command_lead_m = direct_lead_m;
        }
        active_direct_navigation = true;
    } else {
        direct_strategy = DirectStrategy::DIRECT;
        direct_strategy_hold_until_ms = 0;
        direct_strategy_exit_candidate_ms = 0;
    }

    if (have_previous_command) {
        command_loc = limit_command_bearing(previous_command_loc, command_loc, dt_s);
        command_loc = limit_command_step(previous_command_loc, command_loc, dt_s);
    } else {
        command_move_rate_m_s = 0.0f;
        command_bearing_rate_deg_s = 0.0f;
    }
    previous_command_loc = command_loc;
    have_previous_command = true;

    command_heading_error_deg = heading_error_to_target_deg(command_loc);
    active_airspeed_m_s = select_airspeed(target_speed_m_s, plane_to_target_m, active_radius_m, guidance_mode);
    if (stop_orbit_recovery_active) {
        active_airspeed_m_s = MAX(active_airspeed_m_s, FT_STOP_RECOVERY_AIRSPEED_MS);
    }
    plane.new_airspeed_cm = (int32_t)(active_airspeed_m_s * 100.0f);

    set_navigation_target(command_loc, active_direct_navigation, active_radius_m);

    previous_target_heading_rad = target_heading_rad;
    have_previous_target_heading = target_speed_m_s > 0.2f;

    write_follow_log(target_loc, command_loc);
    (void)direct_requested;
    return true;
#endif
}

float ModeFollowTarget::select_precatch_distance(float exit_dist_m, float max_dist_m) const
{
    const float configured_m = plane.g2.follow_target_precatch_dist.get();
    if (configured_m > exit_dist_m && configured_m < max_dist_m) {
        return configured_m;
    }
    return exit_dist_m + (max_dist_m - exit_dist_m) * 0.33f;
}

float ModeFollowTarget::select_predict_time(float distance_m, float exit_dist_m, float max_dist_m) const
{
    const float normal_predict_s = constrain_float(plane.g2.follow_target_predict_time.get(), 0.0f, 20.0f);
    const float catch_predict_s = constrain_float(plane.g2.follow_target_catchup_predict_time.get(), normal_predict_s, 30.0f);

    switch (follow_state) {
    case FollowState::STOP_ORBIT:
        return 0.0f;
    case FollowState::CATCH_UP:
        return catch_predict_s;
    case FollowState::PRE_CATCH: {
        const float ratio = constrain_float((distance_m - exit_dist_m) / MAX(1.0f, max_dist_m - exit_dist_m),
                                            0.0f,
                                            1.0f);
        return normal_predict_s + (catch_predict_s - normal_predict_s) * ratio;
    }
    case FollowState::ORBIT:
        break;
    }
    return normal_predict_s;
}

void ModeFollowTarget::update_distance_rate(float distance_m, float dt_s)
{
    if (!have_previous_distance || dt_s <= 0.0f) {
        previous_plane_to_target_m = distance_m;
        raw_distance_rate_m_s = 0.0f;
        distance_rate_m_s = 0.0f;
        have_previous_distance = true;
        return;
    }

    raw_distance_rate_m_s = (distance_m - previous_plane_to_target_m) / dt_s;
    previous_plane_to_target_m = distance_m;
    const float tau_s = constrain_float(plane.g2.follow_target_rate_tau.get(), 0.0f, 20.0f);
    if (tau_s <= 0.0f) {
        distance_rate_m_s = raw_distance_rate_m_s;
    } else {
        const float alpha = constrain_float(dt_s / (tau_s + dt_s), 0.0f, 1.0f);
        distance_rate_m_s += (raw_distance_rate_m_s - distance_rate_m_s) * alpha;
    }
}

float ModeFollowTarget::select_orbit_radius(float target_speed_ms, float distance_m) const
{
    const float radius_min_m = constrain_float(plane.g2.follow_target_radius_min.get(), 1.0f, 1000.0f);
    const float radius_max_m = MAX(constrain_float(plane.g2.follow_target_radius_max.get(), radius_min_m, 1500.0f),
                                   radius_min_m);
    const float base_radius_m = constrain_float(plane.g2.follow_target_radius.get(), radius_min_m, radius_max_m);

    if (plane.g2.follow_target_adaptive.get() == 0) {
        return base_radius_m;
    }

    if (follow_state == FollowState::STOP_ORBIT) {
        const float stop_radius_m = MAX(constrain_float(plane.g2.follow_target_stop_radius.get(), base_radius_m, 2000.0f),
                                        base_radius_m);
        if (distance_m > base_radius_m * 1.15f) {
            return constrain_float(MAX(base_radius_m, distance_m), base_radius_m, stop_radius_m);
        }
        return base_radius_m;
    }

    const float target_speed_min_ms = constrain_float(plane.g2.follow_target_target_speed_min.get(), 0.0f, 80.0f);
    const float target_speed_max_ms = MAX(constrain_float(plane.g2.follow_target_target_speed_max.get(),
                                                          target_speed_min_ms + 0.1f,
                                                          100.0f),
                                          target_speed_min_ms + 0.1f);
    const float speed_ratio = constrain_float((target_speed_ms - target_speed_min_ms) /
                                              (target_speed_max_ms - target_speed_min_ms),
                                              0.0f,
                                              1.0f);
    const float distance_bias = (follow_state == FollowState::CATCH_UP) ? 1.0f :
                                constrain_float((distance_m - base_radius_m) / MAX(base_radius_m, 1.0f),
                                                0.0f,
                                                1.0f);
    const float state_bias = (follow_state == FollowState::PRE_CATCH) ? 0.65f : 0.35f;
    const float blend = MAX(speed_ratio, state_bias * distance_bias);
    return constrain_float(base_radius_m + ((radius_max_m - base_radius_m) * blend),
                           radius_min_m,
                           radius_max_m);
}

float ModeFollowTarget::select_airspeed(float target_speed_ms, float distance_m, float radius_m, GuidanceMode guidance) const
{
    const float speed_min_ms = constrain_float(plane.g2.follow_target_speed_min.get(), 1.0f, 100.0f);
    const float speed_max_ms = MAX(constrain_float(plane.g2.follow_target_speed_max.get(),
                                                   speed_min_ms + 0.1f,
                                                   120.0f),
                                   speed_min_ms + 0.1f);
    const float speed_cruise_ms = constrain_float(plane.g2.follow_target_speed_cruise.get(),
                                                  speed_min_ms,
                                                  speed_max_ms);

    if (plane.g2.follow_target_adaptive.get() == 0) {
        return speed_cruise_ms;
    }

    const float max_dist_m = MAX(constrain_float(plane.g2.follow_target_max_dist.get(), radius_m + 1.0f, 5000.0f),
                                 radius_m + 1.0f);
    const float distance_ratio = constrain_float((distance_m - radius_m) / (max_dist_m - radius_m),
                                                 0.0f,
                                                 1.0f);
    float speed_margin_ms = 7.0f;
    float extra_fraction = 0.20f;
    if (follow_state == FollowState::CATCH_UP) {
        speed_margin_ms = 10.0f;
        extra_fraction = 0.35f;
    } else if (follow_state == FollowState::PRE_CATCH) {
        speed_margin_ms = 8.0f;
        extra_fraction = 0.25f;
    } else if (follow_state == FollowState::STOP_ORBIT) {
        speed_margin_ms = 4.0f;
        extra_fraction = 0.15f;
    }
    const float base_speed_ms = MAX(speed_cruise_ms, target_speed_ms + speed_margin_ms);
    float desired_speed_ms = base_speed_ms + ((speed_max_ms - base_speed_ms) * extra_fraction * distance_ratio);

    if (guidance == GuidanceMode::DIRECT && command_heading_error_deg >= FT_DIRECT_TURN_HEADING_ERR_DEG) {
        desired_speed_ms = MIN(desired_speed_ms, FT_DIRECT_TURN_SPEED_MS);
    }

    return constrain_float(desired_speed_ms, speed_min_ms, speed_max_ms);
}

float ModeFollowTarget::estimate_half_turn_time_s(float speed_m_s) const
{
    if (speed_m_s <= 1.0f) {
        return 0.0f;
    }
    const float roll_rad = radians(constrain_float(FT_ROLL_LIMIT_DEG, 5.0f, 80.0f));
    const float radius_m = sq(speed_m_s) / MAX(0.1f, GRAVITY_MSS * tanf(roll_rad));
    return (M_PI * radius_m) / speed_m_s;
}

float ModeFollowTarget::dynamic_predict_time(float raw_predict_s, float max_dist_m, float exit_dist_m, float dt_s, float target_heading_rad)
{
    if (target_speed_m_s < 0.2f) {
        active_predict_s = raw_predict_s;
        return raw_predict_s;
    }

    const float rover_max_speed_m_s = MAX(constrain_float(plane.g2.follow_target_target_speed_max.get(), 0.1f, 100.0f), 0.1f);
    const float base_lead_m = target_speed_m_s * raw_predict_s;
    const float high_speed_ratio = constrain_float((target_speed_m_s - 12.0f) / MAX(0.1f, rover_max_speed_m_s - 12.0f), 0.0f, 1.0f);
    const float distance_ratio = constrain_float((plane_to_target_m - exit_dist_m) / MAX(1.0f, max_dist_m - exit_dist_m), 0.0f, 1.0f);
    const float rate_ratio = constrain_float(distance_rate_m_s / 3.0f, 0.0f, 1.0f);
    const float dynamic_ratio = constrain_float(high_speed_ratio * 0.65f + distance_ratio * 0.25f + rate_ratio * 0.10f, 0.0f, 1.0f);
    float strength = dynamic_ratio * 0.60f;
    if (follow_state == FollowState::CATCH_UP) {
        strength = 1.0f;
    } else if (follow_state == FollowState::PRE_CATCH) {
        strength = MAX(dynamic_ratio, 0.75f);
    }

    float target_lead_m = FT_LOOKAHEAD_MIN_M + (FT_LOOKAHEAD_MAX_M - FT_LOOKAHEAD_MIN_M) * dynamic_ratio;
    const float turn_time_s = estimate_half_turn_time_s(FT_DIRECT_TURN_SPEED_MS);
    if (turn_time_s > 0.0f) {
        target_lead_m = MAX(target_lead_m, target_speed_m_s * turn_time_s * FT_LOOKAHEAD_TURN_FACTOR);
    }
    float desired_lead_m = base_lead_m + (MAX(base_lead_m, target_lead_m) - base_lead_m) * strength;

    if (have_previous_target_heading && dt_s > 0.0f) {
        const float turn_rate_deg_s = fabsf(degrees(wrap_PI(target_heading_rad - previous_target_heading_rad))) / dt_s;
        const float turn_ratio = constrain_float(turn_rate_deg_s / FT_LOOKAHEAD_TURN_THRESHOLD_DEG_S, 0.0f, 1.0f);
        float turn_cut = 0.7f * turn_ratio;
        if (follow_state == FollowState::PRE_CATCH || follow_state == FollowState::CATCH_UP || distance_rate_m_s > 0.0f) {
            turn_cut *= 0.25f;
        }
        desired_lead_m = base_lead_m + (desired_lead_m - base_lead_m) * (1.0f - turn_cut);
    }

    desired_lead_m = constrain_float(desired_lead_m, base_lead_m, FT_LOOKAHEAD_MAX_M);
    float selected_s = constrain_float(desired_lead_m / target_speed_m_s, raw_predict_s, FT_LOOKAHEAD_MAX_TIME_S);
    if (have_previous_predict && FT_TARGET_FILTER_TAU_S > 0.0f && dt_s > 0.0f) {
        const float alpha = 1.0f - expf(-dt_s / FT_TARGET_FILTER_TAU_S);
        selected_s = active_predict_s + alpha * (selected_s - active_predict_s);
    }
    return selected_s;
}

Location ModeFollowTarget::smooth_center_step(const Location &previous, const Location &target, float tau_s, float dt_s) const
{
    if (tau_s <= 0.0f || dt_s <= 0.0f) {
        return target;
    }
    const float alpha = 1.0f - expf(-dt_s / tau_s);
    const Vector2f delta_ne = previous.get_distance_NE(target);
    Location smoothed{previous};
    smoothed.offset(delta_ne.x * alpha, delta_ne.y * alpha);
    smoothed.set_alt_cm(target.alt, target.get_alt_frame());
    return smoothed;
}

Location ModeFollowTarget::limit_command_step(const Location &previous, const Location &target, float dt_s)
{
    const float max_rate_ms = constrain_float(plane.g2.follow_target_command_rate.get(), 0.0f, 200.0f);
    if (max_rate_ms <= 0.0f || dt_s <= 0.0f) {
        command_move_rate_m_s = 0.0f;
        return target;
    }

    const float distance_m = previous.get_distance(target);
    command_move_rate_m_s = distance_m / dt_s;
    const float max_step_m = max_rate_ms * dt_s;
    if (distance_m <= max_step_m || distance_m <= 0.01f) {
        return target;
    }

    Location limited{previous};
    limited.offset_bearing(degrees(previous.get_bearing(target)), max_step_m);
    limited.set_alt_cm(target.alt, target.get_alt_frame());
    command_move_rate_m_s = max_rate_ms;
    return limited;
}


Location ModeFollowTarget::limit_command_bearing(const Location &previous, const Location &target, float dt_s)
{
    if (FT_COMMAND_BEARING_RATE_LIMIT_DEG_S <= 0.0f || dt_s <= 0.0f) {
        command_bearing_rate_deg_s = 0.0f;
        return target;
    }

    const Vector2f prev_ne = plane.current_loc.get_distance_NE(previous);
    const Vector2f target_ne = plane.current_loc.get_distance_NE(target);
    const float prev_dist_m = prev_ne.length();
    const float target_dist_m = target_ne.length();
    if (prev_dist_m <= 1.0f || target_dist_m <= 1.0f) {
        command_bearing_rate_deg_s = 0.0f;
        return target;
    }

    const float prev_bearing_rad = atan2f(prev_ne.y, prev_ne.x);
    const float target_bearing_rad = atan2f(target_ne.y, target_ne.x);
    const float delta_rad = wrap_PI(target_bearing_rad - prev_bearing_rad);
    command_bearing_rate_deg_s = fabsf(degrees(delta_rad)) / dt_s;
    const float max_delta_rad = radians(FT_COMMAND_BEARING_RATE_LIMIT_DEG_S * dt_s);
    if (fabsf(delta_rad) <= max_delta_rad) {
        return target;
    }

    const float limited_bearing_rad = prev_bearing_rad + ((delta_rad > 0.0f) ? max_delta_rad : -max_delta_rad);
    Location limited{plane.current_loc};
    limited.offset(cosf(limited_bearing_rad) * target_dist_m, sinf(limited_bearing_rad) * target_dist_m);
    limited.set_alt_cm(target.alt, target.get_alt_frame());
    command_bearing_rate_deg_s = FT_COMMAND_BEARING_RATE_LIMIT_DEG_S;
    return limited;
}

float ModeFollowTarget::heading_error_to_target_deg(const Location &target) const
{
    const float bearing_rad = plane.current_loc.get_bearing(target);
    return fabsf(degrees(wrap_PI(bearing_rad - AP::ahrs().get_yaw())));
}

bool ModeFollowTarget::solve_intercept_time_s(const Vector2f &relative_ne, const Vector2f &target_vel_ne, float intercept_speed_m_s, float &time_s) const
{
    const float a = target_vel_ne.length_squared() - sq(intercept_speed_m_s);
    const float b = 2.0f * (relative_ne.x * target_vel_ne.x + relative_ne.y * target_vel_ne.y);
    const float c = relative_ne.length_squared();

    if (fabsf(a) < 1.0e-3f) {
        if (fabsf(b) < 1.0e-3f) {
            return false;
        }
        const float t = -c / b;
        if (t > 0.0f) {
            time_s = t;
            return true;
        }
        return false;
    }

    const float discriminant = sq(b) - 4.0f * a * c;
    if (discriminant < 0.0f) {
        return false;
    }
    const float root = sqrtf(discriminant);
    const float t1 = (-b - root) / (2.0f * a);
    const float t2 = (-b + root) / (2.0f * a);
    float best = 0.0f;
    if (t1 > 0.0f) {
        best = t1;
    }
    if (t2 > 0.0f && (best <= 0.0f || t2 < best)) {
        best = t2;
    }
    if (best <= 0.0f) {
        return false;
    }
    time_s = best;
    return true;
}

Location ModeFollowTarget::compute_direct_target(const Location &target_loc, const Vector3f &target_vel_ned, float &predict_s, float &lead_m) const
{
    Location direct{target_loc};
    if (target_speed_m_s < 0.2f) {
        predict_s = 0.0f;
        lead_m = 0.0f;
        return direct;
    }

    const Vector2f relative_ne = plane.current_loc.get_distance_NE(target_loc);
    const Vector2f target_vel_ne{target_vel_ned.x, target_vel_ned.y};
    const float intercept_speed_m_s = MAX(constrain_float(plane.g2.follow_target_speed_max.get(), FT_DIRECT_TURN_SPEED_MS, 120.0f),
                                          FT_DIRECT_TURN_SPEED_MS);
    float intercept_time_s = 0.0f;
    if (!solve_intercept_time_s(relative_ne, target_vel_ne, intercept_speed_m_s, intercept_time_s)) {
        const float closing_speed_m_s = MAX(1.0f, intercept_speed_m_s - target_speed_m_s);
        intercept_time_s = relative_ne.length() / closing_speed_m_s;
    }

    float max_lead_m = FT_DIRECT_LOOKAHEAD_MAX_M;
    const float turn_time_s = estimate_half_turn_time_s(FT_DIRECT_TURN_SPEED_MS);
    if (turn_time_s > 0.0f) {
        max_lead_m = MAX(max_lead_m, target_speed_m_s * turn_time_s * FT_DIRECT_LOOKAHEAD_TURN_FACTOR);
    }
    const float max_time_by_lead_s = max_lead_m / MAX(0.2f, target_speed_m_s);
    const float min_time_by_lead_s = FT_DIRECT_LOOKAHEAD_MIN_M / MAX(0.2f, target_speed_m_s);
    predict_s = constrain_float(intercept_time_s, 0.0f, MIN(FT_DIRECT_MAX_TIME_S, max_time_by_lead_s));
    predict_s = MAX(predict_s, min_time_by_lead_s);
    lead_m = constrain_float(target_speed_m_s * predict_s, FT_DIRECT_LOOKAHEAD_MIN_M, max_lead_m);
    predict_s = lead_m / MAX(0.2f, target_speed_m_s);

    direct.offset(target_vel_ned.x * predict_s, target_vel_ned.y * predict_s);
    return direct;
}

Location ModeFollowTarget::compute_tangent_target(const Location &center_loc, float radius_m) const
{
    const Vector2f plane_from_center = center_loc.get_distance_NE(plane.current_loc);
    const float distance_to_center_m = plane_from_center.length();
    radius_m = MAX(radius_m, 1.0f);
    const float theta_plane = atan2f(plane_from_center.y, plane_from_center.x);
    const float orbit_sign = (plane.g2.follow_target_direction.get() == 0) ? 1.0f : -1.0f;
    float candidate_theta_1;
    float candidate_theta_2;

    if (distance_to_center_m > radius_m + 1.0f) {
        const float alpha = acosf(constrain_float(radius_m / distance_to_center_m, -1.0f, 1.0f));
        candidate_theta_1 = theta_plane + alpha;
        candidate_theta_2 = theta_plane - alpha;
    } else {
        const float capture_angle_rad = radians(constrain_float(FT_DIRECT_CAPTURE_ANGLE_DEG, 5.0f, 175.0f));
        candidate_theta_1 = theta_plane + orbit_sign * capture_angle_rad;
        candidate_theta_2 = theta_plane - orbit_sign * capture_angle_rad;
    }

    const float heading_rad = AP::ahrs().get_yaw();
    const Vector2f heading_vec{cosf(heading_rad), sinf(heading_rad)};
    float best_score = -FLT_MAX;
    Vector2f best_vec{0.0f, 0.0f};
    const float candidates[2] = {candidate_theta_1, candidate_theta_2};

    for (uint8_t i = 0; i < 2; i++) {
        const float theta = candidates[i];
        const Vector2f target_vec{cosf(theta) * radius_m, sinf(theta) * radius_m};
        Vector2f line_dir = target_vec - plane_from_center;
        const float line_len = line_dir.length();
        if (line_len <= 1.0e-6f) {
            continue;
        }
        line_dir /= line_len;
        const Vector2f orbit_tangent = (plane.g2.follow_target_direction.get() == 0) ?
                                       Vector2f{-sinf(theta), cosf(theta)} :
                                       Vector2f{sinf(theta), -cosf(theta)};
        const float score = line_dir * orbit_tangent + 2.0f * (line_dir * heading_vec);
        if (score > best_score) {
            best_score = score;
            best_vec = target_vec;
        }
    }

    Location tangent{center_loc};
    tangent.offset(best_vec.x, best_vec.y);
    return tangent;
}

bool ModeFollowTarget::update_guidance_mode(uint32_t now_ms, float max_dist_m, float exit_dist_m, float precatch_dist_m)
{
    const Vector2f gs = AP::ahrs().groundspeed_vector();
    const float airspeed_proxy_m_s = gs.length();
    if (!airspeed_guard_active && airspeed_proxy_m_s < FT_DIRECT_MIN_AIRSPEED_MS) {
        airspeed_guard_active = true;
    } else if (airspeed_guard_active && airspeed_proxy_m_s >= FT_DIRECT_AIRSPEED_RELEASE_MS) {
        airspeed_guard_active = false;
    }

    const bool direct_allowed = !stop_orbit_active &&
                                !airspeed_guard_active &&
                                target_speed_m_s >= FT_DIRECT_MIN_ROVER_SPEED_MS;
    const float direct_entry_distance_m = MAX(FT_DIRECT_ENTRY_DIST_M, max_dist_m);
    const float direct_hard_entry_distance_m = direct_entry_distance_m + MAX(60.0f, max_dist_m - exit_dist_m);
    const float direct_exit_distance_m = exit_dist_m;
    const float high_precatch_distance_m = precatch_dist_m + 0.75f * MAX(0.0f, max_dist_m - precatch_dist_m);
    const bool direct_precatch = follow_state == FollowState::PRE_CATCH &&
                                 plane_to_target_m >= high_precatch_distance_m &&
                                 distance_rate_m_s >= FT_PRE_DIRECT_RATE_MS;
    const bool catchup_direct_candidate = follow_state == FollowState::CATCH_UP &&
                                          plane_to_target_m >= direct_entry_distance_m &&
                                          distance_rate_m_s >= FT_DIRECT_ENTRY_RATE_MS;
    const bool hard_direct_candidate = follow_state == FollowState::CATCH_UP &&
                                       plane_to_target_m >= direct_hard_entry_distance_m;
    const bool direct_candidate = direct_allowed && (catchup_direct_candidate || hard_direct_candidate || direct_precatch);

    bool direct_requested = false;
    if (direct_candidate) {
        if (guidance_mode == GuidanceMode::DIRECT) {
            direct_requested = true;
            direct_entry_candidate_ms = 0;
        } else if (direct_entry_candidate_ms == 0) {
            direct_entry_candidate_ms = now_ms;
        } else if (now_ms - direct_entry_candidate_ms >= FT_DIRECT_ENTRY_STABLE_MS) {
            direct_requested = true;
        }
    } else {
        direct_entry_candidate_ms = 0;
    }

    if (direct_requested) {
        guidance_mode = GuidanceMode::DIRECT;
        direct_hold_until_ms = MAX(direct_hold_until_ms, now_ms + FT_DIRECT_MIN_HOLD_MS);
        direct_exit_candidate_ms = 0;
    } else if (guidance_mode == GuidanceMode::DIRECT && direct_allowed) {
        const bool direct_exit_ready = plane_to_target_m <= direct_exit_distance_m &&
                                       distance_rate_m_s <= FT_DIRECT_EXIT_RATE_MS;
        if (direct_exit_ready) {
            if (direct_exit_candidate_ms == 0) {
                direct_exit_candidate_ms = now_ms;
            } else if (now_ms >= direct_hold_until_ms &&
                       now_ms - direct_exit_candidate_ms >= FT_DIRECT_EXIT_STABLE_MS) {
                reset_direct_state();
            }
        } else {
            direct_exit_candidate_ms = 0;
        }
    } else if (guidance_mode == GuidanceMode::DIRECT) {
        reset_direct_state();
    }

    return direct_requested;
}

void ModeFollowTarget::set_navigation_target(const Location &target, bool direct_navigation, float radius_m)
{
    Location nav_target{target};
    plane.set_guided_WP(nav_target);
    plane.set_target_altitude_location(plane.next_WP_loc);
    plane.loiter.direction = (plane.g2.follow_target_direction.get() == 0) ? 1 : -1;
    plane.auto_state.crosstrack = false;
    active_direct_navigation = direct_navigation;
    active_command_radius_m = direct_navigation ? 0.0f : radius_m;
}


void ModeFollowTarget::write_follow_log(const Location &target_loc, const Location &center_loc)
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_log_ms < 200U) {
        return;
    }
    last_log_ms = now_ms;

    plane.logger.Write(
        "FOLT",
        "TimeUS,S,Stp,Dst,Rt,Raw,TSp,Rad,Prd,Cmd,TLa,TLn,CLa,CLn,Asp",
        "QBBfffffffLLLLf",
        AP_HAL::micros64(),
        (uint8_t)follow_state,
        (uint8_t)(stop_orbit_active ? 1U : 0U),
        plane_to_target_m,
        distance_rate_m_s,
        raw_distance_rate_m_s,
        target_speed_m_s,
        active_radius_m,
        active_predict_s,
        command_move_rate_m_s,
        target_loc.lat,
        target_loc.lng,
        center_loc.lat,
        center_loc.lng,
        active_airspeed_m_s);
#endif
}
