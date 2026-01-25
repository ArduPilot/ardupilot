/*
   FlightDock Formation Controller Implementation

   This implements the proven Phase 7A control logic in C++ for 400 Hz operation.
   Key features:
   - Graduated speed control (range/10 rule)
   - Predictive waypoint placement (0.5s ahead)
   - Smooth phase transitions (INTERCEPT -> TRANSITION -> FORMATION)

   SPEED REFERENCE (2025-12-08):
   All speeds are TRUE AIRSPEED (TAS) - FlightDock uses TAS exclusively.
   LEAD_SPEED = 24.7 m/s TAS is the DHC-2 Beaver's actual speed through the air mass.

   Refactored: 2026-01-24 - Use formation_config.h constants and state structs
*/

#include "formation_controller.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

// Constructor - all state structs use default member initializers
FormationController::FormationController() :
    _active(false),
    _last_update_ms(0),
    _range_to_lead(0.0f),
    _closure_rate(0.0f),
    _last_range(0.0f),
    _last_range_ms(0),
    _desired_airspeed(FC::SpeedControl::LEAD_SPEED_DEFAULT_MPS),
    _roll_command(0.0f),
    _pitch_command(0.0f),
    _last_cross_track_error(0.0f),
    _desired_offset_body(10.0f, 0.0f, 10.0f),
    _predict_lead_s(FC::Geometry::PREDICT_TIME_S),
    _max_bank_rad(radians(FC::BankLimits::MAX_BANK_NORMAL_DEG)),
    _max_pitch_rad(radians(FC::AltitudeControl::MAX_PITCH_DEG))
{
    // State structs use default member initializers
    // Initialize PD gains to mid-range values
    _pd.kp_cross = radians(FC::PDGains::K_CROSS_MID_DEG_PER_M);
    _pd.kd_cross = radians(FC::PDGains::KD_CROSS_DEG_PER_M_S);
}

// Initialize controller
void FormationController::init() {
    // Emit FORM_E_BUILD beacon to prove C++ binary version
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
        "[FORM_E_BUILD] ver=%s features=CPP_STATION_KEEP,OPTION_E,TURN_FOLLOW",
        FORM_E_BUILD_VERSION);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
        "FormationController: init OK - ready for FORM_ENABLE");
    _active = false;
    _lead.valid = false;
}

// Set lead aircraft state from MAVLink GLOBAL_POSITION_INT message
void FormationController::set_lead_state(float lat, float lon, float alt,
                                         float vn, float ve, float vd) {
    _lead.lat = lat;
    _lead.lon = lon;
    _lead.alt = alt;
    _lead.vn = vn;
    _lead.ve = ve;
    _lead.vd = vd;
    _lead.valid = true;
    _lead.last_update_ms = AP_HAL::millis();
}

// Main 400 Hz update function
void FormationController::update(const Location &current_loc,
                                 const Vector3f &current_vel,
                                 float current_heading) {
    uint32_t now_ms = AP_HAL::millis();

    // First-call diagnostic log
    static bool first_call = true;
    if (first_call) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FormationController: first update() - C++ ACTIVE");
        first_call = false;
        _build_beacon.first_update_ms = now_ms;
    }

    // =========================================================================
    // LATE-JOIN SAFE BUILD BEACON (Phase 7D.52)
    // Resend FORM_E_BUILD every 2s for the first 20s after first update().
    // =========================================================================
    if (!_build_beacon.sent && _build_beacon.first_update_ms > 0) {
        uint32_t elapsed_ms = now_ms - _build_beacon.first_update_ms;
        if (elapsed_ms < FC::Timing::BUILD_BEACON_WINDOW_MS) {
            if (now_ms - _build_beacon.last_emit_ms >= FC::Timing::BUILD_BEACON_INTERVAL_MS) {
                _build_beacon.last_emit_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "[FORM_E_BUILD] ver=%s features=CPP_STATION_KEEP,OPTION_E,TURN_FOLLOW",
                    FORM_E_BUILD_VERSION);
            }
        } else {
            _build_beacon.sent = true;
        }
    }

    if (!_active || !_lead.valid) {
        return;
    }

    float dt = (now_ms - _last_update_ms) * 0.001f;

    // Check for lead aircraft timeout (2 seconds)
    if (now_ms - _lead.last_update_ms > 2000) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FormationController: Lead timeout");
        _lead.valid = false;
        return;
    }

    // Lead-telemetry freshness gating (500ms threshold)
    uint32_t lead_age_ms = now_ms - _lead.last_update_ms;
    if (lead_age_ms > FC::SensorFusion::LEAD_STALE_THRESH_MS) {
        _roll_command = 0.0f;
        _pitch_command = 0.0f;

        static uint32_t last_stale_warn_ms = 0;
        if (now_ms - last_stale_warn_ms >= 2000) {
            last_stale_warn_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                "[FORM_E] WARN: lead_age=%lums > %lums - wings-level fallback",
                (unsigned long)lead_age_ms, (unsigned long)FC::SensorFusion::LEAD_STALE_THRESH_MS);
        }
        _last_update_ms = now_ms;
        return;
    }

    // Calculate formation waypoint (with prediction)
    _formation_waypoint = calculate_formation_waypoint();

    // Calculate range to lead aircraft
    Location lead_loc(
        static_cast<int32_t>(_lead.lat * 1e7),
        static_cast<int32_t>(_lead.lon * 1e7),
        static_cast<int32_t>(_lead.alt * 100),
        Location::AltFrame::ABSOLUTE
    );

    float gps_range = calculate_range(current_loc, lead_loc);
    update_fused_range(gps_range);
    _range_to_lead = _sensor_fusion.fused_range;
    _closure_rate = calculate_closure_rate(_range_to_lead, dt);
    _desired_airspeed = calculate_desired_speed(_range_to_lead);

    // ============================================================================
    // STATION-KEEPING PD CONTROLLER
    // ============================================================================
    if (!_active) {
        return;
    }

    // Build Location object for leader
    Location loc_leader;
    loc_leader.lat = static_cast<int32_t>(_lead.lat * 1e7);
    loc_leader.lng = static_cast<int32_t>(_lead.lon * 1e7);
    int32_t alt_cm = static_cast<int32_t>(_lead.alt * 100);
    loc_leader.set_alt_cm(alt_cm, Location::AltFrame::ABSOLUTE);

    Location loc_follower = current_loc;
    Vector3f leader_pos_ned = loc_follower.get_distance_NED(loc_leader);

    // Estimate leader velocity from consecutive positions
    float dt_vel = (now_ms - _lead.vel_last_update_ms) * 0.001f;
    if (dt_vel > 0.0f && dt_vel < 1.0f && _lead.vel_last_update_ms > 0) {
        _lead.vel_estimated = (leader_pos_ned - _lead.pos_prev_ned) / dt_vel;
    }
    _lead.pos_prev_ned = leader_pos_ned;
    _lead.vel_last_update_ms = now_ms;

    // Predict leader position ahead
    Vector3f leader_pos_pred_ned = leader_pos_ned + _lead.vel_estimated * _predict_lead_s;

    // Compute body-frame error
    float leader_yaw = atan2f(_lead.ve, _lead.vn);
    float cos_yaw = cosf(leader_yaw);
    float sin_yaw = sinf(leader_yaw);

    Vector3f rel_ned = leader_pos_pred_ned;
    Vector3f rel_body;
    rel_body.x =  cos_yaw * rel_ned.x + sin_yaw * rel_ned.y;
    rel_body.y = -sin_yaw * rel_ned.x + cos_yaw * rel_ned.y;
    rel_body.z = rel_ned.z;

    // Sanity guard for corrupt geometry
    bool bad_geom = false;
    if (!isfinite(rel_body.x) || !isfinite(rel_body.y) || !isfinite(rel_body.z)) {
        bad_geom = true;
    } else if (fabsf(rel_body.x) > FC::Geometry::BAD_GEOM_THRESH_M ||
               fabsf(rel_body.y) > FC::Geometry::BAD_GEOM_THRESH_M ||
               fabsf(rel_body.z) > FC::Geometry::BAD_GEOM_THRESH_M) {
        bad_geom = true;
    }

    if (bad_geom) {
        _roll_command = 0.0f;
        _pitch_command = 0.0f;
        _last_cross_track_error = 0.0f;

        static uint32_t last_bad_geom_ms = 0;
        if (now_ms - last_bad_geom_ms >= 2000) {
            last_bad_geom_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                "[FORM_E] BAD_GEOM rel=(%.0f,%.0f,%.0f) - wings-level",
                (double)rel_body.x, (double)rel_body.y, (double)rel_body.z);
        }
        _last_update_ms = now_ms;
        return;
    }

    // =========================================================================
    // OUT_OF_ENGAGE SAFETY GUARD
    // =========================================================================
    float behind_m = rel_body.x;
    _direct_throttle.behind_m = behind_m;

    // Compute closure rate from behind_m changes
    static uint32_t last_behind_change_ms = 0;
    static float last_behind_for_rate = 0.0f;

    float behind_delta = fabsf(behind_m - last_behind_for_rate);
    if (behind_delta > 0.05f && last_behind_change_ms > 0) {
        float rate_dt = (now_ms - last_behind_change_ms) * 0.001f;
        if (rate_dt > 0.02f && rate_dt < 1.0f) {
            float behind_change = last_behind_for_rate - behind_m;
            float raw_closure = behind_change / rate_dt;
            if (fabsf(raw_closure) < 20.0f) {
                _direct_throttle.behind_closure_rate = 0.3f * raw_closure + 0.7f * _direct_throttle.behind_closure_rate;
            }
        }
        last_behind_for_rate = behind_m;
        last_behind_change_ms = now_ms;
    } else if (last_behind_change_ms == 0) {
        last_behind_for_rate = behind_m;
        last_behind_change_ms = now_ms;
    }
    _direct_throttle.prev_behind_m = behind_m;

    bool geometry_bad = (_range_to_lead > FC::OutOfEngage::RANGE_M) ||
                        (behind_m < FC::OutOfEngage::BEHIND_MIN_M);
    bool geometry_recovered = (_range_to_lead < (FC::OutOfEngage::RANGE_M - FC::OutOfEngage::HYSTERESIS_M)) &&
                              (behind_m > FC::OutOfEngage::BEHIND_MIN_M);

    if (_out_of_engage.latched) {
        if (geometry_recovered) {
            _out_of_engage.latched = false;
            _out_of_engage.accum_ms = 0;
        }
    } else {
        if (geometry_bad) {
            uint32_t dt_ms = 0;
            if (_out_of_engage.last_check_ms > 0) {
                dt_ms = now_ms - _out_of_engage.last_check_ms;
                if (dt_ms > 100) dt_ms = 100;
            }
            _out_of_engage.last_check_ms = now_ms;
            _out_of_engage.accum_ms += dt_ms;

            if (_out_of_engage.accum_ms >= FC::OutOfEngage::PERSIST_MS) {
                _out_of_engage.latched = true;
            }
        } else {
            _out_of_engage.accum_ms = 0;
            _out_of_engage.last_check_ms = now_ms;
        }
    }

    if (_out_of_engage.latched) {
        _roll_command = 0.0f;
        _pitch_command = 0.0f;
        _last_cross_track_error = 0.0f;

        if (now_ms - _out_of_engage.last_log_ms >= FC::OutOfEngage::LOG_INTERVAL_MS) {
            _out_of_engage.last_log_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                "[FORM_E] OUT_OF_ENG rng=%.0f beh=%.0f",
                (double)_range_to_lead, (double)behind_m);
        }
        _last_update_ms = now_ms;
        return;
    }

    // =========================================================================
    // PD CONTROL LAW WITH RANGE-DEPENDENT GAINS
    // =========================================================================
    float k_cross_deg_per_m;
    if (_range_to_lead > 400.0f) {
        k_cross_deg_per_m = FC::PDGains::K_CROSS_FAR_DEG_PER_M;
    } else if (_range_to_lead > 200.0f) {
        k_cross_deg_per_m = FC::PDGains::K_CROSS_MID_DEG_PER_M;
    } else {
        k_cross_deg_per_m = FC::PDGains::K_CROSS_NEAR_DEG_PER_M;
    }
    _pd.kp_cross = radians(k_cross_deg_per_m);
    _pd.kd_cross = radians(FC::PDGains::KD_CROSS_DEG_PER_M_S);

    // Estimate leader yaw rate for turn feed-forward
    float dt_yaw = (now_ms - _lead.yaw_last_update_ms) * 0.001f;
    if (dt_yaw > 0.0f && dt_yaw < 1.0f && _lead.yaw_last_update_ms > 0) {
        float dyaw = leader_yaw - _lead.yaw_prev_rad;
        if (dyaw > M_PI) dyaw -= 2.0f * M_PI;
        if (dyaw < -M_PI) dyaw += 2.0f * M_PI;
        _lead.yaw_rate_rad_s = dyaw / dt_yaw;
    }
    _lead.yaw_prev_rad = leader_yaw;
    _lead.yaw_last_update_ms = now_ms;

    // Cross-track computation
    float cross_m = -rel_body.y;

    // Cross-track rate from velocity projection
    float cross_rate_raw = 0.0f;
    _pd.spike_this_cycle = false;

    float vrel_n = current_vel.x - _lead.vn;
    float vrel_e = current_vel.y - _lead.ve;

    _direct_throttle.descent_rate = current_vel.z;

    float right_n = -sin_yaw;
    float right_e = cos_yaw;
    cross_rate_raw = vrel_n * right_n + vrel_e * right_e;

    if (!isfinite(cross_rate_raw)) {
        cross_rate_raw = 0.0f;
        _pd.cross_rate_filt = 0.0f;
        _pd.spike_this_cycle = true;
        _pd.spike_count++;
    } else if (fabsf(cross_rate_raw) > FC::PDGains::MAX_CROSS_RATE_M_S) {
        float clamped = copysignf(FC::PDGains::MAX_CROSS_RATE_M_S, cross_rate_raw);
        if (now_ms - _pd.spike_last_log_ms >= 1000) {
            _pd.spike_last_log_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                "[FORM_E] CRS_SPIKE n=%u raw=%+.0f vrel=(%.1f,%.1f)",
                (unsigned)_pd.spike_count,
                (double)cross_rate_raw, (double)vrel_n, (double)vrel_e);
        }
        cross_rate_raw = clamped;
        _pd.spike_this_cycle = true;
        _pd.spike_count++;
    }

    _pd.prev_err_cross = cross_m;

    float alpha = dt / (FC::PDGains::CROSS_RATE_FILTER_TAU_S + dt);
    _pd.cross_rate_filt = (1.0f - alpha) * _pd.cross_rate_filt + alpha * cross_rate_raw;
    _last_cross_track_error = cross_m;

    // =========================================================================
    // TURN-FOLLOWING ROLL GUIDANCE (Phase 7D.49)
    // =========================================================================
    float follower_hdg_deg = degrees(current_heading);
    float leader_hdg_deg = degrees(leader_yaw);
    _turn_follow.hdg_err_deg = follower_hdg_deg - leader_hdg_deg;
    while (_turn_follow.hdg_err_deg > 180.0f) _turn_follow.hdg_err_deg -= 360.0f;
    while (_turn_follow.hdg_err_deg < -180.0f) _turn_follow.hdg_err_deg += 360.0f;

    // Turn-follow unit guard
    bool turn_follow_unit_guard = false;
    if (fabsf(follower_hdg_deg) > 360.0f || fabsf(leader_hdg_deg) > 360.0f) {
        turn_follow_unit_guard = true;
        _turn_follow.w_turn = 0.0f;
        _turn_follow.active = false;

        static uint32_t last_guard_warn_ms = 0;
        if (now_ms - last_guard_warn_ms >= 2000) {
            last_guard_warn_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                "[TURN_FOLLOW_GUARD] bad_units f=%.0f l=%.0f he=%.0f",
                (double)follower_hdg_deg, (double)leader_hdg_deg,
                (double)_turn_follow.hdg_err_deg);
        }
    }

    // Improved turn detection (Phase 7D.60)
    float lead_yr_deg_s = degrees(_lead.yaw_rate_rad_s);
    float yr_abs = fabsf(lead_yr_deg_s);

    if (_turn_follow.yr_latched) {
        if (yr_abs < FC::TurnFollow::YR_THRESH_OFF_DEG_S) {
            _turn_follow.yr_latched = false;
        }
    } else {
        if (yr_abs > FC::TurnFollow::YR_THRESH_ON_DEG_S) {
            _turn_follow.yr_latched = true;
        }
    }

    _turn_follow.cross_rate_detect = fabsf(_pd.cross_rate_filt) > FC::TurnFollow::CROSS_RATE_THRESH_M_S;
    bool large_hdg_err = fabsf(_turn_follow.hdg_err_deg) > FC::TurnFollow::HDG_ERR_TURN_ON_DEG;
    bool large_cross = fabsf(cross_m) > FC::CrossProtect::ON_M;

    if (!turn_follow_unit_guard) {
        _turn_follow.active = _turn_follow.yr_latched || _turn_follow.cross_rate_detect ||
                              large_hdg_err || large_cross;
    }

    // Compute desired yaw rate for turn-following
    _turn_follow.yr_des_deg_s = lead_yr_deg_s - FC::TurnFollow::K_HDG_YR * _turn_follow.hdg_err_deg;
    _turn_follow.yr_des_deg_s = constrain_float(_turn_follow.yr_des_deg_s, -FC::TurnFollow::YR_DES_MAX_DEG_S, FC::TurnFollow::YR_DES_MAX_DEG_S);

    // Compute turn-following bank angle
    float yr_des_rad_s = radians(_turn_follow.yr_des_deg_s);
    float V_tas = get_lead_tas();
    float bank_turn_rad = atanf((V_tas * yr_des_rad_s) / 9.81f);
    _turn_follow.bank_turn_deg = degrees(bank_turn_rad);

    // Compute blend weight W_TURN
    float hdg_err_abs = fabsf(_turn_follow.hdg_err_deg);
    if (hdg_err_abs <= FC::TurnFollow::HDG_ERR_TURN_ON_DEG) {
        _turn_follow.w_turn = 0.0f;
    } else if (hdg_err_abs >= FC::TurnFollow::HDG_ERR_BLEND_FULL_DEG) {
        _turn_follow.w_turn = 1.0f;
    } else {
        _turn_follow.w_turn = (hdg_err_abs - FC::TurnFollow::HDG_ERR_TURN_ON_DEG) /
                              (FC::TurnFollow::HDG_ERR_BLEND_FULL_DEG - FC::TurnFollow::HDG_ERR_TURN_ON_DEG);
    }
    _turn_follow.w_turn = _turn_follow.w_turn * _turn_follow.w_turn * (3.0f - 2.0f * _turn_follow.w_turn);

    // Fix W_TURN collapse (Phase 7D.59)
    _turn_follow.w_min_enforced = false;
    _turn_follow.lead_yr_deg_s = lead_yr_deg_s;
    if (_turn_follow.active && fabsf(_turn_follow.bank_turn_deg) > FC::TurnFollow::BANK_FF_MIN_DEG) {
        if (_turn_follow.w_turn < FC::TurnFollow::W_MIN) {
            _turn_follow.w_turn = FC::TurnFollow::W_MIN;
            _turn_follow.w_min_enforced = true;
        }
    }

    // Rate limiting on bank_turn
    float max_bank_delta_deg = FC::TurnFollow::BANK_RATE_DEG_S * dt;
    float bank_turn_delta = _turn_follow.bank_turn_deg - _turn_follow.prev_bank_turn_deg;
    if (bank_turn_delta > max_bank_delta_deg) {
        _turn_follow.bank_turn_deg = _turn_follow.prev_bank_turn_deg + max_bank_delta_deg;
    } else if (bank_turn_delta < -max_bank_delta_deg) {
        _turn_follow.bank_turn_deg = _turn_follow.prev_bank_turn_deg - max_bank_delta_deg;
    }
    _turn_follow.prev_bank_turn_deg = _turn_follow.bank_turn_deg;

    // Safety guard for turn-follow at far range
    _turn_follow.safety_blocked = false;
    if (_turn_follow.active && _turn_follow.w_turn > 0.0f) {
        bool in_intercept_with_large_cross = (behind_m > FC::TurnFollow::SAFETY_FAR_RANGE_M) &&
                                              (fabsf(cross_m) > FC::TurnFollow::SAFETY_MAX_CROSS_M);
        if (in_intercept_with_large_cross) {
            _turn_follow.safety_blocked = true;
            _turn_follow.w_turn = 0.0f;
        }
    }

    // =========================================================================
    // PD ROLL CONTROL LAW
    // =========================================================================
    float p_roll = -_pd.kp_cross * cross_m;

    float d_roll_raw = -_pd.kd_cross * _pd.cross_rate_filt;
    const float max_d_roll_rad = radians(FC::PDGains::MAX_D_TERM_DEG);
    float d_roll = constrain_float(d_roll_raw, -max_d_roll_rad, max_d_roll_rad);

    float cross_track_roll = p_roll + d_roll;

    // Turn feed-forward
    float turn_ff = FC::TurnFollow::TURN_FF_GAIN * _lead.yaw_rate_rad_s * _predict_lead_s;
    cross_track_roll += turn_ff;

    _turn_follow.bank_cross_deg = degrees(cross_track_roll);

    // Blend cross-track with turn-following roll
    float blended_roll_deg;
    if (_turn_follow.w_turn > 0.001f) {
        blended_roll_deg = (1.0f - _turn_follow.w_turn) * _turn_follow.bank_cross_deg +
                           _turn_follow.w_turn * _turn_follow.bank_turn_deg;
    } else {
        blended_roll_deg = _turn_follow.bank_cross_deg;
    }

    // Enforce minimum nav_roll during turns (Phase 7D.59)
    _turn_follow.cross_min_enforced = false;
    if (_turn_follow.active && fabsf(cross_m) > FC::TurnFollow::CROSS_MIN_NAV_ROLL_M) {
        float min_roll_mag = FC::TurnFollow::CROSS_MIN_ROLL_DEG;
        float cross_correction_sign = (cross_m > 0.0f) ? -1.0f : 1.0f;
        float current_roll_contribution = blended_roll_deg * cross_correction_sign;

        if (current_roll_contribution < min_roll_mag) {
            blended_roll_deg = min_roll_mag * cross_correction_sign;
            _turn_follow.cross_min_enforced = true;
        }
    }

    // =========================================================================
    // CROSS DIVERGENCE PROTECTION (Phase 7D.60)
    // =========================================================================
    _cross_protect.enforced = false;
    float cross_abs = fabsf(cross_m);

    if (_cross_protect.active) {
        if (cross_abs < FC::CrossProtect::OFF_M) {
            _cross_protect.active = false;
            static uint32_t last_xp_off_ms = 0;
            if (now_ms - last_xp_off_ms >= 2000) {
                last_xp_off_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "[XP_OFF] cross=%.0fm < %.0fm", (double)cross_abs, (double)FC::CrossProtect::OFF_M);
            }
        }
    } else {
        if (cross_abs > FC::CrossProtect::ON_M) {
            _cross_protect.active = true;
            static uint32_t last_xp_on_ms = 0;
            if (now_ms - last_xp_on_ms >= 2000) {
                last_xp_on_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                    "[XP_ON] cross=%.0fm > %.0fm tf=%u", (double)cross_abs, (double)FC::CrossProtect::ON_M,
                    _turn_follow.active ? 1 : 0);
            }
        }
    }

    if (_cross_protect.active) {
        float cross_correction_sign = (cross_m > 0.0f) ? -1.0f : 1.0f;
        float cross_excess = cross_abs - FC::CrossProtect::OFF_M;
        float roll_range = FC::CrossProtect::MAX_ROLL_DEG - FC::CrossProtect::MIN_ROLL_DEG;
        float cross_scale = constrain_float(cross_excess / 100.0f, 0.0f, 1.0f);
        float xp_roll_mag = FC::CrossProtect::MIN_ROLL_DEG + roll_range * cross_scale;

        _cross_protect.roll_deg = xp_roll_mag * cross_correction_sign;

        float current_roll_contribution = blended_roll_deg * cross_correction_sign;
        if (current_roll_contribution < xp_roll_mag) {
            blended_roll_deg = _cross_protect.roll_deg;
            _cross_protect.enforced = true;
        }
    }

    float raw_roll = radians(blended_roll_deg);
    _pd.raw_roll_for_log = raw_roll;

    // Turn phase detection
    bool in_turn_phase = _turn_follow.active;
    float effective_max_bank = in_turn_phase ? radians(FC::BankLimits::MAX_BANK_TURN_DEG) : _max_bank_rad;

    // Smooth saturation
    float desired_roll = effective_max_bank * tanhf(raw_roll / effective_max_bank);

    // Roll command rate limiting
    float sat_pct = MIN(100.0f, 100.0f * fabsf(raw_roll) / effective_max_bank);

    float slew_rate_deg_s = FC::BankLimits::ROLL_SLEW_RATE_NORMAL_DEG_S;
    if (fabsf(cross_m) > FC::BankLimits::ROLL_SLEW_BOOST_CROSS_THRESH_M ||
        sat_pct > FC::BankLimits::ROLL_SLEW_BOOST_SAT_THRESH_PCT) {
        slew_rate_deg_s = FC::BankLimits::ROLL_SLEW_RATE_BOOST_DEG_S;
    }
    _pd.slew_rate_used_deg_s = slew_rate_deg_s;

    const float max_roll_rate_rad = radians(slew_rate_deg_s);
    float max_delta = max_roll_rate_rad * dt;
    float roll_pd = desired_roll;

    bool sign_flip = (desired_roll > 0.0f && _pd.prev_roll_cmd < 0.0f) ||
                     (desired_roll < 0.0f && _pd.prev_roll_cmd > 0.0f);
    bool significant_magnitude = fabsf(desired_roll) > radians(8.0f);

    if (sign_flip && significant_magnitude) {
        float accel_max_delta = max_delta * 3.0f;
        if (_pd.prev_roll_cmd > 0.0f) {
            roll_pd = _pd.prev_roll_cmd - accel_max_delta;
            if (roll_pd < 0.0f) {
                roll_pd = MAX(-max_delta, desired_roll);
            }
        } else {
            roll_pd = _pd.prev_roll_cmd + accel_max_delta;
            if (roll_pd > 0.0f) {
                roll_pd = MIN(max_delta, desired_roll);
            }
        }
        if ((roll_pd > 0.0f && desired_roll < -radians(8.0f)) ||
            (roll_pd < 0.0f && desired_roll > radians(8.0f))) {
            roll_pd = 0.0f;
        }
    } else {
        float delta = desired_roll - _pd.prev_roll_cmd;
        if (delta > max_delta) {
            roll_pd = _pd.prev_roll_cmd + max_delta;
        } else if (delta < -max_delta) {
            roll_pd = _pd.prev_roll_cmd - max_delta;
        }
    }

    float roll_cmd = constrain_float(roll_pd, -effective_max_bank, effective_max_bank);

    if (dt > 0.0f) {
        _pd.delta_roll_rate_deg_s = degrees(roll_cmd - _pd.prev_roll_cmd) / dt;
    }
    _pd.prev_roll_cmd = roll_cmd;

    // =========================================================================
    // SIGN DIVERGENCE BRAKE + PANIC BRAKE
    // =========================================================================
    float roll_cmd_deg = degrees(roll_cmd);

    if (_sign_diverge.panic_until_ms > 0 && now_ms < _sign_diverge.panic_until_ms) {
        roll_cmd = 0.0f;
        roll_cmd_deg = 0.0f;
        if (!_sign_diverge.panic_logged) {
            _sign_diverge.panic_logged = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                "[FORM_E] SIGN_PANIC n=%d hold=10s",
                (int)FC::SignDiverge::PANIC_TRIGGER_COUNT);
        }
    } else if (_sign_diverge.brake_until_ms > 0 && now_ms < _sign_diverge.brake_until_ms) {
        roll_cmd = 0.0f;
        roll_cmd_deg = 0.0f;
    } else {
        if (_sign_diverge.brake_until_ms > 0 && now_ms >= _sign_diverge.brake_until_ms) {
            _sign_diverge.brake_until_ms = 0;
            _sign_diverge.brake_logged = false;
        }
        if (_sign_diverge.panic_until_ms > 0 && now_ms >= _sign_diverge.panic_until_ms) {
            _sign_diverge.panic_until_ms = 0;
            _sign_diverge.panic_logged = false;
        }

        bool cross_large = fabsf(cross_m) > FC::SignDiverge::CROSS_THRESH_M;
        bool roll_significant = fabsf(roll_cmd_deg) > FC::SignDiverge::ROLL_THRESH_DEG;
        bool same_sign = (cross_m > 0.0f && roll_cmd_deg > 0.0f) ||
                         (cross_m < 0.0f && roll_cmd_deg < 0.0f);

        if (cross_large && roll_significant && same_sign) {
            uint32_t dt_ms = 0;
            if (_sign_diverge.last_check_ms > 0) {
                dt_ms = now_ms - _sign_diverge.last_check_ms;
                if (dt_ms > 100) dt_ms = 100;
            }
            _sign_diverge.last_check_ms = now_ms;
            _sign_diverge.accum_ms += dt_ms;

            if (_sign_diverge.accum_ms >= FC::SignDiverge::PERSIST_MS) {
                _sign_diverge.brake_until_ms = now_ms + FC::SignDiverge::BRAKE_DURATION_MS;
                _sign_diverge.accum_ms = 0;

                if (!_sign_diverge.brake_logged) {
                    _sign_diverge.brake_logged = true;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                        "[FORM_E] SIGN_DIVERGE c=%+.0f r=%+.0f",
                        (double)cross_m, (double)roll_cmd_deg);
                }

                _sign_diverge.trig_times_ms[_sign_diverge.trig_idx] = now_ms;
                _sign_diverge.trig_idx = (_sign_diverge.trig_idx + 1) % FC::SignDiverge::PANIC_TRIGGER_COUNT;

                uint8_t triggers_in_window = 0;
                for (uint8_t i = 0; i < FC::SignDiverge::PANIC_TRIGGER_COUNT; i++) {
                    if (_sign_diverge.trig_times_ms[i] > 0 &&
                        (now_ms - _sign_diverge.trig_times_ms[i]) <= FC::SignDiverge::PANIC_WINDOW_MS) {
                        triggers_in_window++;
                    }
                }

                if (triggers_in_window >= FC::SignDiverge::PANIC_TRIGGER_COUNT) {
                    _sign_diverge.panic_until_ms = now_ms + FC::SignDiverge::PANIC_DURATION_MS;
                    _sign_diverge.panic_logged = false;
                    for (uint8_t i = 0; i < FC::SignDiverge::PANIC_TRIGGER_COUNT; i++) {
                        _sign_diverge.trig_times_ms[i] = 0;
                    }
                    _sign_diverge.trig_idx = 0;
                }

                roll_cmd = 0.0f;
                roll_cmd_deg = 0.0f;
            }
        } else {
            _sign_diverge.accum_ms = 0;
            _sign_diverge.last_check_ms = now_ms;
        }
    }

    // =========================================================================
    // DIAGNOSTIC LOGS
    // =========================================================================
    static uint32_t last_pd_log_ms = 0;
    static float prev_log_roll_deg = 0.0f;
    if (now_ms - last_pd_log_ms >= 1000 && fabsf(cross_m) > 1.0f) {
        float dt_log = (now_ms - last_pd_log_ms) * 0.001f;
        last_pd_log_ms = now_ms;
        float raw_roll_deg = degrees(_pd.raw_roll_for_log);
        float local_sat_pct = MIN(100.0f, 100.0f * fabsf(_pd.raw_roll_for_log) / effective_max_bank);
        int spike_flag = _pd.spike_this_cycle ? 1 : 0;

        float dr_deg_s = 0.0f;
        if (dt_log > 0.1f) {
            dr_deg_s = (roll_cmd_deg - prev_log_roll_deg) / dt_log;
        }
        prev_log_roll_deg = roll_cmd_deg;

        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "[PD]c%+.0fd%+.0fw%+.0fr%+.0fs%dx%ddr%+.0f",
                      (double)cross_m,
                      (double)_pd.cross_rate_filt,
                      (double)raw_roll_deg,
                      (double)roll_cmd_deg,
                      (int)local_sat_pct, spike_flag, (double)dr_deg_s);
    }

    // Turn-follow diagnostic log
    if (now_ms - _turn_follow.last_log_ms >= 1000) {
        _turn_follow.last_log_ms = now_ms;

        int w_pct = (int)(_turn_follow.w_turn * 100.0f);
        int tf_flag = _turn_follow.active ? 1 : 0;
        int sb_flag = _turn_follow.safety_blocked ? 1 : 0;
        int wm_flag = _turn_follow.w_min_enforced ? 1 : 0;
        int cm_flag = _turn_follow.cross_min_enforced ? 1 : 0;
        int xp_flag = _cross_protect.enforced ? 1 : 0;

        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "[TURN_FOLLOW]he%+.0fyr%+.0fbt%+.0fbc%+.0fw%dtf%dsb%dwm%dcm%dxp%d",
                      (double)_turn_follow.hdg_err_deg,
                      (double)_turn_follow.lead_yr_deg_s,
                      (double)_turn_follow.bank_turn_deg,
                      (double)_turn_follow.bank_cross_deg,
                      w_pct, tf_flag, sb_flag, wm_flag, cm_flag, xp_flag);
    }

    // Detailed turn-follow debug log
    static uint32_t last_tf_dbg_ms = 0;
    if (now_ms - last_tf_dbg_ms >= 1000) {
        if (now_ms - _turn_follow.last_log_ms >= 400) {
            last_tf_dbg_ms = now_ms;

            const char* guard_str = "ok";
            if (turn_follow_unit_guard) {
                guard_str = "bad_units";
            } else if (!_turn_follow.active) {
                guard_str = "straight";
            }

            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "[TF_DBG]tf=%u w=%.2f yr=%.1f bt=%.1f xp=%u",
                          _turn_follow.active ? 1 : 0,
                          (double)_turn_follow.w_turn,
                          (double)_turn_follow.lead_yr_deg_s,
                          (double)_turn_follow.bank_turn_deg,
                          _cross_protect.active ? 1 : 0);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "[TF_DBG2]c=%.0f he=%.1f r=%.1f %s wm%dcm%dxp%d",
                          (double)cross_m,
                          (double)_turn_follow.hdg_err_deg,
                          (double)roll_cmd_deg,
                          guard_str,
                          _turn_follow.w_min_enforced ? 1 : 0,
                          _turn_follow.cross_min_enforced ? 1 : 0,
                          _cross_protect.enforced ? 1 : 0);
        }
    }

    float pitch_cmd = 0.0f;
    _pd.prev_err_initialized = true;

    _roll_command = roll_cmd;
    _pitch_command = pitch_cmd;

    // =========================================================================
    // DIRECT THROTTLE CONTROL (Test 360)
    // =========================================================================
    if (_direct_throttle.behind_m <= FC::DirectThrottle::ENABLE_BEHIND_M && _direct_throttle.behind_m > 0.0f) {
        _direct_throttle.override_active = true;

        float leader_tas = get_lead_tas();
        float follower_tas = _direct_throttle.follower_airspeed;

        static bool direct_throttle_logged = false;
        if (!direct_throttle_logged && _direct_throttle.override_active) {
            direct_throttle_logged = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                "[T360_DIRECT_THR] ENTER behind=%.1f r=%.1f lead_tas=%.1f fol_tas=%.1f",
                (double)_direct_throttle.behind_m, (double)_range_to_lead, (double)leader_tas, (double)follower_tas);
        }

        float airspeed_error = leader_tas - follower_tas;
        float base_throttle = FC::DirectThrottle::BASE_THROTTLE_PCT + airspeed_error * FC::DirectThrottle::KP_AIRSPEED_THROTTLE;

        float range_trim;
        if (_direct_throttle.behind_m > 10.0f) {
            range_trim = (_direct_throttle.behind_m - 10.0f) * 1.0f;
            range_trim = constrain_float(range_trim, 0.0f, 20.0f);
        } else if (_direct_throttle.behind_m > 2.0f) {
            range_trim = 0.0f;
        } else {
            range_trim = (_direct_throttle.behind_m - 2.0f) * 1.5f;
            range_trim = constrain_float(range_trim, -3.0f, 0.0f);
        }

        _direct_throttle.throttle_percent = constrain_float(base_throttle + range_trim,
                                            FC::DirectThrottle::MIN_THROTTLE_PCT, FC::DirectThrottle::MAX_THROTTLE_PCT);

        // Zone-based pitch control (Test 361)
        float pitch_trim_deg;
        if (_direct_throttle.behind_m > FC::PitchControl::ZONE_FAR_M) {
            pitch_trim_deg = FC::PitchControl::CLOSING_DEG;
        } else if (_direct_throttle.behind_m > FC::PitchControl::ZONE_NEAR_M) {
            pitch_trim_deg = FC::PitchControl::FINE_DEG;
        } else {
            pitch_trim_deg = FC::PitchControl::CONTACT_DEG;
        }
        _direct_throttle.pitch_command_cd = (int32_t)(pitch_trim_deg * 100.0f);

        // Altitude floor protection
        if (_direct_throttle.descent_rate > FC::DirectThrottle::ALT_FLOOR_DESCENT_RATE_MPS) {
            _direct_throttle.pitch_command_cd = (int32_t)(FC::DirectThrottle::ALT_FLOOR_PITCH_DEG * 100.0f);
            pitch_cmd = radians(FC::DirectThrottle::ALT_FLOOR_PITCH_DEG);
            _pitch_command = pitch_cmd;

            static uint32_t last_alt_floor_log_ms = 0;
            if (now_ms - last_alt_floor_log_ms >= 2000) {
                last_alt_floor_log_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                    "[T362_ALT_FLOOR] descent=%.1fm/s > %.1f - forcing pitch +%.0f",
                    (double)_direct_throttle.descent_rate, (double)FC::DirectThrottle::ALT_FLOOR_DESCENT_RATE_MPS,
                    (double)FC::DirectThrottle::ALT_FLOOR_PITCH_DEG);
            }
        }

        // Diverge detection (Test 363)
        float velocity_closure = follower_tas - leader_tas;

        if (velocity_closure < FC::DirectThrottle::DIVERGE_THRESHOLD_MPS) {
            _direct_throttle.diverge_timer_ms += 20;
            if (_direct_throttle.diverge_timer_ms > FC::DirectThrottle::DIVERGE_TIMEOUT_MS) {
                _direct_throttle.override_active = false;
                _direct_throttle.diverge_timer_ms = 0;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                    "[T363_ABORT] Diverging Vc=%.2fm/s for 3s - aborting C++",
                    (double)velocity_closure);
            }
        } else {
            _direct_throttle.diverge_timer_ms = 0;
        }

        // Throttle diagnostic logging
        if (now_ms - _direct_throttle.log_last_ms >= FC::DirectThrottle::LOG_INTERVAL_MS) {
            _direct_throttle.log_last_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                "[T363_DUAL] b=%.1f th=%.0f p=%ld Vc=%.2f",
                (double)_direct_throttle.behind_m,
                (double)_direct_throttle.throttle_percent,
                (long)_direct_throttle.pitch_command_cd,
                (double)velocity_closure);
        }

    } else {
        _direct_throttle.override_active = false;
        _direct_throttle.diverge_timer_ms = 0;
    }

    _last_update_ms = now_ms;
}

// Calculate horizontal distance between two locations
float FormationController::calculate_range(const Location &loc1, const Location &loc2) {
    return loc1.get_distance(loc2);
}

// Calculate closure rate (positive = approaching, negative = separating)
float FormationController::calculate_closure_rate(float current_range, float dt) {
    if (dt < 0.001f || _last_range_ms == 0) {
        _last_range = current_range;
        _last_range_ms = AP_HAL::millis();
        return 0.0f;
    }

    float closure = (_last_range - current_range) / dt;

    _last_range = current_range;
    _last_range_ms = AP_HAL::millis();

    return closure;
}

// Calculate desired airspeed using graduated speed control
float FormationController::calculate_desired_speed(float range_to_lead) {
    float desired_speed;
    float leader_tas = get_lead_tas();
    uint32_t now_ms = AP_HAL::millis();
    float dt = 0.0f;
    if (_fine_ctrl.standoff_last_update_ms > 0) {
        dt = (now_ms - _fine_ctrl.standoff_last_update_ms) * 0.001f;
        if (dt > 0.1f) dt = 0.1f;
    }
    _fine_ctrl.standoff_last_update_ms = now_ms;

    if (range_to_lead > FC::SpeedControl::INTERCEPT_THRESHOLD_M) {
        // INTERCEPT PHASE
        float target_closure = range_to_lead / 10.0f;
        target_closure = constrain_float(target_closure, FC::SpeedControl::MIN_CLOSURE_RATE_MPS, FC::SpeedControl::MAX_CLOSURE_RATE_MPS);
        desired_speed = leader_tas + target_closure;
        desired_speed = MIN(desired_speed, FC::SpeedControl::MAX_APPROACH_SPEED_MPS);
        _fine_ctrl.active = false;

    } else if (range_to_lead > FC::SpeedControl::TRANSITION_THRESHOLD_M) {
        // TRANSITION PHASE
        float range_error = range_to_lead - FC::Geometry::OFFSET_M;
        float target_closure = range_error / 5.0f;
        target_closure = MAX(target_closure, FC::SpeedControl::MIN_CLOSURE_RATE_MPS);
        desired_speed = leader_tas + target_closure;
        _fine_ctrl.active = false;

    } else if (range_to_lead > FC::FineControl::ENABLE_RANGE_M) {
        // FORMATION PHASE
        float target_closure = 0.5f;
        desired_speed = leader_tas + target_closure;
        _fine_ctrl.active = false;

    } else if (range_to_lead > FC::FineControl::PREDECEL_ZONE_INNER_M) {
        // PRE-DECEL PHASE (Test 359)
        if (!_fine_ctrl.active) {
            _fine_ctrl.active = true;
            _fine_ctrl.entry_ms = now_ms;
            _fine_ctrl.entry_vc = _closure_rate;
            _fine_ctrl.min_range = range_to_lead;
            _fine_ctrl.standoff_dwell_s = 0.0f;
            _fine_ctrl.standoff_cleared = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                "[T359_FINE] ENTER r=%.1f Vc=%.2f TAS=%.1f",
                (double)range_to_lead, (double)_closure_rate, (double)leader_tas);
        }

        float dist_to_standoff = range_to_lead - FC::FineControl::PREDECEL_ZONE_INNER_M;
        float target_vc = FC::FineControl::PREDECEL_TARGET_VC_MPS + dist_to_standoff * 0.05f;

        float vc_error = _closure_rate - target_vc;
        float speed_adj = constrain_float(vc_error * 0.5f, -FC::FineControl::PREDECEL_MAX_ADJ_MPS, FC::FineControl::PREDECEL_MAX_ADJ_MPS);
        desired_speed = leader_tas - speed_adj;

        if (range_to_lead < _fine_ctrl.min_range) {
            _fine_ctrl.min_range = range_to_lead;
        }

        if (now_ms - _fine_ctrl.last_log_ms >= FC::FineControl::LOG_INTERVAL_MS) {
            _fine_ctrl.last_log_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                "[T359_PREDECEL] r=%.1f Vc=%.2f tgt=%.2f adj=%.2f",
                (double)range_to_lead, (double)_closure_rate, (double)target_vc, (double)speed_adj);
        }

    } else if (range_to_lead > FC::FineControl::STANDOFF_INNER_M) {
        // STANDOFF PHASE (Test 359)
        if (!_fine_ctrl.standoff_cleared) {
            if (_closure_rate < FC::FineControl::STANDOFF_VC_THRESHOLD_MPS &&
                _closure_rate > -FC::FineControl::STANDOFF_VC_THRESHOLD_MPS) {
                _fine_ctrl.standoff_dwell_s += dt;

                if (_fine_ctrl.standoff_dwell_s >= FC::FineControl::STANDOFF_DWELL_REQUIRED_S) {
                    _fine_ctrl.standoff_cleared = true;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                        "[T359_STANDOFF] CLEARED after %.1fs dwell at r=%.2f",
                        (double)_fine_ctrl.standoff_dwell_s, (double)range_to_lead);
                }

                desired_speed = leader_tas;

            } else {
                float speed_adj = constrain_float(_closure_rate * 0.5f, -0.5f, 0.5f);
                desired_speed = leader_tas - speed_adj;
                _fine_ctrl.standoff_dwell_s = 0.0f;

                if (now_ms - _fine_ctrl.last_log_ms >= FC::FineControl::LOG_INTERVAL_MS) {
                    _fine_ctrl.last_log_ms = now_ms;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                        "[T359_STANDOFF] HOLD r=%.2f Vc=%.2f dwell=%.1f adj=%.2f",
                        (double)range_to_lead, (double)_closure_rate,
                        (double)_fine_ctrl.standoff_dwell_s, (double)speed_adj);
                }
            }
        } else {
            desired_speed = leader_tas + 0.02f;
        }

        if (range_to_lead < _fine_ctrl.min_range) {
            _fine_ctrl.min_range = range_to_lead;
        }

    } else {
        // CONTACT PHASE (Test 359)
        float vc_error = _closure_rate - 0.0f;
        float speed_adj = constrain_float(vc_error * 0.3f, -FC::FineControl::CONTACT_MAX_ADJ_MPS, FC::FineControl::CONTACT_MAX_ADJ_MPS);
        desired_speed = leader_tas - speed_adj;

        if (range_to_lead < _fine_ctrl.min_range) {
            _fine_ctrl.min_range = range_to_lead;
        }

        static bool contact_logged = false;
        if (!contact_logged) {
            contact_logged = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                "[T359_CONTACT] ENTER r=%.2f Vc=%.3f min_r=%.2f",
                (double)range_to_lead, (double)_closure_rate, (double)_fine_ctrl.min_range);
        }
    }

    // Overtake protection (Test 359)
    if (range_to_lead < FC::FineControl::PREDECEL_ZONE_INNER_M &&
        _closure_rate > FC::FineControl::OVERTAKE_VC_THRESH_MPS) {
        desired_speed = leader_tas - FC::FineControl::OVERTAKE_BRAKE_ADJ_MPS;
        _fine_ctrl.standoff_dwell_s = 0.0f;
        _fine_ctrl.overtake_count++;

        if (dt > 0.0f) {
            _fine_ctrl.overtake_active_time_s += dt;
        }

        if (now_ms - _fine_ctrl.overtake_last_ms >= 1000) {
            _fine_ctrl.overtake_last_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                "[T359_OVERTAKE] Vc=%.2f at r=%.1fm - BRAKING (n=%u)",
                (double)_closure_rate, (double)range_to_lead, (unsigned)_fine_ctrl.overtake_count);
        }
    }

    return desired_speed;
}

// Calculate formation waypoint with predictive placement
Location FormationController::calculate_formation_waypoint() {
    float predicted_north = _lead.vn * FC::Geometry::PREDICT_TIME_S;
    float predicted_east = _lead.ve * FC::Geometry::PREDICT_TIME_S;

    Location lead_current(
        static_cast<int32_t>(_lead.lat * 1e7),
        static_cast<int32_t>(_lead.lon * 1e7),
        static_cast<int32_t>(_lead.alt * 100),
        Location::AltFrame::ABSOLUTE
    );

    lead_current.offset(predicted_north, predicted_east);

    return lead_current;
}

// Get formation waypoint for guidance system
Location FormationController::get_formation_waypoint() {
    return _formation_waypoint;
}

// Get desired airspeed command
float FormationController::get_desired_airspeed() {
    return _desired_airspeed;
}

// Set UWB range measurement
void FormationController::set_uwb_range(float range_m) {
    if (range_m > 0 && range_m < FC::SensorFusion::UWB_MAX_RANGE_M) {
        _sensor_fusion.uwb_range = range_m;
        _sensor_fusion.uwb_valid = true;
        _sensor_fusion.uwb_last_update_ms = AP_HAL::millis();
    }
}

// Calculate adaptive alpha for GPS/UWB fusion
float FormationController::calculate_adaptive_alpha(float range) {
    if (range > FC::SensorFusion::UWB_MAX_RANGE_M) {
        return 0.0f;
    }

    if (range < 30.0f) {
        return 1.0f - (range / 30.0f) * 0.1f;
    } else if (range < 80.0f) {
        float t = (range - 30.0f) / 50.0f;
        return 0.9f - t * 0.4f;
    } else if (range < 100.0f) {
        return 0.5f;
    } else {
        float t = (range - 100.0f) / 100.0f;
        return 0.5f - t * 0.2f;
    }
}

// Fuse GPS and UWB range measurements
float FormationController::fuse_gps_uwb_range(float gps_range, float uwb_range) {
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _sensor_fusion.uwb_last_update_ms > FC::SensorFusion::UWB_TIMEOUT_MS) {
        _sensor_fusion.uwb_valid = false;
    }

    if (!_sensor_fusion.uwb_valid) {
        return gps_range;
    }

    float range_for_alpha = uwb_range;
    float alpha = calculate_adaptive_alpha(range_for_alpha);
    float fused = alpha * uwb_range + (1.0f - alpha) * gps_range;

    return fused;
}

// Update fused range with smoothing
void FormationController::update_fused_range(float gps_range) {
    float raw_fused = fuse_gps_uwb_range(gps_range, _sensor_fusion.uwb_range);

    if (!_sensor_fusion.fused_range_initialized) {
        _sensor_fusion.fused_range = raw_fused;
        _sensor_fusion.fused_range_initialized = true;
    } else {
        _sensor_fusion.fused_range = FC::SensorFusion::SMOOTH_ALPHA * raw_fused +
                                      (1.0f - FC::SensorFusion::SMOOTH_ALPHA) * _sensor_fusion.fused_range;
    }
}
