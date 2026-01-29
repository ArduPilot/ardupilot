/*
   FlightDock Formation Controller

   Minimal formation flight controller integrated directly into ArduPlane.
   Implements graduated speed control with range/10 rule for smooth intercept.

   Author: FlightDock Research Team
   Date: January 2026
   Version: 7D.63 (T363: Velocity-based closure rate fix)

   Refactored: 2026-01-24 - Organized constants into formation_config.h
*/

#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include "formation_config.h"

// Namespace aliases for cleaner code
namespace FC = FormationConfig;

// =========================================================================
// State Structures for Organized Member Variables
// =========================================================================

// Lead aircraft telemetry state
struct LeadState {
    float lat = 0;              // degrees
    float lon = 0;              // degrees
    float alt = 0;              // meters
    float vn = 0;               // North velocity (m/s)
    float ve = 0;               // East velocity (m/s)
    float vd = 0;               // Down velocity (m/s)
    bool valid = false;
    uint32_t last_update_ms = 0;

    // Leader TAS from Python telemetry
    float tas_actual = 0.0f;
    bool tas_valid = false;

    // Leader yaw rate tracking
    float yaw_prev_rad = 0.0f;
    float yaw_rate_rad_s = 0.0f;
    uint32_t yaw_last_update_ms = 0;

    // Leader position prediction
    Vector3f pos_prev_ned;
    Vector3f vel_estimated;
    uint32_t vel_last_update_ms = 0;
};

// Turn-following state (Phase 7D.49)
struct TurnFollowState {
    float hdg_err_deg = 0.0f;           // Heading error (follower - leader)
    float yr_des_deg_s = 0.0f;          // Desired yaw rate
    float bank_turn_deg = 0.0f;         // Bank from turn-following
    float bank_cross_deg = 0.0f;        // Bank from cross-track PD
    float w_turn = 0.0f;                // Blend weight (0-1)
    bool active = false;                // Turn-following engaged
    float prev_bank_turn_deg = 0.0f;    // For rate limiting
    uint32_t last_log_ms = 0;
    bool safety_blocked = false;        // Deep approach blocked
    bool w_min_enforced = false;        // Phase 7D.59 fix
    bool cross_min_enforced = false;    // Phase 7D.59 fix
    float lead_yr_deg_s = 0.0f;         // Leader yaw rate for logging

    // Turn detection (Phase 7D.60)
    bool yr_latched = false;            // Yaw-rate based detection
    bool cross_rate_detect = false;     // Cross-rate based detection
};

// Cross-track protection state (Phase 7D.60)
struct CrossProtectState {
    bool active = false;                // Hysteresis latched
    bool enforced = false;              // Applied this cycle
    float roll_deg = 0.0f;              // Computed roll command
};

// Sign divergence brake state (Test 233/234)
struct SignDivergeState {
    uint32_t accum_ms = 0;              // Continuous duration accumulator
    uint32_t last_check_ms = 0;         // Last check timestamp
    uint32_t brake_until_ms = 0;        // Brake latch end time
    uint32_t panic_until_ms = 0;        // Panic latch end time
    uint32_t trig_times_ms[3] = {0};    // Ring buffer for PANIC detection
    uint8_t trig_idx = 0;               // Current ring index
    bool brake_logged = false;
    bool panic_logged = false;
};

// Out-of-engage safety state (Test 238)
struct OutOfEngageState {
    uint32_t last_log_ms = 0;
    uint32_t accum_ms = 0;              // Persistence accumulator
    uint32_t last_check_ms = 0;
    bool latched = false;               // Hysteresis state
};

// Fine control state (Test 359)
struct FineControlState {
    float standoff_dwell_s = 0.0f;      // Time at low Vc in standoff
    bool standoff_cleared = false;      // Gate passed
    uint32_t standoff_last_update_ms = 0;
    bool active = false;                // Fine-control engaged
    uint32_t entry_ms = 0;              // Entry timestamp
    float entry_vc = 0.0f;              // Vc at entry
    float min_range = 999.0f;           // Minimum range achieved
    uint32_t overtake_count = 0;        // Overtake events
    uint32_t overtake_last_ms = 0;
    float overtake_active_time_s = 0.0f;
    uint32_t last_log_ms = 0;
};

// Direct throttle control state (Test 360-367)
struct DirectThrottleState {
    float throttle_percent = 50.0f;     // Output (0-100%)
    bool override_active = false;       // C++ owns throttle
    float follower_airspeed = 0.0f;     // Current follower TAS
    float behind_m = 0.0f;              // Along-track distance
    float prev_behind_m = 0.0f;
    float behind_closure_rate = 0.0f;   // Closure from behind_m changes
    float descent_rate = 0.0f;          // Positive = descending
    uint32_t log_last_ms = 0;
    uint32_t diverge_timer_ms = 0;      // Diverge accumulator
    int32_t pitch_command_cd = 0;       // Direct pitch output (centideg)
};

// PD controller state
struct PDControllerState {
    float kp_cross = 0.0f;              // Current proportional gain
    float kd_cross = 0.0f;              // Current derivative gain
    float prev_err_cross = 0.0f;        // Previous cross error
    float prev_err_behind = 0.0f;
    float prev_err_below = 0.0f;
    bool prev_err_initialized = false;
    float cross_rate_filt = 0.0f;       // Filtered cross rate
    float raw_roll_for_log = 0.0f;      // Pre-saturation roll
    uint32_t spike_count = 0;           // Spike rejection counter
    uint32_t spike_last_log_ms = 0;
    bool spike_this_cycle = false;
    float delta_roll_rate_deg_s = 0.0f; // Test 240 diagnostic
    float slew_rate_used_deg_s = 0.0f;
    float prev_roll_cmd = 0.0f;         // For rate limiting
};

// UWB sensor fusion state
struct SensorFusionState {
    float uwb_range = 0.0f;
    bool uwb_valid = false;
    uint32_t uwb_last_update_ms = 0;
    float fused_range = 0.0f;
    bool fused_range_initialized = false;
};

// Build beacon state (Phase 7D.52)
struct BuildBeaconState {
    bool sent = false;
    uint32_t first_update_ms = 0;
    uint32_t last_emit_ms = 0;
};

// =========================================================================
// FormationController Class
// =========================================================================

class FormationController {
public:
    FormationController();

    // Initialize controller
    void init();

    // Main 400 Hz update function
    void update(const Location &current_loc,
                const Vector3f &current_vel,
                float current_heading);

    // Set lead aircraft state (called when GLOBAL_POSITION_INT received)
    void set_lead_state(float lat, float lon, float alt,
                       float vn, float ve, float vd);

    // Set UWB range measurement (called when DISTANCE_SENSOR received)
    void set_uwb_range(float range_m);

    // Get fused range estimate (GPS+UWB)
    float get_fused_range() { return _sensor_fusion.fused_range; }

    // Check if UWB is valid
    bool is_uwb_valid() { return _sensor_fusion.uwb_valid; }

    // Get formation waypoint command
    Location get_formation_waypoint();

    // Get desired airspeed command
    float get_desired_airspeed();

    // Get attitude commands (in radians)
    float get_roll_command() { return _roll_command; }
    float get_pitch_command() { return _pitch_command; }

    // Check if formation mode is active
    bool is_active() { return _active; }

    // Activate/deactivate formation mode
    void set_active(bool active) { _active = active; }

    // Get diagnostic info
    float get_range_to_lead() { return _range_to_lead; }
    float get_closure_rate() { return _closure_rate; }
    float get_formation_distance_error() { return _range_to_lead; }

    // Get cross-track error for REJOIN_STATION debug logging
    float get_cross_track_error() { return _last_cross_track_error; }

    // Get lead telemetry age in milliseconds (for freshness gating)
    uint32_t get_lead_age_ms() {
        if (_lead.last_update_ms == 0) return UINT32_MAX;
        return AP_HAL::millis() - _lead.last_update_ms;
    }

    // Leader TAS setter/getter (Test 359)
    void set_lead_tas(float tas) { _lead.tas_actual = tas; _lead.tas_valid = true; }
    float get_lead_tas() const {
        return _lead.tas_valid ? _lead.tas_actual : FC::SpeedControl::LEAD_SPEED_DEFAULT_MPS;
    }
    bool is_lead_tas_valid() const { return _lead.tas_valid; }

    // Standoff state getters (Test 359)
    float get_standoff_dwell_s() const { return _fine_ctrl.standoff_dwell_s; }
    bool is_standoff_cleared() const { return _fine_ctrl.standoff_cleared; }
    bool is_fine_control_active() const { return _fine_ctrl.active; }

    // Direct throttle getters (Test 360)
    float get_throttle_percent() const { return _direct_throttle.throttle_percent; }
    bool is_throttle_override_active() const { return _direct_throttle.override_active; }

    // Set current follower airspeed for P-control (Test 360)
    void set_follower_airspeed(float airspeed) { _direct_throttle.follower_airspeed = airspeed; }

    // Direct pitch getter (Test 361)
    int32_t get_pitch_command_cd() const { return _direct_throttle.pitch_command_cd; }

private:
    // =========================================================================
    // State Structures
    // =========================================================================
    LeadState _lead;
    TurnFollowState _turn_follow;
    CrossProtectState _cross_protect;
    SignDivergeState _sign_diverge;
    OutOfEngageState _out_of_engage;
    FineControlState _fine_ctrl;
    DirectThrottleState _direct_throttle;
    PDControllerState _pd;
    SensorFusionState _sensor_fusion;
    BuildBeaconState _build_beacon;

    // =========================================================================
    // Core State Variables
    // =========================================================================
    bool _active = false;
    uint32_t _last_update_ms = 0;

    // Formation state
    float _range_to_lead = 0.0f;
    float _closure_rate = 0.0f;
    float _last_range = 0.0f;
    uint32_t _last_range_ms = 0;

    // Calculated outputs
    Location _formation_waypoint;
    float _desired_airspeed = FC::SpeedControl::LEAD_SPEED_DEFAULT_MPS;
    float _roll_command = 0.0f;
    float _pitch_command = 0.0f;

    // Cross-track error for debug logging
    float _last_cross_track_error = 0.0f;

    // Station-keeping PD controller - desired offset in leader body frame (m)
    // (x, y, z) = (behind, right, below) in leader body frame
    Vector3f _desired_offset_body{10.0f, 0.0f, 10.0f};

    // Prediction horizon
    float _predict_lead_s = FC::Geometry::PREDICT_TIME_S;

    // Control output constraints
    float _max_bank_rad = radians(FC::BankLimits::MAX_BANK_NORMAL_DEG);
    float _max_pitch_rad = radians(FC::AltitudeControl::MAX_PITCH_DEG);

    // =========================================================================
    // Internal Functions
    // =========================================================================
    float calculate_range(const Location &loc1, const Location &loc2);
    float calculate_closure_rate(float current_range, float dt);
    float calculate_desired_speed(float range_to_lead);
    Location calculate_formation_waypoint();
    float calculate_adaptive_alpha(float range);
    float fuse_gps_uwb_range(float gps_range, float uwb_range);
    void update_fused_range(float gps_range);
};
