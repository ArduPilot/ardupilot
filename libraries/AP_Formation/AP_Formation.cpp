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

#include "AP_Formation.h"

// Parameter table - exposed as FORM_* parameters
const AP_Param::GroupInfo AP_Formation::var_info[] = {
    // @Param: OFFSET
    // @DisplayName: Formation trail offset
    // @Description: Distance to maintain behind lead aircraft
    // @Units: m
    // @Range: 10 200
    // @User: Standard
    AP_GROUPINFO("OFFSET", 1, AP_Formation, _formation_offset, 50.0f),

    // @Param: LATERAL
    // @DisplayName: Formation lateral offset
    // @Description: Lateral distance from lead aircraft (+ = right)
    // @Units: m
    // @Range: -50 50
    // @User: Standard
    AP_GROUPINFO("LATERAL", 2, AP_Formation, _formation_lateral, 0.0f),

    // @Param: VERTICAL
    // @DisplayName: Formation vertical offset
    // @Description: Vertical distance from lead aircraft (+ = down)
    // @Units: m
    // @Range: -50 50
    // @User: Standard
    AP_GROUPINFO("VERTICAL", 3, AP_Formation, _formation_vertical, 0.0f),

    // @Param: PREDICT
    // @DisplayName: Prediction time
    // @Description: How far ahead to predict lead aircraft position
    // @Units: s
    // @Range: 0.1 3.0
    // @User: Advanced
    AP_GROUPINFO("PREDICT", 4, AP_Formation, _predict_time, 1.0f),

    // @Param: SMOOTH
    // @DisplayName: Range smoothing alpha
    // @Description: Exponential smoothing weight (0=heavy smoothing, 1=no filtering)
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("SMOOTH", 5, AP_Formation, _smooth_alpha, 0.3f),

    // @Param: APPROACH
    // @DisplayName: Approach speed
    // @Description: Speed when far from formation position
    // @Units: m/s
    // @Range: 10 40
    // @User: Standard
    AP_GROUPINFO("APPROACH", 6, AP_Formation, _approach_speed, 26.0f),

    // @Param: MAINTAIN
    // @DisplayName: Maintain speed
    // @Description: Speed when in formation position
    // @Units: m/s
    // @Range: 10 40
    // @User: Standard
    AP_GROUPINFO("MAINTAIN", 7, AP_Formation, _maintain_speed, 24.7f),

    // @Param: SPD_THRESH
    // @DisplayName: Speed threshold
    // @Description: Distance threshold for approach/maintain speed switch
    // @Units: m
    // @Range: 10 100
    // @User: Standard
    AP_GROUPINFO("SPD_THRESH", 8, AP_Formation, _speed_threshold, 55.0f),

    AP_GROUPEND
};

// Constructor - initialize all member variables
AP_Formation::AP_Formation() :
    _uwb_range(0.0f),
    _smooth_range(0.0f),
    _last_update_ms(0),
    _active(false),
    _smooth_range_initialized(false)
{
    // Setup parameter defaults
    AP_Param::setup_object_defaults(this, var_info);

    // Initialize vectors to zero
    _target_pos.zero();
    _target_vel.zero();
    _own_pos.zero();
    _vel_cmd.zero();
    _predicted_target_pos.zero();
}

// Main control loop - runs at 400Hz
// Implements proven Phase 6 GPS+UWB sensor fusion algorithm
void AP_Formation::update(float dt)
{
    // Safety: Don't run if not active
    if (!_active) {
        _vel_cmd.zero();
        return;
    }

    // Check for stale data (5 second timeout)
    uint32_t now = AP_HAL::millis();
    if (_last_update_ms > 0 && now - _last_update_ms > 5000) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Formation: Stale data, disabling");
        _active = false;
        _vel_cmd.zero();
        return;
    }

    // STEP 1: Calculate GPS-based range to target
    Vector3f delta = _target_pos - _own_pos;
    float gps_range = delta.length();

    // STEP 2: Adaptive sensor fusion
    // Use UWB range for alpha calculation if available, otherwise GPS
    float range_for_alpha = (_uwb_range > 0.1f) ? _uwb_range : gps_range;
    float alpha = calculate_adaptive_alpha(range_for_alpha);

    float fused_range;
    if (_uwb_range > 0.1f && _uwb_range < 500.0f) {
        // Valid UWB range - blend with GPS
        fused_range = alpha * _uwb_range + (1.0f - alpha) * gps_range;
    } else {
        // GPS only if UWB invalid
        fused_range = gps_range;
        alpha = 0.0f;
    }

    // STEP 3: Heavy smoothing filter (exponential moving average)
    if (_smooth_range < 0.1f) {
        // Initialize on first update
        _smooth_range = fused_range;
    } else {
        // Exponential smoothing: smoothed = alpha*new + (1-alpha)*old
        _smooth_range = _smooth_alpha * fused_range + (1.0f - _smooth_alpha) * _smooth_range;
    }

    // STEP 4-5: Calculate desired position in formation with prediction
    _predicted_target_pos = calculate_formation_position();

    // STEP 6: Binary speed control
    float target_speed;
    if (_smooth_range > _speed_threshold) {
        target_speed = _approach_speed;  // Far - approach quickly
    } else {
        target_speed = _maintain_speed;  // Close - maintain formation
    }

    // STEP 7: Calculate velocity command toward formation position
    Vector3f direction = _predicted_target_pos - _own_pos;
    float distance = direction.length();

    if (distance > 0.1f) {
        // Normalize direction and scale by target speed
        direction /= distance;
        _vel_cmd = direction * target_speed;
    } else {
        // Already at desired position
        _vel_cmd.zero();
    }

    // Debug logging every 2 seconds
    static uint32_t last_log_ms = 0;
    if (now - last_log_ms > 2000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
            "Formation: range=%.1fm alpha=%.2f speed=%.1fm/s",
            (double)_smooth_range, (double)alpha, (double)target_speed);
        last_log_ms = now;
    }
}

// Set lead aircraft position (NED frame)
// Updates timestamp to track data freshness
void AP_Formation::set_target_position(const Vector3f &pos_ned)
{
    _target_pos = pos_ned;
    _last_update_ms = AP_HAL::millis();
}

// Set lead aircraft velocity (NED frame)
void AP_Formation::set_target_velocity(const Vector3f &vel_ned)
{
    _target_vel = vel_ned;
}

// Set UWB ranging measurement
// Validates range is reasonable before accepting
void AP_Formation::set_uwb_range(float range_m)
{
    // Sanity check: reject invalid or unreasonable ranges
    if (range_m > 0.0f && range_m < 500.0f) {
        _uwb_range = range_m;
    }
}

// Set own position (NED frame)
void AP_Formation::set_own_position(const Vector3f &pos_ned)
{
    _own_pos = pos_ned;
}

// Reset controller state
// Clears all state variables and disables control
void AP_Formation::reset()
{
    _target_pos.zero();
    _target_vel.zero();
    _own_pos.zero();
    _uwb_range = 0.0f;
    _smooth_range = 0.0f;
    _vel_cmd.zero();
    _predicted_target_pos.zero();
    _last_update_ms = 0;
    _active = false;
    _smooth_range_initialized = false;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Formation: Controller reset");
}

// Calculate adaptive GPS/UWB weighting based on range
// Implements Phase 6 validated algorithm:
//   - Close (<50m): Trust UWB heavily (alpha=0.8)
//   - Medium (50-100m): Linear transition (0.8 -> 0.5)
//   - Far (100-200m): Favor GPS (0.5 -> 0.3)
//   - Very far (>200m): GPS only (alpha=0.0)
float AP_Formation::calculate_adaptive_alpha(float range_m) const
{
    // Range zones for adaptive weighting
    const float CLOSE_RANGE = 50.0f;   // UWB optimal
    const float MID_RANGE = 100.0f;    // Transition zone
    const float FAR_RANGE = 200.0f;    // UWB max effective
    const float ALPHA_BASE = 0.8f;     // UWB weight when close

    if (range_m < CLOSE_RANGE) {
        // Close range - trust UWB heavily
        return ALPHA_BASE;
    } else if (range_m < MID_RANGE) {
        // Medium range - linear transition from 0.8 to 0.5
        float t = (range_m - CLOSE_RANGE) / (MID_RANGE - CLOSE_RANGE);
        return ALPHA_BASE - t * 0.3f;  // 0.8 -> 0.5
    } else if (range_m < FAR_RANGE) {
        // Far range - linear transition from 0.5 to 0.3
        float t = (range_m - MID_RANGE) / (FAR_RANGE - MID_RANGE);
        return 0.5f - t * 0.2f;  // 0.5 -> 0.3
    }

    // Beyond UWB range - GPS only
    return 0.0f;
}

// Calculate desired formation position with prediction
// Implements Phase 6 predictive control:
//   1. Predict where lead will be in FORM_PREDICT seconds
//   2. Apply formation offset (trail, lateral, vertical)
//   3. Return target position in NED frame
Vector3f AP_Formation::calculate_formation_position() const
{
    // STEP 1: Predict lead aircraft position
    // Compensates for system lag (sensor->processing->actuation)
    // Default: 1.0 second lookahead
    Vector3f predicted_pos = _target_pos + _target_vel * _predict_time;

    // STEP 2: Calculate formation offset
    // Note: For now using simple NED offset
    // TODO: Transform based on lead aircraft heading for proper body-frame offset
    Vector3f offset;
    offset.x = -_formation_offset;  // Negative = behind (north is forward)
    offset.y = _formation_lateral;  // Positive = right
    offset.z = -_formation_vertical; // Negative = above (down is positive in NED)

    // STEP 3: Formation position = predicted leader + offset
    return predicted_pos + offset;
}

// Calculate GPS-derived range to target (not used in current implementation)
// Kept for compatibility and future extensions
float AP_Formation::calculate_gps_range() const
{
    Vector3f relative_pos = _target_pos - _own_pos;
    return relative_pos.length();
}

// Apply exponential smoothing to range (not used - smoothing done in update())
// Kept for compatibility and future extensions
void AP_Formation::update_smooth_range(float raw_range)
{
    if (!_smooth_range_initialized) {
        _smooth_range = raw_range;
        _smooth_range_initialized = true;
        return;
    }

    float alpha = constrain_float(_smooth_alpha, 0.1f, 1.0f);
    _smooth_range = alpha * raw_range + (1.0f - alpha) * _smooth_range;
    _smooth_range = constrain_float(_smooth_range, 0.0f, 1000.0f);
}

// Calculate velocity command (not used - velocity calculated in update())
// Kept for compatibility and future extensions
Vector3f AP_Formation::calculate_velocity_command(float dt) const
{
    Vector3f position_error = _predicted_target_pos - _own_pos;
    float error_distance = position_error.length();

    if (error_distance < 0.1f) {
        return _target_vel;
    }

    Vector3f direction = position_error / error_distance;
    float proportional_speed = error_distance * 0.5f;

    return direction * proportional_speed;
}

// Calculate target speed (not used - speed calculated in update())
// Kept for compatibility and future extensions
float AP_Formation::calculate_target_speed(float range_m) const
{
    if (range_m > _speed_threshold) {
        return _approach_speed;
    }
    return _maintain_speed;
}
