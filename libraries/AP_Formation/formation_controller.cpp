/*
   FlightDock Formation Controller Implementation
   
   This implements the proven Phase 7A control logic in C++ for 400 Hz operation.
   Key features:
   - Graduated speed control (range/10 rule)
   - Predictive waypoint placement (0.5s ahead)
   - Smooth phase transitions (INTERCEPT → TRANSITION → FORMATION)
*/

#include "formation_controller.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

// Constructor
FormationController::FormationController() :
    _active(false),
    _last_update_ms(0),
    _lead_lat(0),
    _lead_lon(0),
    _lead_alt(0),
    _lead_vn(0),
    _lead_ve(0),
    _lead_vd(0),
    _lead_valid(false),
    _lead_last_update_ms(0),
    _range_to_lead(0),
    _closure_rate(0),
    _last_range(0),
    _last_range_ms(0),
    _desired_airspeed(LEAD_SPEED),
    _uwb_range(0),
    _uwb_valid(false),
    _uwb_last_update_ms(0),
    _fused_range(0),
    _fused_range_initialized(false)
{
}

// Initialize controller
void FormationController::init() {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FormationController: Initialized");
    _active = false;
    _lead_valid = false;
}

// Set lead aircraft state from MAVLink GLOBAL_POSITION_INT message
void FormationController::set_lead_state(float lat, float lon, float alt,
                                         float vn, float ve, float vd) {
    _lead_lat = lat;
    _lead_lon = lon;
    _lead_alt = alt;
    _lead_vn = vn;
    _lead_ve = ve;
    _lead_vd = vd;
    _lead_valid = true;
    _lead_last_update_ms = AP_HAL::millis();
}

// Main 400 Hz update function
void FormationController::update(const Location &current_loc,
                                 const Vector3f &current_vel,
                                 float current_heading) {
    if (!_active || !_lead_valid) {
        return;
    }
    
    uint32_t now_ms = AP_HAL::millis();
    float dt = (now_ms - _last_update_ms) * 0.001f; // Convert to seconds
    
    // Check for lead aircraft timeout (2 seconds)
    if (now_ms - _lead_last_update_ms > 2000) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FormationController: Lead timeout");
        _lead_valid = false;
        return;
    }
    
    // Calculate formation waypoint (with 0.5s prediction)
    _formation_waypoint = calculate_formation_waypoint();
    
    // Calculate range to lead aircraft
    Location lead_loc(
        static_cast<int32_t>(_lead_lat * 1e7),
        static_cast<int32_t>(_lead_lon * 1e7),
        static_cast<int32_t>(_lead_alt * 100),
        Location::AltFrame::ABSOLUTE
    );
    
    // Calculate GPS-only range
    float gps_range = calculate_range(current_loc, lead_loc);
    
    // Fuse with UWB if available
    update_fused_range(gps_range);
    
    // Use fused range for control
    _range_to_lead = _fused_range;
    
    // Calculate closure rate
    _closure_rate = calculate_closure_rate(_range_to_lead, dt);
    
    // Calculate desired airspeed based on range (graduated speed control)
    _desired_airspeed = calculate_desired_speed(_range_to_lead);
    
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
    
    // Closure rate = (previous_range - current_range) / dt
    // Positive = approaching, negative = separating
    float closure = (_last_range - current_range) / dt;
    
    _last_range = current_range;
    _last_range_ms = AP_HAL::millis();
    
    return closure;
}

// Calculate desired airspeed using graduated speed control
// This implements the proven Phase 7A three-phase speed profile
float FormationController::calculate_desired_speed(float range_to_lead) {
    float desired_speed;
    
    if (range_to_lead > INTERCEPT_THRESHOLD) {
        // INTERCEPT PHASE: Use range/10 rule for graduated closure
        float target_closure = range_to_lead / 10.0f;
        target_closure = constrain_float(target_closure, MIN_CLOSURE_RATE, MAX_CLOSURE_RATE);
        desired_speed = LEAD_SPEED + target_closure;
        desired_speed = MIN(desired_speed, MAX_APPROACH_SPEED);
        
    } else if (range_to_lead > TRANSITION_THRESHOLD) {
        // TRANSITION PHASE: Gentle deceleration
        float range_error = range_to_lead - FORMATION_OFFSET;
        float target_closure = range_error / 5.0f;  // 20m error → 4 m/s closure
        target_closure = MAX(target_closure, MIN_CLOSURE_RATE);
        desired_speed = LEAD_SPEED + target_closure;
        
    } else {
        // FORMATION PHASE: Match lead speed
        desired_speed = LEAD_SPEED;
    }
    
    return desired_speed;
}

// Calculate formation waypoint with predictive placement
// This implements the proven Phase 7A waypoint logic
Location FormationController::calculate_formation_waypoint() {
    // Predict lead's future position (0.5 seconds ahead)
    float predicted_north = _lead_vn * PREDICT_TIME;
    float predicted_east = _lead_ve * PREDICT_TIME;
    
    // Create location for lead's current position
    Location lead_current(
        static_cast<int32_t>(_lead_lat * 1e7),
        static_cast<int32_t>(_lead_lon * 1e7),
        static_cast<int32_t>(_lead_alt * 100),
        Location::AltFrame::ABSOLUTE
    );
    
    // Offset to predicted position
    // NOTE: This aims AT the lead aircraft (with prediction)
    // The trail distance is maintained by speed differential, not waypoint offset!
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
    if (range_m > 0 && range_m < UWB_MAX_RANGE) {
        _uwb_range = range_m;
        _uwb_valid = true;
        _uwb_last_update_ms = AP_HAL::millis();
    }
}

// Calculate adaptive alpha for GPS/UWB fusion
// Returns UWB weight: 0.8 @ <50m, 0.0 @ >200m
float FormationController::calculate_adaptive_alpha(float range) {
    if (range < 50.0f) {
        return 0.8f;  // Trust UWB 80% when close
    } else if (range < 100.0f) {
        // Linear transition 50m→100m: 0.8→0.5
        return 0.8f - (range - 50.0f) * 0.006f;
    } else if (range < UWB_MAX_RANGE) {
        // Linear transition 100m→200m: 0.5→0.0
        return 0.5f - (range - 100.0f) * 0.005f;
    } else {
        return 0.0f;  // GPS only when far
    }
}

// Fuse GPS and UWB range measurements
float FormationController::fuse_gps_uwb_range(float gps_range, float uwb_range) {
    // Check UWB timeout
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _uwb_last_update_ms > UWB_TIMEOUT_MS) {
        _uwb_valid = false;
    }
    
    // If no valid UWB, use GPS only
    if (!_uwb_valid) {
        return gps_range;
    }
    
    // Calculate adaptive alpha based on range
    float alpha = calculate_adaptive_alpha(gps_range);
    
    // Fuse: fused = alpha * uwb + (1-alpha) * gps
    float fused = alpha * uwb_range + (1.0f - alpha) * gps_range;
    
    return fused;
}

// Update fused range with smoothing
void FormationController::update_fused_range(float gps_range) {
    // Fuse GPS + UWB
    float raw_fused = fuse_gps_uwb_range(gps_range, _uwb_range);
    
    // Apply exponential smoothing
    if (!_fused_range_initialized) {
        _fused_range = raw_fused;
        _fused_range_initialized = true;
    } else {
        _fused_range = SMOOTH_ALPHA * raw_fused + (1.0f - SMOOTH_ALPHA) * _fused_range;
    }
}

