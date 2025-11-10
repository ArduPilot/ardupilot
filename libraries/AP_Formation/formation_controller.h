/*
   FlightDock Formation Controller
   
   Minimal formation flight controller integrated directly into ArduPlane.
   Implements graduated speed control with range/10 rule for smooth intercept.
   
   Author: FlightDock Research Team
   Date: November 2025
   Version: 7B-Mini (Minimal Integration)
*/

#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

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
    float get_fused_range() { return _fused_range; }
    
    // Check if UWB is valid
    bool is_uwb_valid() { return _uwb_valid; }
    
    // Get formation waypoint command
    Location get_formation_waypoint();
    
    // Get desired airspeed command
    float get_desired_airspeed();
    
    // Check if formation mode is active
    bool is_active() { return _active; }
    
    // Activate/deactivate formation mode
    void set_active(bool active) { _active = active; }
    
    // Get diagnostic info
    float get_range_to_lead() { return _range_to_lead; }
    float get_closure_rate() { return _closure_rate; }
    float get_formation_distance_error() { return _range_to_lead; }

private:
    // Formation parameters (from Phase 7A proven values)
    const float FORMATION_OFFSET = 50.0f;      // Trail distance (m)
    const float FORMATION_TOLERANCE = 5.0f;    // Acceptable error (m)
    const float LEAD_SPEED = 24.7f;            // Lead aircraft speed (m/s)
    const float PREDICT_TIME = 0.5f;           // Prediction horizon (s)
    
    // Speed control thresholds (graduated speed reduction)
    const float INTERCEPT_THRESHOLD = 70.0f;   // Range for intercept phase
    const float TRANSITION_THRESHOLD = 50.0f;  // Range for transition phase
    
    // Speed limits
    const float MAX_APPROACH_SPEED = 26.0f;    // Maximum approach speed (match FORM_APPROACH)
    const float MIN_CLOSURE_RATE = 0.5f;       // Minimum closure rate
    const float MAX_CLOSURE_RATE = 8.0f;       // Maximum closure rate
    
    // State variables
    bool _active;
    uint32_t _last_update_ms;
    
    // Lead aircraft state
    float _lead_lat;
    float _lead_lon;
    float _lead_alt;
    float _lead_vn;  // North velocity (m/s)
    float _lead_ve;  // East velocity (m/s)
    float _lead_vd;  // Down velocity (m/s)
    bool _lead_valid;
    uint32_t _lead_last_update_ms;
    
    // Formation state
    float _range_to_lead;
    float _closure_rate;
    float _last_range;
    uint32_t _last_range_ms;
    
    // Calculated outputs
    Location _formation_waypoint;
    float _desired_airspeed;
    
    // Internal calculation functions
    float calculate_range(const Location &loc1, const Location &loc2);
    float calculate_closure_rate(float current_range, float dt);
    float calculate_desired_speed(float range_to_lead);
    Location calculate_formation_waypoint();

    // UWB sensor fusion (from AP_Formation)
    float _uwb_range;
    bool _uwb_valid;
    uint32_t _uwb_last_update_ms;
    
    // Fused range estimate (GPS+UWB)
    float _fused_range;
    bool _fused_range_initialized;
    
    // Sensor fusion parameters
    const float UWB_TIMEOUT_MS = 500.0f;    // 500ms UWB timeout
    const float UWB_MAX_RANGE = 200.0f;     // Max effective UWB range
    const float SMOOTH_ALPHA = 0.3f;        // Smoothing filter coefficient
    
    // Additional private functions
    float calculate_adaptive_alpha(float range);
    float fuse_gps_uwb_range(float gps_range, float uwb_range);
    void update_fused_range(float gps_range);
};


