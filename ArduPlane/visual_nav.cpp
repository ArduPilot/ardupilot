#include "Plane.h"

// Initialize visual navigation system (no GPS)
void Plane::init_visual_nav()
{
    // Initialize home at (0, 0) with barometer altitude
    visual_nav.home_without_gps.lat = 0;
    visual_nav.home_without_gps.lng = 0;
    visual_nav.home_without_gps.alt = (int32_t)barometer.get_altitude() * 100; // convert to cm
    
    visual_nav.home_set_from_gcs = false;
    visual_nav.last_origin_set_ms = millis();
    visual_nav.origin_offset_ne = Vector2f(0, 0);
    
    // Set home in EKF
    if (!ahrs.set_home(visual_nav.home_without_gps)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to set home in EKF");
        return;
    }
    
    gcs().send_text(MAV_SEVERITY_INFO, "Visual Navigation Init: Home (0,0) at alt %.1fm", 
                    visual_nav.home_without_gps.alt / 100.0f);
}

// Handle "Set EKF Origin Here" command from GCS
void Plane::handle_set_ekf_origin(const Location &origin_loc)
{
    gcs().send_text(MAV_SEVERITY_INFO, "DEBUG: handle_set_ekf_origin called, initialised=%u lat=%ld lng=%ld alt=%ld",
                    origin_loc.initialised(), origin_loc.lat, origin_loc.lng, origin_loc.alt);
    
    if (!origin_loc.initialised()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Invalid origin location");
        return;
    }
    
    // Update home location
    visual_nav.home_without_gps = origin_loc;
    visual_nav.home_set_from_gcs = true;
    visual_nav.last_origin_set_ms = millis();
    
    // Update EKF home
    if (!ahrs.set_home(visual_nav.home_without_gps)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to set EKF home");
        return;
    }

    
    gcs().send_text(MAV_SEVERITY_INFO, "Visual Nav updated: home_set_from_gcs=%d", (int)visual_nav.home_set_from_gcs);
    gcs().send_text(MAV_SEVERITY_INFO, "EKF Origin Set: %.6f, %.6f, alt %.1fm", 
                    origin_loc.lat / 1e7f,
                    origin_loc.lng / 1e7f,
                    origin_loc.alt / 100.0f);
}

// Update visual navigation position
void Plane::update_visual_nav_position()
{
    if (!visual_nav.home_without_gps.initialised()) {
        return;
    }
    
    Location current;
    if (!ahrs.get_location(current)) {
        return;
    }
    // Calculate offset from home using get_distance_NE
    Vector2f offset = current.get_distance_NE(visual_nav.home_without_gps);
    visual_nav.origin_offset_ne = offset;
}

Location Plane::get_visual_nav_location() const
{
    if (!visual_nav.home_without_gps.initialised()) {
        return Location();  // Return invalid location
    }

    Location result = visual_nav.home_without_gps;
    result.offset(visual_nav.origin_offset_ne.x, visual_nav.origin_offset_ne.y);
    
    // Also update altitude from current AHRS location
    Location current;
    if (ahrs.get_location(current)) {
        result.alt = current.alt;
    }

    return result;
}
// Check if using visual navigation (no GPS)
bool Plane::is_visual_nav_mode() const
{
    return !gps.status() && visual_nav.home_without_gps.initialised();
}
