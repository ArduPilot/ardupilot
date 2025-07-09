#include "Copter.h"
#include "mode.h"

#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h> // for MAV_SEVERITY macros

#if MODE_KILOMETER_ENABLED

ModeKilometer::ModeKilometer() :
    current_state(KilometerState::IDLE),
    start_yaw_deg(0.0f),
    mission_active(false),
    state_start_time_ms(0)
{
}

bool ModeKilometer::init(bool ignore_checks)
{
    // Call parent init function first
    if (!ModeGuided::init(ignore_checks)) {
        return false;
    }
    
    // Initialize state
    current_state = KilometerState::IDLE;
    mission_active = false;
    state_start_time_ms = AP_HAL::millis();
    
    // Store starting position and yaw
    start_position = copter.current_loc;
    start_yaw_deg = degrees(copter.ahrs.get_yaw());
    
    gcs().send_text(MAV_SEVERITY_INFO, "Kilometer Mode: Initialized - Ready for mission");
    gcs().send_text(MAV_SEVERITY_INFO, "Start yaw: %.1f deg", start_yaw_deg);
    
    // Automatically start the mission
    return start_kilometer_mission();
}

void ModeKilometer::exit()
{
    // Cleanup and send final status
    mission_active = false;
    current_state = KilometerState::IDLE;
    
    gcs().send_text(MAV_SEVERITY_INFO, "Kilometer Mode: Exited");
    
    // Call parent exit
    ModeGuided::exit();
}

void ModeKilometer::run()
{
    // Handle state machine
    switch (current_state) {
        case KilometerState::IDLE:
            handle_state_idle();
            break;
        case KilometerState::GOING_FORWARD:
            handle_state_going_forward();
            break;
        case KilometerState::GOING_BACK:
            handle_state_going_back();
            break;
        case KilometerState::COMPLETED:
            handle_state_completed();
            break;
        case KilometerState::ABORTED:
            handle_state_aborted();
            break;
    }
    
    // Call parent run method to handle guided mode functionality
    ModeGuided::run();
    
    // Send periodic status updates
    static uint32_t last_status_ms = 0;
    if (AP_HAL::millis() - last_status_ms > 2000) { // Every 2 seconds
        send_mission_status();
        last_status_ms = AP_HAL::millis();
    }
}

bool ModeKilometer::start_kilometer_mission()
{
    if (mission_active) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Kilometer Mode: Mission already active");
        return false;
    }
    
    // Calculate forward position
    calculate_forward_position();
    
    // Start mission
    mission_active = true;
    transition_to_state(KilometerState::GOING_FORWARD);
    
    gcs().send_text(MAV_SEVERITY_INFO, "Kilometer Mode: Mission started");
    gcs().send_text(MAV_SEVERITY_INFO, "Forward target: %d, %d, %d", 
                   forward_position.lat, forward_position.lng, forward_position.alt);
    
    return true;
}

void ModeKilometer::abort_mission()
{
    if (!mission_active) {
        return;
    }
    
    gcs().send_text(MAV_SEVERITY_WARNING, "Kilometer Mode: Mission aborted");
    transition_to_state(KilometerState::ABORTED);
}

void ModeKilometer::calculate_forward_position()
{
    // Calculate position 1km forward in current yaw direction
    float yaw_rad = radians(start_yaw_deg);
    
    // Calculate offset in NED coordinates
    Vector3f offset_ned;
    offset_ned.x = KILOMETER_DISTANCE_M * cosf(yaw_rad); // North
    offset_ned.y = KILOMETER_DISTANCE_M * sinf(yaw_rad); // East
    offset_ned.z = 0.0f; // Same altitude
    

    forward_position = start_position;
    forward_position.offset(offset_ned.x, offset_ned.y);
}

bool ModeKilometer::is_at_target_position(const Location& target)
{
    // Check horizontal distance
    float distance_m = copter.current_loc.get_distance(target);
    if (distance_m > WAYPOINT_RADIUS_M) {
        return false;
    }
    
    // Check altitude difference
    float alt_diff_m = abs(copter.current_loc.alt - target.alt) * 0.01f; // Convert cm to m
    if (alt_diff_m > WAYPOINT_RADIUS_M) {
        return false;
    }
    
    return true;
}

void ModeKilometer::transition_to_state(KilometerState new_state)
{
    if (current_state == new_state) {
        return;
    }
    
    current_state = new_state;
    state_start_time_ms = AP_HAL::millis();
    
    // Log state transitions
    const char* state_names[] = {"IDLE", "GOING_FORWARD", "GOING_BACK", "COMPLETED", "ABORTED"};
    gcs().send_text(MAV_SEVERITY_INFO, "Kilometer Mode: State -> %s", state_names[(int)new_state]);
}

void ModeKilometer::handle_state_idle()
{
    // Do nothing, wait for mission start
}

void ModeKilometer::handle_state_going_forward()
{
    if (!mission_active) {
        return;
    }
    
    // Set destination to forward position using guided mode functionality
    if (!set_destination(forward_position)) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Kilometer Mode: Failed to set forward destination");
        transition_to_state(KilometerState::ABORTED);
        return;
    }
    
    // Check if we've reached the forward position
    if (is_at_target_position(forward_position)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Kilometer Mode: Reached forward position, returning home");
        transition_to_state(KilometerState::GOING_BACK);
    }
}

void ModeKilometer::handle_state_going_back()
{
    if (!mission_active) {
        return;
    }
    
    // Set destination to start position using guided mode functionality
    if (!set_destination(start_position)) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Kilometer Mode: Failed to set return destination");
        transition_to_state(KilometerState::ABORTED);
        return;
    }
    
    // Check if we've reached the start position
    if (is_at_target_position(start_position)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Kilometer Mode: Returned to start position");
        //transition_to_state(KilometerState::COMPLETED);
        transition_to_state(KilometerState::GOING_FORWARD);
    }
}

void ModeKilometer::handle_state_completed()
{
    mission_active = false;
    
    // Hold position at start location
    if (!set_destination(start_position)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Kilometer Mode: Failed to hold position");
    }
    
    gcs().send_text(MAV_SEVERITY_INFO, "Kilometer Mode: Mission completed successfully");
}

void ModeKilometer::handle_state_aborted()
{
    mission_active = false;
    
    // Hold current position
    Location current_pos = copter.current_loc;
    if (!set_destination(current_pos)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Kilometer Mode: Failed to hold position after abort");
    }
    
    gcs().send_text(MAV_SEVERITY_WARNING, "Kilometer Mode: Mission aborted - holding position");
}

float ModeKilometer::get_distance_to_target()
{
    switch (current_state) {
        case KilometerState::GOING_FORWARD:
            return copter.current_loc.get_distance(forward_position);
        case KilometerState::GOING_BACK:
            return copter.current_loc.get_distance(start_position);
        default:
            return 0.0f;
    }
}

void ModeKilometer::send_mission_status()
{
    if (!mission_active) {
        return;
    }
    
    float distance_to_target = get_distance_to_target();
    uint32_t elapsed_time_s = (AP_HAL::millis() - state_start_time_ms) / 1000;
    
    const char* state_names[] = {"IDLE", "FORWARD", "BACK", "COMPLETE", "ABORT"};
    gcs().send_text(MAV_SEVERITY_INFO, "KM Status: %s, Dist: %.1fm, Time: %us", 
                   state_names[(int)current_state], distance_to_target, elapsed_time_s);
}


const AP_Param::GroupInfo ModeKilometer::var_info[] = {
    AP_GROUPEND
};

#endif