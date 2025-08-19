#include "AP_Camera_Tracking.h"

#if AP_CAMERA_OFFBOARD_TRACKING_ENABLED

#include <AP_Mount/AP_Mount.h>
#include <AP_Follow/AP_Follow.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Camera/AP_Camera.h>
extern const AP_HAL::HAL& hal;

void AP_Camera_Tracking::init() {
    if (!hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&AP_Camera_Tracking::convert_poi_to_follow_target_message_thread, void),
        "cam_follow_int",
        4096, 
        AP_HAL::Scheduler::PRIORITY_IO, 
        -1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Camera: failed to start follow integration thread");
    }
}

// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only then top_left is the point
// top_left,bottom_right are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Camera_Tracking::set_tracking(TrackingType tracking_type, const Vector2f& top_left, const Vector2f& bottom_right, uint8_t tracking_device_sysid, uint8_t tracking_device_compid, mavlink_camera_information_t _cam_info)
{
    // if we don't support the required tracking then return
    switch (tracking_type) {
        case TrackingType::TRK_NONE:
            break;
        case TrackingType::TRK_POINT:
            if (!(_cam_info.flags & CAMERA_CAP_FLAGS_HAS_TRACKING_POINT)) {
                gcs().send_text(MAV_SEVERITY_INFO, "Camera Doesn't support Track point, see camera cap flags");
                return false;
            }
            break;
        case TrackingType::TRK_RECTANGLE:
            if (!(_cam_info.flags & CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE)) {
                gcs().send_text(MAV_SEVERITY_INFO, "Camera Doesn't support Track rectangle, see camera cap flags");
                return false;
            }
            break;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"AP_Camera: New Tracking request from sysid %d and comp %d", tracking_device_sysid,tracking_device_compid);

    if (_link == nullptr) {
        uint8_t proxy_device_compid = tracking_device_compid;
        uint8_t proxy_device_sysid {};
        _link = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_ONBOARD_CONTROLLER, proxy_device_compid, proxy_device_sysid);
        if (proxy_device_sysid != tracking_device_sysid) {
            // means the tracking device sysid we found is not same as what we declared through parameters
            _link = nullptr;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"AP_Camera: Found Controller but its different from the declared one in parameters");
            return false;
        }
        if (_link == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"AP_Camera: Could Not find any onboard controller registered");
            return false;
        }
    }

    // prepare and send message
    mavlink_command_long_t pkt = {
        0,                          // param1
        0,                          // param2
        0,                          // param3
        0,                          // param4
        0,                          // param5
        0,                          // param6
        0,                          // param7
        0,                          // command
        tracking_device_sysid,      // target_system
        tracking_device_compid,     // target_component
        0,                          // confirmation
    };

    switch (tracking_type) {
        case TrackingType::TRK_POINT:
            pkt.command = MAV_CMD_CAMERA_TRACK_POINT;
            pkt.param1 = top_left.x;
            pkt.param2 = top_left.y;
            break;
        case TrackingType::TRK_RECTANGLE:
            pkt.command = MAV_CMD_CAMERA_TRACK_RECTANGLE;
            pkt.param1 = top_left.x;
            pkt.param2 = top_left.y;
            pkt.param3 = bottom_right.x;
            pkt.param4 = bottom_right.y;
            break;
        case TrackingType::TRK_NONE:
            pkt.command = MAV_CMD_CAMERA_STOP_TRACKING;
            break;
        default:
            // Unknown
            return false;
    }

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    // Store tracking state for follow integration
    _tracking_active = (tracking_type != TrackingType::TRK_NONE);
    _tracking_type = tracking_type;
    _last_tracking_update_ms = AP_HAL::millis();

    // If follow system is enabled with object follow option, start generating target messages
    AP_Follow *follow = AP_Follow::get_singleton();
    if (follow != nullptr && follow->enabled() && follow->option_is_enabled(AP_Follow::Option::OBJECT_FOLLOW_ON_ENTER)) { 
        if (_tracking_active) {
            // Start internal target message generation
            _generate_follow_messages = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_Camera_Tracking: Tracking and Follow Active");
        } else {
            // Stop internal target message generation
            _generate_follow_messages = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_Camera_Tracking: Tracking and Follow Deactivated ");
        }
    }

    return true;
}

// Call this function periodically (e.g., from update loop) to generate follow target messages
void AP_Camera_Tracking::convert_poi_to_follow_target_message_thread()
{
    while (true) {
        // Run at 10Hz
        hal.scheduler->delay(AP_Follow::get_singleton()->get_poi_delay());

        if (!_generate_follow_messages || !_tracking_active) {
            continue;
        }

        uint32_t now_ms = AP_HAL::millis();

        // Get POI location from mount if available
        uint8_t mount_instance = 0;
        Quaternion quat;
        Location vehicle_loc;
        Location poi_loc;
        
        AP_Mount *mount = AP_Mount::get_singleton();
        if (mount == nullptr) {
            continue;
        }

        if (!mount->get_poi(mount_instance, quat, vehicle_loc, poi_loc)) {
            continue;
        }

        // Check if object is still being tracked and visible
        if (!AP_Camera::get_singleton()->is_tracking_object_visible(0)) {
            continue;
        }

        AP_Follow *follow = AP_Follow::get_singleton();
        if (follow == nullptr) {
            continue;
        }

        // Option 2: Generate FOLLOW_TARGET message (more comprehensive)
        generate_follow_target_message(poi_loc, now_ms);
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_Camera: GENERATED A FOLLOW MESSAGE");
    }
}

// Alternative: Generate a FOLLOW_TARGET message for the tracked object
void AP_Camera_Tracking::generate_follow_target_message(const Location &poi_loc, uint32_t timestamp_ms)
{
    mavlink_message_t msg;
    mavlink_follow_target_t packet;

    packet.timestamp = timestamp_ms * 1000; // Convert to microseconds
    packet.custom_state = 0;
    packet.lat = poi_loc.lat;
    packet.lon = poi_loc.lng;
    int32_t altitude;
    if (!poi_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,altitude)) {
        // No use
    };
    packet.alt =  altitude * 0.01f; // Convert cm to meters

    // To test this code with gazebo center uncomment (This is from the location CMAC), Put a object on gazebo center of of frame
    // Location ref(-353632619,1491652374,0,Location::AltFrame::ABOVE_HOME);
    // ref.change_alt_frame(poi_loc.get_alt_frame());
    // gcs().send_text(MAV_SEVERITY_WARNING, "The distance is %f, poi_lat= %d, poi_lng=%d", ref.get_distance(poi_loc), poi_loc.lat, poi_loc.lng);

    // Estimate velocity from position changes
    if (_last_poi_location_valid && (timestamp_ms > _last_poi_timestamp_ms)) {
        float dt = (timestamp_ms - _last_poi_timestamp_ms) * 0.001f;
        if (dt > 0.05f && dt < 2.0f) { // reasonable time delta
            // Calculate velocity using get_distance_NE which returns Vector2f in meters
            Vector2f distance_ne = _last_poi_location.get_distance_NE(poi_loc);
            
            packet.vel[0] = distance_ne.x / dt; // North velocity in m/s
            packet.vel[1] = distance_ne.y / dt; // East velocity in m/s
            packet.vel[2] = (poi_loc.alt - _last_poi_location.alt) * 0.01f / dt; // Down velocity in m/s (convert cm to m)
        } else {
            packet.vel[0] = 0;
            packet.vel[1] = 0;
            packet.vel[2] = 0;
        }
    } else {
        packet.vel[0] = 0;
        packet.vel[1] = 0;
        packet.vel[2] = 0;
    }

    // No acceleration data available
    packet.acc[0] = 0;
    packet.acc[1] = 0;
    packet.acc[2] = 0;

    // No attitude data available
    packet.attitude_q[0] = 1.0f; // w
    packet.attitude_q[1] = 0.0f; // x
    packet.attitude_q[2] = 0.0f; // y
    packet.attitude_q[3] = 0.0f; // z

    packet.rates[0] = 0;
    packet.rates[1] = 0;
    packet.rates[2] = 0;

    packet.position_cov[0] = 1.0f; // Position covariance
    packet.position_cov[1] = 1.0f;
    packet.position_cov[2] = 1.0f;

    // Set estimation capabilities: position and velocity available
    packet.est_capabilities = (1<<0) | (1<<1); // Position and velocity

    // Store current location for next calculation
    _last_poi_location = poi_loc;
    _last_poi_timestamp_ms = timestamp_ms;
    _last_poi_location_valid = true;

    // Create mavlink message
    const uint8_t object_sysid = AP_Follow::get_singleton()->get_target_sysid();
    msg.sysid = object_sysid;
    msg.compid = 255;
    msg.msgid = MAVLINK_MSG_ID_FOLLOW_TARGET;

    mavlink_msg_follow_target_encode(msg.sysid, msg.compid, &msg, &packet);

    // Send to follow system
    AP_Follow *follow = AP_Follow::get_singleton();
    if (follow != nullptr) {
        if (follow->get_target_sysid() != object_sysid) {
            follow->set_target_sysid(object_sysid);
        }
        follow->handle_msg(msg);
    }
}

#endif // AP_CAMERA_OFFBOARD_TRACKING_ENABLED