#include "AP_Camera_Tracking.h"

#if AP_CAMERA_TRACKING_ENABLED

extern const AP_HAL::HAL& hal;

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
               return false;
            }
            break;
        case TrackingType::TRK_RECTANGLE:
            if (!(_cam_info.flags & CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE)) {
               return false;
            }
            break;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Tracking: New Tracking request");

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
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"sent message to device sysid %d and comp %d",tracking_device_sysid,tracking_device_compid);
    return true;
}

#endif // AP_CAMERA_TRACKING_ENABLED