#include "AP_Mount_Xacti.h"

#if HAL_MOUNT_XACTI_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "Mount"
#define XACTI_PARAM_SINGLESHOT "SingleShot"
#define XACTI_PARAM_RECORDING "Recording"
#define XACTI_PARAM_FOCUSMODE "FocusMode"

#define AP_MOUNT_XACTI_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_XACTI_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xacti: " fmt, ## args); } } while (0)

bool AP_Mount_Xacti::_subscribed = false;
AP_Mount_Xacti::DetectedModules AP_Mount_Xacti::_detected_modules[];
HAL_Semaphore AP_Mount_Xacti::_sem_registry;

// Constructor
AP_Mount_Xacti::AP_Mount_Xacti(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance)
{
    register_backend();

    param_int_cb = FUNCTOR_BIND_MEMBER(&AP_Mount_Xacti::handle_param_get_set_response_int, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    param_save_cb = FUNCTOR_BIND_MEMBER(&AP_Mount_Xacti::handle_param_save_response, void, AP_DroneCAN*, const uint8_t, bool);
}

// init - performs any required initialisation for this instance
void AP_Mount_Xacti::init()
{
    _initialised = true;
}

// update mount position - should be called periodically
void AP_Mount_Xacti::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // periodically send copter attitude and GPS status
    send_copter_att_status();

    // update based on mount mode
    switch (get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &angle_bf_target = _params.retract_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            mnt_target.target_type = MountTargetType::ANGLE;
            mnt_target.angle_rad.set(angle_bf_target*DEG_TO_RAD, false);
            break;
        }

        case MAV_MOUNT_MODE_MAVLINK_TARGETING: {
            // mavlink targets are set while handling the incoming message
            break;
        }

        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's RC inputs
            MountTarget rc_target;
            get_rc_target(mnt_target.target_type, rc_target);
            switch (mnt_target.target_type) {
            case MountTargetType::ANGLE:
                mnt_target.angle_rad = rc_target;
                break;
            case MountTargetType::RATE:
                mnt_target.rate_rads = rc_target;
                break;
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (get_angle_target_to_roi(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to Home location
        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        // point mount to another vehicle
        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (get_angle_target_to_sysid(mnt_target.angle_rad)) {
                mnt_target.target_type = MountTargetType::ANGLE;
            }
            break;

        default:
            // we do not know this mode so raise internal error
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }

    // send target angles or rates depending on the target type
    switch (mnt_target.target_type) {
        case MountTargetType::ANGLE:
            send_target_angles(mnt_target.angle_rad.pitch, mnt_target.angle_rad.yaw, mnt_target.angle_rad.yaw_is_ef);
            break;
        case MountTargetType::RATE:
            send_target_rates(mnt_target.rate_rads.pitch, mnt_target.rate_rads.yaw, mnt_target.rate_rads.yaw_is_ef);
            break;
    }
}

// return true if healthy
bool AP_Mount_Xacti::healthy() const
{
    // unhealthy until gimbal has been found and replied with firmware version info
    if (!_initialised) {
        return false;
    }

    // unhealthy if attitude information NOT received recently
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_attitude_quat_ms > 1000) {
        return false;
    }

    // if we get this far return healthy
    return true;
}

// take a picture.  returns true on success
bool AP_Mount_Xacti::take_picture()
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    // set SingleShot parameter
    return _detected_modules[_instance].ap_dronecan->set_parameter_on_node(_detected_modules[_instance].node_id, XACTI_PARAM_SINGLESHOT, 0, &param_int_cb);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_Xacti::record_video(bool start_recording)
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    // set Recording parameter
    return _detected_modules[_instance].ap_dronecan->set_parameter_on_node(_detected_modules[_instance].node_id, XACTI_PARAM_RECORDING, start_recording ? 1 : 0, &param_int_cb);
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Mount_Xacti::set_focus(FocusType focus_type, float focus_value)
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return SetFocusResult::FAILED;
    }

    // convert focus type and value to parameter value
    uint8_t focus_param_value;
    switch (focus_type) {
    case FocusType::RATE:
    case FocusType::PCT:
        // focus rate and percentage control not supported so simply switch to manual focus
        // FocusMode of 0:Manual Focus
        focus_param_value = 0;
        break;
    case FocusType::AUTO:
        // FocusMode of 1:Single AutoFocus, 2:Continuous AutoFocus
        focus_param_value = 2;
        break;
    default:
        // unsupported forucs mode
        return SetFocusResult::INVALID_PARAMETERS;
    }

    // set FocusMode parameter
    if (!_detected_modules[_instance].ap_dronecan->set_parameter_on_node(_detected_modules[_instance].node_id, XACTI_PARAM_FOCUSMODE, focus_param_value, &param_int_cb)) {
        return SetFocusResult::FAILED;
    }
    return SetFocusResult::ACCEPTED;
}

// send camera information message to GCS
void AP_Mount_Xacti::send_camera_information(mavlink_channel_t chan) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    static const uint8_t vendor_name[32] = "Xacti";
    static uint8_t model_name[32] = "CX-GB100";
    const char cam_definition_uri[140] {};
    const float NaN = nanf("0x4152");

    // capability flags
    const uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
                           CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                           CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS;

    // send CAMERA_INFORMATION message
    mavlink_msg_camera_information_send(
        chan,
        AP_HAL::millis(),       // time_boot_ms
        vendor_name,            // vendor_name uint8_t[32]
        model_name,             // model_name uint8_t[32]
        0,                      // firmware version uint32_t
        NaN,                    // focal_length float (mm)
        NaN,                    // sensor_size_h float (mm)
        NaN,                    // sensor_size_v float (mm)
        0,                      // resolution_h uint16_t (pix)
        0,                      // resolution_v uint16_t (pix)
        0,                      // lens_id uint8_t
        flags,                  // flags uint32_t (CAMERA_CAP_FLAGS)
        0,                      // cam_definition_version uint16_t
        cam_definition_uri,     // cam_definition_uri char[140]
        _instance + 1);         // gimbal_device_id uint8_t
}

// send camera settings message to GCS
void AP_Mount_Xacti::send_camera_settings(mavlink_channel_t chan) const
{
    const float NaN = nanf("0x4152");

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        _recording_video ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE,   // camera mode (0:image, 1:video, 2:image survey)
        0,                  // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaN);               // focusLevel float, percentage from 0 to 100, NaN if unknown
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Xacti::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat = _current_attitude_quat;
    return true;
}

// send target pitch and yaw rates to gimbal
// yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
void AP_Mount_Xacti::send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef)
{
    // send gimbal rate target to gimbal
    send_gimbal_control(3, degrees(pitch_rads) * 100, degrees(yaw_rads) * 100);
}

// send target pitch and yaw angles to gimbal
// yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
void AP_Mount_Xacti::send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef)
{
    // convert yaw to body frame
    const float yaw_bf_rad = yaw_is_ef ? wrap_PI(yaw_rad - AP::ahrs().yaw) : yaw_rad;

    // send angle target to gimbal
    send_gimbal_control(2, degrees(pitch_rad) * 100, degrees(yaw_bf_rad) * 100);
}

// subscribe to Xacti DroneCAN messages
void AP_Mount_Xacti::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    // return immediately if DroneCAN is unavailable
    if (ap_dronecan == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Xacti: DroneCAN subscribe failed");
        return;
    }

    _subscribed = true;

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_gimbal_attitude_status, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("gimbal_attitude_status_sub");
        _subscribed = false;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_gnss_status_req, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("gnss_status_req_sub");
        _subscribed = false;
    }
}

// register backend in detected modules array used to map DroneCAN port and node id to backend
void AP_Mount_Xacti::register_backend()
{
    WITH_SEMAPHORE(_sem_registry);

    // add this backend to _detected_modules array
    _detected_modules[_instance].driver = this;

    // return if devid is zero meaning this backend has not yet been associated with a mount
    const uint32_t devid = (uint32_t)_params.dev_id.get();
    if (devid == 0) {
        return;
    }

    // get DroneCan port from device id
    const uint8_t can_driver_index = AP_HAL::Device::devid_get_bus(devid);
    const uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_DroneCAN *ap_dronecan = AP_DroneCAN::get_dronecan(i);
        if (ap_dronecan != nullptr && ap_dronecan->get_driver_index() == can_driver_index) {
            _detected_modules[_instance].ap_dronecan = ap_dronecan;
        }
    }

    // get node_id from device id
    _detected_modules[_instance].node_id = AP_HAL::Device::devid_get_address(devid);
}

// find backend associated with the given dronecan port and node_id.  also associates backends with zero node ids
// returns pointer to backend on success, nullptr on failure
AP_Mount_Xacti* AP_Mount_Xacti::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id)
{
    WITH_SEMAPHORE(_sem_registry);

    // exit immediately if DroneCAN is unavailable or invalid node id
    if (ap_dronecan == nullptr || node_id == 0) {
        return nullptr;
    }

    // search for backend with matching dronecan port and node id
    for (uint8_t i = 0; i < ARRAY_SIZE(_detected_modules); i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_dronecan == ap_dronecan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
        }
    }

    // if we got this far, this dronecan port and node id are not associated with any backend
    // associate with first backend with node id of zero
    for (uint8_t i = 0; i < ARRAY_SIZE(_detected_modules); i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].node_id == 0) {
                _detected_modules[i].ap_dronecan = ap_dronecan;
                _detected_modules[i].node_id = node_id;
                const auto dev_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                                ap_dronecan->get_driver_index(),
                                                                node_id, 0);
                _detected_modules[i].driver->set_dev_id(dev_id);
                return _detected_modules[i].driver;
        }
    }

    return nullptr;
}

// handle xacti gimbal attitude status message
void AP_Mount_Xacti::handle_gimbal_attitude_status(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GimbalAttitudeStatus &msg)
{
    // fetch the matching backend driver, node id and gimbal id backend instance
    AP_Mount_Xacti* driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id);
    if (driver == nullptr) {
        return;
    }

    // convert body-frame Euler angles to Quaternion.  Note yaw direction is reversed from normal
    driver->_current_attitude_quat.from_euler(radians(msg.gimbal_roll * 0.01), radians(msg.gimbal_pitch * 0.01), radians(-msg.gimbal_yaw * 0.01));
    driver->_last_current_attitude_quat_ms = AP_HAL::millis();
}

// handle xacti gnss status request message
void AP_Mount_Xacti::handle_gnss_status_req(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GnssStatusReq &msg)
{
    // sanity check dronecan port
    if (ap_dronecan == nullptr) {
        return;
    }

    // get current location
    uint8_t gps_status = 2;
    Location loc;
    if (!AP::ahrs().get_location(loc)) {
        gps_status = 0;
    }

    // get date and time
    uint16_t year, ms;
    uint8_t month, day, hour, min, sec;
    if (!AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec, ms)) {
        year = month = day = hour = min = sec = 0;
    }

    // send xacti specific gnss status message
    com_xacti_GnssStatus xacti_gnss_status_msg {};
    xacti_gnss_status_msg.gps_status = gps_status;
    xacti_gnss_status_msg.order = msg.requirement;
    xacti_gnss_status_msg.remain_buffer = 1;
    xacti_gnss_status_msg.utc_year = year;
    xacti_gnss_status_msg.utc_month = month + 1;
    xacti_gnss_status_msg.utc_day = day;
    xacti_gnss_status_msg.utc_hour = hour;
    xacti_gnss_status_msg.utc_minute = min;
    xacti_gnss_status_msg.utc_seconds = sec;
    xacti_gnss_status_msg.latitude = loc.lat * 1e-7;
    xacti_gnss_status_msg.longitude = loc.lng * 1e-7;
    xacti_gnss_status_msg.altitude = loc.alt * 1e-2;
    ap_dronecan->xacti_gnss_status.broadcast(xacti_gnss_status_msg);
}

// handle param get/set response
bool AP_Mount_Xacti::handle_param_get_set_response_int(AP_DroneCAN* ap_dronecan, uint8_t node_id, const char* name, int32_t &value)
{
    // display errors
    const char* err_prefix_str = "Xacti: failed to";
    if (strcmp(name, XACTI_PARAM_SINGLESHOT) == 0) {
        if (value < 0) {
            gcs().send_text(MAV_SEVERITY_ERROR, "%s take pic", err_prefix_str);
        }
        return false;
    }
    if (strcmp(name, XACTI_PARAM_RECORDING) == 0) {
        if (value < 0) {
            _recording_video = false;
            gcs().send_text(MAV_SEVERITY_ERROR, "%s record", err_prefix_str);
        } else {
            _recording_video = (value == 1);
            gcs().send_text(MAV_SEVERITY_INFO, "Xacti: recording %s", _recording_video ? "ON" : "OFF");
        }
        return false;
    }
    if (strcmp(name, XACTI_PARAM_FOCUSMODE) == 0) {
        if (value < 0) {
            gcs().send_text(MAV_SEVERITY_ERROR, "%s change focus", err_prefix_str);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Xacti: %s focus", value == 0 ? "manual" : "auto");
        }
        return false;
    }
    // unhandled parameter get or set
    gcs().send_text(MAV_SEVERITY_INFO, "Xacti: get/set %s res:%ld", name, (long int)value);
    return false;
}

void AP_Mount_Xacti::handle_param_save_response(AP_DroneCAN* ap_dronecan, const uint8_t node_id, bool success)
{
    // display failure to save parameter
    if (!success) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Xacti: CAM%u failed to set param", (int)_instance+1);
    }
}

// send gimbal control message via DroneCAN
// mode is 2:angle control or 3:rate control
// pitch_cd is pitch angle in centi-degrees or pitch rate in cds
// yaw_cd is angle in centi-degrees or yaw rate in cds
void AP_Mount_Xacti::send_gimbal_control(uint8_t mode, int16_t pitch_cd, int16_t yaw_cd)
{
    // exit immediately if no DroneCAN port
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return;
    }

    // send at no faster than 5hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_gimbal_control_ms < 200) {
        return;
    }
    last_send_gimbal_control_ms = now_ms;

    // send xacti specific gimbal control message
    com_xacti_GimbalControlData gimbal_control_data_msg {};
    gimbal_control_data_msg.pitch_cmd_type = mode;
    gimbal_control_data_msg.yaw_cmd_type = mode;
    gimbal_control_data_msg.pitch_cmd_value = pitch_cd;
    gimbal_control_data_msg.yaw_cmd_value = -yaw_cd;
    _detected_modules[_instance].ap_dronecan->xacti_gimbal_control_data.broadcast(gimbal_control_data_msg);
}

// send copter attitude status message to gimbal
void AP_Mount_Xacti::send_copter_att_status()
{
    // exit immediately if no DroneCAN port
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return;
    }

    // send at no faster than 5hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_copter_att_status_ms < 100) {
        return;
    }

    // send xacti specific vehicle attitude message
    Quaternion veh_att;
    if (!AP::ahrs().get_quaternion(veh_att)) {
        return;
    }

    last_send_copter_att_status_ms = now_ms;
    com_xacti_CopterAttStatus copter_att_status_msg {};
    copter_att_status_msg.quaternion_wxyz_e4[0] = veh_att.q1 * 1e4;
    copter_att_status_msg.quaternion_wxyz_e4[1] = veh_att.q2 * 1e4;
    copter_att_status_msg.quaternion_wxyz_e4[2] = veh_att.q3 * 1e4;
    copter_att_status_msg.quaternion_wxyz_e4[3] = veh_att.q4 * 1e4;
    copter_att_status_msg.reserved.len = 2;
    copter_att_status_msg.reserved.data[0] = 0;
    copter_att_status_msg.reserved.data[1] = 0;
    _detected_modules[_instance].ap_dronecan->xacti_copter_att_status.broadcast(copter_att_status_msg);
}

#endif // HAL_MOUNT_XACTI_ENABLED
