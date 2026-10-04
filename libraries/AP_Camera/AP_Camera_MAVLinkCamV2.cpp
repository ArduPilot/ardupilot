#include "AP_Camera_MAVLinkCamV2.h"

#if AP_CAMERA_MAVLINKCAMV2_ENABLED
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_CAMERA_MAVLINKCAMV2_SEARCH_MS    60000   // search for camera for 60 seconds after startup
#define AP_CAMERA_MAVLINKCAMV2_STATUS_INTERVAL_MS 1000
#define AP_CAMERA_MAVLINKCAMV2_STATUS_TIMEOUT_MS 3000
#define AP_CAMERA_MAVLINKCAMV2_STATUS_RETRY_MS 10000
#define AP_CAMERA_MAVLINKCAMV2_EMPTY_STREAM_RETRY_MS 10000

// update - should be called at 50hz
void AP_Camera_MAVLinkCamV2::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_camera();
    }

#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
    if (_initialised &&
        (_cam_info.flags & CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM)) {
        const uint32_t stream_req_elapsed_ms =
            AP_HAL::millis() - _last_stream_info_req_ms;
        if ((!video_stream_information_complete() &&
             stream_req_elapsed_ms > 1000) ||
            (_video_stream_info_empty &&
             stream_req_elapsed_ms >
             AP_CAMERA_MAVLINKCAMV2_EMPTY_STREAM_RETRY_MS)) {
            request_video_stream_information();
        }
    }
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED

    const uint32_t status_interval_ms = _capture_status_requests >= 3 ?
        AP_CAMERA_MAVLINKCAMV2_STATUS_RETRY_MS : AP_CAMERA_MAVLINKCAMV2_STATUS_INTERVAL_MS;
    if (_initialised &&
        (_cam_info.flags & (CAMERA_CAP_FLAGS_CAPTURE_IMAGE | CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) &&
        AP_HAL::millis() - _last_capture_status_req_ms >=
        status_interval_ms) {
        request_camera_capture_status();
    }

    // call parent update
    AP_Camera_Backend::update();
}

// entry point to actually take a picture.  returns true on success
bool AP_Camera_MAVLinkCamV2::trigger_pic()
{
    // exit immediately if have not found camera or does not support taking pictures
    if (_link == nullptr || !(_cam_info.flags & CAMERA_CAP_FLAGS_CAPTURE_IMAGE)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.target_system = _sysid;
    pkt.target_component = _compid;
    pkt.command = MAV_CMD_IMAGE_START_CAPTURE;
    pkt.param3 = 1;             // number of images to take
    pkt.param4 = image_index+1; // starting sequence number

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Camera_MAVLinkCamV2::record_video(bool start_recording)
{
    return record_video_stream(start_recording, 0, 0);
}

bool AP_Camera_MAVLinkCamV2::record_video_stream(bool start_recording, uint8_t stream_id, float status_frequency)
{
    // exit immediately if have not found camera or does not support recording video
    if (_link == nullptr || !(_cam_info.flags & CAMERA_CAP_FLAGS_CAPTURE_VIDEO)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.target_system = _sysid;
    pkt.target_component = _compid;

    if (start_recording) {
        pkt.command = MAV_CMD_VIDEO_START_CAPTURE;
        pkt.param2 = status_frequency;
    } else {
        pkt.command = MAV_CMD_VIDEO_STOP_CAPTURE;
    }
    pkt.param1 = stream_id;

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// set zoom specified as a rate or percentage
bool AP_Camera_MAVLinkCamV2::set_zoom(ZoomType zoom_type, float zoom_value)
{
    // exit immediately if have not found camera or does not support zoom
    if (_link == nullptr || !(_cam_info.flags & CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM)) {
        return false;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.target_system = _sysid;
    pkt.target_component = _compid;
    pkt.command = MAV_CMD_SET_CAMERA_ZOOM;
    switch (zoom_type) {
    case ZoomType::RATE:
        pkt.param1 = ZOOM_TYPE_CONTINUOUS;
        break;
    case ZoomType::PCT:
        pkt.param1 = ZOOM_TYPE_RANGE;
        break;
    }
    pkt.param2 = zoom_value;            // Zoom Value

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return true;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Camera_MAVLinkCamV2::set_focus(FocusType focus_type, float focus_value)
{
    // exit immediately if have not found camera or does not support focus
    if (_link == nullptr || !(_cam_info.flags & CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS)) {
        return SetFocusResult::FAILED;
    }

    // prepare and send message
    mavlink_command_long_t pkt {};
    pkt.target_system = _sysid;
    pkt.target_component = _compid;
    pkt.command = MAV_CMD_SET_CAMERA_FOCUS;
    switch (focus_type) {
    case FocusType::RATE:
        // focus in, out or hold (focus in = -1, hold = 0, focus out = 1). Same as FOCUS_TYPE_CONTINUOUS
        pkt.param1 = FOCUS_TYPE_CONTINUOUS;
        break;
    case FocusType::PCT:
        // focus to a percentage (from 0 to 100) of the full range. Same as FOCUS_TYPE_RANGE
        pkt.param1 = FOCUS_TYPE_RANGE;
        break;
    case FocusType::AUTO:
        // focus automatically. Same as FOCUS_TYPE_AUTO
        pkt.param1 = FOCUS_TYPE_AUTO;
        break;
    }
    pkt.param2 = focus_value;

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);

    return SetFocusResult::ACCEPTED;
}

// handle incoming mavlink message including CAMERA_INFORMATION
void AP_Camera_MAVLinkCamV2::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    // exit immediately if this is not our camera's message on its own link
    if (_link == nullptr || chan != _link->get_chan() || msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    case MAVLINK_MSG_ID_CAMERA_SETTINGS:
    case MAVLINK_MSG_ID_STORAGE_INFORMATION:
    case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
    case MAVLINK_MSG_ID_CAMERA_FOV_STATUS:
    case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
    case MAVLINK_MSG_ID_PARAM_EXT_ACK:
    case MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE:
    case MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS:
    case MAVLINK_MSG_ID_CAMERA_TRACKING_GEO_STATUS:
    case MAVLINK_MSG_ID_VIDEO_STREAM_STATUS:
        resend_message(chan, msg);
        break;

    case MAVLINK_MSG_ID_CAMERA_INFORMATION: {
        resend_message(chan, msg);
        mavlink_msg_camera_information_decode(&msg, &_cam_info);

        const uint8_t fw_ver_major = _cam_info.firmware_version & 0x000000FF;
        const uint8_t fw_ver_minor = (_cam_info.firmware_version & 0x0000FF00) >> 8;
        const uint8_t fw_ver_revision = (_cam_info.firmware_version & 0x00FF0000) >> 16;
        const uint8_t fw_ver_build = (_cam_info.firmware_version & 0xFF000000) >> 24;

        // display camera info to user
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Camera: %.32s %.32s fw:%u.%u.%u.%u",
                      _cam_info.vendor_name,
                      _cam_info.model_name,
                      (unsigned)fw_ver_major,
                      (unsigned)fw_ver_minor,
                      (unsigned)fw_ver_revision,
                      (unsigned)fw_ver_build);

        _got_camera_info = true;
#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
        if (_cam_info.flags & CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM) {
            request_video_stream_information();
        } else {
            reset_video_stream_information(0);
            _video_stream_info_empty = true;
        }
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
        break;
    }

    case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
        resend_message(chan, msg);
        mavlink_msg_camera_capture_status_decode(&msg, &_capture_status);
        _got_capture_status = true;
        _last_capture_status_ms = AP_HAL::millis();
        _capture_status_requests = 0;
        break;

#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
    case MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: {
        resend_message(chan, msg);
        mavlink_video_stream_information_t stream_info {};
        mavlink_msg_video_stream_information_decode(&msg, &stream_info);
        if (stream_info.count == 0) {
            // A camera may advertise stream support but report that no streams
            // are currently available.  Treat that as a complete response and
            // retry later at a reduced rate in case streams become available.
            reset_video_stream_information(0);
            _video_stream_info_empty = true;
            _last_stream_info_req_ms = AP_HAL::millis();
            break;
        }
        if (stream_info.stream_id == 0 ||
            stream_info.stream_id > stream_info.count) {
            break;
        }

        const uint8_t stream_count = MIN(
            stream_info.count,
            AP_CAMERA_MAVLINKCAMV2_MAX_VIDEO_STREAMS);
        if (_video_stream_info_empty ||
            stream_count != _video_stream_count) {
            reset_video_stream_information(stream_count);
        }
        _video_stream_info_empty = false;
        if (stream_info.stream_id > stream_count) {
            break;
        }

        const uint8_t slot = stream_info.stream_id - 1U;
        if (_video_stream_info[slot] == nullptr) {
            _video_stream_info[slot] =
                NEW_NOTHROW mavlink_video_stream_information_t;
            if (_video_stream_info[slot] == nullptr) {
                break;
            }
        }
        stream_info.count = stream_count;
        *_video_stream_info[slot] = stream_info;
        break;
    }
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED

    default:
        break;
    }
}

void AP_Camera_MAVLinkCamV2::resend_message(mavlink_channel_t chan, const mavlink_message_t &msg) const
{
    // Only relay our camera's isolated link; broadcasts are already routed.
    if (chan != _link->get_chan() || (!_link->is_private() && !_link->is_unicast())) {
        return;
    }

    // Keep the camera identity: replies without a camera instance field
    // cannot be distinguished by a GCS if they all use the FC component.
    for (uint8_t i = 0; i < gcs().num_gcs(); i++) {
        GCS_MAVLINK &out_link = *gcs().chan(i);
        if (!out_link.is_active() || out_link.is_private() || out_link.is_unicast()) {
            continue;
        }
#if HAL_HIGH_LATENCY2_ENABLED
        if (out_link.is_high_latency_link) {
            continue;
        }
#endif  // HAL_HIGH_LATENCY2_ENABLED
        WITH_SEMAPHORE(comm_chan_lock(out_link.get_chan()));
        if (out_link.check_payload_size(msg.len)) {
            _mavlink_resend_uart(out_link.get_chan(), &msg);
        }
    }
}

bool AP_Camera_MAVLinkCamV2::send_camera_message(mavlink_channel_t chan, uint32_t msgid, const void *packet) const
{
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    GCS_MAVLINK *out_link = gcs().chan(chan);
    if (_link == nullptr || out_link == nullptr || entry == nullptr) {
        return true;
    }
    WITH_SEMAPHORE(comm_chan_lock(chan));
    if (msgid > 255 && out_link->sending_mavlink1()) {
        return true;
    }
    if (!out_link->check_payload_size(entry->max_msg_len)) {
        return false;
    }
    // Finalize cached payloads with the camera identity and this channel's
    // sequence/signing state; do not change the global MAVLink identity.
    mavlink_message_t msg {};
    msg.msgid = msgid;
    memcpy(_MAV_PAYLOAD_NON_CONST(&msg), packet, entry->max_msg_len);
    mavlink_finalize_message_chan(&msg, _sysid, _compid, chan,
                                  entry->min_msg_len, entry->max_msg_len, entry->crc_extra);
    _mavlink_resend_uart(chan, &msg);
    return true;
}

// send the remote camera's cached capture and recording status to the GCS
void AP_Camera_MAVLinkCamV2::send_camera_capture_status(mavlink_channel_t chan) const
{
    if (!_got_capture_status) {
        return;
    }
    if (AP_HAL::millis() - _last_capture_status_ms > AP_CAMERA_MAVLINKCAMV2_STATUS_TIMEOUT_MS) {
        return;
    }

    // ArduPilot implements interval capture by sending individual shots to
    // the camera, so the camera cannot report our interval setting itself.
    const bool interval_active = time_interval_settings.num_remaining != 0;
    mavlink_camera_capture_status_t status = _capture_status;
    status.image_status |= interval_active ? 2 : 0;
    if (interval_active) {
        status.image_interval = time_interval_settings.time_interval_ms * 0.001f;
    }
    send_camera_message(chan, MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, &status);
}

// send camera information message to GCS
void AP_Camera_MAVLinkCamV2::send_camera_information(mavlink_channel_t chan) const
{
    // exit immediately if we have not yet received cam info
    if (!_got_camera_info) {
        return;
    }

    mavlink_camera_information_t info = _cam_info;
    // The FC may carry a standalone camera on a separately configured mount.
    // Do not override an advertised association or refer to our mount from
    // a camera belonging to a different MAVLink system.
    if (info.gimbal_device_id == 0 && _sysid == mavlink_system.sysid) {
        info.gimbal_device_id = get_gimbal_device_id();
    }
    send_camera_message(chan, MAVLINK_MSG_ID_CAMERA_INFORMATION, &info);
}

#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
// send cached video stream information messages to GCS
bool AP_Camera_MAVLinkCamV2::send_video_stream_information(mavlink_channel_t chan, uint8_t &next_stream) const
{
    WITH_SEMAPHORE(comm_chan_lock(chan));
    for (; next_stream < _video_stream_count; next_stream++) {
        const uint8_t i = next_stream;
        if (_video_stream_info[i] == nullptr) {
            continue;
        }
        if (_video_stream_info[i]->stream_id != i + 1U) {
            continue;
        }
        if (!send_camera_message(chan, MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, _video_stream_info[i])) {
            return false;
        }
    }
    return true;
}

bool AP_Camera_MAVLinkCamV2::video_stream_information_complete() const
{
    if (_video_stream_info_empty) {
        return true;
    }

    if (_video_stream_count == 0) {
        return false;
    }
    for (uint8_t i = 0; i < _video_stream_count; i++) {
        if (_video_stream_info[i] == nullptr ||
            _video_stream_info[i]->stream_id != i + 1U) {
            return false;
        }
    }
    return true;
}

void AP_Camera_MAVLinkCamV2::reset_video_stream_information(
    uint8_t stream_count)
{
    _video_stream_count = stream_count;
    for (auto *stream_info : _video_stream_info) {
        if (stream_info == nullptr) {
            continue;
        }
        stream_info->stream_id = 0;
    }
}

void AP_Camera_MAVLinkCamV2::request_video_stream_information()
{
    if (_link == nullptr) {
        return;
    }

    uint8_t requested_stream_id = 0;
    for (uint8_t i = 0; i < _video_stream_count; i++) {
        if (_video_stream_info[i] == nullptr ||
            _video_stream_info[i]->stream_id != i + 1U) {
            requested_stream_id = i + 1U;
            break;
        }
    }

    mavlink_command_long_t pkt {};
    pkt.param1 = MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION;
    pkt.param2 = requested_stream_id;
    pkt.command = MAV_CMD_REQUEST_MESSAGE;
    pkt.target_system = _sysid;
    pkt.target_component = _compid;
    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char *)&pkt);
    _last_stream_info_req_ms = AP_HAL::millis();
}
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED

// search for camera in GCS_MAVLink routing table
void AP_Camera_MAVLinkCamV2::find_camera()
{
    // do not look for camera for first 10 seconds so user may see banner
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms < 10000) {
        return;
    }

    // search for camera for 60 seconds or until armed
    if ((now_ms > AP_CAMERA_MAVLINKCAMV2_SEARCH_MS) && hal.util->get_soft_armed()) {
        return;
    }

    // search for a mavlink enabled camera
    if (_link == nullptr) {
        const int16_t compid = _params.mavlink_compid(_instance);
        if (compid <= AP_CAMERA_MAX_ATTACHED_DEVICE_ID || compid > 255) {
            if (now_ms - _last_config_warning_ms >= 10000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAM%u_COMPID must be 0 or 7..255", unsigned(_instance + 1));
                _last_config_warning_ms = now_ms;
            }
            return;
        }
        // Discovery is component-based, so only the first configured slot
        // may claim an ID, including an earlier slot's implicit default.
        for (uint8_t i = 0; i < _instance; i++) {
            const auto &params = _frontend._params[i];
            if (AP_Camera::CameraType(params.type.get()) != AP_Camera::CameraType::MAVLINK_CAMV2) {
                continue;
            }
            if (compid != params.mavlink_compid(i)) {
                continue;
            }
            if (now_ms - _last_config_warning_ms >= 10000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CAM%u_COMPID duplicates CAM%u", unsigned(_instance + 1), unsigned(i + 1));
                _last_config_warning_ms = now_ms;
            }
            return;
        }
        _link = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_CAMERA, compid, _sysid);
        if (_link == nullptr) {
            // have not yet found a camera so return
            return;
        }
        _compid = compid;
    }

    // request CAMERA_INFORMATION
    if (!_got_camera_info) {
        if (now_ms - _last_caminfo_req_ms > 1000) {
            _last_caminfo_req_ms = now_ms;
            request_camera_information();
        }
        return;
    }

    _initialised = true;
}

// request CAMERA_INFORMATION (holds vendor and model name)
void AP_Camera_MAVLinkCamV2::request_camera_information() const
{
    if (_link == nullptr) {
        return;
    }

    const mavlink_command_long_t pkt {
        MAVLINK_MSG_ID_CAMERA_INFORMATION,  // param1
        0,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_REQUEST_MESSAGE,
        _sysid,
        _compid,
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
}

// request CAMERA_CAPTURE_STATUS from the remote camera
void AP_Camera_MAVLinkCamV2::request_camera_capture_status()
{
    if (_link == nullptr) {
        return;
    }

    mavlink_command_long_t pkt {};
    pkt.param1 = MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS;
    pkt.command = MAV_CMD_REQUEST_MESSAGE;
    pkt.target_system = _sysid;
    pkt.target_component = _compid;
    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char *)&pkt);
    _last_capture_status_req_ms = AP_HAL::millis();
    if (_capture_status_requests < 3) {
        _capture_status_requests++;
    }
}

#endif // AP_CAMERA_MAVLINKCAMV2_ENABLED
