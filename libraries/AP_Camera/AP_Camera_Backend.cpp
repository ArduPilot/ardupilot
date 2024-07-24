#include "AP_Camera_Backend.h"

#if AP_CAMERA_ENABLED
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Mount/AP_Mount.h>

#if 0
    #define debug(fmt, args ...) do { hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
    #define debug(fmt, args ...)
#endif

#if AP_CAMERA_JSON_INFO_ENABLED
#include <AP_Filesystem/AP_Filesystem.h>
#include <stdio.h>
#endif // AP_CAMERA_JSON_INFO_ENABLED

extern const AP_HAL::HAL& hal;

// Constructor
AP_Camera_Backend::AP_Camera_Backend(AP_Camera &frontend, AP_Camera_Params &params, uint8_t instance) :
    _frontend(frontend),
    _params(params),
    _instance(instance)
{}

void AP_Camera_Backend::init() {
#if AP_CAMERA_JSON_INFO_ENABLED
    init_camera_information_from_json();
#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
    init_video_stream_information_from_json();
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
#endif // AP_CAMERA_JSON_INFO_ENABLED
}

// update - should be called at 50hz
void AP_Camera_Backend::update()
{
    // Check camera options and start/stop recording based on arm/disarm
    if ((_params.options.get() & (uint8_t)Options::RecordWhileArmed) != 0) {
        if (hal.util->get_soft_armed() != last_is_armed) {
            last_is_armed = hal.util->get_soft_armed();
            if (!record_video(last_is_armed)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Camera: failed to %s recording", last_is_armed ? "start" : "stop");
            }
        }
    }

    // try to take picture if pending
    if (trigger_pending) {
        take_picture();
    }

    // check feedback pin
    check_feedback();

    // time based triggering
    // if time and distance triggering both are enabled then we only do time based triggering
    if (time_interval_settings.num_remaining != 0) {
        uint32_t delta_ms = AP_HAL::millis() - last_picture_time_ms;
        if (delta_ms > time_interval_settings.time_interval_ms) {
            if (take_picture()) {
                // decrease num_remaining except when its -1 i.e. capture forever
                if (time_interval_settings.num_remaining > 0) {
                    time_interval_settings.num_remaining--;
                }
            }
        }
        return;
    }

    // implement trigger distance
    if (!is_positive(_params.trigg_dist)) {
        last_location.lat = 0;
        last_location.lng = 0;
        return;
    }

    // check GPS status
    if (AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }

    // check vehicle flight mode supports trigg dist
    if (!_frontend.vehicle_mode_ok_for_trigg_dist()) {
        return;
    }

    // check vehicle roll angle is less than configured maximum
    const AP_AHRS &ahrs = AP::ahrs();
    if ((_frontend.get_roll_max() > 0) && (fabsf(AP::ahrs().roll_sensor * 1e-2f) > _frontend.get_roll_max())) {
        return;
    }

    // get current location. ignore failure because AHRS will provide its best guess
    Location current_loc;
    IGNORE_RETURN(ahrs.get_location(current_loc));

    // initialise last location to current location
    if (last_location.lat == 0 && last_location.lng == 0) {
        last_location = current_loc;
        return;
    }
    if (last_location.lat == current_loc.lat && last_location.lng == current_loc.lng) {
        // we haven't moved - this can happen as update() may
        // be called without a new GPS fix
        return;
    }

    // check vehicle has moved at least trigg_dist meters
    if (current_loc.get_distance(last_location) < _params.trigg_dist) {
        return;
    }

    take_picture();
}

// get corresponding mount instance for the camera
uint8_t AP_Camera_Backend::get_mount_instance() const
{
    // instance 0 means default
    if (_params.mount_instance.get() == 0) {
        return _instance;
    }
    return _params.mount_instance.get() - 1;
}

// get mavlink gimbal device id which is normally mount_instance+1
uint8_t AP_Camera_Backend::get_gimbal_device_id() const
{
#if HAL_MOUNT_ENABLED
    const uint8_t mount_instance = get_mount_instance();
    AP_Mount* mount = AP::mount();
    if (mount != nullptr) {
        if (mount->get_mount_type(mount_instance) != AP_Mount::Type::None) {
            return (mount_instance + 1);
        }
    }
#endif
    return 0;
}


// take a picture.  returns true on success
bool AP_Camera_Backend::take_picture()
{
    // setup feedback pin interrupt or timer
    setup_feedback_callback();

    // check minimum time interval since last picture taken
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_picture_time_ms < (uint32_t)(_params.interval_min * 1000)) {
        trigger_pending = true;
        return false;
    }

    trigger_pending = false;

    // trigger actually taking picture and update image count
    if (trigger_pic()) {
        image_index++;
        last_picture_time_ms = now_ms;
        IGNORE_RETURN(AP::ahrs().get_location(last_location));
#if HAL_LOGGING_ENABLED
        log_picture();
#endif
        return true;
    }

    return false;
}

// take multiple pictures, time_interval between two consecutive pictures is in miliseconds
// total_num is number of pictures to be taken, -1 means capture forever
void AP_Camera_Backend::take_multiple_pictures(uint32_t time_interval_ms, int16_t total_num)
{
    time_interval_settings = {time_interval_ms, total_num};
}

// stop capturing multiple image sequence
void AP_Camera_Backend::stop_capture()
{
    time_interval_settings = {0, 0};
}

// handle camera control
void AP_Camera_Backend::control(float session, float zoom_pos, float zoom_step, float focus_lock, int32_t shooting_cmd, int32_t cmd_id)
{
    // take picture
    if (shooting_cmd == 1) {
        take_picture();
    }
}

// send camera feedback message to GCS
void AP_Camera_Backend::send_camera_feedback(mavlink_channel_t chan)
{
    int32_t altitude = 0;
    if (camera_feedback.location.initialised() && !camera_feedback.location.get_alt_cm(Location::AltFrame::ABSOLUTE, altitude)) {
        // completely ignore this failure!  this is a shouldn't-happen
        // as current_loc should never be in an altitude we can't
        // convert.
    }
    int32_t altitude_rel = 0;
    if (camera_feedback.location.initialised() && !camera_feedback.location.get_alt_cm(Location::AltFrame::ABOVE_HOME, altitude_rel)) {
        // completely ignore this failure!  this is a shouldn't-happen
        // as current_loc should never be in an altitude we can't
        // convert.
    }

    // send camera feedback message
    mavlink_msg_camera_feedback_send(
        chan,
        camera_feedback.timestamp_us,       // image timestamp
        0,                                  // target system id
        _instance,                          // camera id
        image_index,                        // image index
        camera_feedback.location.lat,       // latitude
        camera_feedback.location.lng,       // longitude
        altitude*1e-2f,                     // alt MSL
        altitude_rel*1e-2f,                 // alt relative to home
        camera_feedback.roll_sensor*1e-2f,  // roll angle (deg)
        camera_feedback.pitch_sensor*1e-2f, // pitch angle (deg)
        camera_feedback.yaw_sensor*1e-2f,   // yaw angle (deg)
        0.0f,                               // focal length
        CAMERA_FEEDBACK_PHOTO,              // flags
        camera_feedback.feedback_trigger_logged_count); // completed image captures
}

// send camera information message to GCS
void AP_Camera_Backend::send_camera_information(mavlink_channel_t chan) const
{
#if AP_CAMERA_JSON_INFO_ENABLED
    if (camera_info.is_valid) {
        // This is a const function, so we need to clone the camera_info msg in
        // order to update the time_boot_ms field.
        auto cam_info = camera_info.msg;
        cam_info.time_boot_ms = AP_HAL::millis();
        mavlink_msg_camera_information_send_struct(chan, &cam_info);
    }
#else
    // prepare vendor, model and cam definition strings
    const uint8_t vendor_name[32] {};
    const uint8_t model_name[32] {};
    const char cam_definition_uri[140] {};
    const uint32_t cap_flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
    const float NaN = nanf("0x4152");

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
        0,                      // lens_id, uint8_t
        cap_flags,              // flags uint32_t (CAMERA_CAP_FLAGS)
        0,                      // cam_definition_version uint16_t
        cam_definition_uri,     // cam_definition_uri char[140]
        get_gimbal_device_id());// gimbal_device_id uint8_t
#endif // AP_CAMERA_JSON_INFO_ENABLED
}

// send camera settings message to GCS
void AP_Camera_Backend::send_camera_settings(mavlink_channel_t chan) const
{
    const float NaN = nanf("0x4152");

    // send CAMERA_SETTINGS message
    mavlink_msg_camera_settings_send(
        chan,
        AP_HAL::millis(),   // time_boot_ms
        CAMERA_MODE_IMAGE,  // camera mode (0:image, 1:video, 2:image survey)
        NaN,                // zoomLevel float, percentage from 0 to 100, NaN if unknown
        NaN);               // focusLevel float, percentage from 0 to 100, NaN if unknown
}

#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
// send camera field of view status
void AP_Camera_Backend::send_camera_fov_status(mavlink_channel_t chan) const
{
    // getting corresponding mount instance for camera
    const AP_Mount* mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    Quaternion quat;
    Location loc;
    Location poi_loc;
    if (!mount->get_poi(get_mount_instance(), quat, loc, poi_loc)) {
        return;
    }
    // send camera fov status message only if the last calculated values aren't stale
    const float NaN = nanf("0x4152");
    const float quat_array[4] = {
        quat.q1,
        quat.q2,
        quat.q3,
        quat.q4
    };
    mavlink_msg_camera_fov_status_send(
        chan,
        AP_HAL::millis(),
        loc.lat,
        loc.lng,
        loc.alt,
        poi_loc.lat,
        poi_loc.lng,
        poi_loc.alt,
        quat_array,
        horizontal_fov() > 0 ? horizontal_fov() : NaN,
        vertical_fov() > 0 ? vertical_fov() : NaN
    );
}
#endif

// send camera capture status message to GCS
void AP_Camera_Backend::send_camera_capture_status(mavlink_channel_t chan) const
{
    const float NaN = nanf("0x4152");

    // Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
    const uint8_t image_status = (time_interval_settings.num_remaining > 0) ? 2 : 0;

    // send CAMERA_CAPTURE_STATUS message
    mavlink_msg_camera_capture_status_send(
        chan,
        AP_HAL::millis(),
        image_status,
        0,                // current status of video capturing (0: idle, 1: capture in progress)
        static_cast<float>(time_interval_settings.time_interval_ms) / 1000.0, // image capture interval (s)
        0,                // elapsed time since recording started (ms)
        NaN,              // available storage capacity (ms)
        image_index);     // total number of images captured
}

#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
// send video stream information message to GCS
void AP_Camera_Backend::send_video_stream_information(mavlink_channel_t chan) const {
#if AP_CAMERA_JSON_INFO_ENABLED
    if (video_stream_info.is_valid) {
        mavlink_msg_video_stream_information_send_struct(chan, &video_stream_info.msg);
    }
#endif // AP_CAMERA_JSON_INFO_ENABLED
}
#endif

// setup a callback for a feedback pin. When on PX4 with the right FMU
// mode we can use the microsecond timer.
void AP_Camera_Backend::setup_feedback_callback()
{
    if (_params.feedback_pin <= 0 || timer_installed || isr_installed) {
        // invalid or already installed
        return;
    }

    // ensure we are in input mode
    hal.gpio->pinMode(_params.feedback_pin, HAL_GPIO_INPUT);

    // enable pullup/pulldown
    uint8_t trigger_polarity = _params.feedback_polarity == 0 ? 0 : 1;
    hal.gpio->write(_params.feedback_pin, !trigger_polarity);

    if (hal.gpio->attach_interrupt(_params.feedback_pin, FUNCTOR_BIND_MEMBER(&AP_Camera_Backend::feedback_pin_isr, void, uint8_t, bool, uint32_t),
                                   trigger_polarity?AP_HAL::GPIO::INTERRUPT_RISING:AP_HAL::GPIO::INTERRUPT_FALLING)) {
        isr_installed = true;
    } else {
        // install a 1kHz timer to check feedback pin
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Camera_Backend::feedback_pin_timer, void));

        timer_installed = true;
    }
}

// interrupt handler for interrupt based feedback trigger
void AP_Camera_Backend::feedback_pin_isr(uint8_t pin, bool high, uint32_t timestamp_us)
{
    feedback_trigger_timestamp_us = timestamp_us;
    feedback_trigger_count++;
}

// check if feedback pin is high for timer based feedback trigger, when
// attach_interrupt fails
void AP_Camera_Backend::feedback_pin_timer()
{
    uint8_t pin_state = hal.gpio->read(_params.feedback_pin);
    uint8_t trigger_polarity = _params.feedback_polarity == 0 ? 0 : 1;
    if (pin_state == trigger_polarity &&
        last_pin_state != trigger_polarity) {
        feedback_trigger_timestamp_us = AP_HAL::micros();
        feedback_trigger_count++;
    }
    last_pin_state = pin_state;
}

// check for feedback pin update and log if necessary
void AP_Camera_Backend::check_feedback()
{
    if (feedback_trigger_logged_count != feedback_trigger_count) {
#if HAL_LOGGING_ENABLED
        const uint32_t timestamp32 = feedback_trigger_timestamp_us;
#endif
        feedback_trigger_logged_count = feedback_trigger_count;

        // we should consider doing this inside the ISR and pin_timer
        prep_mavlink_msg_camera_feedback(feedback_trigger_timestamp_us);

#if HAL_LOGGING_ENABLED
        // log camera message
        uint32_t tdiff = AP_HAL::micros() - timestamp32;
        uint64_t timestamp = AP_HAL::micros64();
        Write_Camera(timestamp - tdiff);
#endif
    }
}

void AP_Camera_Backend::prep_mavlink_msg_camera_feedback(uint64_t timestamp_us)
{
    const AP_AHRS &ahrs = AP::ahrs();
    if (!ahrs.get_location(camera_feedback.location)) {
        // completely ignore this failure!  AHRS will provide its best guess.
    }
    camera_feedback.timestamp_us = timestamp_us;
    camera_feedback.roll_sensor = ahrs.roll_sensor;
    camera_feedback.pitch_sensor = ahrs.pitch_sensor;
    camera_feedback.yaw_sensor = ahrs.yaw_sensor;
    camera_feedback.feedback_trigger_logged_count = feedback_trigger_logged_count;

    GCS_SEND_MESSAGE(MSG_CAMERA_FEEDBACK);
}

#if HAL_LOGGING_ENABLED
// log picture
void AP_Camera_Backend::log_picture()
{
    const bool using_feedback_pin = _params.feedback_pin > 0;

    if (!using_feedback_pin) {
        // if we're using a feedback pin then when the event occurs we
        // stash the feedback data.  Since we're not using a feedback
        // pin, we just use "now".
        prep_mavlink_msg_camera_feedback(AP::gps().time_epoch_usec());
    }

    if (!using_feedback_pin) {
        Write_Camera();
    } else {
        Write_Trigger();
    }
}
#endif

#if AP_CAMERA_JSON_INFO_ENABLED
// Read a JSON file from the expected folder on the SD card. If not found, will
// fall back to looking for the same folder in ROMFS.
//
// This allocates a AP_JSON::value object that will need to be freed.
AP_JSON::value * AP_Camera_Backend::_load_mount_msg_json(const char* json_filename)
{
    char* romfs_json_path = nullptr;
    int alloc = asprintf(&romfs_json_path, "@ROMFS/mav_msg_def/AP_Camera/%s", json_filename);
    if (alloc < 0) {
        debug("json load bad alloc");
        return nullptr;
    }

    // drop the "@ROMFS/" so we can search local storage first...
    char const * json_path = &romfs_json_path[7];

    struct stat st;
    if (AP::FS().stat(json_path, &st) != 0) {
        // not in local storage, so try full ROMFS path
        debug("failed to load '%s'", json_path);
        json_path = romfs_json_path;
        if (AP::FS().stat(json_path, &st) != 0) {
            debug("failed to load '%s'", json_path);
            json_path = nullptr;
        }
    }

    AP_JSON::value * obj = nullptr;
    if (json_path) {
        obj = AP_JSON::load_json(json_path);
        if (obj == nullptr) {
            debug("failed to parse '%s'", json_path);
        }
    }

    free(romfs_json_path);
    return obj;
}

// helper function to copy a JSON double into a msg struct
template <typename T>
bool AP_Camera_Backend::_copy_json_field_double(const AP_JSON::value* obj, const char* key, T& dst)
{
    auto v = obj->get(key);
    if (!v.is<double>()) {
        return false;
    }

    dst = v.get<double>();
    return true;
}

// helper function to copy a JSON string into a msg struct
bool AP_Camera_Backend::_copy_json_field_string(const AP_JSON::value* obj, const char* key, char* dst, size_t dst_sz)
{
    auto v = obj->get(key);
    if (!v.is<std::string>()) {
        return false;
    }

    const auto src = v.get<std::string>().c_str();
    strncpy(dst, src, dst_sz - 1);
    dst[dst_sz - 1] = '\0';
    return true;
}

#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
void AP_Camera_Backend::init_video_stream_information_from_json()
{
    video_stream_info.is_valid = false;

    const auto filename = "video_stream_information.json";
    auto *obj = _load_mount_msg_json(filename);
    if (obj == nullptr) {
        return;
    }

    if (!_copy_json_field_double(obj, "type", video_stream_info.msg.type)) goto err;
    if (!_copy_json_field_double(obj, "framerate", video_stream_info.msg.framerate)) goto err;
    if (!_copy_json_field_double(obj, "resolution_h", video_stream_info.msg.resolution_h)) goto err;
    if (!_copy_json_field_double(obj, "resolution_v", video_stream_info.msg.resolution_v)) goto err;
    if (!_copy_json_field_double(obj, "bitrate", video_stream_info.msg.bitrate)) goto err;
    if (!_copy_json_field_double(obj, "rotation", video_stream_info.msg.rotation)) goto err;
    if (!_copy_json_field_double(obj, "hfov", video_stream_info.msg.hfov)) goto err;

    if (!_copy_json_field_string(obj, "name", video_stream_info.msg.name, sizeof(video_stream_info.msg.name))) goto err;
    if (!_copy_json_field_string(obj, "uri", video_stream_info.msg.uri, sizeof(video_stream_info.msg.uri))) goto err;

    // Populate the fields that shouldn't change.
    video_stream_info.msg.stream_id = 1; // We currently only support a single stream defined in JSON
    video_stream_info.msg.count = 1;
    video_stream_info.msg.flags = VIDEO_STREAM_STATUS_FLAGS_RUNNING;

    // If we got this far, then we've got all the required fields.
    video_stream_info.is_valid = true;

err:
    delete obj;

    if (video_stream_info.is_valid) {
        debug("Loaded video_stream_info from '%s'", filename);
        debug("    video_stream_info.msg.stream_id=%" PRIu8, video_stream_info.msg.stream_id);
        debug("    video_stream_info.msg.count=%" PRIu8 "", video_stream_info.msg.count);
        debug("    video_stream_info.msg.type=%" PRIu8 "", video_stream_info.msg.type);
        debug("    video_stream_info.msg.flags=%" PRIu16 "", video_stream_info.msg.flags);
        debug("    video_stream_info.msg.framerate=%f", video_stream_info.msg.framerate);
        debug("    video_stream_info.msg.resolution_h=%" PRIu16 "", video_stream_info.msg.resolution_h);
        debug("    video_stream_info.msg.resolution_v=%" PRIu16 "", video_stream_info.msg.resolution_v);
        debug("    video_stream_info.msg.bitrate=%" PRIu32 "", video_stream_info.msg.bitrate);
        debug("    video_stream_info.msg.rotation=%" PRIu16 "", video_stream_info.msg.rotation);
        debug("    video_stream_info.msg.hfov=%" PRIu16 "", video_stream_info.msg.hfov);

        debug("    video_stream_info.msg.name='%s'", video_stream_info.msg.name);
        debug("    video_stream_info.msg.uri='%s'", video_stream_info.msg.uri);
    }
}
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED

void AP_Camera_Backend::init_camera_information_from_json()
{
    camera_info.is_valid = false;

    const auto filename = "camera_information.json";
    auto *obj = _load_mount_msg_json(filename);
    if (obj == nullptr) {
        return;
    }

    if (!_copy_json_field_double(obj, "firmware_version", camera_info.msg.firmware_version)) goto err;
    if (!_copy_json_field_double(obj, "focal_length", camera_info.msg.focal_length)) goto err;
    if (!_copy_json_field_double(obj, "sensor_size_h", camera_info.msg.sensor_size_h)) goto err;
    if (!_copy_json_field_double(obj, "sensor_size_v", camera_info.msg.sensor_size_v)) goto err;
    if (!_copy_json_field_double(obj, "resolution_h", camera_info.msg.resolution_h)) goto err;
    if (!_copy_json_field_double(obj, "resolution_v", camera_info.msg.resolution_v)) goto err;
    if (!_copy_json_field_double(obj, "flags", camera_info.msg.flags)) goto err;
    if (!_copy_json_field_double(obj, "cam_definition_version", camera_info.msg.cam_definition_version)) goto err;

    if (!_copy_json_field_string(obj, "vendor_name", (char*)camera_info.msg.vendor_name, sizeof(camera_info.msg.vendor_name))) goto err;
    if (!_copy_json_field_string(obj, "model_name", (char*)camera_info.msg.model_name, sizeof(camera_info.msg.model_name))) goto err;
    if (!_copy_json_field_string(obj, "cam_definition_uri", camera_info.msg.cam_definition_uri, sizeof(camera_info.msg.cam_definition_uri))) goto err;

    // Populate the fields that shouldn't change. We don't populate time_boot_ms
    // here because it is set when the message is sent.

    // To disambiguate when we have multiple cameras, the lens_id is populated
    // with the instance number.
    camera_info.msg.lens_id = _instance + 1;
    camera_info.msg.gimbal_device_id = get_gimbal_device_id();

    // If we got this far, then we've got all the required fields.
    camera_info.is_valid = true;

err:
    delete obj;

    if (camera_info.is_valid) {
        debug("Loaded camera_info from '%s'", filename);
        debug("    camera_info.msg.time_boot_ms=%" PRIu32, camera_info.msg.time_boot_ms);
        debug("    camera_info.msg.vendor_name='%s'", camera_info.msg.vendor_name);
        debug("    camera_info.msg.model_name='%s'", camera_info.msg.model_name);
        debug("    camera_info.msg.firmware_version=%" PRIu32, camera_info.msg.firmware_version);
        debug("    camera_info.msg.focal_length=%f", camera_info.msg.focal_length);
        debug("    camera_info.msg.sensor_size_h=%f", camera_info.msg.sensor_size_h);
        debug("    camera_info.msg.sensor_size_v=%f", camera_info.msg.sensor_size_v);
        debug("    camera_info.msg.resolution_h=%" PRIu16, camera_info.msg.resolution_h);
        debug("    camera_info.msg.resolution_v=%" PRIu16, camera_info.msg.resolution_v);
        debug("    camera_info.msg.lens_id=%" PRIu8, camera_info.msg.lens_id);
        debug("    camera_info.msg.flags=%" PRIu32, camera_info.msg.flags);
        debug("    camera_info.msg.cam_definition_version=%" PRIu16, camera_info.msg.cam_definition_version);
        debug("    camera_info.msg.cam_definition_uri='%s'", camera_info.msg.cam_definition_uri);
        debug("    camera_info.msg.gimbal_device_id=%" PRIu8, camera_info.msg.gimbal_device_id);
    }
}

#endif // AP_CAMERA_JSON_INFO_ENABLED


#endif // AP_CAMERA_ENABLED
