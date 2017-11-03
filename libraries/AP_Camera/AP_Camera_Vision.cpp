/*
 *  Camera wrapper class for optional tight camera autopilot synchronization
 *  Camera Vision overloads the camera functions to add behaviors suited to machine vision
 *
 *  Samuel Dudley 28/10/2017
 *
 *  dudley.samuel@gmail.com
 *
 */
#include "AP_Camera_Vision.h"

#define CAM_DEBUG_VISION true

// table of user settable parameters
const AP_Param::GroupInfo AP_Camera_Vision::var_info[] = {
    // parameters from parent class
    AP_NESTEDGROUPINFO(AP_Camera, 0),

    // @Param: FEEDBACK_ID
    // @DisplayName: Camera feedback target component ID
    // @Description: Target ID of the component which handles camera feedback AHRS MAVLink messages
    // @User: Standard
    AP_GROUPINFO("FEEDBACK_ID",  1, AP_Camera_Vision, _vision_feedback_target_component, AP_CAMERA_VISION_DEFAULT_FEEDBACK_COMPONENT_ID),

    // @Param: MAX_SAMP_AGE
    // @DisplayName: Max camera trigger and AHRS sample time delta
    // @Description: Maximum acceptable delta between camera trigger time and AHRS sample time in us before the AHRS sample is unhealthy
    // @Units: us
    // @Range: 0 1000000
    // @User: Advanced
    AP_GROUPINFO("MAX_SAMP_AGE", 2, AP_Camera_Vision, _maximum_ahrs_sample_age_us, AP_CAMERA_VISION_DEFAULT_MAX_SAMPLE_AGE),

    // @Param: MAX_GCS_RATE
    // @DisplayName: Max camera feedback message Hz
    // @Description: Maximum rate that camera feedback messages will be sent to the GCS (Hz). 0 = suppress all feedback messages to the GCS, -1 = send all feedback messages to the GCS
    // @Units: Hz
    // @Range: -1 100
    // @User: Advanced
    AP_GROUPINFO("MAX_GCS_RATE", 3, AP_Camera_Vision, _gcs_feedback_hz, AP_CAMERA_VISION_DEFAULT_GCS_FEEDBACK_HZ),

    AP_GROUPEND
};

// check the camera trigger status and action
// this function is called by Arduplane.cpp at 50Hz by default but needs to be called
// at 2 x frame rate of camera to ensure all frames are reported
void AP_Camera_Vision::update_trigger() {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && CAM_DEBUG_VISION
    snapshot_ahrs();
#endif
    // if the hardware trigger has not been installed then do it now
    setup_feedback_callback();
    if (check_trigger_pin()) {
        // registered that a photo was taken (via the feedback pin)
        if (_ahrs_data_good) {
            read_ahrs_summary();
        }
        _image_index++;

        // TODO use the flag bit fields to indicate what the errors are
        if (_ahrs_data_good) {
            _flags = 1;
        } else {
            _flags = 0;
        }

        // send a low rate message feedback to the GCS while sending full rate to the CC
        if (should_send_feedback_to_gcs()) {
            gcs().send_message(MSG_CAMERA_FEEDBACK); // this calls send_feedback in the standard camera library
            _last_gcs_feedback_time = AP_HAL::millis();
        }
        // send feedback info and AHRS state to the CC
        send_feedback_ahrs();

        // store the image capture info to dataflash
        DataFlash_Class *df = DataFlash_Class::instance();
        if (df != nullptr) {
            if (df->should_log(log_camera_bit)) {
                // log the AHRS synchronised camera data
                df->Log_Write_Camera_Vision1(_ahrs_summary, _flags, _camera_feedback_time, _image_index);
                df->Log_Write_Camera_Vision2(_ahrs_summary, _flags, _camera_feedback_time, _image_index);
                // log the default camera data
                df->Log_Write_Camera(ahrs, gps, current_loc);
            }
        }
    }
}

// send camera feedback to message to components of this system
void AP_Camera_Vision::send_feedback_ahrs() {
    mavlink_message_t msg;
    mavlink_camera_feedback_ahrs_t camera_feedback_ahrs = { };

    camera_feedback_ahrs.trigger_time = _camera_feedback_time;
    camera_feedback_ahrs.sample_time = _ahrs_summary.ahrs_update_time;
    camera_feedback_ahrs.x = _ahrs_summary.ned_pos_rel_home.x;
    camera_feedback_ahrs.y = _ahrs_summary.ned_pos_rel_home.y;
    camera_feedback_ahrs.z = _ahrs_summary.ned_pos_rel_home.z;
    camera_feedback_ahrs.vx = _ahrs_summary.velocity.x;
    camera_feedback_ahrs.vy = _ahrs_summary.velocity.y;
    camera_feedback_ahrs.vz = _ahrs_summary.velocity.z;
    camera_feedback_ahrs.q1 = _ahrs_summary.quat.q1;
    camera_feedback_ahrs.q2 = _ahrs_summary.quat.q2;
    camera_feedback_ahrs.q3 = _ahrs_summary.quat.q3;
    camera_feedback_ahrs.q4 = _ahrs_summary.quat.q4;
    camera_feedback_ahrs.home_lat = _ahrs_summary.home.lat;
    camera_feedback_ahrs.home_lon = _ahrs_summary.home.lng;
    camera_feedback_ahrs.home_alt = _ahrs_summary.home.alt;
    camera_feedback_ahrs.lat = _ahrs_summary.location.lat;
    camera_feedback_ahrs.lon = _ahrs_summary.location.lng;
    camera_feedback_ahrs.alt = _ahrs_summary.location.alt;
    camera_feedback_ahrs.img_idx = _image_index;
    camera_feedback_ahrs.ekf_type = _ahrs_summary.ekf_type;
    camera_feedback_ahrs.target_component = _vision_feedback_target_component;
    camera_feedback_ahrs.status = _flags;

    // encode camera feedback ahrs into MAVLink msg
    mavlink_msg_camera_feedback_ahrs_encode(0, 0, &msg, &camera_feedback_ahrs);

    // send to all components
    GCS_MAVLINK::send_to_components(&msg);
}

// read the AHRS summary
void AP_Camera_Vision::read_ahrs_summary(void) {
    // make sure the AHRS summary is ready to read
    if (_current_summary->get_ready_to_read()) {
        // copy the data locally
        memcpy(&_ahrs_summary, _current_summary, sizeof(_ahrs_summary));
        // check the health of the data copied
        // we expect _camera_feedback_time to be close to ahrs_summary.ahrs_update_time
        // the delta is used to check health
        _ahrs_sample_age = abs(_camera_feedback_time - _ahrs_summary.ahrs_update_time);

        if (_ahrs_sample_age > _maximum_ahrs_sample_age_us) {
            // the AHRS summary was too old
            _ahrs_data_good = false;
        } else {
            _ahrs_data_good = true;
        }
    } else {
        // the AHRS summary was not ready to read
        _ahrs_data_good = false;
    }
    // release the summary for writing
    _current_summary->set_ready_to_write(true);
}

bool AP_Camera_Vision::should_send_feedback_to_gcs(void) {
    if (is_negative(_gcs_feedback_hz)) {
        return true;
    } else if (is_zero(_gcs_feedback_hz)) {
        // handle the zero case here to avoid the divide by zero in the next case
        return false;
    } else if (AP_HAL::millis() - _last_gcs_feedback_time > (1.0/_gcs_feedback_hz)*1000) {
        // sufficient time has passed since we last sent a msg to the GCS
       return true;
    } else {
        // insufficient time has passed, don't send a message to the GCS this image
       return false;
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
// callback for hardware timer capture on PX4.
// machine vision camera exposure output is connected to this pin
void AP_Camera_Vision::capture_callback(void *context, uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow) {
    snapshot_ahrs();
}
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AP_Camera_Vision::snapshot_ahrs(void) {
    _camera_feedback_time = AP_HAL::micros64();
    _camera_triggered = true;
    // snapshot the memory address of the summary
    _current_summary = ahrs.summary.current_summary;
    // make sure the AHRS summary is valid
    if (_current_summary != nullptr) {
        // lock the summary from being over written
        _current_summary->set_ready_to_write(false);
        _ahrs_data_good = true;
    } else {
        // valid current summary does not yet exist
        _ahrs_data_good = false;
    }
}
#endif
