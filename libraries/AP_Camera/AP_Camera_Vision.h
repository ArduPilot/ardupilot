/*
 *  Camera wrapper class for optional tight camera autopilot synchronization
 *  Camera Vision overloads the camera functions to add behaviors suited to machine vision
 *
 *  Samuel Dudley 28/10/2017
 *
 *  dudley.samuel@gmail.com
 *
 */

#pragma once

#include "AP_Camera.h"
#include <AP_AHRS/AP_AHRS.h>

// with a 400Hz AHRS update rate we expect the data to be no older than 2500us
// TODO assess the logged values to determine a suitable default time
#define AP_CAMERA_VISION_DEFAULT_MAX_SAMPLE_AGE          100000 // set to 100ms for now
#define AP_CAMERA_VISION_DEFAULT_FEEDBACK_COMPONENT_ID   191
#define AP_CAMERA_VISION_DEFAULT_GCS_FEEDBACK_HZ         1 // 1 message per second

class AP_Camera_Vision: public AP_Camera {
public:
    static AP_Camera_Vision create(AP_Relay *obj_relay,
                            uint32_t _log_camera_bit,
                            const struct Location &_loc,
                            const AP_GPS &_gps,
                            AP_AHRS &_ahrs) {
        return AP_Camera_Vision{obj_relay, _log_camera_bit, _loc, _gps, _ahrs};
    }

    constexpr AP_Camera_Vision(AP_Camera_Vision &&other) = default;

    /* Do not allow copies */
    AP_Camera_Vision(const AP_Camera_Vision &other) = delete;
    AP_Camera_Vision &operator=(const AP_Camera_Vision&) = delete;

    // overloaded function from the standard AP_Camera class
    // check to see if a trigger event has occurred and action accordingly
    void update_trigger();



    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Camera_Vision(AP_Relay *obj_relay, uint32_t _log_camera_bit,
            const struct Location &_loc, const AP_GPS &_gps, AP_AHRS &_ahrs)
            : AP_Camera(obj_relay, _log_camera_bit, _loc, _gps, _ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _last_gcs_feedback_time = 0;
    }

    // determine if the GCS should be informed about this image capture event
    bool should_send_feedback_to_gcs(void);

    // send AHRS summary MAVLink message to attached components
    void send_feedback_ahrs(void);

    // read the AHRS summary captured at the feedback event
    void read_ahrs_summary(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    // overloaded function from the standard AP_Camera class
    // called on hardware trigger event
    void capture_callback(void *context, uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    void snapshot_ahrs(void);
#endif

    // component ID of the CC which will receive the AHRS summary MAVLink message
    AP_Int16 _vision_feedback_target_component;

    // maximum time difference between the AHRS data sample time and the camera trigger time
    // before the sampled AHRS data is considered unhealthy
    AP_Int32 _maximum_ahrs_sample_age_us;

    uint32_t _last_gcs_feedback_time;

    // the cameras local copy of the AHRS summary structure
    AP_AHRS::AHRS_Summary _ahrs_summary;

    AP_AHRS::AHRS_Summary *_current_summary;

    // the time that the last hardware trigger event occurred
    uint64_t _camera_feedback_time;

    // the absolute difference between the camera feedback time and the AHRS sample time
    AP_Int32 _ahrs_sample_age;

    // maximum rate at which the GCS will receive camera feedback infomation
    AP_Float _gcs_feedback_hz;

    volatile bool _ahrs_data_good;

    uint8_t _flags;

};
