/*
  DroneCAN gimbal driver

  Implements gimbal control and attitude feedback using the DroneCAN / DSDL / com / xacti messages
  see https://github.com/dronecan/DSDL/tree/master/com/xacti

 */

#pragma once

#include "AP_Mount_Backend.h"

#if HAL_MOUNT_XACTI_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include "AP_Mount.h"

class AP_Mount_Xacti : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_Xacti(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Xacti);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    //
    // camera controls
    //

    // take a picture.  returns true on success
    bool take_picture() override;

    // start or stop video recording
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // set camera lens as a value from 0 to 5
    bool set_lens(uint8_t lens) override;

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan) const override;

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

    // subscribe to Xacti DroneCAN messages
    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);

    // xacti specific message handlers
    static void handle_gimbal_attitude_status(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GimbalAttitudeStatus &msg);
    static void handle_gnss_status_req(AP_DroneCAN* ap_dronecan, const CanardRxTransfer& transfer, const com_xacti_GnssStatusReq &msg);

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // send text prefix string to reduce flash cost
    static const char* send_text_prefix;

    // Sensor mode enumeration (aka lens)
    enum class SensorsMode : uint8_t {
        RGB = 0,    // RGB (aka "visible)")
        IR = 1,     // Infrared, aka thermal
        PIP = 2,    // RGB with IR PIP
        NDVI = 3,   // NDVI (vegetation greenness)
    };
    // array of sensor mode enumeration strings for display to user
    // if enum above is updated, also update sensor_mode_str definition in cpp
    static const char* sensor_mode_str[];

    // send target pitch and yaw rates to gimbal
    // yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
    void send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef);

    // send target pitch and yaw angles to gimbal
    // yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
    void send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef);

    // register backend in detected modules array used to map DroneCAN port and node id to backend
    void register_backend();

    // find backend associated with the given dronecan port and node_id.  also associates backends with zero node ids
    // returns pointer to backend on success, nullptr on failure
    static AP_Mount_Xacti* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    // DroneCAN parameter handling methods
    FUNCTOR_DECLARE(param_int_cb, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    FUNCTOR_DECLARE(param_save_cb, void, AP_DroneCAN*, const uint8_t, bool);
    bool handle_param_get_set_response_int(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, int32_t &value);
    void handle_param_save_response(AP_DroneCAN* ap_dronecan, const uint8_t node_id, bool success);

    // helper function to set integer parameters
    bool set_param_int32(const char* param_name, int32_t param_value);

    // send gimbal control message via DroneCAN
    // mode is 2:angle control or 3:rate control
    // pitch_cd is pitch angle in centi-degrees or pitch rate in cds
    // yaw_cd is angle in centi-degrees or yaw rate in cds
    void send_gimbal_control(uint8_t mode, int16_t pitch_cd, int16_t yaw_cd);

    // send vehicle attitude to gimbal via DroneCAN
    // returns true if sent so that we avoid immediately trying to also send other messages
    bool send_copter_att_status();

    // update zoom rate controller
    // returns true if sent so that we avoid immediately trying to also send other messages
    bool update_zoom_rate_control();

    // check if safe to send message (if messages sent too often camera will not respond)
    bool is_safe_to_send() const;

    // internal variables
    bool _initialised;                              // true once the driver has been initialised

    // attitude received from gimbal
    Quaternion _current_attitude_quat;              // current attitude as a quaternion
    uint32_t _last_current_attitude_quat_ms;        // system time _current_angle_rad was updated
    bool _recording_video;                          // true if recording video
    uint16_t _last_zoom_param_value = 100;          // last digital zoom parameter value sent to camera.  100 ~ 1000 (interval 100)
    struct {
        bool enabled;                               // true if zoom rate control is enabled
        int8_t increment;                           // zoom increment on each update (+100 or -100)
        uint32_t last_update_ms;                    // system time that zoom rate control last updated zoom
    } _zoom_rate_control;

    // DroneCAN related variables
    static bool _subscribed;                        // true once subscribed to receive DroneCAN messages
    static struct DetectedModules {
        AP_Mount_Xacti *driver;                     // pointer to Xacti backends
        AP_DroneCAN* ap_dronecan;                   // DroneCAN interface used by this backend
        uint8_t node_id;                            // DroneCAN node id associated by this backend
    } _detected_modules[AP_MOUNT_MAX_INSTANCES];
    static HAL_Semaphore _sem_registry;             // semaphore protecting access to _detected_modules table
    uint32_t last_send_gimbal_control_ms;           // system time that send_gimbal_control was last called (used to slow down sends to 5hz)
    uint32_t last_send_copter_att_status_ms;        // system time that send_copter_att_status was last called (used to slow down sends to 10hz)
    uint32_t last_send_set_param_ms;                // system time that a set parameter message was sent
};

#endif // HAL_MOUNT_XACTI_ENABLED
