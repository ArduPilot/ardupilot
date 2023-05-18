/*
  Scripting mount/gimbal driver
 */

#pragma once

#include "AP_Mount_Backend.h"

#if HAL_MOUNT_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

class AP_Mount_Scripting : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Scripting);

    // init - performs any required initialisation for this instance
    void init() override {};

    // update mount position - should be called periodically
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    // take a picture.  returns true on success
    bool take_picture() override;

    // start or stop video recording.  returns true on success
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;

    // set camera zoom step.  returns true on success
    // zoom out = -1, hold = 0, zoom in = 1
    bool set_zoom_step(int8_t zoom_step) override;

    // set focus in, out or hold.  returns true on success
    // focus in = -1, focus hold = 0, focus out = 1
    bool set_manual_focus_step(int8_t focus_step) override;

    // auto focus.  returns true on success
    bool set_auto_focus() override;

    // accessors for scripting backends
    bool get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame) override;
    bool get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame) override;
    bool get_location_target(Location& _target_loc) override;
    void set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg) override;
    bool get_camera_state(uint16_t& pic_count, bool& record_video, int8_t& zoom_step, int8_t& focus_step, bool& auto_focus) override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // internal variables
    uint32_t last_update_ms;        // system time of last call to one of the get_ methods.  Used for health reporting
    Vector3f current_angle_deg;     // current gimbal angles in degrees (x=roll, y=pitch, z=yaw)

    MountTarget target_rate_rads;   // rate target in rad/s
    bool target_rate_rads_valid;    // true if _target_rate_degs holds a valid rate target

    MountTarget target_angle_rad;   // angle target in radians
    bool target_angle_rad_valid;    // true if _target_rate_degs holds a valid rate target

    Location target_loc;            // target location
    bool target_loc_valid;          // true if target_loc holds a valid target location

    // camera related internal variables
    uint16_t picture_count;         // number of pictures that have been taken (or at least requested)
    bool recording_video;           // true when record video has been requested
    int8_t manual_zoom_step;        // manual zoom step.  zoom out = -1, hold = 0, zoom in = 1
    int8_t manual_focus_step;       // manual focus step.  focus in = -1, focus hold = 0, focus out = 1
    bool auto_focus_active;         // auto focus active
};

#endif // HAL_MOUNT_SIYISERIAL_ENABLED
