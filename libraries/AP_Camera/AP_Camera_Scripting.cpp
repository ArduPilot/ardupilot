#include "AP_Camera_Scripting.h"

#if AP_CAMERA_SCRIPTING_ENABLED

extern const AP_HAL::HAL& hal;

// entry point to actually take a picture
bool AP_Camera_Scripting::trigger_pic()
{
    // increment counter to allow backend to notice request
    _cam_state.take_pic_incr++;
    return true;
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Camera_Scripting::record_video(bool start_recording)
{
    _cam_state.recording_video = start_recording;
    return true;
}

// set zoom specified as a rate or percentage
bool AP_Camera_Scripting::set_zoom(ZoomType zoom_type, float zoom_value)
{
    _cam_state.zoom_type = (uint8_t)zoom_type;
    _cam_state.zoom_value = zoom_value;
    return true;
}

// set focus specified as rate, percentage or auto
// focus in = -1, focus hold = 0, focus out = 1
SetFocusResult AP_Camera_Scripting::set_focus(FocusType focus_type, float focus_value)
{
    _cam_state.focus_type = (uint8_t)focus_type;
    _cam_state.focus_value = focus_value;
    return SetFocusResult::ACCEPTED;
}

#if AP_CAMERA_TRACKING_ENABLED
// set tracking to none, point or rectangle (see TrackingType enum)
// if POINT only then top_left is the point
// top_left,bottom_right are in range 0 to 1.  0 is left or top, 1 is right or bottom
bool AP_Camera_Scripting::set_tracking_internal(TrackingType tracking_type, const Vector2f& top_left, const Vector2f& bottom_right)
{
    _cam_state.tracking_type = (uint8_t)tracking_type;
    _cam_state.tracking_p1 = top_left;
    _cam_state.tracking_p2 = bottom_right;
    return true;
}
#endif

// access for scripting backend to retrieve state
// returns true on success and cam_state is filled in
bool AP_Camera_Scripting::get_state(AP_Camera::camera_state_t& cam_state)
{
    cam_state = _cam_state;
    return true;
}

#endif // AP_CAMERA_SCRIPTING_ENABLED
