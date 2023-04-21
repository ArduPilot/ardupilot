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

// set focus in, out or hold.  returns true on success
// focus in = -1, focus hold = 0, focus out = 1
bool AP_Camera_Scripting::set_manual_focus_step(int8_t focus_step)
{
    _cam_state.focus_step = focus_step;
    _cam_state.auto_focus = false;
    return true;
}

// auto focus.  returns true on success
bool AP_Camera_Scripting::set_auto_focus()
{
    _cam_state.auto_focus = true;
    _cam_state.focus_step = 0;
    return true;
}

// access for scripting backend to retrieve state
// returns true on success and cam_state is filled in
bool AP_Camera_Scripting::get_state(AP_Camera::camera_state_t& cam_state)
{
    cam_state = _cam_state;
    return true;
}

#endif // AP_CAMERA_SCRIPTING_ENABLED
