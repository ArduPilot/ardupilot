#include "AP_Camera.h"

#if AP_CAMERA_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Mount/AP_Mount.h>
#include "AP_Camera_SoloGimbal.h"

// ------------------------------
#define CAM_DEBUG DISABLED

const AP_Param::GroupInfo AP_Camera::var_info[] = {
    // @Param: TRIGG_TYPE
    // @DisplayName: Camera shutter (trigger) type
    // @Description: how to trigger the camera to take a picture
    // @Values: 0:Servo,1:Relay, 2:GoPro in Solo Gimbal, 3:Mount (Siyi)
    // @User: Standard
    AP_GROUPINFO("TRIGG_TYPE",  0, AP_Camera, _trigger_type, 0),

    // @Param: DURATION
    // @DisplayName: Duration that shutter is held open
    // @Description: How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
    // @Units: ds
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("DURATION",    1, AP_Camera, _trigger_duration, AP_CAMERA_TRIGGER_DEFAULT_DURATION),

    // @Param: SERVO_ON
    // @DisplayName: Servo ON PWM value
    // @Description: PWM value in microseconds to move servo to when shutter is activated
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_ON",    2, AP_Camera, _servo_on_pwm, AP_CAMERA_SERVO_ON_PWM),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: PWM value in microseconds to move servo to when shutter is deactivated
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF",   3, AP_Camera, _servo_off_pwm, AP_CAMERA_SERVO_OFF_PWM),

    // @Param: TRIGG_DIST
    // @DisplayName: Camera trigger distance
    // @Description: Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
    // @User: Standard
    // @Units: m
    // @Range: 0 1000
    AP_GROUPINFO("TRIGG_DIST",  4, AP_Camera, _trigg_dist, 0),

    // @Param: RELAY_ON
    // @DisplayName: Relay ON value
    // @Description: This sets whether the relay goes high or low when it triggers. Note that you should also set RELAY_DEFAULT appropriately for your camera
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("RELAY_ON",    5, AP_Camera, _relay_on, 1),

    // @Param: MIN_INTERVAL
    // @DisplayName: Minimum time between photos
    // @Description: Postpone shooting if previous picture was taken less than preset time(ms) ago.
    // @User: Standard
    // @Units: ms
    // @Range: 0 10000
    AP_GROUPINFO("MIN_INTERVAL",  6, AP_Camera, _min_interval, 0),

    // @Param: MAX_ROLL
    // @DisplayName: Maximum photo roll angle.
    // @Description: Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).
    // @User: Standard
    // @Units: deg
    // @Range: 0 180
    AP_GROUPINFO("MAX_ROLL",  7, AP_Camera, _max_roll, 0),

    // @Param: FEEDBACK_PIN
    // @DisplayName: Camera feedback pin
    // @Description: pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection.  Some common values are given, but see the Wiki's "GPIOs" page for how to determine the pin number for a given autopilot. See also the CAM_FEEDBACK_POL option.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FEEDBACK_PIN",  8, AP_Camera, _feedback_pin, AP_CAMERA_FEEDBACK_DEFAULT_FEEDBACK_PIN),

    // @Param: FEEDBACK_POL
    // @DisplayName: Camera feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the feedback pin should go high on trigger. If set to 0 then it should go low
    // @Values: 0:TriggerLow,1:TriggerHigh
    // @User: Standard
    AP_GROUPINFO("FEEDBACK_POL",  9, AP_Camera, _feedback_polarity, 1),

    // @Param: AUTO_ONLY
    // @DisplayName: Distance-trigging in AUTO mode only
    // @Description: When enabled, trigging by distance is done in AUTO mode only.
    // @Values: 0:Always,1:Only when in AUTO
    // @User: Standard
    AP_GROUPINFO("AUTO_ONLY",  10, AP_Camera, _auto_mode_only, 0),

    // @Param: TYPE
    // @DisplayName: Type of camera (0: None, 1: BMMCC)
    // @Description: Set the camera type that is being used, certain cameras have custom functions that need further configuration, this enables that.
    // @Values: 0:Default,1:BMMCC
    // @User: Standard
    AP_GROUPINFO("TYPE",  11, AP_Camera, _type, 0),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/// Servo operated camera
void
AP_Camera::servo_pic()
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_cam_trigger, _servo_on_pwm);

    // leave a message that it should be active for this many loops (assumes 50hz loops)
    _trigger_counter = constrain_int16(_trigger_duration*5,0,255);
}

/// basic relay activation
void
AP_Camera::relay_pic()
{
    AP_Relay *_apm_relay = AP::relay();
    if (_apm_relay == nullptr) {
        return;
    }
    if (_relay_on) {
        _apm_relay->on(0);
    } else {
        _apm_relay->off(0);
    }

    // leave a message that it should be active for this many loops (assumes 50hz loops)
    _trigger_counter = constrain_int16(_trigger_duration*5,0,255);
}

/// single entry point to take pictures
void AP_Camera::trigger_pic()
{
    setup_feedback_callback();

    _image_index++;
    switch (get_trigger_type()) {
    case CamTrigType::servo:
        servo_pic();            // Servo operated camera
        break;
    case CamTrigType::relay:
        relay_pic();            // basic relay activation
        break;
#if HAL_SOLO_GIMBAL_ENABLED
    case CamTrigType::gopro:  // gopro in Solo Gimbal
        AP_Camera_SoloGimbal::gopro_shutter_toggle();
        break;
#endif
#if HAL_MOUNT_ENABLED
    case CamTrigType::mount: {
        AP_Mount* mount = AP::mount();
        if (mount != nullptr) {
            mount->take_picture(0);
        }
        break;
    }
#endif
    default:
        break;
    }

    log_picture();
}

/// de-activate the trigger after some delay, but without using a delay() function
/// should be called at 50hz
void
AP_Camera::trigger_pic_cleanup()
{
    if (_trigger_counter) {
        _trigger_counter--;
    } else {
        switch (get_trigger_type()) {
        case CamTrigType::servo:
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_trigger, _servo_off_pwm);
            break;
        case CamTrigType::relay: {
            AP_Relay *_apm_relay = AP::relay();
            if (_apm_relay == nullptr) {
                break;
            }
            if (_relay_on) {
                _apm_relay->off(0);
            } else {
                _apm_relay->on(0);
            }
            break;
        }
        case CamTrigType::gopro:
        case CamTrigType::mount:
            // nothing to do
            break;
        }
    }

    if (_trigger_counter_cam_function) {
        _trigger_counter_cam_function--;
    } else {
        switch (_type) {
        case AP_Camera::CAMERA_TYPE_BMMCC:
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_iso, _servo_off_pwm);
            break;
        }
    }
}

void AP_Camera::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
        control_msg(msg);
        break;
#if HAL_SOLO_GIMBAL_ENABLED
    case MAVLINK_MSG_ID_GOPRO_HEARTBEAT:
        // heartbeat from the Solo gimbal with a GoPro
        if (get_trigger_type() == CamTrigType::gopro) {
            AP_Camera_SoloGimbal::handle_gopro_heartbeat(chan, msg);
            break;
        }
        break;
#endif
    }
}

/// momentary switch to cycle camera modes
void AP_Camera::cam_mode_toggle()
{
    switch (get_trigger_type()) {
#if HAL_SOLO_GIMBAL_ENABLED
    case CamTrigType::gopro:
        AP_Camera_SoloGimbal::gopro_capture_mode_toggle();
        break;
#endif
    default:
        // no other cameras use this yet
        break;
    }
}

/// decode deprecated MavLink message that controls camera.
void
AP_Camera::control_msg(const mavlink_message_t &msg)
{
    __mavlink_digicam_control_t packet;
    mavlink_msg_digicam_control_decode(&msg, &packet);

    control(packet.session, packet.zoom_pos, packet.zoom_step, packet.focus_lock, packet.shot, packet.command_id);
}

void AP_Camera::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{
    // we cannot process the configure command so convert to mavlink message
    // and send to all components in case they and process it

    mavlink_command_long_t mav_cmd_long = {};

    // convert mission command to mavlink command_long
    mav_cmd_long.command = MAV_CMD_DO_DIGICAM_CONFIGURE;
    mav_cmd_long.param1 = shooting_mode;
    mav_cmd_long.param2 = shutter_speed;
    mav_cmd_long.param3 = aperture;
    mav_cmd_long.param4 = ISO;
    mav_cmd_long.param5 = exposure_type;
    mav_cmd_long.param6 = cmd_id;
    mav_cmd_long.param7 = engine_cutoff_time;

    // send to all components
    GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&mav_cmd_long, sizeof(mav_cmd_long));

    if (_type == AP_Camera::CAMERA_TYPE_BMMCC) {
        // Set a trigger for the additional functions that are flip controlled (so far just ISO and Record Start / Stop use this method, will add others if required)
        _trigger_counter_cam_function = constrain_int16(_trigger_duration*5,0,255);

        // If the message contains non zero values then use them for the below functions
        if (ISO > 0) {
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_iso, _servo_on_pwm);
        }

        if (aperture > 0) {
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_aperture, (int)aperture);
        }

        if (shutter_speed > 0) {
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_shutter_speed, (int)shutter_speed);
        }

        // Use the shooting mode PWM value for the BMMCC as the focus control - no need to modify or create a new MAVlink message type.
        if (shooting_mode > 0) {
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_focus, (int)shooting_mode);
        }
    }
}

void AP_Camera::control(float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id)
{
    // take picture
    if (is_equal(shooting_cmd,1.0f)) {
        trigger_pic();
    }

    mavlink_command_long_t mav_cmd_long = {};

    // convert command to mavlink command long
    mav_cmd_long.command = MAV_CMD_DO_DIGICAM_CONTROL;
    mav_cmd_long.param1 = session;
    mav_cmd_long.param2 = zoom_pos;
    mav_cmd_long.param3 = zoom_step;
    mav_cmd_long.param4 = focus_lock;
    mav_cmd_long.param5 = shooting_cmd;
    mav_cmd_long.param6 = cmd_id;

    // send to all components
    GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&mav_cmd_long, sizeof(mav_cmd_long));
}

/*
  Send camera feedback to the GCS
 */
void AP_Camera::send_feedback(mavlink_channel_t chan) const
{

    int32_t altitude = 0;
    if (feedback.location.initialised() && !feedback.location.get_alt_cm(Location::AltFrame::ABSOLUTE, altitude)) {
        // completely ignore this failure!  this is a shouldn't-happen
        // as current_loc should never be in an altitude we can't
        // convert.
    }
    int32_t altitude_rel = 0;
    if (feedback.location.initialised() && !feedback.location.get_alt_cm(Location::AltFrame::ABOVE_HOME, altitude_rel)) {
        // completely ignore this failure!  this is a shouldn't-happen
        // as current_loc should never be in an altitude we can't
        // convert.
    }

    mavlink_msg_camera_feedback_send(
        chan,
        feedback.timestamp_us,
        0, 0, _image_index,
        feedback.location.lat,
        feedback.location.lng,
        altitude*1e-2f, altitude_rel*1e-2f,
        feedback.roll_sensor*1e-2f,
        feedback.pitch_sensor*1e-2f,
        feedback.yaw_sensor*1e-2f,
        0.0f, CAMERA_FEEDBACK_PHOTO,
        feedback.camera_trigger_logged);
}


/*
  update; triggers by distance moved and camera trigger
*/
void AP_Camera::update()
{
    update_trigger();

    if (AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }

    if (is_zero(_trigg_dist)) {
        _last_location.lat = 0;
        _last_location.lng = 0;
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();
    Location current_loc;
    if (!ahrs.get_location(current_loc)) {
        // completely ignore this failure!  AHRS will provide its best guess.
    }

    if (_last_location.lat == 0 && _last_location.lng == 0) {
        _last_location = current_loc;
        return;
    }
    if (_last_location.lat == current_loc.lat && _last_location.lng == current_loc.lng) {
        // we haven't moved - this can happen as update() may
        // be called without a new GPS fix
        return;
    }

    if (current_loc.get_distance(_last_location) < _trigg_dist) {
        return;
    }

    if (_max_roll > 0 && fabsf(AP::ahrs().roll_sensor*1e-2f) > _max_roll) {
        return;
    }

    if (_is_in_auto_mode != true && _auto_mode_only != 0) {
        return;
    }

    uint32_t tnow = AP_HAL::millis();
    if (tnow - _last_photo_time < (unsigned) _min_interval) {
        return;
    }

    take_picture();

    _last_location = current_loc;
    _last_photo_time = tnow;
}

/*
  interrupt handler for interrupt based feedback trigger
 */
void AP_Camera::feedback_pin_isr(uint8_t pin, bool high, uint32_t timestamp_us)
{
    _feedback_trigger_timestamp_us = timestamp_us;
    _camera_trigger_count++;
}

/*
  check if feedback pin is high for timer based feedback trigger, when
  attach_interrupt fails
 */
void AP_Camera::feedback_pin_timer(void)
{
    uint8_t pin_state = hal.gpio->read(_feedback_pin);
    uint8_t trigger_polarity = _feedback_polarity==0?0:1;
    if (pin_state == trigger_polarity &&
        _last_pin_state != trigger_polarity) {
        _feedback_trigger_timestamp_us = AP_HAL::micros();
        _camera_trigger_count++;
    }
    _last_pin_state = pin_state;
}

/*
  setup a callback for a feedback pin. When on PX4 with the right FMU
  mode we can use the microsecond timer.
 */
void AP_Camera::setup_feedback_callback(void)
{
    if (_feedback_pin <= 0 || _timer_installed || _isr_installed) {
        // invalid or already installed
        return;
    }

    // ensure we are in input mode
    hal.gpio->pinMode(_feedback_pin, HAL_GPIO_INPUT);

    // enable pullup/pulldown
    uint8_t trigger_polarity = _feedback_polarity==0?0:1;
    hal.gpio->write(_feedback_pin, !trigger_polarity);

    if (hal.gpio->attach_interrupt(_feedback_pin, FUNCTOR_BIND_MEMBER(&AP_Camera::feedback_pin_isr, void, uint8_t, bool, uint32_t),
                                   trigger_polarity?AP_HAL::GPIO::INTERRUPT_RISING:AP_HAL::GPIO::INTERRUPT_FALLING)) {
        _isr_installed = true;
    } else {
        // install a 1kHz timer to check feedback pin
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Camera::feedback_pin_timer, void));

        _timer_installed = true;
    }
}

// log_picture - log picture taken and send feedback to GCS
void AP_Camera::log_picture()
{
    if (!using_feedback_pin()) {
        // if we're using a feedback pin then when the event occurs we
        // stash the feedback data.  Since we're not using a feedback
        // pin, we just use "now".
        prep_mavlink_msg_camera_feedback(AP::gps().time_epoch_usec());
    }

    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }
    if (!logger->should_log(log_camera_bit)) {
        return;
    }

    if (!using_feedback_pin()) {
        Write_Camera();
    } else {
        Write_Trigger();
    }
}

// take_picture - take a picture
void AP_Camera::take_picture()
{
    // take a local picture:
    trigger_pic();

    // tell all of our components to take a picture:
    mavlink_command_long_t cmd_msg {};
    cmd_msg.command = MAV_CMD_DO_DIGICAM_CONTROL;
    cmd_msg.param5 = 1;

    // forward to all components
    GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&cmd_msg, sizeof(cmd_msg));
}

// start/stop recording video.  returns true on success
// start_recording should be true to start recording, false to stop recording
bool AP_Camera::record_video(bool start_recording)
{
#if HAL_MOUNT_ENABLED
    // only mount implements recording video
    if (get_trigger_type() == CamTrigType::mount) {
        AP_Mount* mount = AP::mount();
        if (mount != nullptr) {
            return mount->record_video(0, start_recording);
        }
    }
#endif
    return false;
}

// zoom in, out or hold.  returns true on success
// zoom out = -1, hold = 0, zoom in = 1
bool AP_Camera::set_zoom_step(int8_t zoom_step)
{
#if HAL_MOUNT_ENABLED
    // only mount implements set_zoom_step
    if (get_trigger_type() == CamTrigType::mount) {
        AP_Mount* mount = AP::mount();
        if (mount != nullptr) {
            return mount->set_zoom_step(0, zoom_step);
        }
    }
#endif
    return false;
}

// focus in, out or hold.  returns true on success
// focus in = -1, focus hold = 0, focus out = 1
bool AP_Camera::set_manual_focus_step(int8_t focus_step)
{
#if HAL_MOUNT_ENABLED
    // only mount implements set_manual_focus_step
    if (get_trigger_type() == CamTrigType::mount) {
        AP_Mount* mount = AP::mount();
        if (mount != nullptr) {
            return mount->set_manual_focus_step(0, focus_step);
        }
    }
#endif
    return false;
}

// auto focus.  returns true on success
bool AP_Camera::set_auto_focus()
{
#if HAL_MOUNT_ENABLED
    // only mount implements set_auto_focus
    if (get_trigger_type() == CamTrigType::mount) {
        AP_Mount* mount = AP::mount();
        if (mount != nullptr) {
            return mount->set_auto_focus(0);
        }
    }
#endif
    return false;
}

void AP_Camera::prep_mavlink_msg_camera_feedback(uint64_t timestamp_us)
{
    const AP_AHRS &ahrs = AP::ahrs();
    if (!ahrs.get_location(feedback.location)) {
        // completely ignore this failure!  AHRS will provide its best guess.
    }
    feedback.timestamp_us = timestamp_us;
    feedback.roll_sensor = ahrs.roll_sensor;
    feedback.pitch_sensor = ahrs.pitch_sensor;
    feedback.yaw_sensor = ahrs.yaw_sensor;
    feedback.camera_trigger_logged = _camera_trigger_logged;

    gcs().send_message(MSG_CAMERA_FEEDBACK);
}

/*
  update camera trigger - 50Hz
 */
void AP_Camera::update_trigger()
{
    trigger_pic_cleanup();
    
    if (_camera_trigger_logged != _camera_trigger_count) {
        uint32_t timestamp32 = _feedback_trigger_timestamp_us;
        _camera_trigger_logged = _camera_trigger_count;

        // we should consider doing this inside the ISR and pin_timer
        prep_mavlink_msg_camera_feedback(_feedback_trigger_timestamp_us);

        AP_Logger *logger = AP_Logger::get_singleton();
        if (logger != nullptr) {
            if (logger->should_log(log_camera_bit)) {
                uint32_t tdiff = AP_HAL::micros() - timestamp32;
                uint64_t timestamp = AP_HAL::micros64();
                Write_Camera(timestamp - tdiff);
            }
        }
    }
}

AP_Camera::CamTrigType AP_Camera::get_trigger_type(void)
{
    uint8_t type = _trigger_type.get();

    switch ((CamTrigType)type) {
        case CamTrigType::servo:
        case CamTrigType::relay:
        case CamTrigType::gopro:
        case CamTrigType::mount:
            return (CamTrigType)type;
        default:
            return CamTrigType::servo;
    }
}

// singleton instance
AP_Camera *AP_Camera::_singleton;

namespace AP {

AP_Camera *camera()
{
    return AP_Camera::get_singleton();
}

}

#endif
