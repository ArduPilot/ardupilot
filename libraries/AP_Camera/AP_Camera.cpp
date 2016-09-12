// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_Camera.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <drivers/drv_input_capture.h>
#include <drivers/drv_pwm_output.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

// ------------------------------
#define CAM_DEBUG DISABLED

const AP_Param::GroupInfo AP_Camera::var_info[] = {
    // @Param: TRIGG_TYPE
    // @DisplayName: Camera shutter (trigger) type
    // @Description: how to trigger the camera to take a picture
    // @Values: 0:Servo,1:Relay
    // @User: Standard
    AP_GROUPINFO("TRIGG_TYPE",  0, AP_Camera, _trigger_type, AP_CAMERA_TRIGGER_DEFAULT_TRIGGER_TYPE),

    // @Param: DURATION
    // @DisplayName: Duration that shutter is held open
    // @Description: How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
    // @Units: seconds
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("DURATION",    1, AP_Camera, _trigger_duration, AP_CAMERA_TRIGGER_DEFAULT_DURATION),

    // @Param: SERVO_ON
    // @DisplayName: Servo ON PWM value
    // @Description: PWM value to move servo to when shutter is activated
    // @Units: pwm
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_ON",    2, AP_Camera, _servo_on_pwm, AP_CAMERA_SERVO_ON_PWM),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: PWM value to move servo to when shutter is deactivated
    // @Units: pwm
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF",   3, AP_Camera, _servo_off_pwm, AP_CAMERA_SERVO_OFF_PWM),

    // @Param: TRIGG_DIST
    // @DisplayName: Camera trigger distance
    // @Description: Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
    // @User: Standard
    // @Units: meters
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
    // @Units: milliseconds
    // @Range: 0 10000
    AP_GROUPINFO("MIN_INTERVAL",  6, AP_Camera, _min_interval, 0),

    // @Param: MAX_ROLL
    // @DisplayName: Maximum photo roll angle.
    // @Description: Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).
    // @User: Standard
    // @Units: Degrees
    // @Range: 0 180
    AP_GROUPINFO("MAX_ROLL",  7, AP_Camera, _max_roll, 0),
 
    // @Param: FEEDBACK_PIN
    // @DisplayName: Camera feedback pin
    // @Description: pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection. See also the CAM_FEEDBACK_POL option. If using AUX4 pin on a Pixhawk then a fast capture method is used that allows for the trigger time to be as short as one microsecond.
    // @Values: -1:Disabled,50:PX4 AUX1,51:PX4 AUX2,52:PX4 AUX3,53:PX4 AUX4(fast capture),54:PX4 AUX5,55:PX4 AUX6
    // @User: Standard
    AP_GROUPINFO("FEEDBACK_PIN",  8, AP_Camera, _feedback_pin, AP_CAMERA_FEEDBACK_DEFAULT_FEEDBACK_PIN),

    // @Param: FEEDBACK_POL
    // @DisplayName: Camera feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the feedback pin should go high on trigger. If set to 0 then it should go low
    // @Values: 0:TriggerLow,1:TriggerHigh
    // @User: Standard
    AP_GROUPINFO("FEEDBACK_POL",  9, AP_Camera, _feedback_polarity, 1),
    
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/*
  static trigger var for PX4 callback
 */
volatile bool   AP_Camera::_camera_triggered;

/// Servo operated camera
void
AP_Camera::servo_pic()
{
	RC_Channel_aux::set_radio(RC_Channel_aux::k_cam_trigger, _servo_on_pwm);

	// leave a message that it should be active for this many loops (assumes 50hz loops)
	_trigger_counter = constrain_int16(_trigger_duration*5,0,255);
}

/// basic relay activation
void
AP_Camera::relay_pic()
{
    if (_relay_on) {
        _apm_relay->on(0);
    } else {
        _apm_relay->off(0);
    }

    // leave a message that it should be active for this many loops (assumes 50hz loops)
    _trigger_counter = constrain_int16(_trigger_duration*5,0,255);
}

/// single entry point to take pictures
///  set send_mavlink_msg to true to send DO_DIGICAM_CONTROL message to all components
void
AP_Camera::trigger_pic(bool send_mavlink_msg)
{
    setup_feedback_callback();

    _image_index++;
    switch (_trigger_type)
    {
    case AP_CAMERA_TRIGGER_TYPE_SERVO:
        servo_pic();                    // Servo operated camera
        break;
    case AP_CAMERA_TRIGGER_TYPE_RELAY:
        relay_pic();                    // basic relay activation
        break;
    }

    if (send_mavlink_msg) {
        // create command long mavlink message
        mavlink_command_long_t cmd_msg;
        memset(&cmd_msg, 0, sizeof(cmd_msg));
        cmd_msg.command = MAV_CMD_DO_DIGICAM_CONTROL;
        cmd_msg.param5 = 1;
        // create message
        mavlink_message_t msg;
        mavlink_msg_command_long_encode(0, 0, &msg, &cmd_msg);

        // forward to all components
        GCS_MAVLINK::send_to_components(&msg);
    }
}

/// de-activate the trigger after some delay, but without using a delay() function
/// should be called at 50hz
void
AP_Camera::trigger_pic_cleanup()
{
    if (_trigger_counter) {
        _trigger_counter--;
    } else {
        switch (_trigger_type) {
            case AP_CAMERA_TRIGGER_TYPE_SERVO:
                RC_Channel_aux::set_radio(RC_Channel_aux::k_cam_trigger, _servo_off_pwm);
                break;
            case AP_CAMERA_TRIGGER_TYPE_RELAY:
                if (_relay_on) {
                    _apm_relay->off(0);
                } else {
                    _apm_relay->on(0);
                }
                break;
        }
    }
}

/// decode deprecated MavLink message that controls camera.
void
AP_Camera::control_msg(mavlink_message_t* msg)
{
    __mavlink_digicam_control_t packet;
    mavlink_msg_digicam_control_decode(msg, &packet);

    control(packet.session, packet.zoom_pos, packet.zoom_step, packet.focus_lock, packet.shot, packet.command_id);
}

void AP_Camera::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{
    // we cannot process the configure command so convert to mavlink message
    // and send to all components in case they and process it

    mavlink_message_t msg;
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

    // Encode Command long into MAVLINK msg
    mavlink_msg_command_long_encode(0, 0, &msg, &mav_cmd_long);

    // send to all components
    GCS_MAVLINK::send_to_components(&msg);
}

bool AP_Camera::control(float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id)
{
    bool ret = false;
    
    // take picture
    if (is_equal(shooting_cmd,1.0f)) {
        trigger_pic(false);
        ret = true;
    }

    mavlink_message_t msg;
    mavlink_command_long_t mav_cmd_long = {};

    // convert command to mavlink command long
    mav_cmd_long.command = MAV_CMD_DO_DIGICAM_CONTROL;
    mav_cmd_long.param1 = session;
    mav_cmd_long.param2 = zoom_pos;
    mav_cmd_long.param3 = zoom_step;
    mav_cmd_long.param4 = focus_lock;
    mav_cmd_long.param5 = shooting_cmd;
    mav_cmd_long.param6 = cmd_id;

    // Encode Command long into MAVLINK msg
    mavlink_msg_command_long_encode(0, 0, &msg, &mav_cmd_long);

    // send to all components
    GCS_MAVLINK::send_to_components(&msg);
    return ret;
}

/*
  Send camera feedback to the GCS
 */
void AP_Camera::send_feedback(mavlink_channel_t chan, AP_GPS &gps, const AP_AHRS &ahrs, const Location &current_loc)
{
    float altitude, altitude_rel;
    if (current_loc.flags.relative_alt) {
        altitude = current_loc.alt+ahrs.get_home().alt;
        altitude_rel = current_loc.alt;
    } else {
        altitude = current_loc.alt;
        altitude_rel = current_loc.alt - ahrs.get_home().alt;
    }

    mavlink_msg_camera_feedback_send(chan, 
        gps.time_epoch_usec(),
        0, 0, _image_index,
        current_loc.lat, current_loc.lng,
        altitude/100.0f, altitude_rel/100.0f,
        ahrs.roll_sensor/100.0f, ahrs.pitch_sensor/100.0f, ahrs.yaw_sensor/100.0f,
        0.0f,CAMERA_FEEDBACK_PHOTO);
}


/*  update location, for triggering by GPS distance moved
    This function returns true if a picture should be taken
    The caller is responsible for taking the picture based on the return value of this function.
    The caller is also responsible for logging the details about the photo
*/
bool AP_Camera::update_location(const struct Location &loc, const AP_AHRS &ahrs)
{
    if (is_zero(_trigg_dist)) {
        return false;
    }
    if (_last_location.lat == 0 && _last_location.lng == 0) {
        _last_location = loc;
        return false;
    }
    if (_last_location.lat == loc.lat && _last_location.lng == loc.lng) {
        // we haven't moved - this can happen as update_location() may
        // be called without a new GPS fix
        return false;
    }

    if (get_distance(loc, _last_location) < _trigg_dist) {
        return false;
    }

    if (_max_roll > 0 && labs(ahrs.roll_sensor/100) > _max_roll) {
        return false;
    }

    uint32_t tnow = AP_HAL::millis();
    if (tnow - _last_photo_time < (unsigned) _min_interval) {
        return false;
    }  else {
        _last_location = loc;
        _last_photo_time = tnow;
        return true;
    }
}

/*
  check if feedback pin is high
 */
void AP_Camera::feedback_pin_timer(void)
{
    int8_t dpin = hal.gpio->analogPinToDigitalPin(_feedback_pin);
    if (dpin == -1) {
        return;
    }
    // ensure we are in input mode
    hal.gpio->pinMode(dpin, HAL_GPIO_INPUT);

    // enable pullup
    hal.gpio->write(dpin, 1);

    uint8_t pin_state = hal.gpio->read(dpin);
    uint8_t trigger_polarity = _feedback_polarity==0?0:1;
    if (pin_state == trigger_polarity &&
        _last_pin_state != trigger_polarity) {
        _camera_triggered = true;
    }
    _last_pin_state = pin_state;
}

/*
  check if camera has triggered
 */
bool AP_Camera::check_trigger_pin(void)
{
    if (_camera_triggered) {
        _camera_triggered = false;
        return true;
    }
    return false;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
/*
  callback for timer capture on PX4
 */
void AP_Camera::capture_callback(void *context, uint32_t chan_index,
                                 hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow)
{
    _camera_triggered = true;    
}
#endif

/*
  setup a callback for a feedback pin. When on PX4 with the right FMU
  mode we can use the microsecond timer.
 */
void AP_Camera::setup_feedback_callback(void)
{
    if (_feedback_pin <= 0 || _timer_installed) {
        // invalid or already installed
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    /*
      special case for pin 53 on PX4. We can use the fast timer support
     */
    if (_feedback_pin == 53) {
        int fd = open("/dev/px4fmu", 0);
        if (fd != -1) {
            if (ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_MODE_3PWM1CAP) != 0) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Camera: unable to setup 3PWM1CAP\n");
                close(fd);
                goto failed;
            }   
            if (up_input_capture_set(3, _feedback_polarity==1?Rising:Falling, 0, capture_callback, this) != 0) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Camera: unable to setup timer capture\n");
                close(fd);
                goto failed;
            }
            close(fd);
            _timer_installed = true;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Camera: setup fast trigger capture\n");
        }
    }
failed:
#endif // CONFIG_HAL_BOARD

    if (!_timer_installed) {
        // install a 1kHz timer to check feedback pin
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Camera::feedback_pin_timer, void));
    }
    _timer_installed = true;
}
