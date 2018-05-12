#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount_STorM32_native.h"

extern const AP_HAL::HAL& hal;

AP_Mount_STorM32_native::AP_Mount_STorM32_native(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance)
{
    //these need to be initialized to the following values
// but doesn't need to be done explicitly, since they are zero
//    _initialised = false;
//    _armed = false;
//    _task_time_last = 0;
//    _task_counter = TASK_SLOT0;
//    _send_armeddisarmed = false;
//    _mount_type = AP_Mount::Mount_Type_None; //the mount type will be determined in init()

    _bitmask = SEND_STORM32LINK_V2 | SEND_CMD_SETINPUTS | SEND_CMD_DOCAMERA;
}

//------------------------------------------------------
// AP_Mount_STorM32_native interface functions
//------------------------------------------------------

// init - performs any required initialisation for this instance
void AP_Mount_STorM32_native::init(const AP_SerialManager& serial_manager)
{
    //from instance we can determine its type, we keep it here since that's easier/shorter
    _mount_type = _frontend.get_mount_type(_instance);

    // it should never happen that it's not one of them, but let's enforce it, to not depend on the outside
    if (_mount_type != AP_Mount::Mount_Type_STorM32_native) {
        _mount_type = AP_Mount::Mount_Type_None;
    }

    if (_mount_type == AP_Mount::Mount_Type_STorM32_native) {
        _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_STorM32_Native, 0);
        if (_uart) {
            _serial_is_initialised = true; //tell the STorM32_lib class
            //we do not set _initialised = true, since we first need to pass find_gimbal()
        } else {
            _serial_is_initialised = false; //tell the STorM32_lib class
            _mount_type = AP_Mount::Mount_Type_None; //this prevents many things from happening, safety guard
        }
    }

    set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get()); //set mode to default value set by user via parameter
    _target_mode_last = _state._mode;
}

// update mount position - should be called periodically
// this function must be defined in any case
void AP_Mount_STorM32_native::update()
{
    if (!_initialised) {
        find_gimbal_native(); //this searches for a gimbal on serial
        return;
    }

    send_text_to_gcs();
}

// 400 Hz loop
void AP_Mount_STorM32_native::update_fast()
{
    if (!_initialised) {
        return;
    }

    //slow down everything to 100 Hz
    // we can't use update(), since 50 Hz isn't compatible with the desired 20 Hz STorM32Link rate
    // each message is send at 20 Hz i.e. 50 ms, for 5 task slots => 10 ms per task slot
    uint64_t current_time_ms = AP_HAL::millis64();
    if ((current_time_ms - _task_time_last) >= 10) {
        _task_time_last = current_time_ms;

        const uint16_t LIVEDATA_FLAGS = LIVEDATA_STATUS_V2 | LIVEDATA_ATTITUDE_RELATIVE;

        switch (_task_counter) {
            case TASK_SLOT0:
                if (_bitmask & SEND_STORM32LINK_V2) {
                    send_storm32link_v2(_frontend._ahrs); //2.3ms
                }

                break;
            case TASK_SLOT1:
                // trigger live data
                if (_mount_type == AP_Mount::Mount_Type_STorM32_native) {
                    receive_reset_wflush(); //we are brutal and kill all incoming bytes
                    send_cmd_getdatafields(LIVEDATA_FLAGS); //0.6ms
                }

                // send do_camera here
                // not currently supported

                break;
            case TASK_SLOT2:
                set_target_angles_bymountmode();
                send_target_angles(); //1.7 ms or 1.0ms

                break;
            case TASK_SLOT3:
                if (_bitmask & SEND_CMD_SETINPUTS) {
                    send_cmd_setinputs(); //2.4ms
                }

                break;
            case TASK_SLOT4:
                // receive live data
                if (_mount_type == AP_Mount::Mount_Type_STorM32_native) {
                    do_receive(); //we had now 4/5*50ms = 40ms time, this should be more than enough
                    if (message_received() && (_serial_in.cmd == 0x06) && (_serial_in.getdatafields.flags == LIVEDATA_FLAGS)) {
                        // attitude angles are in STorM32 convention
                        // convert from STorM32 to ArduPilot convention, this need correction p:-1,r:+1,y:-1
                        set_status_angles_deg(
                            -_serial_in.getdatafields.livedata_attitude.pitch_deg,
                             _serial_in.getdatafields.livedata_attitude.roll_deg,
                            -_serial_in.getdatafields.livedata_attitude.yaw_deg );
                        // we also can check if the gimbal is in normal mode
                        bool _armed_new = is_normal_state(_serial_in.getdatafields.livedata_status.state);
                        if (_armed_new != _armed) {
                            _send_armeddisarmed = true;
                        }
                        _armed = _armed_new;
                    }
                }

                break;
        }

        _task_counter++;
        if (_task_counter >= TASK_SLOTNUMBER) {
            _task_counter = 0;
        }
    }
}

// set_mode - sets mount's mode
void AP_Mount_STorM32_native::set_mode(enum MAV_MOUNT_MODE mode)
{
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_STorM32_native::status_msg(mavlink_channel_t chan)
{
    if (!_initialised) {
        return;
    }

    float pitch_deg, roll_deg, yaw_deg;

    get_status_angles_deg(&pitch_deg, &roll_deg, &yaw_deg);

    // MAVLink MOUNT_STATUS: int32_t pitch(deg*100), int32_t roll(deg*100), int32_t yaw(deg*100)
    mavlink_msg_mount_status_send(chan, 0, 0, pitch_deg*100.0f, roll_deg*100.0f, yaw_deg*100.0f);
}

//------------------------------------------------------
// AP_Mount_STorM32_native private function
//------------------------------------------------------

void AP_Mount_STorM32_native::set_target_angles_bymountmode(void)
{
    uint16_t pitch_pwm, roll_pwm, yaw_pwm;

    bool get_pwm_target_from_radio = (_bitmask & GET_PWM_TARGET_FROM_RADIO) ? true : false;

    // flag to trigger sending target angles to gimbal
    bool send_ef_target = false;
    bool send_pwm_target = false;

    // update based on mount mode
    enum MAV_MOUNT_MODE mount_mode = get_mode();

    switch (mount_mode) {
        // move mount to a "retracted" position.
        case MAV_MOUNT_MODE_RETRACT:
            {
                const Vector3f &target = _state._retract_angles.get();
                _angle_ef_target_rad.x = radians(target.x);
                _angle_ef_target_rad.y = radians(target.y);
                _angle_ef_target_rad.z = radians(target.z);
                send_ef_target = true;
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
                const Vector3f &target = _state._neutral_angles.get();
                _angle_ef_target_rad.x = radians(target.x);
                _angle_ef_target_rad.y = radians(target.y);
                _angle_ef_target_rad.z = radians(target.z);
                send_ef_target = true;
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            send_ef_target = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            if (get_pwm_target_from_radio) {
                get_pwm_target_angles_from_radio(&pitch_pwm, &roll_pwm, &yaw_pwm);
                send_pwm_target = true;
            } else {
                update_targets_from_rc();
                send_ef_target = true;
            }
            if (is_failsafe()) {
                pitch_pwm = roll_pwm = yaw_pwm = 1500;
                _angle_ef_target_rad.y = _angle_ef_target_rad.x = _angle_ef_target_rad.z = 0.0f;
            }
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true);
                send_ef_target = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // send target angles
    if (send_ef_target) {
        set_target_angles_rad(_angle_ef_target_rad.y, _angle_ef_target_rad.x, _angle_ef_target_rad.z, mount_mode);
    } else if (send_pwm_target) {
        set_target_angles_pwm(pitch_pwm, roll_pwm, yaw_pwm, mount_mode);
    }
}

void AP_Mount_STorM32_native::get_pwm_target_angles_from_radio(uint16_t* pitch_pwm, uint16_t* roll_pwm, uint16_t* yaw_pwm)
{
    get_valid_pwm_from_channel(_state._tilt_rc_in, pitch_pwm);
    get_valid_pwm_from_channel(_state._roll_rc_in, roll_pwm);
    get_valid_pwm_from_channel(_state._pan_rc_in, yaw_pwm);
}

void AP_Mount_STorM32_native::get_valid_pwm_from_channel(uint8_t rc_in, uint16_t* pwm)
{
    #define rc_ch(i) RC_Channels::rc_channel(i-1)

    if (rc_in && (rc_ch(rc_in))) {
        *pwm = rc_ch(rc_in)->get_radio_in();
    } else {
        *pwm = 1500;
    }
}

void AP_Mount_STorM32_native::set_target_angles_rad(float pitch_rad, float roll_rad, float yaw_rad, enum MAV_MOUNT_MODE mount_mode)
{
    _target.deg.pitch = degrees(pitch_rad);
    _target.deg.roll = degrees(roll_rad);
    _target.deg.yaw = degrees(yaw_rad);
    _target.type = ANGLES_DEG;
    _target.mode = mount_mode;
}

void AP_Mount_STorM32_native::set_target_angles_pwm(uint16_t pitch_pwm, uint16_t roll_pwm, uint16_t yaw_pwm, enum MAV_MOUNT_MODE mount_mode)
{
    _target.pwm.pitch = pitch_pwm;
    _target.pwm.roll = roll_pwm;
    _target.pwm.yaw = yaw_pwm;
    _target.type = ANGLES_PWM;
    _target.mode = mount_mode;
}

void AP_Mount_STorM32_native::send_target_angles(void)
{
    if (_target.mode <= MAV_MOUNT_MODE_NEUTRAL) { //RETRACT and NEUTRAL
        if (_target_mode_last != _target.mode) { // only do it once, i.e., when mode has just changed
            // trigger a recenter camera, this clears all internal Remote states
            // the camera does not need to be recentered explicitly, thus return
            send_cmd_recentercamera();
            _target_mode_last = _target.mode;
        }
        return;
    }

    // update to current mode, to avoid repeated actions on some mount mode changes
    _target_mode_last = _target.mode;

    if (_target.type == ANGLES_PWM) {
        uint16_t pitch_pwm = _target.pwm.pitch;
        uint16_t roll_pwm = _target.pwm.roll;
        uint16_t yaw_pwm = _target.pwm.yaw;

        const uint16_t DZ = 10;

        if (pitch_pwm < 10) {
            pitch_pwm = 1500;
        }
        if (pitch_pwm < 1500-DZ) {
            pitch_pwm += DZ;
        } else if (pitch_pwm > 1500+DZ) {
            pitch_pwm -= DZ;
        } else {
            pitch_pwm = 1500;
        }

        if (roll_pwm < 10) {
            roll_pwm = 1500;
        }
        if (roll_pwm < 1500-DZ) {
            roll_pwm += DZ;
        } else if (roll_pwm > 1500+DZ) {
            roll_pwm -= DZ;
        } else {
            roll_pwm = 1500;
        }

        if (yaw_pwm < 10) {
            yaw_pwm = 1500;
        }
        if (yaw_pwm < 1500-DZ) {
            yaw_pwm += DZ;
        } else if (yaw_pwm > 1500+DZ) {
            yaw_pwm -= DZ;
        } else {
            yaw_pwm = 1500;
        }

        send_cmd_setpitchrollyaw(pitch_pwm, roll_pwm, yaw_pwm);
    } else {
        float pitch_deg = _target.deg.pitch;
        float roll_deg = _target.deg.roll;
        float yaw_deg = _target.deg.yaw;

        //convert from ArduPilot to STorM32 convention, this need correction p:-1,r:+1,y:-1
        send_cmd_setangles(-pitch_deg, roll_deg, -yaw_deg, 0);
    }
}

//------------------------------------------------------
// status angles handlers, angles are in ArduPilot convention
//------------------------------------------------------

void AP_Mount_STorM32_native::set_status_angles_deg(float pitch_deg, float roll_deg, float yaw_deg)
{
    _status.pitch_deg = pitch_deg;
    _status.roll_deg = roll_deg;
    _status.yaw_deg = yaw_deg;
}

void AP_Mount_STorM32_native::get_status_angles_deg(float* pitch_deg, float* roll_deg, float* yaw_deg)
{
    *pitch_deg = _status.pitch_deg;
    *roll_deg = _status.roll_deg;
    *yaw_deg = _status.yaw_deg;
}

//------------------------------------------------------
// discovery functions
//------------------------------------------------------

void AP_Mount_STorM32_native::find_gimbal_native(void)
{
    if (_mount_type != AP_Mount::Mount_Type_STorM32_native) {
        return;
    }

    if (!_serial_is_initialised) {
        return;
    }

    uint64_t current_time_ms = AP_HAL::millis64();

#if FIND_GIMBAL_MAX_SEARCH_TIME_MS
    if (current_time_ms > FIND_GIMBAL_MAX_SEARCH_TIME_MS) {
        _initialised = false; //should be already false, but it can't hurt to ensure that
        _serial_is_initialised = false; //switch off STorM32_lib
        _mount_type = AP_Mount::Mount_Type_None; //switch off finally, also makes find_gimbal() to stop searching
        return;
    }
#endif

    if ((current_time_ms - _task_time_last) > 100) { //try it every 100 ms
        _task_time_last = current_time_ms;

        switch (_task_counter) {
            case TASK_SLOT0:
                // send GETVERSIONSTR
                receive_reset_wflush(); //we are brutal and kill all incoming bytes
                send_cmd_getversionstr();
                break;
            case TASK_SLOT1:
                // receive GETVERSIONSTR response
                do_receive();
                if (message_received() && (_serial_in.cmd == 0x02)) {
                    _task_counter = TASK_SLOT0;
                    _initialised = true;
                    return; //done, get out of here
                }
                break;
            default:
                // skip
                break;
        }
        _task_counter++;
        if (_task_counter >= 3) {
            _task_counter = 0;
        }
    }
}

void AP_Mount_STorM32_native::send_text_to_gcs(void)
{
    if (!_initialised) {
        return;
    }

    if (_send_armeddisarmed) {
        _send_armeddisarmed = false;
        gcs().send_text(MAV_SEVERITY_INFO, (_armed) ? "  STorM32: ARMED" : "  STorM32: DISARMED" );
    }
}

//------------------------------------------------------
// interfaces to STorM32_lib
//------------------------------------------------------

size_t AP_Mount_STorM32_native::_serial_txspace(void)
{
    if (_mount_type == AP_Mount::Mount_Type_STorM32_native) {
        return (size_t)_uart->txspace();
    }
    return 0;
}

size_t AP_Mount_STorM32_native::_serial_write(const uint8_t *buffer, size_t size, uint8_t priority)
{
    if (_mount_type == AP_Mount::Mount_Type_STorM32_native) {
        return _uart->write(buffer, size);
    }
    return 0;
}

uint32_t AP_Mount_STorM32_native::_serial_available(void)
{
    if (_mount_type == AP_Mount::Mount_Type_STorM32_native) {
        return _uart->available();
    }
    return 0;
}

int16_t AP_Mount_STorM32_native::_serial_read(void)
{
    if (_mount_type == AP_Mount::Mount_Type_STorM32_native) {
        return _uart->read();
    }
    return 0;
}

uint16_t AP_Mount_STorM32_native::_rcin_read(uint8_t ch)
{
    return hal.rcin->read(ch);
}

//------------------------------------------------------
// helper
//------------------------------------------------------

bool AP_Mount_STorM32_native::is_failsafe(void)
{
    #define rc_ch(i) RC_Channels::rc_channel(i-1)

    uint8_t roll_rc_in = _state._roll_rc_in;
    uint8_t tilt_rc_in = _state._tilt_rc_in;
    uint8_t pan_rc_in = _state._pan_rc_in;

    if (roll_rc_in && (rc_ch(roll_rc_in)) && (rc_ch(roll_rc_in)->get_radio_in() < 700)) {
        return true;
    }
    if (tilt_rc_in && (rc_ch(tilt_rc_in)) && (rc_ch(tilt_rc_in)->get_radio_in() < 700)) {
        return true;
    }
    if (pan_rc_in && (rc_ch(pan_rc_in)) && (rc_ch(pan_rc_in)->get_radio_in() < 700)) {
        return true;
    }

    return false;
}





