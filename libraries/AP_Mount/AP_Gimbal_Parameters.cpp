#include "AP_Gimbal_Parameters.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const uint32_t AP_Gimbal_Parameters::_retry_period = 3000;
const uint8_t AP_Gimbal_Parameters::_max_fetch_attempts = 5;

const char AP_Gimbal_Parameters::_names[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS][16] = {
    "GMB_OFF_ACC_X",
    "GMB_OFF_ACC_Y",
    "GMB_OFF_ACC_Z",
    "GMB_GN_ACC_X",
    "GMB_GN_ACC_Y",
    "GMB_GN_ACC_Z",
    "GMB_OFF_GYRO_X",
    "GMB_OFF_GYRO_Y",
    "GMB_OFF_GYRO_Z",
    "GMB_OFF_JNT_X",
    "GMB_OFF_JNT_Y",
    "GMB_OFF_JNT_Z",
    "GMB_K_RATE",
    "GMB_POS_HOLD",
    "GMB_MAX_TORQUE",
    "GMB_SYSID"
};

AP_Gimbal_Parameters::AP_Gimbal_Parameters():
_last_request_ms(0)
{
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        _states[i] = GMB_PARAM_NOT_YET_READ;
        _values[i] = 0.0f;
        _param_seen[i] = false;
        _fetch_attempts[i] = 0;
    }
}

void AP_Gimbal_Parameters::fetch_params()
{
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (_states[i] != GMB_PARAM_NOT_YET_READ) {
            _states[i] = GMB_PARAM_FETCH_AGAIN;
        }
    }
}

bool AP_Gimbal_Parameters::initialized()
{
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if(_states[i] == GMB_PARAM_NOT_YET_READ) {
            return false;
        }
    }
    return true;
}

bool AP_Gimbal_Parameters::received_all()
{
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if(_states[i] == GMB_PARAM_NOT_YET_READ || _states[i] == GMB_PARAM_FETCH_AGAIN) {
            return false;
        }
    }
    return true;
}

void AP_Gimbal_Parameters::get_param(char const* name, float& value, float def_val) {
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (!strcmp(name, _names[i])) {
            if (!_param_seen[i]) {
                value = def_val;
            } else {
                value = _values[i];
            }
            return;
        }
    }
}

void AP_Gimbal_Parameters::set_param(mavlink_channel_t chan, char const* name, float value) {
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (!strcmp(name, _names[i])) {
            if ((_states[i] == GMB_PARAM_CONSISTENT && _values[i] == value) || _states[i] == GMB_PARAM_NONEXISTANT) {
                return;
            }
            _states[i] = GMB_PARAM_ATTEMPTING_TO_SET;
            if (_values[i] != value) {
                _values[i] = value;
                mavlink_msg_param_set_send(chan, 0, MAV_COMP_ID_GIMBAL, _names[i], _values[i], MAV_PARAM_TYPE_REAL32);
            }
            _last_request_ms = hal.scheduler->millis();
            return;
        }
    }
}

void AP_Gimbal_Parameters::update(mavlink_channel_t chan)
{
    uint32_t tnow_ms = hal.scheduler->millis();

    // retry initial param retrieval
    if(!received_all()){
        if (tnow_ms-_last_request_ms > _retry_period) {
            _last_request_ms = tnow_ms;
            mavlink_msg_param_request_list_send(chan, 0, MAV_COMP_ID_GIMBAL);

            for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
                if (!_param_seen[i]) {
                    _fetch_attempts[i]++;
                }
            }
        }
    }

    // retry param_set
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (_states[i] == GMB_PARAM_ATTEMPTING_TO_SET && tnow_ms - _last_request_ms > _retry_period) {
            mavlink_msg_param_set_send(chan, 0, MAV_COMP_ID_GIMBAL, _names[i], _values[i], MAV_PARAM_TYPE_REAL32);
            if (!_param_seen[i]) {
                _fetch_attempts[i]++;
            }
        }
    }

    // check for nonexistant parameters
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (!_param_seen[i] && _fetch_attempts[i] > _max_fetch_attempts) {
            _states[i] = GMB_PARAM_NONEXISTANT;
            hal.console->printf("Gimbal parameter %s timed out\n", _names[i]);
        }
    }
}

void AP_Gimbal_Parameters::handle_param_value(DataFlash_Class *dataflash, mavlink_message_t *msg)
{
    mavlink_param_value_t packet;
    mavlink_msg_param_value_decode(msg, &packet);

    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (!strcmp(packet.param_id, _names[i])) {
            _param_seen[i] = true;
            switch(_states[i]) {
                case GMB_PARAM_NONEXISTANT:
                case GMB_PARAM_NOT_YET_READ:
                case GMB_PARAM_FETCH_AGAIN:
                    dataflash->Log_Write_Parameter(packet.param_id, packet.param_value);
                    _values[i] = packet.param_value;
                    _states[i] = GMB_PARAM_CONSISTENT;
                    break;
                case GMB_PARAM_CONSISTENT:
                    _values[i] = packet.param_value;
                    break;
                case GMB_PARAM_ATTEMPTING_TO_SET:
                    if (packet.param_value == _values[i]) {
                        _states[i] = GMB_PARAM_CONSISTENT;
                    }
                    break;
            }
        }
    }
}

Vector3f AP_Gimbal_Parameters::get_accel_bias()
{
    Vector3f ret;
    get_param("GMB_OFF_ACC_X",ret.x);
    get_param("GMB_OFF_ACC_Y",ret.y);
    get_param("GMB_OFF_ACC_Z",ret.z);
    return ret;
}
Vector3f AP_Gimbal_Parameters::get_accel_gain()
{
    Vector3f ret;
    get_param("GMB_GN_ACC_X",ret.x,1.0f);
    get_param("GMB_GN_ACC_Y",ret.y,1.0f);
    get_param("GMB_GN_ACC_Z",ret.z,1.0f);
    return ret;
}
Vector3f AP_Gimbal_Parameters::get_gyro_bias()
{
    Vector3f ret;
    get_param("GMB_OFF_GYRO_X",ret.x);
    get_param("GMB_OFF_GYRO_Y",ret.y);
    get_param("GMB_OFF_GYRO_Z",ret.z);
    return ret;
}
Vector3f AP_Gimbal_Parameters::get_joint_bias()
{
    Vector3f ret;
    get_param("GMB_OFF_JNT_X",ret.x);
    get_param("GMB_OFF_JNT_Y",ret.y);
    get_param("GMB_OFF_JNT_Z",ret.z);
    return ret;
}
float AP_Gimbal_Parameters::get_K_rate()
{
    float ret;
    get_param("GMB_K_RATE",ret);
    return ret;
}
