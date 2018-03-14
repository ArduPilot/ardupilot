#include "SoloGimbal_Parameters.h"
#include <AP_HAL/AP_HAL.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

const uint32_t SoloGimbal_Parameters::_retry_period = 3000;
const uint8_t SoloGimbal_Parameters::_max_fetch_attempts = 5;

SoloGimbal_Parameters::SoloGimbal_Parameters()
{
    reset();
}


void SoloGimbal_Parameters::reset()
{
    memset(_params,0,sizeof(_params));
    _last_request_ms = 0;
    _flashing_step = GMB_PARAM_NOT_FLASHING;
}

const char* SoloGimbal_Parameters::get_param_name(gmb_param_t param)
{
    switch(param) {
        case GMB_PARAM_GMB_OFF_ACC_X:
            return "GMB_OFF_ACC_X";
        case GMB_PARAM_GMB_OFF_ACC_Y:
            return "GMB_OFF_ACC_Y";
        case GMB_PARAM_GMB_OFF_ACC_Z:
            return "GMB_OFF_ACC_Z";
        case GMB_PARAM_GMB_GN_ACC_X:
            return "GMB_GN_ACC_X";
        case GMB_PARAM_GMB_GN_ACC_Y:
            return "GMB_GN_ACC_Y";
        case GMB_PARAM_GMB_GN_ACC_Z:
            return "GMB_GN_ACC_Z";
        case GMB_PARAM_GMB_OFF_GYRO_X:
            return "GMB_OFF_GYRO_X";
        case GMB_PARAM_GMB_OFF_GYRO_Y:
            return "GMB_OFF_GYRO_Y";
        case GMB_PARAM_GMB_OFF_GYRO_Z:
            return "GMB_OFF_GYRO_Z";
        case GMB_PARAM_GMB_OFF_JNT_X:
            return "GMB_OFF_JNT_X";
        case GMB_PARAM_GMB_OFF_JNT_Y:
            return "GMB_OFF_JNT_Y";
        case GMB_PARAM_GMB_OFF_JNT_Z:
            return "GMB_OFF_JNT_Z";
        case GMB_PARAM_GMB_K_RATE:
            return "GMB_K_RATE";
        case GMB_PARAM_GMB_POS_HOLD:
            return "GMB_POS_HOLD";
        case GMB_PARAM_GMB_MAX_TORQUE:
            return "GMB_MAX_TORQUE";
        case GMB_PARAM_GMB_SND_TORQUE:
            return "GMB_SND_TORQUE";
        case GMB_PARAM_GMB_SYSID:
            return "GMB_SYSID";
        case GMB_PARAM_GMB_FLASH:
            return "GMB_FLASH";
        default:
            return "";
    };
}

void SoloGimbal_Parameters::fetch_params()
{
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (_params[i].state != GMB_PARAMSTATE_NOT_YET_READ) {
            _params[i].state = GMB_PARAMSTATE_FETCH_AGAIN;
        }
    }
}

bool SoloGimbal_Parameters::initialized()
{
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if(_params[i].state == GMB_PARAMSTATE_NOT_YET_READ) {
            return false;
        }
    }
    return true;
}

bool SoloGimbal_Parameters::received_all()
{
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if(_params[i].state == GMB_PARAMSTATE_NOT_YET_READ || _params[i].state == GMB_PARAMSTATE_FETCH_AGAIN) {
            return false;
        }
    }
    return true;
}

void SoloGimbal_Parameters::get_param(gmb_param_t param, float& value, float def_val) {
    if (!_params[param].seen) {
        value = def_val;
    } else {
        value = _params[param].value;
    }
}

void SoloGimbal_Parameters::set_param(gmb_param_t param, float value) {
    if ((_params[param].state == GMB_PARAMSTATE_CONSISTENT && param != GMB_PARAM_GMB_FLASH && is_equal(_params[param].value,value)) || _params[param].state == GMB_PARAMSTATE_NONEXISTANT) {
        return;
    }

    _params[param].state = GMB_PARAMSTATE_ATTEMPTING_TO_SET;
    _params[param].value = value;
    mavlink_msg_param_set_send(_chan, 0, MAV_COMP_ID_GIMBAL, get_param_name(param), _params[param].value, MAV_PARAM_TYPE_REAL32);

    _last_request_ms = AP_HAL::millis();
}

void SoloGimbal_Parameters::update()
{
    uint32_t tnow_ms = AP_HAL::millis();

    // retry initial param retrieval
    if(!received_all()){
        if (tnow_ms-_last_request_ms > _retry_period) {
            _last_request_ms = tnow_ms;
            mavlink_msg_param_request_list_send(_chan, 0, MAV_COMP_ID_GIMBAL);

            for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
                if (!_params[i].seen) {
                    _params[i].fetch_attempts++;
                }
            }
        }
    }

    // retry param_set
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (_params[i].state == GMB_PARAMSTATE_ATTEMPTING_TO_SET && tnow_ms - _last_request_ms > _retry_period) {
            mavlink_msg_param_set_send(_chan, 0, MAV_COMP_ID_GIMBAL, get_param_name((gmb_param_t)i), _params[i].value, MAV_PARAM_TYPE_REAL32);
            if (!_params[i].seen) {
                _params[i].fetch_attempts++;
            }
        }
    }

    // check for nonexistent parameters
    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (!_params[i].seen && _params[i].fetch_attempts > _max_fetch_attempts) {
            _params[i].state = GMB_PARAMSTATE_NONEXISTANT;
            hal.console->printf("Gimbal parameter %s timed out\n", get_param_name((gmb_param_t)i));
        }
    }

    if(_flashing_step == GMB_PARAM_FLASHING_WAITING_FOR_SET) {
        bool done = true;
        for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
            if (_params[i].state == GMB_PARAMSTATE_ATTEMPTING_TO_SET) {
                done = false;
                break;
            }
        }

        if (done) {
            _flashing_step = GMB_PARAM_FLASHING_WAITING_FOR_ACK;
            set_param(GMB_PARAM_GMB_FLASH,69.0f);
        }
    }
}

void SoloGimbal_Parameters::handle_param_value(mavlink_message_t *msg)
{
    mavlink_param_value_t packet;
    mavlink_msg_param_value_decode(msg, &packet);

    DataFlash_Class *dataflash = DataFlash_Class::instance();
    if (dataflash != nullptr) {
        dataflash->Log_Write_Parameter(packet.param_id, packet.param_value);
    }

    for(uint8_t i=0; i<MAVLINK_GIMBAL_NUM_TRACKED_PARAMS; i++) {
        if (!strcmp(packet.param_id, get_param_name((gmb_param_t)i))) {
            _params[i].seen = true;
            switch(_params[i].state) {
                case GMB_PARAMSTATE_NONEXISTANT:
                case GMB_PARAMSTATE_NOT_YET_READ:
                case GMB_PARAMSTATE_FETCH_AGAIN:
                    _params[i].value = packet.param_value;
                    _params[i].state = GMB_PARAMSTATE_CONSISTENT;
                    break;
                case GMB_PARAMSTATE_CONSISTENT:
                    _params[i].value = packet.param_value;
                    break;
                case GMB_PARAMSTATE_ATTEMPTING_TO_SET:
                    if (i == GMB_PARAM_GMB_FLASH) {
                        if (_flashing_step == GMB_PARAM_FLASHING_WAITING_FOR_ACK && (int)packet.param_value == 1) {
                            _flashing_step = GMB_PARAM_NOT_FLASHING;
                        }
                        _params[i].value = 0;
                        _params[i].state = GMB_PARAMSTATE_CONSISTENT;
                    } else if (is_equal(packet.param_value,_params[i].value)) {
                        _params[i].state = GMB_PARAMSTATE_CONSISTENT;
                    }
                    break;
            }
            break;
        }
    }
}

Vector3f SoloGimbal_Parameters::get_accel_bias()
{
    Vector3f ret;
    get_param(GMB_PARAM_GMB_OFF_ACC_X,ret.x);
    get_param(GMB_PARAM_GMB_OFF_ACC_Y,ret.y);
    get_param(GMB_PARAM_GMB_OFF_ACC_Z,ret.z);
    return ret;
}
Vector3f SoloGimbal_Parameters::get_accel_gain()
{
    Vector3f ret;
    get_param(GMB_PARAM_GMB_GN_ACC_X,ret.x,1.0f);
    get_param(GMB_PARAM_GMB_GN_ACC_Y,ret.y,1.0f);
    get_param(GMB_PARAM_GMB_GN_ACC_Z,ret.z,1.0f);
    return ret;
}

void SoloGimbal_Parameters::set_accel_bias(const Vector3f& bias)
{
    set_param(GMB_PARAM_GMB_OFF_ACC_X, bias.x);
    set_param(GMB_PARAM_GMB_OFF_ACC_Y, bias.y);
    set_param(GMB_PARAM_GMB_OFF_ACC_Z, bias.z);
}

void SoloGimbal_Parameters::set_accel_gain(const Vector3f& gain)
{
    set_param(GMB_PARAM_GMB_GN_ACC_X, gain.x);
    set_param(GMB_PARAM_GMB_GN_ACC_Y, gain.y);
    set_param(GMB_PARAM_GMB_GN_ACC_Z, gain.z);
}

Vector3f SoloGimbal_Parameters::get_gyro_bias()
{
    Vector3f ret;
    get_param(GMB_PARAM_GMB_OFF_GYRO_X,ret.x);
    get_param(GMB_PARAM_GMB_OFF_GYRO_Y,ret.y);
    get_param(GMB_PARAM_GMB_OFF_GYRO_Z,ret.z);
    return ret;
}

void SoloGimbal_Parameters::set_gyro_bias(const Vector3f& bias)
{
    set_param(GMB_PARAM_GMB_OFF_GYRO_X,bias.x);
    set_param(GMB_PARAM_GMB_OFF_GYRO_Y,bias.y);
    set_param(GMB_PARAM_GMB_OFF_GYRO_Z,bias.z);
}

Vector3f SoloGimbal_Parameters::get_joint_bias()
{
    Vector3f ret;
    get_param(GMB_PARAM_GMB_OFF_JNT_X,ret.x);
    get_param(GMB_PARAM_GMB_OFF_JNT_Y,ret.y);
    get_param(GMB_PARAM_GMB_OFF_JNT_Z,ret.z);
    return ret;
}
float SoloGimbal_Parameters::get_K_rate()
{
    float ret;
    get_param(GMB_PARAM_GMB_K_RATE,ret);
    return ret;
}

void SoloGimbal_Parameters::flash()
{
    _flashing_step = GMB_PARAM_FLASHING_WAITING_FOR_SET;
}

bool SoloGimbal_Parameters::flashing()
{
    return _flashing_step != GMB_PARAM_NOT_FLASHING;
}
