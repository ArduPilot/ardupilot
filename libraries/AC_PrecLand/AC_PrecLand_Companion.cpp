#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS.h>
#include "AC_PrecLand_Companion.h"

// perform any required initialisation of backend
void AC_PrecLand_Companion::init()
{
    // set healthy
    _state.healthy = true;
    _have_los_meas = false;
    _have_los_meas_local_frd = false;
    _have_los_meas_local_ned = false;
    _have_los_meas_local_offset_ned = false;
    _unsupported_frame_msg_sent = false;
    
}

// retrieve updates from sensor
void AC_PrecLand_Companion::update()
{
    _have_los_meas = _have_los_meas && AP_HAL::millis()-_los_meas_time_ms <= 1000;
    _have_los_meas_local_frd = _have_los_meas_local_frd && AP_HAL::millis()-_los_meas_local_frd_time_ms <= 1000;
    _have_los_meas_local_ned = _have_los_meas_local_ned && AP_HAL::millis()-_los_meas_local_ned_time_ms <= 1000;
    _have_los_meas_local_offset_ned = _have_los_meas_local_offset_ned && AP_HAL::millis()-_los_meas_local_offset_ned_time_ms <= 1000;
}

// provides a unit vector towards the target in body frame
// returns same as have_los_meas()
bool AC_PrecLand_Companion::get_los_body(Vector3f& ret) {
    if (have_los_meas()) {
        ret = _los_meas_body;
        return true;
    }
    return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_Companion::los_meas_time_ms() {
    return _los_meas_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_Companion::have_los_meas() {
    return _have_los_meas;
}

// provides a local FRD position
// FRD local frame (x: Front, y: Right, z: Down) with origin that travels with the vehicle Z - down to earth.
// returns same as have_los_meas_local_frd() 
bool AC_PrecLand_Companion::get_los_local_frd_target(Vector3f& local_frd){
    if(have_los_meas_local_frd()) {
        local_frd = _los_meas_local_frd;
        return true;
    }
    return false;
};

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_Companion::los_meas_local_frd_time_ms(){
     
     return _los_meas_local_frd_time_ms;
};

// return true if there is a valid los measurement available
bool AC_PrecLand_Companion::have_los_meas_local_frd(){

    return _have_los_meas_local_frd;
};

// provides a local  NED position
// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
// returns same as have_los_meas_local_ned()
bool AC_PrecLand_Companion::get_los_local_ned_target(Vector3f& local_ned) {
    if(have_los_meas_local_ned()) {
        local_ned = _los_meas_local_ned;
        return true;
    }
    
    return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_Companion::los_meas_local_ned_time_ms() {
    return _los_meas_local_ned_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_Companion::have_los_meas_local_ned(){

    return _have_los_meas_local_ned;
}

// provides a local offset NED position
// NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
// returns same as have_los_meas_local_offset_ned() 
bool AC_PrecLand_Companion::get_los_local_offset_ned_target(Vector3f& local_offset_ned){

        if(have_los_meas_local_offset_ned()) {
        local_offset_ned = _los_meas_local_offset_ned;
        return true;
    }
    return false;
}

// returns system time in milliseconds of last los measurement
uint32_t AC_PrecLand_Companion::los_meas_local_offset_ned_time_ms() {

    return _los_meas_local_offset_ned_time_ms;
}

// return true if there is a valid los measurement available
bool AC_PrecLand_Companion::have_los_meas_local_offset_ned(){

    return _have_los_meas_local_offset_ned;
}

// return distance to target
float AC_PrecLand_Companion::distance_to_target(){

    return _distance_to_target;
}

void AC_PrecLand_Companion::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms){

    if (packet.position_valid) {
        
        switch (packet.frame) {
            
            case MAV_FRAME_BODY_FRD:{ // Body fixed frame of reference, Z-down (vehicle bottom) (x: Forward, y: Right, z: Down).
                 _distance_to_target = packet.distance;
                _los_meas_body = Vector3f(packet.x,packet.y,packet.z);
                _los_meas_body /= _distance_to_target;
                _los_meas_time_ms = timestamp_ms;
                _have_los_meas = true; 
            }
            break;

            case MAV_FRAME_LOCAL_FRD:{ // Body location fixed frame of reference, Z-down to earth (x: Forward, y: Right, z: Down).
                 _distance_to_target = packet.distance;
                _los_meas_local_frd = Vector3f(packet.x,packet.y,packet.z);
                _los_meas_local_frd_time_ms = timestamp_ms;
                _have_los_meas_local_frd = true; 
            }
            break;

            case MAV_FRAME_LOCAL_OFFSET_NED:{//Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
                _distance_to_target = packet.distance;
                _los_meas_local_offset_ned = Vector3f(packet.x,packet.y,packet.z);
                _los_meas_local_offset_ned_time_ms = timestamp_ms;
                _have_los_meas_local_offset_ned = true;
            }
            break;

            case MAV_FRAME_LOCAL_NED:{// Local coordinate frame, Z-down (x: North, y: East, z: Down).
                _distance_to_target = packet.distance;
                _los_meas_local_ned = Vector3f(packet.x,packet.y,packet.z);
                _los_meas_local_ned_time_ms = timestamp_ms;
                _have_los_meas_local_ned = true;
            }
            break;

            default:{
                if (!_unsupported_frame_msg_sent) {
                    _unsupported_frame_msg_sent = true;
                    gcs().send_text(MAV_SEVERITY_INFO,"PrecLand_Companion: Frame not supported: (%d).",packet.frame);
                }
                
            break;
            }
        }
    }else{
        _distance_to_target = packet.distance;

        // compute unit vector towards target
        _los_meas_body = Vector3f(-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f);
        _los_meas_body /= _los_meas_body.length();

        _los_meas_time_ms = timestamp_ms;
        _have_los_meas = true;
    }
}
 