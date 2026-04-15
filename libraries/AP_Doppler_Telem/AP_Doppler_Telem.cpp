

#include "AP_Doppler_config.h"



#include "AP_Doppler_Telem.h"
#include "AP_Doppler_Backend.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <cmath>



extern const AP_HAL::HAL& hal;

AP_Doppler_Telem *AP_Doppler_Telem::singleton;

AP_Doppler_Telem::AP_Doppler_Telem()
{
    singleton = this;
    _backend = nullptr;
    port = nullptr;
    _fitsurface = new AP_Doppler_FitSurface(*this);
    _doppler_parameters = &AP::vehicle()->doppler_parameters;
}


AP_Doppler_Telem::~AP_Doppler_Telem(void)
{
    singleton = nullptr;
    if (_backend!= nullptr) {
        delete _backend;
        _backend = nullptr;
    }
    delete _fitsurface;
    _fitsurface = nullptr;
}




/*
 * init - perform required initialisation
 */
bool AP_Doppler_Telem::init(const AP_SerialManager &serial_manager)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Doppler INIT");

    if (_doppler_parameters->sim_enabled()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Doppler SIM ENABLED");
        return true;
    }

    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have Doppler on multiple serial ports)

    if ((port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Doppler, 0)))
    {
        _backend = new AP_Doppler_Backend(port);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Doppler FIND SERIAL");
    }

    if (_backend == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Doppler  NO SERIAL");
        return false;
        
    }

    if (!_backend->init()) {
        delete _backend;
        _backend = nullptr;
        return false;
    }
    
    gcs().send_text(MAV_SEVERITY_CRITICAL, "OK");
    return true;
}


void AP_Doppler_Telem::update()
{
    if (_doppler_parameters->sim_enabled()) {
        update_simulated_messages();
        return;
    }

    if (_backend == nullptr) {
        return;
    }

    fit_health = _fitsurface->update();
}


void AP_Doppler_Telem::send()
{
    if (_backend != nullptr) {
        _backend->send();
    }
    else {
        return;
    }
}

bool AP_Doppler_Telem::get_velocity_body(Vector3f &vel_body_mps, uint32_t &t_ms, float &quality, DVL_LockState &lock) const
{
    if (_doppler_parameters->sim_enabled()) {
        return false;
    }

    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_velocity_body(vel_body_mps, t_ms, quality, lock);
}

bool AP_Doppler_Telem::get_bi_msg(DVL_BI_Msg &msg) const
{
    if (_doppler_parameters->sim_enabled()) {
        WITH_SEMAPHORE(_sim_sem);
        msg = _sim_bi_msg;
        return msg.valid;
    }

    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_bi_msg(msg);
}

bool AP_Doppler_Telem::get_bd_msg(DVL_BD_Msg &msg) const
{
    if (_doppler_parameters->sim_enabled()) {
        WITH_SEMAPHORE(_sim_sem);
        msg = _sim_bd_msg;
        return msg.valid;
    }

    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_bd_msg(msg);
}

bool AP_Doppler_Telem::get_wi_msg(DVL_WI_Msg &msg) const
{
    if (_doppler_parameters->sim_enabled()) {
        WITH_SEMAPHORE(_sim_sem);
        msg = _sim_wi_msg;
        return msg.valid;
    }

    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_wi_msg(msg);
}

bool AP_Doppler_Telem::get_ua_msg(DVL_U_Msg &msg) const
{
    if (_doppler_parameters->sim_enabled()) {
        WITH_SEMAPHORE(_sim_sem);
        msg = _sim_ua_msg;
        return msg.valid;
    }

    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_ua_msg(msg);
}

bool AP_Doppler_Telem::get_ub_msg(DVL_U_Msg &msg) const
{
    if (_doppler_parameters->sim_enabled()) {
        WITH_SEMAPHORE(_sim_sem);
        msg = _sim_ub_msg;
        return msg.valid;
    }

    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_ub_msg(msg);
}

bool AP_Doppler_Telem::get_uc_msg(DVL_U_Msg &msg) const
{
    if (_doppler_parameters->sim_enabled()) {
        WITH_SEMAPHORE(_sim_sem);
        msg = _sim_uc_msg;
        return msg.valid;
    }

    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_uc_msg(msg);
}

bool AP_Doppler_Telem::get_ud_msg(DVL_U_Msg &msg) const
{
    if (_doppler_parameters->sim_enabled()) {
        WITH_SEMAPHORE(_sim_sem);
        msg = _sim_ud_msg;
        return msg.valid;
    }

    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_ud_msg(msg);
}

void AP_Doppler_Telem::update_simulated_messages()
{
    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - _sim_last_update_ms) < 100U) {
        return;
    }
    _sim_last_update_ms = now_ms;

    const uint64_t now_us = AP_HAL::micros64();
    const float t = now_ms * 0.001f;
    constexpr uint8_t sim_status = 'A';

    WITH_SEMAPHORE(_sim_sem);

    _sim_bi_msg.time_usec = now_us;
    _sim_bi_msg.sequence++;
    _sim_bi_msg.valid = true;
    _sim_bi_msg.status = sim_status;
    _sim_bi_msg.vx_mps = 0.35f + 0.08f * sinf(0.40f * t);
    _sim_bi_msg.vy_mps = 0.05f * cosf(0.30f * t);
    _sim_bi_msg.vz_mps = 0.02f * sinf(0.20f * t);
    _sim_bi_msg.vel_error_mps = 0.01f;

    _sim_bd_msg.time_usec = now_us;
    _sim_bd_msg.sequence++;
    _sim_bd_msg.valid = true;
    _sim_bd_msg.status = sim_status;
    _sim_bd_msg.east_m = 1.0f + 0.25f * sinf(0.10f * t);
    _sim_bd_msg.north_m = 0.8f + 0.20f * cosf(0.12f * t);
    _sim_bd_msg.up_m = 0.1f * sinf(0.08f * t);
    _sim_bd_msg.bottom_m = 4.2f + 0.15f * sinf(0.18f * t);
    _sim_bd_msg.time_since_valid_s = 0.0f;

    _sim_wi_msg.time_usec = now_us;
    _sim_wi_msg.sequence++;
    _sim_wi_msg.valid = true;
    _sim_wi_msg.status = sim_status;
    _sim_wi_msg.vx_mps = 0.18f + 0.03f * sinf(0.35f * t);
    _sim_wi_msg.vy_mps = -0.04f + 0.02f * cosf(0.28f * t);
    _sim_wi_msg.vz_mps = 0.01f * sinf(0.22f * t);
    _sim_wi_msg.vel_error_mps = 0.015f;

    _sim_ua_msg.time_usec = now_us;
    _sim_ua_msg.sequence++;
    _sim_ua_msg.valid = true;
    _sim_ua_msg.status = sim_status;
    _sim_ua_msg.velocity_mps = 0.22f + 0.02f * sinf(0.40f * t);
    _sim_ua_msg.distance_m = 5.10f + 0.08f * sinf(0.09f * t);
    _sim_ua_msg.rssi = 48.0f + 2.0f * sinf(0.11f * t);
    _sim_ua_msg.nsd = 3.2f + 0.2f * cosf(0.14f * t);

    _sim_ub_msg.time_usec = now_us;
    _sim_ub_msg.sequence++;
    _sim_ub_msg.valid = true;
    _sim_ub_msg.status = sim_status;
    _sim_ub_msg.velocity_mps = 0.24f + 0.02f * sinf(0.43f * t + 0.4f);
    _sim_ub_msg.distance_m = 5.05f + 0.08f * sinf(0.09f * t + 0.2f);
    _sim_ub_msg.rssi = 49.0f + 2.0f * sinf(0.11f * t + 0.3f);
    _sim_ub_msg.nsd = 3.3f + 0.2f * cosf(0.14f * t + 0.2f);

    _sim_uc_msg.time_usec = now_us;
    _sim_uc_msg.sequence++;
    _sim_uc_msg.valid = true;
    _sim_uc_msg.status = sim_status;
    _sim_uc_msg.velocity_mps = 0.21f + 0.02f * sinf(0.46f * t + 0.8f);
    _sim_uc_msg.distance_m = 4.95f + 0.08f * sinf(0.09f * t + 0.4f);
    _sim_uc_msg.rssi = 47.0f + 2.0f * sinf(0.11f * t + 0.6f);
    _sim_uc_msg.nsd = 3.5f + 0.2f * cosf(0.14f * t + 0.4f);

    _sim_ud_msg.time_usec = now_us;
    _sim_ud_msg.sequence++;
    _sim_ud_msg.valid = true;
    _sim_ud_msg.status = sim_status;
    _sim_ud_msg.velocity_mps = 0.23f + 0.02f * sinf(0.49f * t + 1.2f);
    _sim_ud_msg.distance_m = 5.00f + 0.08f * sinf(0.09f * t + 0.6f);
    _sim_ud_msg.rssi = 50.0f + 2.0f * sinf(0.11f * t + 0.9f);
    _sim_ud_msg.nsd = 3.4f + 0.2f * cosf(0.14f * t + 0.6f);
}




namespace AP
{
AP_Doppler_Telem *Doppler_telem()
{
    return AP_Doppler_Telem::get_singleton();
}
};
