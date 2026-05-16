

#include "AP_Doppler_config.h"



#include "AP_Doppler_Telem.h"
#include "AP_Doppler_Backend.h"
#include "AP_Doppler_Parameters.h"
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
        WITH_SEMAPHORE(_sim_sem);
        if (_sim_odom_sample.sequence == 0) {
            return false;
        }
        vel_body_mps = _sim_odom_sample.vel_body_mps;
        t_ms = _sim_odom_sample.time_ms;
        quality = _sim_odom_sample.quality;
        lock = _sim_odom_sample.lock;
        return true;
    }

    if (_backend == nullptr) {
        return false;
    }
    return _backend->get_velocity_body(vel_body_mps, t_ms, quality, lock);
}

bool AP_Doppler_Telem::get_odom_sample(DVLBodyOdomSample &sample) const
{
    if (_doppler_parameters->sim_enabled()) {
        WITH_SEMAPHORE(_sim_sem);
        if (_sim_odom_sample.sequence == 0) {
            return false;
        }
        sample = _sim_odom_sample;
        sample.vel_body_mps.rotate(_doppler_parameters->orientation());
        return sample.quality >= _doppler_parameters->min_quality();
    }

    if (_backend == nullptr) {
        return false;
    }

    EPD6VelocitySample backend_sample {};
    if (!_backend->get_velocity_sample(backend_sample, _doppler_parameters->use_water_track())) {
        return false;
    }

    sample.vel_body_mps = backend_sample.vel_body_mps;
    sample.vel_body_mps.rotate(_doppler_parameters->orientation());
    sample.time_ms = backend_sample.update_ms;
    sample.sequence = backend_sample.sequence;
    sample.vel_error_mps = backend_sample.vel_error_mps;
    sample.quality = constrain_float(backend_sample.quality, 0.0f, 100.0f);
    sample.lock = backend_sample.lock;

    if (sample.lock == DVL_LockState::WATER_TRACK) {
        sample.quality = MIN(sample.quality, 60.0f);
    }

    return sample.quality >= _doppler_parameters->min_quality();
}

bool AP_Doppler_Telem::odom_enabled() const
{
    return _doppler_parameters != nullptr &&
           (_backend != nullptr || _doppler_parameters->sim_enabled()) &&
           _doppler_parameters->body_odom_enabled();
}

bool AP_Doppler_Telem::extnav_vel_enabled() const
{
    return _doppler_parameters != nullptr &&
           (_backend != nullptr || _doppler_parameters->sim_enabled()) &&
           _doppler_parameters->extnav_vel_enabled();
}

bool AP_Doppler_Telem::odom_healthy() const
{
    DVLBodyOdomSample sample {};
    return odom_enabled() && get_odom_sample(sample);
}

const Vector3f &AP_Doppler_Telem::odom_pos_offset() const
{
    return _doppler_parameters->pos_offset();
}

uint16_t AP_Doppler_Telem::odom_delay_ms() const
{
    return _doppler_parameters->delay_ms();
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
    const Vector3f sim_velocity = _doppler_parameters->sim_velocity();
    const float sim_altitude_m = _doppler_parameters->sim_altitude_m();
    const float sim_quality = _doppler_parameters->sim_quality();
    const float sim_vel_error_mps = 0.03f;
    constexpr uint8_t sim_status = 'A';
    constexpr float beam_angle_rad = radians(22.5f);
    const float beam_distance_m = sim_altitude_m / cosf(beam_angle_rad);

    WITH_SEMAPHORE(_sim_sem);

    _sim_odom_sample.vel_body_mps = sim_velocity;
    _sim_odom_sample.time_ms = now_ms;
    _sim_odom_sample.sequence++;
    _sim_odom_sample.vel_error_mps = sim_vel_error_mps;
    _sim_odom_sample.quality = sim_quality;
    _sim_odom_sample.lock = DVL_LockState::BOTTOM_LOCK;

    _sim_bi_msg.time_usec = now_us;
    _sim_bi_msg.sequence++;
    _sim_bi_msg.valid = true;
    _sim_bi_msg.status = sim_status;
    _sim_bi_msg.vx_mps = sim_velocity.x;
    _sim_bi_msg.vy_mps = sim_velocity.y;
    _sim_bi_msg.vz_mps = sim_velocity.z;
    _sim_bi_msg.vel_error_mps = sim_vel_error_mps;

    _sim_bd_msg.time_usec = now_us;
    _sim_bd_msg.sequence++;
    _sim_bd_msg.valid = true;
    _sim_bd_msg.status = sim_status;
    _sim_bd_msg.east_m = 0.0f;
    _sim_bd_msg.north_m = 0.0f;
    _sim_bd_msg.up_m = 0.0f;
    _sim_bd_msg.bottom_m = sim_altitude_m;
    _sim_bd_msg.time_since_valid_s = 0.0f;

    _sim_wi_msg.time_usec = now_us;
    _sim_wi_msg.sequence++;
    _sim_wi_msg.valid = true;
    _sim_wi_msg.status = sim_status;
    _sim_wi_msg.vx_mps = sim_velocity.x;
    _sim_wi_msg.vy_mps = sim_velocity.y;
    _sim_wi_msg.vz_mps = sim_velocity.z;
    _sim_wi_msg.vel_error_mps = sim_vel_error_mps;

    _sim_ua_msg.time_usec = now_us;
    _sim_ua_msg.sequence++;
    _sim_ua_msg.valid = true;
    _sim_ua_msg.status = sim_status;
    _sim_ua_msg.velocity_mps = sim_velocity.x;
    _sim_ua_msg.distance_m = beam_distance_m;
    _sim_ua_msg.rssi = 48.0f;
    _sim_ua_msg.nsd = 3.2f;

    _sim_ub_msg.time_usec = now_us;
    _sim_ub_msg.sequence++;
    _sim_ub_msg.valid = true;
    _sim_ub_msg.status = sim_status;
    _sim_ub_msg.velocity_mps = sim_velocity.y;
    _sim_ub_msg.distance_m = beam_distance_m;
    _sim_ub_msg.rssi = 49.0f;
    _sim_ub_msg.nsd = 3.3f;

    _sim_uc_msg.time_usec = now_us;
    _sim_uc_msg.sequence++;
    _sim_uc_msg.valid = true;
    _sim_uc_msg.status = sim_status;
    _sim_uc_msg.velocity_mps = -sim_velocity.x;
    _sim_uc_msg.distance_m = beam_distance_m;
    _sim_uc_msg.rssi = 47.0f;
    _sim_uc_msg.nsd = 3.5f;

    _sim_ud_msg.time_usec = now_us;
    _sim_ud_msg.sequence++;
    _sim_ud_msg.valid = true;
    _sim_ud_msg.status = sim_status;
    _sim_ud_msg.velocity_mps = -sim_velocity.y;
    _sim_ud_msg.distance_m = beam_distance_m;
    _sim_ud_msg.rssi = 50.0f;
    _sim_ud_msg.nsd = 3.4f;
}




namespace AP
{
AP_Doppler_Telem *Doppler_telem()
{
    return AP_Doppler_Telem::get_singleton();
}
};
