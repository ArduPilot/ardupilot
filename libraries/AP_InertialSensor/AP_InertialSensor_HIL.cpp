/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_InertialSensor_HIL.h"
#include <AP_HAL.h>
const extern AP_HAL::HAL& hal;

AP_InertialSensor_HIL::AP_InertialSensor_HIL() : AP_InertialSensor() {
    _accel[0] = Vector3f(0, 0, -GRAVITY_MSS);
}

uint16_t AP_InertialSensor_HIL::_init_sensor( Sample_rate sample_rate ) {
    switch (sample_rate) {
    case RATE_50HZ:
        _sample_period_ms = 20;
        break;
    case RATE_100HZ:
        _sample_period_ms = 10;
        break;
    case RATE_200HZ:
        _sample_period_ms = 5;
        break;
    }
    return AP_PRODUCT_ID_NONE;
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_HIL::update( void ) {
    uint32_t now = hal.scheduler->millis();
    _delta_time_usec = (now - _last_update_ms) * 1000;
    _last_update_ms = now;
    return true;
}

float AP_InertialSensor_HIL::get_delta_time() {
    return _delta_time_usec * 1.0e-6;
}

float AP_InertialSensor_HIL::get_gyro_drift_rate(void) {
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

bool AP_InertialSensor_HIL::_sample_available()
{
    uint16_t ret = (hal.scheduler->millis() - _last_update_ms) 
        / _sample_period_ms;
    
    return ret > 0;
}

bool AP_InertialSensor_HIL::wait_for_sample(uint16_t timeout_ms)
{
    if (_sample_available()) {
        return true;
    }
    uint32_t start = hal.scheduler->millis();
    while ((hal.scheduler->millis() - start) < timeout_ms) {
        hal.scheduler->delay(1);
        if (_sample_available()) {
            return true;
        }
    }
    return false;
}

void AP_InertialSensor_HIL::set_accel(const Vector3f &accel)
{
    _previous_accel[0] = _accel[0];
    _accel[0] = accel;
    _last_accel_usec = hal.scheduler->micros();
}

void AP_InertialSensor_HIL::set_gyro(const Vector3f &gyro)
{
    _gyro[0] = gyro;
    _last_gyro_usec = hal.scheduler->micros();
}

/**
   try to detect bad accel/gyro sensors
 */
bool AP_InertialSensor_HIL::healthy(void) const
{
    uint32_t tnow = hal.scheduler->micros();
    if ((tnow - _last_accel_usec) > 40000) {
        // accels have not updated
        return false;
    }
    if ((tnow - _last_gyro_usec) > 40000) {
        // gyros have not updated
        return false;
    }
    if (fabs(_accel[0].x) > 30 && fabs(_accel[0].y) > 30 && fabs(_accel[0].z) > 30 &&
        (_previous_accel[0] - _accel[0]).length() < 0.01f) {
        // unchanging accel, large in all 3 axes. This is a likely
        // accelerometer failure
        return false;
    }
    return true;
}
