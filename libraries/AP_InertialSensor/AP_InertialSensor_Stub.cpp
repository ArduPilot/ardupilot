/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_InertialSensor_Stub.h"

uint16_t AP_InertialSensor_Stub::_init_sensor( AP_PeriodicProcess * scheduler, Sample_rate sample_rate ) {
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

bool AP_InertialSensor_Stub::update( void ) {
    uint32_t now = millis();
    _delta_time_usec = (now - _last_update_ms) * 1000;
    _last_update_ms = now;
    return true;
}
bool AP_InertialSensor_Stub::new_data_available( void ) {
    return num_samples_available() > 0;
}


float AP_InertialSensor_Stub::gx() {
    return 0.0f;
}
float AP_InertialSensor_Stub::gy() {
    return 0.0f;
}
float AP_InertialSensor_Stub::gz() {
    return 0.0f;
}

float AP_InertialSensor_Stub::ax() {
    return 0.0f;
}
float AP_InertialSensor_Stub::ay() {
    return 0.0f;
}
float AP_InertialSensor_Stub::az() {
    return 0.0f;
}

float AP_InertialSensor_Stub::temperature() {
    return 0.0;
}
uint32_t AP_InertialSensor_Stub::get_delta_time_micros() {
    return _delta_time_usec;
}
uint32_t AP_InertialSensor_Stub::get_last_sample_time_micros() {
    return _last_update_ms * 1000;
}
float AP_InertialSensor_Stub::get_gyro_drift_rate(void) {
    return 0.0;
}
uint16_t AP_InertialSensor_Stub::num_samples_available()
{
    uint16_t ret = (millis() - _last_update_ms) / _sample_period_ms;
    
    return ret;
}
