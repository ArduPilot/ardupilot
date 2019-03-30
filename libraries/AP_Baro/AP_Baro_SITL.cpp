#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Baro_SITL.h"

extern const AP_HAL::HAL& hal;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_SITL::AP_Baro_SITL(AP_Baro &baro) :
    _sitl(AP::sitl()),
    _has_sample(false),
    AP_Baro_Backend(baro)
{
    if (_sitl != nullptr) {
        _instance = _frontend.register_sensor();
#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
        _frontend.set_type(_instance, AP_Baro::BARO_TYPE_WATER);
#endif
        hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &AP_Baro_SITL::_timer, void));
    }
}

// adjust for board temperature
void AP_Baro_SITL::temperature_adjustment(float &p, float &T)
{
    const float tsec = AP_HAL::millis() * 0.001f;
    const float T0 = _sitl->temp_start;
    const float T1 = _sitl->temp_flight;
    const float tconst = _sitl->temp_tconst;
    const float baro_factor = _sitl->temp_baro_factor;
    const float Tzero = 30.0f;  // start baro adjustment at 30C
    T = T1 - (T1 - T0) * expf(-tsec / tconst);
    if (is_positive(baro_factor)) {
        // this produces a pressure change with temperature that
        // closely matches what has been observed with a ICM-20789
        // barometer. A typical factor is 1.2.
        p -= powf(MAX(T - Tzero, 0), baro_factor);
    }
}

void AP_Baro_SITL::_timer()
{

    // 100Hz
    const uint32_t now = AP_HAL::millis();
    if ((now - _last_sample_time) < 10) {
        return;
    }
    _last_sample_time = now;

    float sim_alt = _sitl->state.altitude;

    if (_sitl->baro_disable) {
        // barometer is disabled
        return;
    }

    sim_alt += _sitl->baro_drift * now / 1000.0f;
    sim_alt += _sitl->baro_noise * rand_float();

    // add baro glitch
    sim_alt += _sitl->baro_glitch;

    // add delay
    uint32_t best_time_delta = 200;  // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index = 0;  // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - _last_store_time >= 10) {  // store data every 10 ms.
        _last_store_time = now;
        if (_store_index > _buffer_length - 1) {  // reset buffer index if index greater than size of buffer
            _store_index = 0;
        }
        _buffer[_store_index].data = sim_alt;  // add data to current index
        _buffer[_store_index].time = _last_store_time;  // add time_stamp to current index
        _store_index = _store_index + 1;  // increment index
    }

    // return delayed measurement
    const uint32_t delayed_time = now - _sitl->baro_delay;  // get time corresponding to delay

    // find data corresponding to delayed time in buffer
    for (uint8_t i = 0; i <= _buffer_length - 1; i++) {
        // find difference between delayed time and time stamp in buffer
        uint32_t time_delta = abs(
                (int32_t)(delayed_time - _buffer[i].time));
        // if this difference is smaller than last delta, store this time
        if (time_delta < best_time_delta) {
            best_index = i;
            best_time_delta = time_delta;
        }
    }
    if (best_time_delta < 200) {  // only output stored state if < 200 msec retrieval error
        sim_alt = _buffer[best_index].data;
    }

#if !APM_BUILD_TYPE(APM_BUILD_ArduSub)
    float sigma, delta, theta;

    AP_Baro::SimpleAtmosphere(sim_alt * 0.001f, sigma, delta, theta);
    float p = SSL_AIR_PRESSURE * delta;
    float T = 303.16f * theta - C_TO_KELVIN;  // Assume 30 degrees at sea level - converted to degrees Kelvin

    temperature_adjustment(p, T);
#else
    float rho, delta, theta;
    AP_Baro::SimpleUnderWaterAtmosphere(-sim_alt * 0.001f, rho, delta, theta);
    float p = SSL_AIR_PRESSURE * delta;
    float T = 303.16f * theta - C_TO_KELVIN;  // Assume 30 degrees at sea level - converted to degrees Kelvin
#endif

    _recent_press = p;
    _recent_temp = T;
    _has_sample = true;
}

// Read the sensor
void AP_Baro_SITL::update(void)
{
    if (!_has_sample) {
        return;
    }

    WITH_SEMAPHORE(_sem);
    _copy_to_frontend(_instance, _recent_press, _recent_temp);
    _has_sample = false;
}

#endif  // CONFIG_HAL_BOARD
