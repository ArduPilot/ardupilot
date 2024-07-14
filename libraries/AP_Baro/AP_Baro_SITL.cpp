#include "AP_Baro_SITL.h"

#if AP_SIM_BARO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

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
        set_bus_id(_instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, 0, _instance, DEVTYPE_BARO_SITL));
        hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &AP_Baro_SITL::_timer, void));
    }
}

// adjust for board temperature warmup on start-up
void AP_Baro_SITL::temperature_adjustment(float &p, float &T)
{
    const float tsec = AP_HAL::millis() * 0.001f;
    const float T_sensor = T + AP::sitl()->temp_board_offset;
    const float tconst = AP::sitl()->temp_tconst;
    if (tsec < 23 * tconst) { // time which past the equation below equals T_sensor within approx. 1E-9
        const float T0 = AP::sitl()->temp_start;
        T = T_sensor - (T_sensor - T0) * expf(-tsec / tconst);
    }
    else {
        T = T_sensor;
    }

    const float baro_factor = AP::sitl()->temp_baro_factor;
    const float Tzero = 30.0f;  // start baro adjustment at 30C
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

    if (_sitl->baro[_instance].disable) {
        // barometer is disabled
        return;
    }

    const auto drift_delta_t_ms = now - last_drift_delta_t_ms;
    last_drift_delta_t_ms = now;
    total_alt_drift += _sitl->baro[_instance].drift * drift_delta_t_ms * 0.001f;

    sim_alt += total_alt_drift;
    sim_alt += _sitl->baro[_instance].noise * rand_float();

    // add baro glitch
    sim_alt += _sitl->baro[_instance].glitch;

    // add delay
    uint32_t best_time_delta = 200;  // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index = 0;  // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - _last_store_time >= 10) {  // store data every 10 ms.
        _last_store_time = now;
        if (_store_index > _buffer_length - 1) {  // reset buffer index if index greater than size of buffer
            _store_index = 0;
        }

        // if freezed barometer, report altitude to last recorded altitude
        if (_sitl->baro[_instance].freeze == 1) {
            sim_alt = _last_altitude;
        } else {
            _last_altitude = sim_alt;
        }

        _buffer[_store_index].data = sim_alt;  // add data to current index
        _buffer[_store_index].time = _last_store_time;  // add time_stamp to current index
        _store_index = _store_index + 1;  // increment index
    }

    // return delayed measurement
    const uint32_t delayed_time = now - _sitl->baro[_instance].delay;  // get time corresponding to delay

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
    float p, T_K;
    AP_Baro::get_pressure_temperature_for_alt_amsl(sim_alt, p, T_K);
    float T = KELVIN_TO_C(T_K);
    temperature_adjustment(p, T);
#else
    float rho, delta, theta;
    AP_Baro::SimpleUnderWaterAtmosphere(-sim_alt * 0.001f, rho, delta, theta);
    float p = SSL_AIR_PRESSURE * delta;
    float T = KELVIN_TO_C(SSL_AIR_TEMPERATURE * theta);
#endif

    // add in correction for wind effects
    p += wind_pressure_correction(_instance);

    _recent_press = p;
    _recent_temp = T;
    _has_sample = true;
}

// unhealthy if baro is turned off or beyond supported instances
bool AP_Baro_SITL::healthy(uint8_t instance) 
{
    return _last_sample_time != 0 && !_sitl->baro[instance].disable;
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

/*
  return pressure correction for wind based on SIM_BARO_WCF parameters
 */
float AP_Baro_SITL::wind_pressure_correction(uint8_t instance)
{
    const auto &bp = AP::sitl()->baro[instance];

    // correct for static pressure position errors
    const Vector3f &airspeed_vec_bf = AP::sitl()->state.velocity_air_bf;

    float error = 0.0;
    const float sqx = sq(airspeed_vec_bf.x);
    const float sqy = sq(airspeed_vec_bf.y);
    const float sqz = sq(airspeed_vec_bf.z);

    if (is_positive(airspeed_vec_bf.x)) {
        error += bp.wcof_xp * sqx;
    } else {
        error += bp.wcof_xn * sqx;
    }
    if (is_positive(airspeed_vec_bf.y)) {
        error += bp.wcof_yp * sqy;
    } else {
        error += bp.wcof_yn * sqy;
    }
    if (is_positive(airspeed_vec_bf.z)) {
        error += bp.wcof_zp * sqz;
    } else {
        error += bp.wcof_zn * sqz;
    }

    return error * 0.5 * SSL_AIR_DENSITY * AP::baro()._get_air_density_ratio();
}

#endif  // AP_SIM_BARO_ENABLED
