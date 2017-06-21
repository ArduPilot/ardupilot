#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Baro_SITL.h"

extern const AP_HAL::HAL& hal;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_SITL::AP_Baro_SITL(AP_Baro &baro) :
    _has_sample(false),
    AP_Baro_Backend(baro)
{
    sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    if (sitl != nullptr) {
        instance = _frontend.register_sensor();
        hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &AP_Baro_SITL::_timer, void));
    }
}

// adjust for board temperature
void AP_Baro_SITL::temperature_adjustment(float &p, float &T)
{
    float tsec = AP_HAL::millis() * 0.001;
    const float T0 = sitl->temp_start;
    const float T1 = sitl->temp_flight;
    const float tconst = sitl->temp_tconst;
    const float baro_factor = sitl->temp_baro_factor;
    const float Tzero = 30; // start baro adjustment at 30C
    T = T1 - (T1 - T0)*expf(-tsec / tconst);
    if (baro_factor > 0) {
        // this produces a pressure change with temperature that
        // closely matches what has been observed with a ICM-20789
        // barometer. A typical factor is 1.2.
        p -= powf(MAX(T - Tzero, 0), baro_factor);
    }
}

void AP_Baro_SITL::_timer()
{

    // 100Hz
    uint32_t now = AP_HAL::millis();
    if ((now - _last_sample_time) < 10) {
        return;
    }
    _last_sample_time = now;

    float sim_alt = sitl->state.altitude;

    if (sitl->baro_disable) {
        // barometer is disabled
        return;
    }

    sim_alt += sitl->baro_drift * now / 1000;
    sim_alt += sitl->baro_noise * rand_float();

    // add baro glitch
    sim_alt += sitl->baro_glitch;

    // add delay
    uint32_t best_time_delta = 200; // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index = 0; // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - last_store_time >= 10) { // store data every 10 ms.
        last_store_time = now;
        if (store_index > buffer_length-1) { // reset buffer index if index greater than size of buffer
            store_index = 0;
        }
        buffer[store_index].data = sim_alt; // add data to current index
        buffer[store_index].time = last_store_time; // add time_stamp to current index
        store_index = store_index + 1; // increment index
    }

    // return delayed measurement
    uint32_t delayed_time = now - sitl->baro_delay; // get time corresponding to delay

    // find data corresponding to delayed time in buffer
    for (uint8_t i=0; i<=buffer_length-1; i++) {
        // find difference between delayed time and time stamp in buffer
        uint32_t time_delta = abs(
                (int32_t)(delayed_time - buffer[i].time));
        // if this difference is smaller than last delta, store this time
        if (time_delta < best_time_delta) {
            best_index = i;
            best_time_delta = time_delta;
        }
    }
    if (best_time_delta < 200) { // only output stored state if < 200 msec retrieval error
        sim_alt = buffer[best_index].data;
    }

    float sigma, delta, theta;
    const float p0 = 101325;

    AP_Baro::SimpleAtmosphere(sim_alt*0.001f, sigma, delta, theta);
    float p = p0 * delta;
    float T = 303.16f * theta - 273.16f; // Assume 30 degrees at sea level - converted to degrees Kelvin

    temperature_adjustment(p, T);

    _recent_press = p;
    _recent_temp = T;
    _has_sample = true;
}

// Read the sensor
void AP_Baro_SITL::update(void)
{
    if (_sem->take_nonblocking()) {
        if (!_has_sample) {
            _sem->give();
            return;
        }

        _copy_to_frontend(instance, _recent_press, _recent_temp);
        _has_sample = false;
        _sem->give();
    }
}

#endif // CONFIG_HAL_BOARD
