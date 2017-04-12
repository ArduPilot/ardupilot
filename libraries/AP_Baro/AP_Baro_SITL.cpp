#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Baro_SITL.h"

extern const AP_HAL::HAL& hal;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_SITL::AP_Baro_SITL(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    if (sitl != nullptr) {
        instance = _frontend.register_sensor();
    }
}

// Read the sensor
void AP_Baro_SITL::update(void)
{
    float sim_alt = sitl->state.altitude;

    if (sitl->baro_disable) {
        // barometer is disabled
        return;
    }

    uint32_t now = AP_HAL::millis();
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

    _copy_to_frontend(instance, p, T);
}

#endif // CONFIG_HAL_BOARD
