#include "AP_Compass_SITL.h"

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
extern const AP_HAL::HAL& hal;

AP_Compass_SITL::AP_Compass_SITL(Compass &compass):
    _has_sample(false),
    AP_Compass_Backend(compass)
{
    _sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    if (_sitl != nullptr) {
        _compass._setup_earth_field();
        for (uint8_t i=0; i<SITL_NUM_COMPASSES; i++) {
            _compass_instance[i] = register_compass();
        }
        hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &AP_Compass_SITL::_timer, void));
    }
}

void AP_Compass_SITL::_timer()
{
    // TODO: Refactor delay buffer with AP_Baro_SITL.

    // Sampled at 100Hz
    uint32_t now = AP_HAL::millis();
    if ((now - _last_sample_time) < 10) {
        return;
    }
    _last_sample_time = now;

    // calculate sensor noise and add to 'truth' field in body frame
    // units are milli-Gauss
    Vector3f noise = rand_vec3f() * _sitl->mag_noise;
    Vector3f new_mag_data = _sitl->state.bodyMagField + noise;

    // add delay
    uint32_t best_time_delta = 1000; // initialise large time representing buffer entry closest to current time - delay.
    uint8_t best_index = 0; // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
    if (now - last_store_time >= 10) { // store data every 10 ms.
        last_store_time = now;
        if (store_index > buffer_length-1) { // reset buffer index if index greater than size of buffer
            store_index = 0;
        }
        buffer[store_index].data = new_mag_data; // add data to current index
        buffer[store_index].time = last_store_time; // add time to current index
        store_index = store_index + 1; // increment index
    }

    // return delayed measurement
    uint32_t delayed_time = now - _sitl->mag_delay; // get time corresponding to delay
    // find data corresponding to delayed time in buffer
    for (uint8_t i=0; i<=buffer_length-1; i++) {
        // find difference between delayed time and time stamp in buffer
        uint32_t time_delta = abs((int32_t)(delayed_time - buffer[i].time));
        // if this difference is smaller than last delta, store this time
        if (time_delta < best_time_delta) {
            best_index= i;
            best_time_delta = time_delta;
        }
    }
    if (best_time_delta < 1000) { // only output stored state if < 1 sec retrieval error
        new_mag_data = buffer[best_index].data;
    }

    new_mag_data -= _sitl->mag_ofs.get();

    for (uint8_t i=0; i<SITL_NUM_COMPASSES; i++) {
        rotate_field(new_mag_data, _compass_instance[i]);
        publish_raw_field(new_mag_data, AP_HAL::micros(), _compass_instance[i]);
        correct_field(new_mag_data, _compass_instance[i]);
    }

    if (!_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }

    _mag_accum += new_mag_data;
    _accum_count++;
    if (_accum_count == 10) {
        _mag_accum /= 2;
        _accum_count = 5;
        _has_sample = true;
    }
    _sem->give();
}

void AP_Compass_SITL::read()
{
    if (_sem->take_nonblocking()) {
        if (!_has_sample) {
            _sem->give();
            return;
        }

        Vector3f field(_mag_accum);
        field /= _accum_count;
        _mag_accum.zero();
        _accum_count = 0;

        for (uint8_t i=0; i<SITL_NUM_COMPASSES; i++) {
            publish_filtered_field(field, _compass_instance[i]);
        }

        _has_sample = false;
        _sem->give();
    }

}
#endif
