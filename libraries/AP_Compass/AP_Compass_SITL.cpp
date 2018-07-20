#include "AP_Compass_SITL.h"

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
extern const AP_HAL::HAL& hal;

AP_Compass_SITL::AP_Compass_SITL(Compass &compass):
    _sitl(AP::sitl()),
    _has_sample(false),
    AP_Compass_Backend(compass)
{
    if (_sitl != nullptr) {
        _compass._setup_earth_field();
        for (uint8_t i=0; i<SITL_NUM_COMPASSES; i++) {
            // default offsets to correct value
            compass.set_offsets(i, _sitl->mag_ofs);
            
            _compass_instance[i] = register_compass();
            set_dev_id(_compass_instance[i], AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, i, 0, DEVTYPE_SITL));

            // save so the compass always comes up configured in SITL
            save_dev_id(_compass_instance[i]);
        }
        
        // make first compass external
        set_external(_compass_instance[0], true);

        hal.scheduler->register_timer_process(FUNCTOR_BIND(this, &AP_Compass_SITL::_timer, void));
    }
}


/*
  create correction matrix for diagnonals and off-diagonals
*/
void AP_Compass_SITL::_setup_eliptical_correcion(void)
{
    Vector3f diag = _sitl->mag_diag.get();
    if (diag.is_zero()) {
        diag(1,1,1);
    }
    const Vector3f &diagonals = diag;
    const Vector3f &offdiagonals = _sitl->mag_offdiag;
    
    if (diagonals == _last_dia && offdiagonals == _last_odi) {
        return;
    }
    
    _eliptical_corr = Matrix3f(diagonals.x,    offdiagonals.x, offdiagonals.y,
                               offdiagonals.x, diagonals.y,    offdiagonals.z,
                               offdiagonals.y, offdiagonals.z, diagonals.z);
    if (!_eliptical_corr.invert()) {
        _eliptical_corr.identity();
    }
    _last_dia = diag;
    _last_odi = offdiagonals;
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

    _setup_eliptical_correcion();        
    
    new_mag_data = _eliptical_corr * new_mag_data;
    new_mag_data -= _sitl->mag_ofs.get();

    for (uint8_t i=0; i<SITL_NUM_COMPASSES; i++) {
        Vector3f f = new_mag_data;
        if (i == 0) {
            // rotate the first compass, allowing for testing of external compass rotation
            f.rotate_inverse((enum Rotation)_sitl->mag_orient.get());
            f.rotate(get_board_orientation());
        }
        
        rotate_field(f, _compass_instance[i]);
        publish_raw_field(f, _compass_instance[i]);
        correct_field(f, _compass_instance[i]);

        _mag_accum[i] += f;
    }

    if (!_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }

    _accum_count++;
    if (_accum_count == 10) {
        for (uint8_t i=0; i<SITL_NUM_COMPASSES; i++) {
            _mag_accum[i] /= 2;
        }
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

        for (uint8_t i=0; i<SITL_NUM_COMPASSES; i++) {
            Vector3f field(_mag_accum[i]);
            field /= _accum_count;
            _mag_accum[i].zero();
            publish_filtered_field(field, _compass_instance[i]);
        }
        _accum_count = 0;

        _has_sample = false;
        _sem->give();
    }

}
#endif
