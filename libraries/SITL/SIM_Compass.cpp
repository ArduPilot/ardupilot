#include "SIM_Compass.h"

#if AP_SIM_ENABLED

#include "SITL.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Compass/AP_Compass.h>

using namespace SITL;

/*
  apply the transformation a simulated compass applies to a vector once
  SIM_MAGn_OFS has been taken off it.  AP_Compass_SITL applies this to
  the field it reports; get_mag_offsets() applies it to the offset
  itself.  Keeping the two in one place stops them diverging.
 */
void SIM::mag_sensor_transform(uint8_t instance, Vector3f &v) const
{
    v.rotate_inverse((enum Rotation)mag_orient[instance].get());
    v.rotate(AP::compass().get_board_orientation());

    // SIM_BRD_TRIM: rigid board mounting offset (same rotation applied
    // to the accels and gyros), keeping the compass consistent with the
    // IMU
    const Vector3f &trim = board_trim.get();
    if (!trim.is_zero()) {
        Matrix3f trim_rotation;
        trim_rotation.from_euler(trim.x, trim.y, trim.z);
        v = trim_rotation.transposed() * v;
    }

    // scale the compass to simulate sensor scale factor errors
    v *= mag_scaling[instance];
}

bool SIM::get_mag_offsets(uint8_t instance, Vector3f &offsets) const
{
    if (instance >= ARRAY_SIZE(mag_ofs)) {
        return false;
    }

    offsets = mag_ofs[instance];
    mag_sensor_transform(instance, offsets);

    return true;
}

/*
  create correction matrix for diagonals and off-diagonals
*/
void CompassSim::setup_eliptical_correction()
{
    const auto *_sitl = AP::sitl();

    Vector3f diag = _sitl->mag_diag[instance].get();
    if (diag.is_zero()) {
        diag = {1,1,1};
    }
    const Vector3f &diagonals = diag;
    const Vector3f &offdiagonals = _sitl->mag_offdiag[instance];

    if (diagonals == last_dia && offdiagonals == last_odi) {
        return;
    }

    eliptical_corr = Matrix3f(diagonals.x,    offdiagonals.x, offdiagonals.y,
                              offdiagonals.x, diagonals.y,    offdiagonals.z,
                              offdiagonals.y, offdiagonals.z, diagonals.z);
    if (!eliptical_corr.invert()) {
        eliptical_corr.identity();
    }
    last_dia = diag;
    last_odi = offdiagonals;
}

bool CompassSim::update(Vector3f &field)
{
    const auto *_sitl = AP::sitl();
    if (_sitl == nullptr) {
        return false;
    }
    if (instance >= ARRAY_SIZE(_sitl->mag_ofs)) {
        return false;
    }

    // TODO: Refactor delay buffer with AP_Baro_SITL.

    // Sampled at 100Hz
    const uint32_t now = AP_HAL::millis();
    if ((now - last_sample_time) < 10) {
        return false;
    }
    last_sample_time = now;

    // calculate sensor noise and add to 'truth' field in body frame
    // units are milli-Gauss
    const Vector3f noise = rand_vec3f() * _sitl->mag_noise;
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
    const uint32_t delayed_time = now - _sitl->mag_delay; // get time corresponding to delay
    // find data corresponding to delayed time in buffer
    for (uint8_t i=0; i<=buffer_length-1; i++) {
        // find difference between delayed time and time stamp in buffer
        const uint32_t time_delta = abs((int32_t)(delayed_time - buffer[i].time));
        // if this difference is smaller than last delta, store this time
        if (time_delta < best_time_delta) {
            best_index = i;
            best_time_delta = time_delta;
        }
    }
    if (best_time_delta < 1000) { // only output stored state if < 1 sec retrieval error
        new_mag_data = buffer[best_index].data;
    }

    setup_eliptical_correction();
    Vector3f f = (eliptical_corr * new_mag_data) - _sitl->mag_ofs[instance].get();
    _sitl->mag_sensor_transform(instance, f);

    switch (_sitl->mag_fail[instance]) {
    case 0:
        last_data = f;
        field = f;
        return true;
    case 1:
        // no data
        return false;
    case 2:
        // frozen compass
        field = last_data;
        return true;
    }

    return false;
}

#endif  // AP_SIM_ENABLED
