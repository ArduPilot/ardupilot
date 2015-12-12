/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "Compass.h"
#include "AP_Compass_Backend.h"

extern const AP_HAL::HAL& hal;

AP_Compass_Backend::AP_Compass_Backend(Compass &compass) :
    _compass(compass)
{}

/*
 * A compass measurement is expected to pass through the following functions:
 * 1. rotate_field - this rotates the measurement in-place from sensor frame
 *      to body frame
 * 2. publish_raw_field - this provides an uncorrected point-sample for
 *      calibration libraries
 * 3. correct_field - this corrects the measurement in-place for hard iron,
 *      soft iron, motor interference, and non-orthagonality errors
 * 4. publish_unfiltered_field - this (optionally) provides a corrected
 *      point sample for fusion into the EKF
 * 5. publish_filtered_field - legacy filtered magnetic field
 *
 * All those functions expect the mag field to be in milligauss.
 */

void AP_Compass_Backend::rotate_field(Vector3f &mag, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];
    mag.rotate(MAG_BOARD_ORIENTATION);

    if (!state.external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        mag.rotate(_compass._board_orientation);
    } else {
        // add user selectable orientation
        mag.rotate((enum Rotation)state.orientation.get());
    }
}

void AP_Compass_Backend::publish_raw_field(const Vector3f &mag, uint32_t time_us, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];

    state.last_update_ms = AP_HAL::millis();

    // note that we do not set last_update_usec here as otherwise the
    // EKF and DCM would end up consuming compass data at the full
    // sensor rate. We want them to consume only the filtered fields
    
    state.raw_field = mag;
    state.raw_meas_time_us = time_us;
    state.updated_raw_field = true;
    state.has_raw_field = true;

    _compass._calibrator[instance].new_sample(mag);
}

void AP_Compass_Backend::correct_field(Vector3f &mag, uint8_t i)
{
    Compass::mag_state &state = _compass._state[i];

    if (state.diagonals.get().is_zero()) {
        state.diagonals.set(Vector3f(1.0f,1.0f,1.0f));
    }

    const Vector3f &offsets = state.offset.get();
    const Vector3f &diagonals = state.diagonals.get();
    const Vector3f &offdiagonals = state.offdiagonals.get();
    const Vector3f &mot = state.motor_compensation.get();

    /*
     * note that _motor_offset[] is kept even if compensation is not
     * being applied so it can be logged correctly
     */
    mag += offsets;
    if(_compass._motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && !is_zero(_compass._thr_or_curr)) {
        state.motor_offset = mot * _compass._thr_or_curr;
        mag += state.motor_offset;
    } else {
        state.motor_offset.zero();
    }

    Matrix3f mat(
        diagonals.x, offdiagonals.x, offdiagonals.y,
        offdiagonals.x,    diagonals.y, offdiagonals.z,
        offdiagonals.y, offdiagonals.z,    diagonals.z
    );

    mag = mat * mag;
}

void AP_Compass_Backend::publish_unfiltered_field(const Vector3f &mag, uint32_t time_us, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];

    state.unfiltered_field = mag;
    state.raw_meas_time_us = time_us;
    state.updated_unfiltered_field = true;
    state.has_unfiltered_field = true;
}

/*
  copy latest data to the frontend from a backend
 */
void AP_Compass_Backend::publish_filtered_field(const Vector3f &mag, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];

    state.field = mag;

    state.last_update_ms = AP_HAL::millis();
    state.last_update_usec = AP_HAL::micros();

    state.has_raw_field = state.updated_raw_field;
    state.updated_raw_field = false;

    state.has_unfiltered_field = state.updated_unfiltered_field;
    state.updated_unfiltered_field = false;
}

/*
  register a new backend with frontend, returning instance which
  should be used in publish_field()
 */
uint8_t AP_Compass_Backend::register_compass(void) const
{ 
    return _compass.register_compass(); 
}


/*
  set dev_id for an instance
*/
void AP_Compass_Backend::set_dev_id(uint8_t instance, uint32_t dev_id)
{
    _compass._state[instance].dev_id.set(dev_id);
}

/*
  set external for an instance
*/
void AP_Compass_Backend::set_external(uint8_t instance, bool external)
{
    _compass._state[instance].external.set(external);
}
