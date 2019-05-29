#include <AP_HAL/AP_HAL.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_Compass_Backend::AP_Compass_Backend()
    : _compass(AP::compass())
{
}

void AP_Compass_Backend::rotate_field(Vector3f &mag, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];
    mag.rotate(MAG_BOARD_ORIENTATION);
    mag.rotate(state.rotation);

    if (!state.external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        if (_compass._board_orientation == ROTATION_CUSTOM && _compass._custom_rotation) {
            mag = *_compass._custom_rotation * mag;
        } else {
            mag.rotate(_compass._board_orientation);
        }
    } else {
        // add user selectable orientation
        mag.rotate((enum Rotation)state.orientation.get());
    }
}

void AP_Compass_Backend::publish_raw_field(const Vector3f &mag, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];

    // note that we do not set last_update_usec here as otherwise the
    // EKF and DCM would end up consuming compass data at the full
    // sensor rate. We want them to consume only the filtered fields
    state.last_update_ms = AP_HAL::millis();

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

    // add in the basic offsets
    mag += offsets;

    // apply eliptical correction
    Matrix3f mat(
        diagonals.x, offdiagonals.x, offdiagonals.y,
        offdiagonals.x,    diagonals.y, offdiagonals.z,
        offdiagonals.y, offdiagonals.z,    diagonals.z
    );

    mag = mat * mag;

    /*
      calculate motor-power based compensation
      note that _motor_offset[] is kept even if compensation is not
      being applied so it can be logged correctly
    */    
    state.motor_offset.zero();
    if (_compass._per_motor.enabled() && i == 0) {
        // per-motor correction is only valid for first compass
        _compass._per_motor.compensate(state.motor_offset);
    } else if (_compass._motor_comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
        state.motor_offset = mot * _compass._thr;
    } else if (_compass._motor_comp_type == AP_COMPASS_MOT_COMP_CURRENT) {
        AP_BattMonitor &battery = AP::battery();
        if (battery.has_current()) {
            state.motor_offset = mot * battery.current_amps();
        }
    }

    /*
      we apply the motor offsets after we apply the eliptical
      correction. This is needed to match the way that the motor
      compensation values are calculated, as they are calculated based
      on final field outputs, not on the raw outputs
    */
    mag += state.motor_offset;
}

void AP_Compass_Backend::accumulate_sample(Vector3f &field, uint8_t instance,
                                           uint32_t max_samples)
{
    /* rotate raw_field from sensor frame to body frame */
    rotate_field(field, instance);

    /* publish raw_field (uncorrected point sample) for calibration use */
    publish_raw_field(field, instance);

    /* correct raw_field for known errors */
    correct_field(field, instance);

    if (!field_ok(field)) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    Compass::mag_state &state = _compass._state[instance];
    state.accum += field;
    state.accum_count++;
    if (max_samples && state.accum_count >= max_samples) {
        state.accum_count /= 2;
        state.accum /= 2;
    }
}

void AP_Compass_Backend::drain_accumulated_samples(uint8_t instance,
                                                   const Vector3f *scaling)
{
    WITH_SEMAPHORE(_sem);

    Compass::mag_state &state = _compass._state[instance];

    if (state.accum_count == 0) {
        return;
    }

    if (scaling) {
        state.accum *= *scaling;
    }
    state.accum /= state.accum_count;

    publish_filtered_field(state.accum, instance);

    state.accum.zero();
    state.accum_count = 0;
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
}

void AP_Compass_Backend::set_last_update_usec(uint32_t last_update, uint8_t instance)
{
    Compass::mag_state &state = _compass._state[instance];
    state.last_update_usec = last_update;
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
    _compass._state[instance].dev_id.set_and_notify(dev_id);
    _compass._state[instance].detected_dev_id = dev_id;
}

/*
  save dev_id, used by SITL
*/
void AP_Compass_Backend::save_dev_id(uint8_t instance)
{
    _compass._state[instance].dev_id.save();
}

/*
  set external for an instance
*/
void AP_Compass_Backend::set_external(uint8_t instance, bool external)
{
    if (_compass._state[instance].external != 2) {
        _compass._state[instance].external.set_and_notify(external);
    }
}

bool AP_Compass_Backend::is_external(uint8_t instance)
{
    return _compass._state[instance].external;
}

// set rotation of an instance
void AP_Compass_Backend::set_rotation(uint8_t instance, enum Rotation rotation)
{
    _compass._state[instance].rotation = rotation;
}

static constexpr float FILTER_KOEF = 0.1f;

/* Check that the compass value is valid by using a mean filter. If
 * the value is further than filtrer_range from mean value, it is
 * rejected. 
*/
bool AP_Compass_Backend::field_ok(const Vector3f &field)
{

    
    if (field.is_inf() || field.is_nan()) {
        return false;
    }

    const float range = (float)_compass.get_filter_range();
    if (range <= 0) {
        return true;
    }

    const float length = field.length();

    if (is_zero(_mean_field_length)) {
        _mean_field_length = length;
        return true;
    }

    bool ret = true;
    const float d = fabsf(_mean_field_length - length) / (_mean_field_length + length);  // diff divide by mean value in percent ( with the *200.0f on later line)
    float koeff = FILTER_KOEF;

    if (d * 200.0f > range) {  // check the difference from mean value outside allowed range
        // printf("\nCompass field length error: mean %f got %f\n", (double)_mean_field_length, (double)length );
        ret = false;
        koeff /= (d * 10.0f);  // 2.5 and more, so one bad sample never change mean more than 4%
        _error_count++;
    }
    _mean_field_length = _mean_field_length * (1 - koeff) + length * koeff;  // complimentary filter 1/k

    return ret;
}


enum Rotation AP_Compass_Backend::get_board_orientation(void) const
{
    return _compass._board_orientation;
}
