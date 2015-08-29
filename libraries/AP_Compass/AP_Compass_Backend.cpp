/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "Compass.h"
#include "AP_Compass_Backend.h"

extern const AP_HAL::HAL& hal;

AP_Compass_Backend::AP_Compass_Backend(Compass &compass) :
    _compass(compass)
{}

/*
  copy latest data to the frontend from a backend
 */
void AP_Compass_Backend::publish_field(const Vector3f &mag, uint8_t instance) 
{
    Compass::mag_state &state = _compass._state[instance];

    state.field = mag;

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    state.field.rotate(MAG_BOARD_ORIENTATION);

    if (!state.external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        state.field.rotate(_compass._board_orientation);
    } else {
        // add user selectable orientation
        state.field.rotate((enum Rotation)state.orientation.get());
    }

    apply_corrections(state.field, instance);

    state.last_update_ms = hal.scheduler->millis();
    state.last_update_usec = hal.scheduler->micros();
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
  apply offset and motor compensation corrections
 */
void AP_Compass_Backend::apply_corrections(Vector3f &mag, uint8_t i)
{
    Compass::mag_state &state = _compass._state[i];
    const Vector3f &offsets = state.offset.get();
    const Vector3f &mot = state.motor_compensation.get();

    /*
      note that _motor_offset[] is kept even if compensation is not
      being applied so it can be logged correctly
     */
    mag += offsets;
    if(_compass._motor_comp_type != AP_COMPASS_MOT_COMP_DISABLED && !is_zero(_compass._thr_or_curr)) {
        state.motor_offset = mot * _compass._thr_or_curr;
        mag += state.motor_offset;
    } else {
        state.motor_offset.zero();
    }
}


/*
  set dev_id for an instance
*/
void AP_Compass_Backend::set_dev_id(uint8_t instance, uint32_t dev_id)
{
#if COMPASS_MAX_INSTANCES > 1
    _compass._state[instance].dev_id.set(dev_id);
#endif
}

/*
  set external for an instance
*/
void AP_Compass_Backend::set_external(uint8_t instance, bool external)
{
    _compass._state[instance].external.set(external);
}
