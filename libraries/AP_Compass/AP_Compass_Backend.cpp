/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "Compass.h"
#include "AP_Compass_Backend.h"

AP_Compass_Backend::AP_Compass_Backend(Compass &compass) :
    _compass(compass),
    product_id(AP_PRODUCT_ID_NONE)
{}

/*
  copy latest data to the frontend from a backend
 */
void AP_Compass_Backend::_copy_to_frontend(uint8_t instance) {

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _compass._field[instance].rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _compass._field[instance].rotate((enum Rotation)_compass._orientation[instance].get());

    if (!_compass._external[instance]) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        _compass._field[instance].rotate(_compass.get_board_orientation());
    }

    _compass.apply_corrections(_compass._field[instance],instance);
}