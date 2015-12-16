/*
  SITL handling

  This simulates a range finder

  Alex Buyval, August 2015
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>


/*
  setup the range_finder with new input
  range input is in meters
 */
void SITL_State::_update_range_finder(float range)
{
	if (_range_finder == NULL) {
        // no range_finder in this sketch
        return;
    }

    RangeFinder::RangeFinder_State state;
    state.instance = 0;
    state.distance_cm = range*100;
    state.status = RangeFinder::RangeFinder_Good;

    // TODO: Add lag
    _range_finder->setHIL(0, state);
}


#endif
