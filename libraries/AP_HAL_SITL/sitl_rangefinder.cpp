/*
  SITL handling

  This simulates a range_finder

  Andrew Tridgell November 2011
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include "AP_HAL_SITL/AP_HAL_SITL_Namespace.h"
#include "AP_HAL_SITL/HAL_SITL_Class.h"

#include <AP_Math/AP_Math.h>
#include "../AP_RangeFinder/RangeFinder.h"
#include "../SITL/SITL.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

/*
  setup the range_finder with new input
  all inputs are in degrees
 */
void SITL_State::_update_range_finder(float altitude)
{
	if (_range_finder == NULL) {
        // no range_finder in this sketch
        return;
    }

    RangeFinder::RangeFinder_State state;
    state.instance = 0;
    state.distance_cm = altitude*100;
    state.status = RangeFinder::RangeFinder_Good;

    _range_finder->setHIL(0, state);

}

#endif
