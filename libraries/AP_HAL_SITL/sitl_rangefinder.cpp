/*
  SITL handling

  This simulates a range_finder

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_HAL_SITL.h>
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"

#include <AP_Math.h>
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

    //fprintf(stdout, "2. altitude = %f \n", altitude);

    _range_finder->setHIL(0, state);

//     uint32_t now = hal.scheduler->millis();
//     // add delay
//     uint32_t best_time_delta_range = 1000; // initialise large time representing buffer entry closest to current time - delay.
//     uint8_t best_index_range = 0; // initialise number representing the index of the entry in buffer closest to delay.
// 
//     // storing data from sensor to buffer
//     if (now - last_store_time_range >= 10) { // store data every 10 ms.
//         last_store_time_range = now;
//         if (store_index_range > mag_buffer_length-1) { // reset buffer index if index greater than size of buffer
//             store_index_range = 0;
//         }
//         buffer_range[store_index_range].data = altitude; // add data to current index
//         buffer_range[store_index_range].time = last_store_time_range; // add time to current index
//         store_index_range = store_index_range + 1; // increment index
//     }
// 
//     // return delayed measurement
//     delayed_time_range = now - _sitl->range_delay; // get time corresponding to delay
//     // find data corresponding to delayed time in buffer
//     for (uint8_t i=0; i<=mag_buffer_length-1; i++) {
//         time_delta_range = abs(delayed_time_range - buffer_range[i].time); // find difference between delayed time and time stamp in buffer
//         // if this difference is smaller than last delta, store this time
//         if (time_delta_range < best_time_delta_range) {
//             best_index_range = i;
//             best_time_delta_range = time_delta_range;
//         }
//     }
//     if (best_time_delta_range < 1000) { // only output stored state if < 1 sec retrieval error
//         new_range_data = buffer_range[best_index_range].data;
//     }
// 
//     _range_finder->setHIL(0, new_range_data);
}

#endif
