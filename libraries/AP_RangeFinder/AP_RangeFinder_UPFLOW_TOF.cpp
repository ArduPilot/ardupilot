/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_UPFLOW_TOF.h"

#if AP_RANGEFINDER_UPFLOW_Tx_ENABLED


#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <ctype.h>
#include <AP_OpticalFlow/AP_OpticalFlow_UPFLOW_Tx.h>
//#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;
static int e = 0;

AP_RangeFinder_UPFLOW_TOF::AP_RangeFinder_UPFLOW_TOF(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    set_status(RangeFinder::Status::NoData);
}

bool AP_RangeFinder_UPFLOW_TOF::detect()
{
    // Assume that if the user set the RANGEFINDER_TYPE parameter to MSP,
    // there is an attached MSP rangefinder
    return true;
}

/*
  update distance_m
 */
void AP_RangeFinder_UPFLOW_TOF::update(void)
{
    static UPFLOW_TOF* tof_data = get_upflow_tof();
    // check validity of the data
    if( tof_data->if_opt_ok == true && tof_data->tof_valid != 0 ){
        state.distance_m = tof_data->ground_distance * 0.001f;
        tof_data->if_opt_ok = false;
        e = 0;
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "tof: OK");
    }else{
        e++;
        if(e>10){
            state.distance_m = 0.0f;
            set_status(RangeFinder::Status::NoData);
            //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "tof: err");
        }
    }
    
    // update range_valid state based on distance measured
    update_status();
}

#endif  // AP_RANGEFINDER_UPFLOW_Tx_ENABLED
