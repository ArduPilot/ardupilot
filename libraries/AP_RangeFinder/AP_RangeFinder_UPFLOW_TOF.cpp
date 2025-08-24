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
 
 extern const AP_HAL::HAL& hal;
 
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
 
 //update distance_m
 // The TOF data is parsed in the AP_OpticalFlow_UPFLOW_Tx 
 void AP_RangeFinder_UPFLOW_TOF::update(void)
 {
     AP_OpticalFlow_UPFLOW_Tx::Uplx_TOF tof_data;
     // check validity of the data
     if(AP_OpticalFlow_UPFLOW_Tx::get_tof_data(tof_data)){
         state.distance_m = tof_data.distance_mm * 0.001f;
         state.signal_quality_pct = tof_data.quality;
         state.last_reading_ms = tof_data.last_update_ms;
         state.status = RangeFinder::Status::Good;
     }else{
         glitch_count++;
         if(glitch_count>10){
             state.distance_m = 0.0f;
             set_status(RangeFinder::Status::NoData);
         }
     }
     
     // update range_valid state based on distance measured
     update_status();
 }
 
 #endif  // AP_RANGEFINDER_UPFLOW_Tx_ENABLED
 