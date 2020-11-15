/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Code by David "Buzz" Bussenschutt and others
 */
#include "RCInput.h"
#include <stdio.h>


#include <AP_Math/AP_Math.h>

#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif

#define SIG_DETECT_TIMEOUT_US 500000
using namespace ESP32;
extern const AP_HAL::HAL& hal;
void RCInput::init()
{
   printf("RCInput::init()\n");

    sig_reader.init();
    rcin_prot.init();

    _init = true;
}

bool RCInput::new_input()
{
    if (!_init) {
        return false;
    }
    if (!rcin_mutex.take_nonblocking()) {
        return false;
    }
    bool valid = _rcin_timestamp_last_signal != _last_read;

    _last_read = _rcin_timestamp_last_signal;
    rcin_mutex.give();
  
    return valid;
}

uint8_t RCInput::num_channels()
{
    if (!_init) {
        return 0;
    }
    return _num_channels;
}

uint16_t RCInput::read(uint8_t channel)
{
    if (!_init || (channel >= MIN(RC_INPUT_MAX_CHANNELS, _num_channels))) {
        return 0;
    }
    rcin_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER);
    uint16_t v = _rc_values[channel];
    rcin_mutex.give();

    return v;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len)
{
    if (!_init) {
        return false;
    }
 
    if (len > RC_INPUT_MAX_CHANNELS) {
        len = RC_INPUT_MAX_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++){
        periods[i] = read(i);
    }
    return len;
}

void RCInput::_timer_tick(void)
{

	//printf("RCInput _timer_tick debug");

    if (!_init) { //BUZZ PUT THIS BACK HACK
        return;
    }

        uint32_t width_s0, width_s1;
        while(sig_reader.read(width_s0, width_s1)) {
            //printf(" %d %d . ",width_s0 ,width_s1);
            rcin_prot.process_pulse(width_s0, width_s1);
            
        }
        //printf("\n");

        const char *rc_protocol = nullptr;

        if (rcin_prot.new_input()) {
        
        //printf("new_input\n");
            rcin_mutex.take(HAL_SEMAPHORE_BLOCK_FOREVER);
            _rcin_timestamp_last_signal = AP_HAL::micros();
            _num_channels = rcin_prot.num_channels();
            _num_channels = MIN(_num_channels, RC_INPUT_MAX_CHANNELS);
            
            // throw away channels that aren't totalling 8, corruption.
            //if ( _num_channels != 8 ) { 
            //	for (uint8_t i=0; i<_num_channels; i++) {
            //    	rcin_prot.read(i);
            //   }
            //   printf("not8\n");
            //}
            // range check and throw ones outside range: 

            // otherwise accept them
            //else { 
            //	int drop = 0;
            
            static unsigned int show_rc = 0;
                   
            
	        	for (uint8_t i=0; i<_num_channels; i++) {
	        	
	        		uint16_t tmpv = rcin_prot.read(i);
	        		
	        		//if ( drop == 1 ) continue; 
	        		
	        		// skip out-of-range ones, just drop the entire rest of the frame and assume it'll be ok next frame.
	        		//if (( tmpv < 980 ) || ( tmpv > 2020 )) { drop=1; continue;}  
	        		
	        		// ignore things a long way off from the current value by about 90%
	        		//if ( abs ( _rc_values[i] - tmpv ) > 150 ) {  _rc_values[i] = (_rc_values[i]/10)*9 + tmpv/9; continue; } 
	        		
	        		// average last few values.., ie use about 30%
					//_rc_values[i] = (_rc_values[i]/3)*2 + tmpv/3;
	        		
	                _rc_values[i] = tmpv;
	                
	                if (show_rc == 0){ printf("RC %d  ",_rc_values[i]); }
	            }
	            if (show_rc == 0){ printf("\n");}
	        //}
	        
	        show_rc++; if ( show_rc > 200 ) show_rc = 0; // a few secs
            
            rcin_mutex.give();

            rc_protocol = rcin_prot.protocol_name();

    }

        if (rc_protocol && rc_protocol != last_protocol) {
            last_protocol = rc_protocol;
            gcs().send_text(MAV_SEVERITY_DEBUG, "RCInput: decoding %s", last_protocol);
            printf("RCInput: decoding %s\n", last_protocol);
        }

    // note, we rely on the vehicle code checking new_input()
    // and a timeout for the last valid input to handle failsafe
}

/*
  start a bind operation, if supported
 */
//bool RCInput::rc_bind(int dsmMode)
//{
  // not impl
 //   return true;
//}
