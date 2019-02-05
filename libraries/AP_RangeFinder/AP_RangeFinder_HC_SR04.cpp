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

#include "AP_RangeFinder_HC_SR04.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder.
*/
AP_RangeFinder_HC_SR04::AP_RangeFinder_HC_SR04(RangeFinder::RangeFinder_State &_state,
                                       AP_Int16 &_powersave_range,
                                       float &_estimated_terrain_height) :
    AP_RangeFinder_Backend(_state),
    powersave_range(_powersave_range),
    estimated_terrain_height(_estimated_terrain_height)
{
	// set the pin for send and receive
	hal.gpio->pinMode(state.pin, HAL_GPIO_OUTPUT);
	hal.gpio->write(state.pin, false);
	hal.gpio->pinMode(state.address, HAL_GPIO_INPUT);
		
	// Switch on interrupt
	if (!hal.gpio->attach_interrupt(
        state.addr,
        FUNCTOR_BIND_MEMBER(&AP_RangeFinder_HC_SR04::irq_handler,
            void,
            uint8_t,
            bool,
            uint32_t),
            AP_HAL::GPIO::INTERRUPT_RISING)) {
            // failed to attach interrupt
            gcs().send_text(MAV_SEVERITY_WARNING,
                        "RangeFinder__HC_SR04: Failed to detach from pin %u",
                        state.address);
    }  
	
	// calculate the maximum response time
	distance_m = state.max_distance_cm / 100;
	max_response_delay_ms = disdance_m * 2.912f; // 2,912 ms / m
	max_response_delay_ms = max_response_delay_ms + (max_response_delay_ms / 10); // plus 10% to be sure
	is_in_range = true;
	startMeasurement();
}

/*
   send a trigger pulse to the sensor to initiate the measurement
*/
void AP_RangeFinder_HC_SR04::startMeasurement()
{
	if (is_in_range) {
	    hal.gpio->write(state.pin, true); 
	    // usleep(10);
		hal.scheduler->micros(10) 		
	    hal.gpio->write(state.pin, false); 
	    irq_pulse_start_us = AP_HAL::micros();
	}
}

/*
   interrupt handler to read the answer
*/
void AP_RangeFinder_HC_SR04::irq_handler(uint8_t pin, bool pin_high, uint32_t timestamp_us)
{
	if (state.address == pin){
	    if (pin_high) {
		    if (irq_pulse_start_us != 0) {
		     irq_value_us += timestamp_us - irq_pulse_start_us;
             irq_pulse_start_us = 0;
			 startMeasurement();
		    } else {
			    gcs().send_text(MAV_SEVERITY_WARNING,
                            "RangeFinder__HC_SR04: Answer without question %u",
                            last_pin);	
		    }   
        } else {
		gcs().send_text(MAV_SEVERITY_WARNING,
                            "RangeFinder__HC_SR04: Wrong level detect %u",
                            last_pin);
		}
    } else {
		gcs().send_text(MAV_SEVERITY_WARNING,
                            "RangeFinder__HC_SR04: Wrong pin detect %u",
                            last_pin);
	}
}

/*
    there's no sensible way of detecting rangefinder because 
	I do not know how far the sensor is from the next object
*/
bool AP_RangeFinder_HC_SR04::detect()
{
    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_HC_SR04::get_reading(uint16_t &reading_cm)
{
    // disable interrupts and grab state
    void *irqstate = hal.scheduler->disable_interrupts_save();
    const uint32_t value_us = irq_value_us;
    irq_value_us = 0;
    hal.scheduler->restore_interrupts(irqstate);

    if (value_us == 0) {
        return false;
    }
    reading_cm = (value_us/2) / 29.1;
    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_HC_SR04::update(void)
{
    if (state.pin <= 0) {
        // disabled (by configuration)
        return;
    }
	
    #if OUT_OF_RANGE_DETECT == ENABLED	
	const bool oor = out_of_range();
	if (oor) {
		if (!was_out_of_range) {
			// we are above the power saving range. Disable the sensor
			is_in_range = false;
			set_status(RangeFinder::RangeFinder_NoData);
			state.distance_cm = 0;
			was_out_of_range = oor;
		}
		return;
	}
	// re-enable the sensor:
	if (!oor && was_out_of_range) {
		is_in_range = true;
		startMeasurement();
		was_out_of_range = oor;
	}
	#endif

    if (!get_reading(state.distance_cm)) {
        // failure; consider changing our state
        if (AP_HAL::millis() - state.last_reading_ms > max_response_delay_ms) {
            set_status(RangeFinder::RangeFinder_NoData);
			
			// restart the sensor
			startMeasurement();
        }
        return;
    }

    // update range_valid state based on distance measured
    state.last_reading_ms = AP_HAL::millis();
    update_status();
}

#if OUT_OF_RANGE_DETECT == ENABLED	
// return true if we are beyond the power saving range
bool AP_RangeFinder_HC_SR04::out_of_range(void) const {
    return powersave_range > 0 && estimated_terrain_height > powersave_range;
}
#endif