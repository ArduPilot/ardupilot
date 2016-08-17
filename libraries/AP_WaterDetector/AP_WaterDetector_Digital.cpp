#include "AP_WaterDetector.h"
#include "AP_WaterDetector_Digital.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_WaterDetector_Digital::AP_WaterDetector_Digital(AP_WaterDetector &_water_detector, AP_WaterDetector::WaterDetector_State &_state) :
AP_WaterDetector_Backend(_water_detector, _state)
{}

void AP_WaterDetector_Digital::read()
{
	if(water_detector._pin[state.instance] >= 0) {
		hal.gpio->pinMode(water_detector._pin[state.instance], HAL_GPIO_INPUT);
		state.status = hal.gpio->read(water_detector._pin[state.instance])==water_detector._default_reading[state.instance]?false:true;
	} else {
		state.status = false;
	}
}
