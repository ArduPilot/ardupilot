#include "AP_WaterDetector.h"
#include "AP_WaterDetector_Analog.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_WaterDetector_Analog::AP_WaterDetector_Analog(AP_WaterDetector &_water_detector, AP_WaterDetector::WaterDetector_State &_state) :
AP_WaterDetector_Backend(_water_detector, _state)
{
    source = hal.analogin->channel(water_detector._pin[state.instance]);
}

void AP_WaterDetector_Analog::read()
{
	if(source != NULL && water_detector._pin[state.instance] >= 0) {
		source->set_pin(water_detector._pin[state.instance]);
		state.status = source->voltage_average() > 2.0f;
		state.status = state.status==water_detector._default_reading[state.instance]?false:true;
	} else {
		state.status = false;
	}
}
