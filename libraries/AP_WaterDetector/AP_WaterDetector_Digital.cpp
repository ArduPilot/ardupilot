#include <AP_HAL/AP_HAL.h>
#include "AP_WaterDetector_Digital.h"

extern const AP_HAL::HAL& hal;

AP_WaterDetector_Digital::AP_WaterDetector_Digital() :
AP_WaterDetector(),
_pin(55), // pixhawk aux6
_default(true)
{

}

void AP_WaterDetector_Digital::read()
{
	if(_pin >= 0) {
		hal.gpio->pinMode(_pin, HAL_GPIO_INPUT);
		_status = hal.gpio->read(_pin)==_default?0:1;
	}
}
