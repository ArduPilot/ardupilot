#include "AP_WaterDetector.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_WaterDetector::var_info[] = {

	// @Param: PIN
	// @DisplayName: Pin that water detector is connected to
	// @Description:
    // @Values: 55:Pixhawk Aux6
	// @User: Standard
    AP_GROUPINFO("PIN",        0, AP_WaterDetector, _pin, 55),

	// @Param: DEFAULT
	// @DisplayName: Default reading of water detector when dry
	// @Description:
	// @Values: 0:Low, 1:High
	// @User: Standard
	AP_GROUPINFO("DEFAULT",        1, AP_WaterDetector, _default_reading, 0),

    AP_GROUPEND
};

AP_WaterDetector::AP_WaterDetector() :
	_status(false)
{
	AP_Param::setup_object_defaults(this, var_info);
};

void AP_WaterDetector::read()
{
	if(_pin >= 0) {
		hal.gpio->pinMode(_pin, HAL_GPIO_INPUT);
		_status |= hal.gpio->read(_pin)==_default_reading?0:1; // once water is detected, status is always true, regardless of result of subsequent calls to read
	}
}
