#include "AP_WaterDetector.h"
#include "AP_WaterDetector_Digital.h"
#include "AP_WaterDetector_Analog.h"

#include <AP_HAL/AP_HAL.h>

const AP_Param::GroupInfo AP_WaterDetector::var_info[] = {

	// @Param: PIN
	// @DisplayName: Pin that water detector is connected to
	// @Description:
    // @Values: -1:Disabled, 50:Pixhawk Aux1, 51:Pixhawk Aux2, 52:Pixhawk Aux3, 53:Pixhawk Aux4, 54:Pixhawk Aux5, 55:Pixhawk Aux6, 13:Pixhawk 3.3ADC1, 14:Pixhawk 3.3ADC2, 15:Pixhawk 6.6ADC
	// @User: Standard
    AP_GROUPINFO("1_PIN", 0, AP_WaterDetector, _pin[0], -1),

	// @Param: DEFAULT
	// @DisplayName: Default reading of water detector when dry
	// @Description:
	// @Values: 0:Low, 1:High
	// @User: Standard
	AP_GROUPINFO("1_DEFAULT", 1, AP_WaterDetector, _default_reading[0], 1),

	// @Param: TYPE
	// @DisplayName: Water detector instance type
	// @Description:
	// @Values: 0:None, 1:Digital, 2:Analog, 3:Mavlink
	// @User: Standard
	AP_GROUPINFO("1_TYPE", 2, AP_WaterDetector, _type[0], WATERDETECTOR_TYPE_NONE),

#if WATERDETECTOR_MAX_INSTANCES > 1
	// @Param: PIN
	// @DisplayName: Pin that water detector is connected to
	// @Description:
    // @Values: -1:Disabled, 50:Pixhawk Aux1, 51:Pixhawk Aux2, 52:Pixhawk Aux3, 53:Pixhawk Aux4, 54:Pixhawk Aux5, 55:Pixhawk Aux6, 13:Pixhawk 3.3ADC1, 14:Pixhawk 3.3ADC2, 15:Pixhawk 6.6ADC
	// @User: Standard
    AP_GROUPINFO("2_PIN", 3, AP_WaterDetector, _pin[1], -1),

	// @Param: DEFAULT
	// @DisplayName: Default reading of water detector when dry
	// @Description:
	// @Values: 0:Low, 1:High
	// @User: Standard
	AP_GROUPINFO("2_DEFAULT", 4, AP_WaterDetector, _default_reading[1], 1),

	// @Param: TYPE
	// @DisplayName: Water detector instance type
	// @Description:
	// @Values: 0:None, 1:Digital, 2:Analog, 3:Mavlink
	// @User: Standard
	AP_GROUPINFO("2_TYPE", 5, AP_WaterDetector, _type[1], WATERDETECTOR_TYPE_NONE),
#endif

#if WATERDETECTOR_MAX_INSTANCES > 2
	// @Param: PIN
	// @DisplayName: Pin that water detector is connected to
	// @Description:
    // @Values: -1:Disabled, 50:Pixhawk Aux1, 51:Pixhawk Aux2, 52:Pixhawk Aux3, 53:Pixhawk Aux4, 54:Pixhawk Aux5, 55:Pixhawk Aux6, 13:Pixhawk 3.3ADC1, 14:Pixhawk 3.3ADC2, 15:Pixhawk 6.6ADC
	// @User: Standard
    AP_GROUPINFO("3_PIN", 6, AP_WaterDetector, _pin[2], -1),

	// @Param: DEFAULT
	// @DisplayName: Default reading of water detector when dry
	// @Description:
	// @Values: 0:Low, 1:High
	// @User: Standard
	AP_GROUPINFO("3_DEFAULT", 7, AP_WaterDetector, _default_reading[2], 1),

	// @Param: TYPE
	// @DisplayName: Water detector instance type
	// @Description:
	// @Values: 0:None, 1:Digital, 2:Analog, 3:Mavlink
	// @User: Standard
	AP_GROUPINFO("3_TYPE", 8, AP_WaterDetector, _type[2], WATERDETECTOR_TYPE_NONE),
#endif

    AP_GROUPEND
};

AP_WaterDetector::AP_WaterDetector() :
	_status(false),
	_last_detect_ms(0)
{
	AP_Param::setup_object_defaults(this, var_info);

    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
};

void AP_WaterDetector::init()
{
	for(int i = 0; i < WATERDETECTOR_MAX_INSTANCES; i++) {
		switch (_type[i]) {
		case WATERDETECTOR_TYPE_DIGITAL:
			state[i].instance = i;
			drivers[i] = new AP_WaterDetector_Digital(*this, state[i]);
			break;
		case WATERDETECTOR_TYPE_ANALOG:
			state[i].instance = i;
			drivers[i] = new AP_WaterDetector_Analog(*this, state[i]);
			break;
		case WATERDETECTOR_TYPE_NONE:
		default:
			drivers[i] = NULL;
			break;
		}
	}
}

bool AP_WaterDetector::update()
{
	uint32_t tnow = AP_HAL::millis();

	for(int i = 0; i < WATERDETECTOR_MAX_INSTANCES; i++) {
		if(drivers[i] != NULL) {
			drivers[i]->read();
			if(state[i].status) {
				_last_detect_ms = tnow;
			}
		}
	}

	_status = tnow < _last_detect_ms + WATERDETECTOR_COOLDOWN_MS;

	return _status;
}

void AP_WaterDetector::set_detect()
{
	_last_detect_ms = AP_HAL::millis();
}
