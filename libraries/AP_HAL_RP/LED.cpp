#include "LED.h"

using namespace RP;

extern const AP_HAL::HAL& hal;

bool LED::init(void) {
	bool status = _led.init();
	if (status) {
		hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&LED::update, void));
	} 
	return status;
}

void LED::update(void) {
	_led.update();
}
