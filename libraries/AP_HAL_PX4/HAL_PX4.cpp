#include <stdio.h>
#include "HAL_PX4.h"
using namespace AP_HAL_PX4;

void HAL_PX4::init(void* opts) const {
	console->init(NULL);
}
