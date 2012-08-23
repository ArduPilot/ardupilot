
#include "HAL_AVR.h"
using namespace AP_HAL_AVR;

void HAL_AVR::init(void* opts) const {
    scheduler->init();
};

