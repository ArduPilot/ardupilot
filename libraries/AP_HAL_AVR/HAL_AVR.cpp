
#include "HAL_AVR.h"
using namespace AP_HAL_AVR;

void HAL_AVR::init(void* opts) const {

    scheduler->init();

    /* The AVR RCInput drivers take an AP_HAL_AVR::ISRRegistry*
     * as the init argument */
    rcin->init((void*)&isr_registry);

    rcout->init(NULL);
};

