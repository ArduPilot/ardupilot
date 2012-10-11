
#include "HAL_AVR.h"
using namespace AP_HAL_AVR;

void HAL_AVR::init(void* opts) const {

    scheduler->init((void*)&isr_registry);
   
    /* uart0 is the serial port used for the console, so lets make sure
     * it is initialized at boot */
    uart0->begin(115200);
    console->init((void*)uart0);
    /* The AVR RCInput drivers take an AP_HAL_AVR::ISRRegistry*
     * as the init argument */
    rcin->init((void*)&isr_registry);
    rcout->init(NULL);
    spi->init(NULL);
    i2c->begin();
    i2c->setTimeout(100);
    analogin->init(NULL);
};

