
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_private.h>
#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"

// XXX placeholder
uint8_t isrRegistry;

static AVRScheduler avrScheduler;

HAL_AVR_SITL::HAL_AVR_SITL() :
    AP_HAL::HAL(
        NULL, /* uartA */
        NULL, /* uartB */
        NULL, /* uartC */
        NULL, /* i2c */
        NULL, /* spi */
        NULL, /* analogin */
        NULL, /* storage */
        NULL, /* console */
        NULL, /* gpio */
        NULL, /* rcinput */
        NULL, /* rcoutput */
        &avrScheduler) /* scheduler */
{}

void HAL_AVR_SITL::init(void* machtnichts) const {

    scheduler->init((void*)&isrRegistry);
    uartA->begin(115200);
    console->init((void*) uartA);

    rcin->init((void*)&isrRegistry);
    rcout->init(NULL);
    spi->init(NULL);
    i2c->begin();
    i2c->setTimeout(100);
    analogin->init(NULL);
}

const HAL_AVR_SITL AP_HAL_AVR_SITL;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
