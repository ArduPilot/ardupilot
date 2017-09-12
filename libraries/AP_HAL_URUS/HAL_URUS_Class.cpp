#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS

#include <assert.h>

#include "AP_HAL_URUS.h"
#include "HAL_URUS_Class.h"
#include "CORE_URUS/CORE_URUS.h"

const NSCORE_URUS::CLCORE_URUS& _urus_core = NSCORE_URUS::get_CORE();
static NSCORE_URUS::CLCoreUrusScheduler& urusScheduler = *NSCORE_URUS::get_scheduler();
static NSCORE_URUS::CLCoreUrusUARTDriver& urusUartA = *NSCORE_URUS::get_uartDriver();

HAL_URUS::HAL_URUS() :
    AP_HAL::HAL(
        &urusUartA,  /* uartA */
        nullptr,  /* uartB */
        nullptr,  /* uartC */
        nullptr,  /* uartD */
        nullptr,  /* uartE */
        nullptr,  /* uartF */
        nullptr,
        nullptr, /* spi */
        nullptr, /* analogin */
        nullptr, /* storage */
        &urusUartA, /* console */
        nullptr, /* gpio */
        nullptr,  /* rcinput */
        nullptr, /* rcoutput */
        &urusScheduler, /* scheduler */
        nullptr, /* util */
        nullptr) /* onboard optical flow */
{
    scheduler = NSCORE_URUS::get_scheduler();
    uartA = NSCORE_URUS::get_uartDriver();
    console = uartA;
}

void HAL_URUS::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    assert(callbacks);

    _urus_core.init_core();

    scheduler->init();
    uartA->begin(115200);

    callbacks->setup();

    for (;;) {
        callbacks->loop();
    }
}

const AP_HAL::HAL& AP_HAL::get_HAL() {
    static const HAL_URUS hal;
    return hal;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_URUS
