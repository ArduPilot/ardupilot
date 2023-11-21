#include <assert.h>

#include "HAL.h"

namespace AP_HAL {

HAL::FunCallbacks::FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void))
    : _setup(setup_fun)
    , _loop(loop_fun)
{
}

}

// access serial ports using SERIALn numbering
AP_HAL::UARTDriver* AP_HAL::HAL::serial(uint8_t sernum) const
{
    if (sernum >= ARRAY_SIZE(serial_array)) {
        return nullptr;
    }
    return serial_array[sernum];
}
