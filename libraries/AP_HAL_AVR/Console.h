
#ifndef __AP_HAL_AVR_CONSOLE_H__
#define __AP_HAL_AVR_CONSOLE_H__

#include <AP_HAL.h>
#include "UARTDriver.h"

class AP_HAL::AVRUARTConsole : public AP_HAL::Console {
public:
    AVRUARTConsole( AVRUARTDriver* driver ) : _driver(driver), _init(0) {}
    void init(int machtnicht) { _init = 1; }
private:
    const AVRUARTDriver* _driver;
    int _init;
};


#endif // __AP_HAL_AVR_CONSOLE_H__

