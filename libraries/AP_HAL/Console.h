
#ifndef __AP_HAL_CONSOLE_H__
#define __AP_HAL_CONSOLE_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::Console {
public:
    Console() {}
    virtual void init(int machtnicht) = 0;
};

#endif // __AP_HAL_CONSOLE_H__

