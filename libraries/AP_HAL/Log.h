
#ifndef __AP_HAL_LOG_H__
#define __AP_HAL_LOG_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::Log{
public:
    Log() {}
    virtual void init(int machtnicht) = 0;
};

#endif // __AP_HAL_LOG_H__

