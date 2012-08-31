
#ifndef __AP_HAL_DATAFLASH_H__
#define __AP_HAL_DATAFLASH_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::Dataflash {
public:
    virtual void init(int machtnicht) = 0;
};

#endif // __AP_HAL_DATAFLASH_H__

