
#ifndef __AP_HAL_STORAGE_H__
#define __AP_HAL_STORAGE_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::Storage {
public:
    Storage() {}
    virtual void init(int machtnicht) = 0;
};

#endif // __AP_HAL_STORAGE_H__

