
#ifndef __AP_HAL_RC_INPUT_H__
#define __AP_HAL_RC_INPUT_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::RCInput {
public:
    RCInput() {}
    virtual void init(int machtnicht) = 0;
};

#endif // __AP_HAL_RC_INPUT_H__

