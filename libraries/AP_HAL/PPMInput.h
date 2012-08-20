
#ifndef __AP_HAL_PPM_INPUT_H__
#define __AP_HAL_PPM_INPUT_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::PPMInput {
public:
    PPMInput() {}
    virtual void init(int machtnicht) = 0;
};

#endif // __AP_HAL_PPM_INPUT_H__

