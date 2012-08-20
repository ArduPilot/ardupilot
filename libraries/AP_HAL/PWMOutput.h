
#ifndef __AP_HAL_PWM_OUTPUT_H__
#define __AP_HAL_PWM_OUTPUT_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::PWMOutput {
public:
    PWMOutput() {}
    virtual void init(int machtnicht) = 0;
};

#endif // __AP_HAL_PWM_OUTPUT_H__

