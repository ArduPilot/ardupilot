
#ifndef __AP_HAL_AVR_H__
#define __AP_HAL_AVR_H__

#include <AP_HAL.h>

#include "HAL_AVR_APM1_Class.h"
#include "HAL_AVR_APM2_Class.h"

/**
 * This module exports AP_HAL instances only.
 * All internal drivers must conform to AP_HAL interfaces
 * and not expose implementation details.
 */

#define AP_HAL_MAIN() extern "C" {\
    int main (void) {\
        hal.init(NULL);\
        setup();\
        for(;;) loop();\
        return 0;\
    }\
    }


#endif // __AP_HAL_AVR_H__

