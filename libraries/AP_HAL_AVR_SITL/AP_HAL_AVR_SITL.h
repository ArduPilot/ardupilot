
#ifndef __AP_HAL_AVR_SITL_H__
#define __AP_HAL_AVR_SITL_H__

#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
#define AP_HAL_MAIN() extern "C" {\
    int main (int argc, const char *argv[]) {	\
	hal.init(argc, argv);		\
        setup();\
        for(;;) loop();\
        return 0;\
    }\
    }
#endif

#endif // __AP_HAL_AVR_SITL_H__

