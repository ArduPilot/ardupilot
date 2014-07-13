

#ifndef __AP_HAL_LINUX_MAIN_H__
#define __AP_HAL_LINUX_MAIN_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_ERLE 
#define AP_HAL_MAIN() extern "C" {\
int main (int argc, char * const argv[]) {        \
	hal.init(argc, argv);			\
        setup();\
        hal.scheduler->system_initialized(); \
        for(;;) { loop(); \
            hal.scheduler->delay_microseconds(500); }    \
        return 0;\
    }\
    }
#endif // HAL_BOARD_LINUX || HAL_BOARD_ERLE

#endif // __AP_HAL_LINUX_MAIN_H__
