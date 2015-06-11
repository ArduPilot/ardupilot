

#ifndef __AP_HAL_LINUX_MAIN_H__
#define __AP_HAL_LINUX_MAIN_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#define AP_HAL_MAIN() extern "C" {\
int main (int argc, char * const argv[]) {        \
	hal.init(argc, argv);			\
        hal.scheduler->system_initialized(); \
        setup();\
        for(;;) loop();\
        return 0;\
    }\
    }
#endif // HAL_BOARD_LINUX

#endif // __AP_HAL_LINUX_MAIN_H__
