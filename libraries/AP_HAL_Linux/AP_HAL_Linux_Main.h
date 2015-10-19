

#ifndef __AP_HAL_LINUX_MAIN_H__
#define __AP_HAL_LINUX_MAIN_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#define AP_HAL_MAIN() extern "C" {\
int main (int argc, char * const argv[]) {        \
        AP_HAL::HAL::FunCallbacks callbacks(setup, loop); \
        hal.run(argc, argv, &callbacks); \
        return 0; \
    }\
    }
#endif // HAL_BOARD_LINUX

#endif // __AP_HAL_LINUX_MAIN_H__
