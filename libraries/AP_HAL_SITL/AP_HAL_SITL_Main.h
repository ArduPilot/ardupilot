
#ifndef __AP_HAL_SITL_MAIN_H__
#define __AP_HAL_SITL_MAIN_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define AP_HAL_MAIN() extern "C" {\
    int main (int argc, char * const argv[]) { \
        AP_HAL::HAL::FunCallbacks callbacks(setup, loop); \
        hal.run(argc, argv, &callbacks); \
        return 0; \
    }\
    }
#endif

#endif // __AP_HAL_SITL_MAIN_H__
