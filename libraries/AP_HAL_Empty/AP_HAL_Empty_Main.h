

#ifndef __AP_HAL_EMPTY_MAIN_H__
#define __AP_HAL_EMPTY_MAIN_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
#define AP_HAL_MAIN() extern "C" {\
    int main (void) {\
        AP_HAL::HAL::FunCallbacks callbacks(setup, loop); \
        hal.run(0, NULL, &callbacks); \
        return 0; \
    }\
    }
#endif // HAL_BOARD_EMPTY

#endif // __AP_HAL_EMPTY_MAIN_H__
