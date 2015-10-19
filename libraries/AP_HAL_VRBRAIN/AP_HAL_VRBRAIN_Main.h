#ifndef __AP_HAL_VRBRAIN_MAIN_H__
#define __AP_HAL_VRBRAIN_MAIN_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#define AP_HAL_MAIN() \
    extern "C" __EXPORT int SKETCH_MAIN(int argc, char * const argv[]); \
    int SKETCH_MAIN(int argc, char * const argv[]) {	\
        AP_HAL::HAL::FunCallbacks callbacks(setup, loop); \
        hal.run(argc, argv, &callbacks); \
	return OK; \
    }

#endif
#endif // __AP_HAL_VRBRAIN_MAIN_H__
