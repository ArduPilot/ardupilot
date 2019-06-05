#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ULTRA96
#include <AP_Math/vectorN.h>
#include <AP_Math/AP_Math.h>
#include <AP_Declination/AP_Declination.h>


class AP_Compass_ULTRA96 : public AP_Compass_Backend {
public:
    AP_Compass_ULTRA96();
	void read(void) ;

private:
    volatile int32_t *data_pointer;
	void init() ;
    void update(void) ;
    uint8_t _compass_instance;
};
#endif // CONFIG_HAL_BOARD
