#pragma once

#include "AP_Baro_Backend.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ULTRA96
#include <AP_Math/vectorN.h>

class AP_Baro_ULTRA96 : public AP_Baro_Backend {
public:
    AP_Baro_ULTRA96(AP_Baro &);

    void update(void);

private:
	//MY var
    uint8_t _instance;
	volatile int32_t *data_pointer;
	//End of my VAR

    float _recent_temp;
    float _recent_press;

};
#endif  // CONFIG_HAL_BOARD
