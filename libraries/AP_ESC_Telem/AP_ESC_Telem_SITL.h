#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_ESC_Telem_Backend.h"
#include <SITL/SITL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

class AP_ESC_Telem_SITL
#if HAL_WITH_ESC_TELEM
 : public AP_ESC_Telem_Backend
#endif
{

public:
    AP_ESC_Telem_SITL();

    void update();

protected:

private:
};

#endif
