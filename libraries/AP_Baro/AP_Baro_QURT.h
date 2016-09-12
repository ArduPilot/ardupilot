/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Baro_Backend.h"
#include <AP_HAL_QURT/Semaphores.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
extern "C" {
#include "bmp280_api.h"
}

class AP_Baro_QURT : public AP_Baro_Backend
{
public:
    // Constructor
    AP_Baro_QURT(AP_Baro &baro);

    /* AP_Baro public interface: */
    void update() override;

private:
    void timer(void);

    uint32_t instance;
    uint32_t handle;
    uint32_t last_timer_ms;
    uint64_t last_counter;
    
    uint32_t count;
    float temp_sum;
    float press_sum;
    QURT::Semaphore lock;
};
#endif // CONFIG_HAL_BOARD
