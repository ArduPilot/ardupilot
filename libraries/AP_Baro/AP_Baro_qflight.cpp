/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_QFLIGHT
#include <unistd.h>
#include <stdio.h>
#include "AP_Baro_qflight.h"
#include <AP_HAL_Linux/qflight/qflight_util.h>
#include <AP_HAL_Linux/qflight/qflight_dsp.h>

extern const AP_HAL::HAL& hal;

/*
  constructor - opens the QFLIGHT drivers
 */
AP_Baro_QFLIGHT::AP_Baro_QFLIGHT(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    instance = _frontend.register_sensor();
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Baro_QFLIGHT::timer_update, void));
    hal.scheduler->delay(100);
}

// Read the sensor
void AP_Baro_QFLIGHT::timer_update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_check_ms < 25) {
        return;
    }
    last_check_ms = now;
    
    if (barobuf == nullptr) {
        barobuf = QFLIGHT_RPC_ALLOCATE(DSPBuffer::BARO);
        if (barobuf == nullptr) {
            AP_HAL::panic("Unable to allocate baro buffer");
        }
    }
    int ret = qflight_get_baro_data((uint8_t *)barobuf, sizeof(*barobuf));
    if (ret != 0) {
        return;
    }

    for (uint16_t i=0; i<barobuf->num_samples; i++) {
        DSPBuffer::BARO::BUF &b = barobuf->buf[i];
        pressure_sum += b.pressure_pa;
        temperature_sum += b.temperature_C;
        sum_count++;
    }
}

// Read the sensor
void AP_Baro_QFLIGHT::update(void)
{
    if (sum_count > 0) {
        hal.scheduler->suspend_timer_procs();
        _copy_to_frontend(instance,
                          pressure_sum/sum_count,
                          temperature_sum/sum_count);
        sum_count = 0;
        pressure_sum = 0;
        temperature_sum = 0;
        hal.scheduler->resume_timer_procs();
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE

