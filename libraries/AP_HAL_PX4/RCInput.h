#pragma once

#include "AP_HAL_PX4.h"
#include <drivers/drv_rc_input.h>
#include <systemlib/perf_counter.h>
#include <pthread.h>
#include <AP_Radio/AP_Radio.h>


#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18u
#endif

class PX4::PX4RCInput : public AP_HAL::RCInput {
public:
    void init() override;
    bool new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t read(uint16_t* periods, uint8_t len) override;

    int16_t get_rssi(void) override {
        return _rssi;
    }
        
    void _timer_tick(void);

    bool rc_bind(int dsmMode) override;

private:
    /* override state */
    struct rc_input_values _rcin;
    int _rc_sub;
    uint64_t _last_read;
    perf_counter_t _perf_rcin;
    pthread_mutex_t rcin_mutex;
    int16_t _rssi = -1;

    uint8_t last_input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;
    const char *input_source_name(uint8_t id) const;

#if HAL_RCINPUT_WITH_AP_RADIO
    AP_Radio *radio;
    uint32_t last_radio_us;
#endif
};
