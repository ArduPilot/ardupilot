
#ifndef __AP_HAL_VRBRAIN_RCINPUT_H__
#define __AP_HAL_VRBRAIN_RCINPUT_H__

#include "AP_HAL_VRBRAIN.h"
#include <drivers/drv_rc_input.h>
#include <systemlib/perf_counter.h>
#include <pthread.h>

class VRBRAIN::VRBRAINRCInput : public AP_HAL::RCInput {
public:
    void init();
    bool new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

    void _timer_tick(void);

private:
    /* override state */
    uint16_t _override[RC_INPUT_MAX_CHANNELS];
    struct rc_input_values _rcin;
    int _rc_sub;
    uint64_t _last_read;
    bool _override_valid;
    perf_counter_t _perf_rcin;
    pthread_mutex_t rcin_mutex;
};

#endif // __AP_HAL_VRBRAIN_RCINPUT_H__
