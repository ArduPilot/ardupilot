
#ifndef __AP_HAL_PX4_RCINPUT_H__
#define __AP_HAL_PX4_RCINPUT_H__

#include <AP_HAL_PX4.h>
#include <drivers/drv_rc_input.h>
#include <systemlib/perf_counter.h>

class PX4::PX4RCInput : public AP_HAL::RCInput {
public:
    void init(void* machtnichts);
    override uint8_t  valid_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    void _timer_tick(void);

protected:
    virtual void set_overrides_valid() { _override_valid = true; }

private:
    /* override state */
    uint16_t _override[RC_INPUT_MAX_CHANNELS];
    struct rc_input_values _rcin;
    int _rc_sub;
    uint64_t _last_read;
    uint64_t _last_input;
    bool _override_valid;
    perf_counter_t _perf_rcin;
};

#endif // __AP_HAL_PX4_RCINPUT_H__
