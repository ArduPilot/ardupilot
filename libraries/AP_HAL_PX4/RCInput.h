
#ifndef __AP_HAL_PX4_RCINPUT_H__
#define __AP_HAL_PX4_RCINPUT_H__

#include <AP_HAL_PX4.h>

#define PX4_NUM_RCINPUT_CHANNELS 8

class PX4::PX4RCInput : public AP_HAL::RCInput {
public:
    void init(void* machtnichts);
    uint8_t  valid();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

private:
    /* override state */
    uint16_t _override[PX4_NUM_RCINPUT_CHANNELS];
    uint64_t _last_read;
};

#endif // __AP_HAL_PX4_RCINPUT_H__
