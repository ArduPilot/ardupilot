
#ifndef __AP_HAL_SMACCM_RCINPUT_H__
#define __AP_HAL_SMACCM_RCINPUT_H__

#include <AP_HAL_SMACCM.h>
#include <hwf4/timer.h>

class SMACCM::SMACCMRCInput : public AP_HAL::RCInput {
public:
    SMACCMRCInput();
    void init(void *unused);
    uint8_t  valid();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

    bool set_overrides(int16_t *overrides, uint8_t len);
    bool set_override(uint8_t channel, int16_t override);
    void clear_overrides();

private:
    uint16_t _override[PPM_MAX_CHANNELS];
};

#endif // __AP_HAL_SMACCM_RCINPUT_H__
