
#ifndef __AP_HAL_SMACCM_RCINPUT_H__
#define __AP_HAL_SMACCM_RCINPUT_H__

#include <AP_HAL_SMACCM.h>

#define SMACCM_RCINPUT_CHANNELS 8

class SMACCM::SMACCMRCInput : public AP_HAL::RCInput {
public:
    SMACCMRCInput();
    void init(void *unused);
    uint8_t  valid_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

private:
    uint16_t _override[SMACCM_RCINPUT_CHANNELS];
};

#endif // __AP_HAL_SMACCM_RCINPUT_H__
