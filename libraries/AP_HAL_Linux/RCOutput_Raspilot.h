#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/SPIDevice.h>

namespace Linux {

class RCOutput_Raspilot : public AP_HAL::RCOutput {
public:
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
    void     cork(void) override;
    void     push(void) override;

private:
    void reset();
    void _update(void);

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    uint32_t _last_update_timestamp;
    uint16_t _frequency;
    uint16_t _new_frequency;
    uint16_t _period_us[8];
    bool _corked;
};

}
