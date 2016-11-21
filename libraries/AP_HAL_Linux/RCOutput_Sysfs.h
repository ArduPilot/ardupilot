#pragma once

#include "AP_HAL_Linux.h"
#include "PWM_Sysfs.h"

namespace Linux {

class RCOutput_Sysfs : public AP_HAL::RCOutput {
public:
    RCOutput_Sysfs(uint8_t chip, uint8_t channel_base, uint8_t channel_count);
    ~RCOutput_Sysfs();

    static RCOutput_Sysfs *from(AP_HAL::RCOutput *rcoutput)
    {
        return static_cast<RCOutput_Sysfs *>(rcoutput);
    }

    void init();
    void set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void enable_ch(uint8_t ch);
    void disable_ch(uint8_t ch);
    void write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void read(uint16_t *period_us, uint8_t len);
    void cork(void) override;
    void push(void) override;

private:
    const uint8_t _chip;
    const uint8_t _channel_base;
    const uint8_t _channel_count;
    PWM_Sysfs_Base **_pwm_channels;

    // for handling cork()/push()
    bool _corked;
    uint16_t *_pending;
    uint32_t _pending_mask;
};

}
