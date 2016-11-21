#pragma once

#include <AP_HAL/AP_HAL.h>

namespace Linux {

#define MAX_ZYNQ_PWMS            8	/* number of pwm channels */

class RCOutput_ZYNQ : public AP_HAL::RCOutput {
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
    static const int TICK_PER_US=100;
    static const int TICK_PER_S=100000000;

    // Period|Hi 32 bits each
    struct s_period_hi {
        uint32_t period;
        uint32_t hi;
    };
    struct pwm_cmd {
        struct s_period_hi periodhi[MAX_ZYNQ_PWMS];
    };
    volatile struct pwm_cmd *sharedMem_cmd;

    uint16_t pending[MAX_ZYNQ_PWMS];
    bool corked;
    uint32_t pending_mask;
};

}
