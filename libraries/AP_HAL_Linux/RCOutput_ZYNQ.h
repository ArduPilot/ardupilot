#pragma once

#include <AP_HAL/AP_HAL.h>

namespace Linux {

#define MAX_ZYNQ_PWMS            8	/* number of pwm channels */

class RCOutput_ZYNQ : public AP_HAL::RCOutput {
public:
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    void     cork(void) override;
    void     push(void) override;

private:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ
    static const int TICK_PER_US=50;
    static const int TICK_PER_S=50000000;
#else
    static const int TICK_PER_US=100;
    static const int TICK_PER_S=100000000;
#endif

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
