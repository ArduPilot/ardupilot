#pragma once

#include "AP_HAL_Linux.h"
#define RCOUT_PRUSS_SHAREDRAM_BASE     0x4a310000
#define MAX_PWMS                 12
#define PWM_CMD_MAGIC            0xf00fbaaf
#define PWM_REPLY_MAGIC          0xbaaff00f
#define PWM_CMD_CONFIG	         0	/* full configuration in one go */
#define PWM_CMD_ENABLE	         1	/* enable a pwm */
#define PWM_CMD_DISABLE	         2	/* disable a pwm */
#define PWM_CMD_MODIFY	         3	/* modify a pwm */
#define PWM_CMD_SET	         4	/* set a pwm output explicitly */
#define PWM_CMD_CLR	         5	/* clr a pwm output explicitly */
#define PWM_CMD_TEST	         6	/* various crap */

namespace Linux {

class RCOutput_PRU : public AP_HAL::RCOutput {
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
    static const int TICK_PER_US=200;
    static const int TICK_PER_S=200000000;
    struct pwm_cmd {
        uint32_t magic;
        uint32_t enmask;     /* enable mask */
        uint32_t offmsk;     /* state when pwm is off */
        uint32_t periodhi[MAX_PWMS][2];
        uint32_t hilo_read[MAX_PWMS][2];
        uint32_t enmask_read;
    };
    volatile struct pwm_cmd *sharedMem_cmd;

    uint16_t pending[MAX_PWMS];
    bool corked;
    uint32_t pending_mask;    
};
}
