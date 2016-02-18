#pragma once

#include "AP_HAL_Linux.h"
#define RCOUT_ZYNQ_PWM_BASE	 0x43c00000	//FIXME hardcoding is the devil's work
#define MAX_ZYNQ_PWMS            8	/* number of pwm channels */
#define PWM_CMD_CONFIG	         0	/* full configuration in one go */
#define PWM_CMD_ENABLE	         1	/* enable a pwm */
#define PWM_CMD_DISABLE	         2	/* disable a pwm */
#define PWM_CMD_MODIFY	         3	/* modify a pwm */
#define PWM_CMD_SET	         4	/* set a pwm output explicitly */
#define PWM_CMD_CLR	         5	/* clr a pwm output explicitly */
#define PWM_CMD_TEST	         6	/* various crap */


class Linux::RCOutput_ZYNQ : public AP_HAL::RCOutput {
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

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
};
