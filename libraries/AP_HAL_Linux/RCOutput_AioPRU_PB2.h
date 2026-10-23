#pragma once

#include "AP_HAL_Linux.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET2

#define RCOUT_PB2_PRUSS_RAM_BASE  0x30040000
#define RCOUT_PB2_PRUSS_CTRL_BASE 0x30062000
#define RCOUT_PB2_PRUSS_IRAM_BASE 0x30074000

#define RC_CHAN_COUNT 12

namespace Linux {

class RCOutput_AioPRU_PB2 : public AP_HAL::RCOutput {
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
    void     set_output_mode(uint32_t mask, const enum output_mode mode) override;
    void     timer_tick(void) override;

private:
    static const uint32_t TICK_PER_US = 250;
    static const uint32_t TICK_PER_S = 250000000;

    struct rcoutput {
        volatile uint32_t channelenable;
        volatile uint32_t failsafe; // Has to be kept set to prevent channels from being disabled.
        union {
            struct {
                volatile uint32_t time_high;
                volatile uint32_t time_t;
            } pwm_out[RC_CHAN_COUNT];
            struct {
                volatile uint32_t bitpos; // Position of bit currently read
                volatile uint32_t ch_frame; // Frame, only lower 16 bits used
            } dshot_out[RC_CHAN_COUNT];
        };
    };
    
    volatile struct rcoutput *rcoutput;
    uint32_t* iram;
    uint32_t* ctrl;
    uint16_t period[RC_CHAN_COUNT];
    uint32_t pending_mask;
    bool corked;
    enum output_mode _mode;
    
    void push_internal();
    bool is_dshot_mode(enum RCOutput::output_mode mode);
    uint16_t create_dshot_packet(const uint16_t value, bool telem_request);
};

}
#endif //HAL_BOARD_SUBTYPE_LINUX_POCKET2
