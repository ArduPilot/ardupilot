// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.
#pragma once

#include "AP_HAL_Linux.h"
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
#define RCOUT_PRUSS_RAM_BASE 0x4a300000
#define RCOUT_PRUSS_CTRL_BASE 0x4a322000
#define RCOUT_PRUSS_IRAM_BASE 0x4a334000
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET2
#define RCOUT_PRUSS_RAM_BASE  0x30040000
#define RCOUT_PRUSS_CTRL_BASE 0x30062000
#define RCOUT_PRUSS_IRAM_BASE 0x30074000
#else
#define RCOUT_PRUSS_RAM_BASE 0x4a302000
#define RCOUT_PRUSS_CTRL_BASE 0x4a324000
#define RCOUT_PRUSS_IRAM_BASE 0x4a338000
#endif
#define PWM_CHAN_COUNT 12

namespace Linux {

class RCOutput_AioPRU : public AP_HAL::RCOutput {
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
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET2
   static const uint32_t TICK_PER_US = 250;
   static const uint32_t TICK_PER_S = 250000000;
#else
   static const uint32_t TICK_PER_US = 200;
   static const uint32_t TICK_PER_S = 200000000;
#endif
   
   struct pwm {
      volatile uint32_t channelenable;
      struct {
        volatile uint32_t time_high;
        volatile uint32_t time_t;
        } channel[PWM_CHAN_COUNT];
    };

    volatile struct pwm *pwm;
    uint16_t pending[PWM_CHAN_COUNT];
    uint32_t pending_mask;
    bool corked;
};

}
