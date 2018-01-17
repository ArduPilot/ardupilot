/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once

#include "AP_HAL_ChibiOS.h"
#include "ch.h"
#include "hal.h"

class ChibiOS::RCOutput : public AP_HAL::RCOutput {
public:
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
    uint16_t read_last_sent(uint8_t ch) override;
    void     read_last_sent(uint16_t* period_us, uint8_t len) override;
    void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override {
        _esc_pwm_min = min_pwm;
        _esc_pwm_max = max_pwm;
    }
    void set_output_mode(enum output_mode mode) override;

    void     cork(void) override;
    void     push(void) override;

    /*
      force the safety switch on, disabling PWM output from the IO board
     */
    bool force_safety_on(void) override;

    /*
      force the safety switch off, enabling PWM output from the IO board
     */
    void force_safety_off(void) override;

    bool enable_px4io_sbus_out(uint16_t rate_hz) override;

    /*
      set default update rate
     */
    void set_default_rate(uint16_t rate_hz) override;

    /*
      timer push (for oneshot min rate)
     */
    void timer_tick(void) override;
    
private:
    struct pwm_group {
        // only advanced timers can do high clocks needed for more than 400Hz
        bool advanced_timer;
        uint8_t chan[4]; // chan number, zero based, 255 for disabled
        PWMConfig pwm_cfg;
        PWMDriver* pwm_drv;
    };
    enum output_mode _output_mode = MODE_PWM_NORMAL;

    static pwm_group pwm_group_list[];
    uint16_t _esc_pwm_min;
    uint16_t _esc_pwm_max;

    // offset of first local channel
    uint8_t chan_offset;

    // total number of channels
    uint8_t total_channels;
    
    // last sent values are for all channels
    uint16_t last_sent[16];
    
    // these values are for the local channels. Non-local channels are handled by IOMCU
    uint32_t en_mask;
    uint16_t period[16];
    uint8_t num_channels;
    bool corked;
    // mask of channels that are running in high speed
    uint16_t fast_channel_mask;

    // min time to trigger next pulse to prevent overlap
    uint64_t min_pulse_trigger_us;

    // mutex for oneshot triggering
    mutex_t trigger_mutex;

    // which output groups need triggering
    uint8_t trigger_groups;

    // widest pulse for oneshot triggering
    uint16_t trigger_widest_pulse;
    
    // push out values to local PWM
    void push_local(void);

    // trigger oneshot pulses
    void trigger_oneshot(void);
};
