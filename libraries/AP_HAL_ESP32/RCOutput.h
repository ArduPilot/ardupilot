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
 * Code by Charles "Silvanosky" Villard, David "Buzz" Bussenschutt,
 * Andrey "ARg" Romanov, and Thomas "tpw_rules" Watson'
 *
 */

#pragma once


#include <AP_HAL/RCOutput.h>
#include "HAL_ESP32_Namespace.h"
#include <AP_HAL/Util.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

namespace ESP32
{

class RCOutput : public AP_HAL::RCOutput
{
public:
    RCOutput() {};

    ~RCOutput() {};

    static RCOutput *from(AP_HAL::RCOutput *rcoutput)
    {
        return static_cast<RCOutput *>(rcoutput);
    }

    void init() override;

    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t chan) override;

    void enable_ch(uint8_t chan) override;
    void disable_ch(uint8_t chan) override;

    void write(uint8_t chan, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t* period_us, uint8_t len) override;

    void set_output_mode(uint32_t mask, const enum output_mode mode) override;

    void cork() override;
    void push() override;

    void set_default_rate(uint16_t rate_hz) override;

    /*
       force the safety switch on, disabling PWM output from the IO board
       */
    bool force_safety_on() override;

    /*
       force the safety switch off, enabling PWM output from the IO board
       */
    void force_safety_off() override;

    /*
       set PWM to send to a set of channels when the safety switch is
       in the safe state
       */
    void set_safety_pwm(uint32_t chmask, uint16_t period_us) ;

    /*
       get safety switch state, used by Util.cpp
       */
    AP_HAL::Util::safety_state _safety_switch_state();

    /*
       set PWM to send to a set of channels if the FMU firmware dies
       */
    void set_failsafe_pwm(uint32_t chmask, uint16_t period_us) override;

    /*
       set safety mask for IOMCU
       */
    void set_safety_mask(uint32_t mask)
    {
        safety_mask = mask;
    }


    void timer_tick() override;


private:

    struct pwm_group {
        // SDK objects for the group
        uint8_t mcpwm_group_id;
        mcpwm_timer_handle_t h_timer;
        mcpwm_oper_handle_t h_oper;

        uint32_t rc_frequency; // frequency in Hz
        uint32_t ch_mask; // mask of channels in this group
        enum output_mode current_mode; // output mode (none, normal, brushed)
    };

    struct pwm_chan {
        // SDK objects for the channel
        mcpwm_cmpr_handle_t h_cmpr;
        mcpwm_gen_handle_t h_gen;
        pwm_group *group; // associated group

        uint8_t gpio_num; // associated GPIO number (always defined)
        int value; // output value in microseconds
    };

    uint32_t fast_channel_mask;

    uint32_t constrain_freq(pwm_group &group);

    void set_group_mode(pwm_group &group);

    void write_int(uint8_t chan, uint16_t period_us);

    static pwm_group pwm_group_list[];
    static pwm_chan pwm_chan_list[];

    bool _corked;
    uint32_t _pending_mask;
    uint16_t _pending[12];
    uint16_t safe_pwm[12]; // pwm to use when safety is on

    // safety switch state
    AP_HAL::Util::safety_state safety_state;
    uint32_t safety_update_ms;
    uint8_t led_counter;
    int8_t safety_button_counter;
    uint8_t safety_press_count; // 0.1s units

    // mask of channels to allow when safety on
    uint32_t safety_mask;

    // update safety switch and LED
    void safety_update(void);


    bool _initialized;

};

}
