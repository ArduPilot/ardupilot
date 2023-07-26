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
 * Code by Charles "Silvanosky" Villard and David "Buzz" Bussenschutt
 *
 */

#pragma once


#include <AP_HAL/RCOutput.h>
#include "HAL_ESP32_Namespace.h"
#include "driver/mcpwm.h"
#define HAL_PARAM_DEFAULTS_PATH nullptr
#include <AP_HAL/Util.h>

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
    struct pwm_out {
        int gpio_num;
        mcpwm_unit_t unit_num;
        mcpwm_timer_t timer_num;
        mcpwm_io_signals_t io_signal;
        mcpwm_operator_t op;
        uint8_t chan;
    };


    void write_int(uint8_t chan, uint16_t period_us);

    static pwm_out pwm_group_list[];

    bool _corked;
    uint16_t _pending[12]; //Max channel with 2 unit MCPWM
    uint32_t _pending_mask;

    uint16_t safe_pwm[16]; // pwm to use when safety is on

    uint16_t _max_channels;

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
