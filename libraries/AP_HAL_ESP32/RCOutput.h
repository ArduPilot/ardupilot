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

    // queue a DShot special command (arm/beep/spin-direction/3D/save) on a single
    // DShot channel or all of them; the rcout task transmits it repeat_count times.
    void send_dshot_command(uint8_t command, uint8_t chan, uint32_t command_timeout_ms,
                            uint16_t repeat_count, bool priority) override;

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

        // RMT backend handles (rmt_channel_handle_t / rmt_encoder_handle_t), kept
        // as void* so the new RMT driver headers stay out of RCOutput.h: they
        // clash with the legacy RMT driver that RCInput (RmtSigReader) uses when
        // both are pulled into one translation unit. Used only in a DShot mode
        // (nullptr otherwise). The S3 has 4 RMT TX channels -> max 4 DShot outputs.
        void *rmt_chan;
        void *rmt_encoder;
        uint8_t dshot_buf[2]; // persistent TX buffer for the async RMT frame

        // Pending DShot special command (arm/beep/spin-direction/3D/save). While
        // dshot_command_repeat > 0 the rcout task transmits dshot_command (value
        // 0..47, telemetry bit set, as commands require) instead of the throttle,
        // decrementing once per frame; 0 means "send throttle normally". Updated
        // lock-free from the main thread (same approach as `value`): the writer
        // sets dshot_command before dshot_command_repeat so the task, which gates
        // on the count, never pairs a fresh count with a stale command.
        uint16_t dshot_command;
        uint16_t dshot_command_repeat;
    };

    uint32_t fast_channel_mask;

    uint32_t constrain_freq(pwm_group &group);

    void set_group_mode(pwm_group &group);
    // DShot uses the RMT peripheral (MCPWM cannot generate the digital frame).
    // Set up / tear down the RMT backend for a group switched to a DShot mode.
    void set_group_mode_dshot(pwm_group &group);
    void dshot_free_chan(pwm_chan &ch); // release a channel's RMT resources
    // build a 16-bit DShot frame (value<<1 | telem, then 4-bit CRC)
    static uint16_t create_dshot_packet(uint16_t value, bool telem_request);
    // encode + asynchronously transmit one DShot frame on a channel
    void dshot_send_chan(pwm_chan &ch, uint16_t value, bool telem_request);
    // scale an ArduPilot PWM value (us) to a DShot throttle command (0, 48..2047)
    static uint16_t dshot_throttle_from_pwm(uint16_t period_us);
    // periodic task that re-transmits DShot frames at the DShot rate
    void start_dshot_task();
    void dshot_task();
    static void dshot_task_entry(void *arg);

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
    bool _dshot_task_started = false; // periodic DShot transmit task running?

};

}
