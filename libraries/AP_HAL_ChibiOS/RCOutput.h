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
#include "shared_dma.h"
#include "ch.h"
#include "hal.h"

#if HAL_USE_PWM == TRUE

#if !STM32_DMA_ADVANCED
#define DISABLE_DSHOT
#endif

class ChibiOS::RCOutput : public AP_HAL::RCOutput {
public:
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    uint16_t read_last_sent(uint8_t ch) override;
    void     read_last_sent(uint16_t* period_us, uint8_t len) override;
    void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override {
        _esc_pwm_min = min_pwm;
        _esc_pwm_max = max_pwm;
    }
    bool     get_esc_scaling(uint16_t &min_pwm, uint16_t &max_pwm) override {
        min_pwm = _esc_pwm_min;
        max_pwm = _esc_pwm_max;
        return true;
    }
    void set_output_mode(uint16_t mask, const enum output_mode mode) override;
    bool get_output_mode_banner(char banner_msg[], uint8_t banner_msg_len) const override;

    float scale_esc_to_unity(uint16_t pwm) override {
        return 2.0 * ((float) pwm - _esc_pwm_min) / (_esc_pwm_max - _esc_pwm_min) - 1.0;
    }

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

    /*
      set PWM to send to a set of channels when the safety switch is
      in the safe state
     */
    void set_safety_pwm(uint32_t chmask, uint16_t period_us) override;

    bool enable_px4io_sbus_out(uint16_t rate_hz) override;

    /*
      set default update rate
     */
    void set_default_rate(uint16_t rate_hz) override;

    /*
      timer push (for oneshot min rate)
     */
    void timer_tick(void) override;

    /*
      setup for serial output to a set of ESCs, using the given
      baudrate. Assumes 1 start bit, 1 stop bit, LSB first and 8
      databits. This is used for ESC configuration and firmware
      flashing
     */
    bool setup_serial_output(uint16_t chan_mask, ByteBuffer *buffer, uint32_t baudrate);

    /*
      setup for serial output to an ESC using the given
      baudrate. Assumes 1 start bit, 1 stop bit, LSB first and 8
      databits. This is used for passthrough ESC configuration and
      firmware flashing

      While serial output is active normal output to this channel is
      suspended. Output to some other channels (such as those in the
      same channel timer group) may also be stopped, depending on the
      implementation
     */
    bool serial_setup_output(uint8_t chan, uint32_t baudrate, uint16_t motor_mask) override;

    /*
      write a set of bytes to an ESC, using settings from
      serial_setup_output. This is a blocking call
     */
    bool serial_write_bytes(const uint8_t *bytes, uint16_t len) override;

    /*
      read a byte from a port, using serial parameters from serial_setup_output()
      return the number of bytes read
     */
    uint16_t serial_read_bytes(uint8_t *buf, uint16_t len) override;

    /*
      stop serial output. This restores the previous output mode for
      the channel and any other channels that were stopped by
      serial_setup_output()
     */
    void serial_end(void) override;

    /*
      enable telemetry request for a mask of channels. This is used
      with DShot to get telemetry feedback
     */
    void set_telem_request_mask(uint16_t mask) override { telem_request_mask = (mask >> chan_offset); }

    /*
      get safety switch state, used by Util.cpp
    */
    AP_HAL::Util::safety_state _safety_switch_state(void);

    /*
      set PWM to send to a set of channels if the FMU firmware dies
     */
    void set_failsafe_pwm(uint32_t chmask, uint16_t period_us) override;

    /*
      set safety mask for IOMCU
     */
    void set_safety_mask(uint16_t mask) { safety_mask = mask; }

    /*
     * mark the channels in chanmask as reversible. This is needed for some ESC types (such as DShot)
     * so that output scaling can be performed correctly. The chanmask passed is added (ORed) into
     * any existing mask.
     */
    void set_reversible_mask(uint16_t chanmask) override {
        reversible_mask |= chanmask;
    }

    /*
      setup serial LED output for a given channel number, with
      the given max number of LEDs in the chain.
     */
    bool set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode = MODE_PWM_NONE, uint16_t clock_mask = 0) override;

    /*
      setup serial LED output data for a given output channel
      and LEDs number. LED -1 is all LEDs
     */
    void set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue) override;

    /*
      trigger send of serial LED data
     */
    void serial_led_send(const uint16_t chan) override;

private:
    struct pwm_group {
        // only advanced timers can do high clocks needed for more than 400Hz
        bool advanced_timer;
        uint8_t chan[4]; // chan number, zero based, 255 for disabled
        PWMConfig pwm_cfg;
        PWMDriver* pwm_drv;
        bool have_up_dma; // can we do DMAR outputs for DShot?
        uint8_t dma_up_stream_id;
        uint8_t dma_up_channel;
        uint8_t alt_functions[4];
        ioline_t pal_lines[4];

        // below this line is not initialised by hwdef.h
        enum output_mode current_mode;
        uint16_t frequency_hz;
        uint16_t ch_mask;
        const stm32_dma_stream_t *dma;
        Shared_DMA *dma_handle;
        uint32_t *dma_buffer;
        uint16_t dma_buffer_len;
        bool have_lock;
        bool pwm_started;
        uint32_t bit_width_mul;
        uint32_t rc_frequency;
        bool in_serial_dma;
        uint64_t last_dmar_send_us;
        virtual_timer_t dma_timeout;
        uint8_t serial_nleds;
        uint8_t clock_mask;
        bool serial_led_pending;
        bool prepared_send;

        // serial output
        struct {
            // expected time per bit
            uint32_t bit_time_us;

            // channel to output to within group (0 to 3)
            uint8_t chan;

            // thread waiting for byte to be written
            thread_t *waiter;
        } serial;
    };

    /*
      structure for IRQ handler for soft-serial input
     */
    static struct irq_state {
        // ioline for port being read
        ioline_t line;

        // time the current byte started
        systime_t byte_start_tick;

        // number of bits we have read in this byte
        uint8_t nbits;

        // bitmask of bits so far (includes start and stop bits)
        uint16_t bitmask;

        // value of completed byte (includes start and stop bits)
        uint16_t byteval;

        // expected time per bit in system ticks
        systime_t bit_time_tick;

        // the bit value of the last bit received
        uint8_t last_bit;

        // thread waiting for byte to be read
        thread_t *waiter;

        // timeout for byte read
        virtual_timer_t serial_timeout;
        bool timed_out;
    } irq;


    // the group being used for serial output
    struct pwm_group *serial_group;
    thread_t *serial_thread;
    tprio_t serial_priority;

    static pwm_group pwm_group_list[];
    uint16_t _esc_pwm_min;
    uint16_t _esc_pwm_max;

    // offset of first local channel
    uint8_t chan_offset;

    // total number of channels on FMU
    uint8_t num_fmu_channels;

    // number of active fmu channels
    uint8_t active_fmu_channels;

    static const uint8_t max_channels = 16;

    // last sent values are for all channels
    uint16_t last_sent[max_channels];

    // these values are for the local channels. Non-local channels are handled by IOMCU
    uint32_t en_mask;
    uint16_t period[max_channels];
    uint16_t safe_pwm[max_channels]; // pwm to use when safety is on
    bool corked;
    // mask of channels that are running in high speed
    uint16_t fast_channel_mask;
    uint16_t io_fast_channel_mask;
    uint16_t reversible_mask;

    // min time to trigger next pulse to prevent overlap
    uint64_t min_pulse_trigger_us;

    // mutex for oneshot triggering
    mutex_t trigger_mutex;

    // which output groups need triggering
    uint8_t trigger_groupmask;

    // widest pulse for oneshot triggering
    uint16_t trigger_widest_pulse;

    // iomcu output mode (pwm, oneshot or oneshot125)
    enum output_mode iomcu_mode = MODE_PWM_NORMAL;

    // find a channel group given a channel number
    struct pwm_group *find_chan(uint8_t chan, uint8_t &group_idx);

    // push out values to local PWM
    void push_local(void);

    // trigger group pulses
    void trigger_groups(void);

    // setup output frequency for a group
    void set_freq_group(pwm_group &group);

    // safety switch state
    AP_HAL::Util::safety_state safety_state;
    uint32_t safety_update_ms;
    uint8_t led_counter;
    int8_t safety_button_counter;
    uint8_t safety_press_count; // 0.1s units

    // mask of channels to allow when safety on
    uint16_t safety_mask;

    // update safety switch and LED
    void safety_update(void);

    /*
      DShot handling
     */
    // the pre-bit is needed with TIM5, or we can get some corrupt frames
    const uint8_t dshot_pre = 1;
    const uint8_t dshot_post = 2;
    const uint16_t dshot_bit_length = 16 + dshot_pre + dshot_post;
    const uint16_t dshot_buffer_length = dshot_bit_length*4*sizeof(uint32_t);
    static const uint16_t dshot_min_gap_us = 100;
    uint32_t dshot_pulse_time_us;
    uint16_t telem_request_mask;

    /*
      Serial lED handling. Max of 32 LEDs uses max 12k of memory per group
      return true if send was successful
    */
    bool serial_led_send(pwm_group &group);
    bool serial_led_pending;

    void dma_allocate(Shared_DMA *ctx);
    void dma_deallocate(Shared_DMA *ctx);
    uint16_t create_dshot_packet(const uint16_t value, bool telem_request);
    void fill_DMA_buffer_dshot(uint32_t *buffer, uint8_t stride, uint16_t packet, uint16_t clockmul);
    void dshot_send(pwm_group &group, bool blocking);
    static void dma_irq_callback(void *p, uint32_t flags);
    static void dma_unlock(void *p);
    bool mode_requires_dma(enum output_mode mode) const;
    bool setup_group_DMA(pwm_group &group, uint32_t bitrate, uint32_t bit_width, bool active_high, const uint16_t buffer_length, bool choose_high);
    void send_pulses_DMAR(pwm_group &group, uint32_t buffer_length);
    void set_group_mode(pwm_group &group);
    bool is_dshot_protocol(const enum output_mode mode) const;
    uint32_t protocol_bitrate(const enum output_mode mode) const;

    /*
      setup neopixel (WS2812B) output data for a given output channel
     */
    void _set_neopixel_rgb_data(pwm_group *grp, uint8_t idx, uint8_t led, uint8_t red, uint8_t green, uint8_t blue);

    /*
      setup ProfiLED output data for a given output channel
     */
    void _set_profiled_rgb_data(pwm_group *grp, uint8_t idx, uint8_t led, uint8_t red, uint8_t green, uint8_t blue);
    void _set_profiled_clock(pwm_group *grp, uint8_t idx, uint8_t led);
    void _set_profiled_blank_frame(pwm_group *grp, uint8_t idx, uint8_t led);

    // serial output support
    static const eventmask_t serial_event_mask = EVENT_MASK(1);
    bool serial_write_byte(uint8_t b);
    bool serial_read_byte(uint8_t &b);
    void fill_DMA_buffer_byte(uint32_t *buffer, uint8_t stride, uint8_t b , uint32_t bitval);
    static void serial_bit_irq(void);
    static void serial_byte_timeout(void *ctx);

};

#endif // HAL_USE_PWM
