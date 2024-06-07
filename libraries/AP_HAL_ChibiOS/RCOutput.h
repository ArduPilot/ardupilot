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

#include <hal.h>
#include "AP_HAL_ChibiOS.h"
#include <AP_HAL/Semaphores.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>

#include "shared_dma.h"

#if HAL_USE_PWM == TRUE

#if defined(STM32F1)
#ifdef HAL_WITH_BIDIR_DSHOT
typedef uint16_t dmar_uint_t; // save memory to allow dshot on IOMCU
typedef int16_t dmar_int_t;
#else
typedef uint8_t dmar_uint_t; // save memory to allow dshot on IOMCU
typedef int8_t dmar_int_t;
#endif
#else
typedef uint32_t dmar_uint_t;
typedef int32_t dmar_int_t;
#endif

#ifdef AP_RCOUT_USE_32BIT_TIME
typedef uint32_t rcout_timer_t;
#define rcout_micros() AP_HAL::micros()
#else
typedef uint64_t rcout_timer_t;
#define rcout_micros() AP_HAL::micros64()
#endif

#define RCOU_DSHOT_TIMING_DEBUG 0

class ChibiOS::RCOutput : public AP_HAL::RCOutput
#ifdef HAL_WITH_BIDIR_DSHOT
  , AP_ESC_Telem_Backend
#endif
{
public:
    // disabled channel marker
    const static uint8_t CHAN_DISABLED = 255;

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

    // surface dshot telemetry for use by the harmonic notch and status information
#ifdef HAL_WITH_BIDIR_DSHOT
    uint16_t get_erpm(uint8_t chan) const override { return _bdshot.erpm[chan]; }
    float get_erpm_error_rate(uint8_t chan) const override {
      return 100.0f * float(_bdshot.erpm_errors[chan]) / (1 + _bdshot.erpm_errors[chan] + _bdshot.erpm_clean_frames[chan]);
    }
    /*
      allow all erpm values to be read and for new updates to be detected - primarily for IOMCU
     */
    bool  new_erpm() override { return _bdshot.update_mask != 0; }
    uint32_t read_erpm(uint16_t* erpm, uint8_t len) override;
#endif
    void set_output_mode(uint32_t mask, const enum output_mode mode) override;
    enum output_mode get_output_mode(uint32_t& mask) override;
    bool get_output_mode_banner(char banner_msg[], uint8_t banner_msg_len) const override;

    /*
     * return mask of channels that must be disabled because they share a group with a digital channel
     */
    uint32_t get_disabled_channels(uint32_t digital_mask) override;

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
    void timer_tick(rcout_timer_t cycle_start_us, rcout_timer_t timeout_period_us);

    /*
      LED push
     */
    void led_timer_tick(rcout_timer_t cycle_start_us, rcout_timer_t timeout_period_us);

#if defined(IOMCU_FW) && HAL_DSHOT_ENABLED
    void timer_tick() override;
    static void dshot_send_trampoline(void *p);
#endif
    /*
      setup for serial output to a set of ESCs, using the given
      baudrate. Assumes 1 start bit, 1 stop bit, LSB first and 8
      databits. This is used for ESC configuration and firmware
      flashing
     */
#if HAL_SERIAL_ESC_COMM_ENABLED
    bool setup_serial_output(uint32_t chan_mask, ByteBuffer *buffer, uint32_t baudrate);

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
    bool serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t motor_mask) override;

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
#endif

    /*
      enable telemetry request for a mask of channels. This is used
      with Dshot to get telemetry feedback
      The mask uses servo channel numbering
     */
    void set_telem_request_mask(uint32_t mask) override;

    /*
      enable bi-directional telemetry request for a mask of channels. This is used
      with Dshot to get telemetry feedback
      The mask uses servo channel numbering
     */
    void set_bidir_dshot_mask(uint32_t mask) override;
#ifdef HAL_WITH_BIDIR_DSHOT
    void set_motor_poles(uint8_t poles) override { _bdshot.motor_poles = poles; }
#endif

    /*
      Set the dshot rate as a multiple of the loop rate
     */
    void set_dshot_rate(uint8_t dshot_rate, uint16_t loop_rate_hz) override;

#if defined(IOMCU_FW)
    /*
      Get/Set the dshot period in us, only for use by the IOMCU
     */
    void set_dshot_period(uint32_t period_us, uint8_t dshot_rate) override {
      _dshot_period_us = period_us;
      _dshot_rate = dshot_rate;
    }
    uint32_t get_dshot_period_us() const override { return _dshot_period_us; }
#endif

#if HAL_DSHOT_ENABLED
    /*
      Set/get the dshot esc_type
     */
    void set_dshot_esc_type(DshotEscType dshot_esc_type) override;

    DshotEscType get_dshot_esc_type() const override { return _dshot_esc_type; }
#endif

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
    void set_safety_mask(uint32_t mask) { safety_mask = mask; }

#if HAL_DSHOT_ENABLED
    /*
     * mark the channels in chanmask as reversible. This is needed for some ESC types (such as Dshot)
     * so that output scaling can be performed correctly. The chanmask passed is added (ORed) into any existing mask.
     * The mask uses servo channel numbering
     */
    void set_reversible_mask(uint32_t chanmask) override;

    /*
     * mark the channels in chanmask as reversed.
     * The chanmask passed is added (ORed) into any existing mask.
     * The mask uses servo channel numbering
     */
    void set_reversed_mask(uint32_t chanmask) override;
    uint32_t get_reversed_mask() override { return _reversed_mask; }

    /*
      mark escs as active for the purpose of sending dshot commands
      The mask uses servo channel numbering
     */
    void set_active_escs_mask(uint32_t chanmask) override { _active_escs_mask |= (chanmask >> chan_offset); }

    /*
      Send a dshot command, if command timout is 0 then 10 commands are sent
      chan is the servo channel to send the command to
     */
    void send_dshot_command(uint8_t command, uint8_t chan = ALL_CHANNELS, uint32_t command_timeout_ms = 0, uint16_t repeat_count = 10, bool priority = false) override;

    /*
     * Update channel masks at 1Hz allowing for actions such as dshot commands to be sent
     */
    void update_channel_masks() override;

    /*
     * Allow channel mask updates to be temporarily suspended
     */
    void disable_channel_mask_updates() override { _disable_channel_mask_updates = true; }
    void enable_channel_mask_updates() override { _disable_channel_mask_updates = false; }
#endif

    /*
      setup serial LED output for a given channel number, with
      the given max number of LEDs in the chain.
     */
#if HAL_SERIALLED_ENABLED
    bool set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode = MODE_PWM_NONE, uint32_t clock_mask = 0) override;

    /*
      setup serial LED output data for a given output channel
      and LEDs number. LED -1 is all LEDs
     */
    bool set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue) override;

    /*
      trigger send of serial LED data
     */
    bool serial_led_send(const uint16_t chan) override;
#endif
    /*
      rcout thread
     */
    void rcout_thread();

    /*
      Force group trigger from all callers rather than just from the main thread
    */
    void force_trigger_groups(bool onoff) override { force_trigger = onoff; }

    /*
     timer information
     */
    void timer_info(ExpandingString &str) override;

private:
    enum class DshotState {
      IDLE = 0,
      SEND_START = 1,
      SEND_COMPLETE = 2,
      RECV_START = 3,
      RECV_COMPLETE = 4,
      RECV_FAILED = 5
    };

    struct PACKED SerialLed {
      uint8_t red;
      uint8_t green;
      uint8_t blue;
    };

    /*
      DShot handling
     */
    // the pre-bit is needed with TIM5, or we can get some corrupt frames
    static const uint8_t dshot_pre = 1;
    static const uint8_t dshot_post = 2;
    static const uint16_t dshot_bit_length = 16 + dshot_pre + dshot_post;
    static const uint16_t DSHOT_BUFFER_LENGTH = dshot_bit_length * 4 * sizeof(dmar_uint_t);
    static const uint16_t MIN_GCR_BIT_LEN = 7;
    static const uint16_t MAX_GCR_BIT_LEN = 22;
    static const uint16_t TELEM_IC_SAMPLE = 16;
    static const uint16_t GCR_TELEMETRY_BIT_LEN = MAX_GCR_BIT_LEN;
    // input capture is expecting TELEM_IC_SAMPLE (16) ticks per transition (22) so the maximum
    // value of the counter in CCR registers is 16*22 == 352, so must be 16-bit
    static const uint16_t GCR_TELEMETRY_BUFFER_LEN = GCR_TELEMETRY_BIT_LEN*sizeof(dmar_uint_t);
    static const uint16_t INVALID_ERPM = 0xffffU;
    static const uint16_t ZERO_ERPM = 0x0fffU;

    struct pwm_group {
        // only advanced timers can do high clocks needed for more than 400Hz
        bool advanced_timer;
        uint8_t chan[4]; // chan number, zero based, 255 for disabled
        PWMConfig pwm_cfg;
        PWMDriver* pwm_drv;
        uint8_t timer_id;
        bool have_up_dma; // can we do DMAR outputs for DShot?
        uint8_t dma_up_stream_id;
        uint8_t dma_up_channel;
#ifdef HAL_WITH_BIDIR_DSHOT
        struct {
            bool have_dma;
            uint8_t stream_id;
            uint8_t channel;
        } dma_ch[4];
#ifdef HAL_TIM_UP_SHARED
        bool shared_up_dma; // do we need to wait for TIMx_UP DMA to be finished after use
#endif
#endif
        uint8_t alt_functions[4];
        ioline_t pal_lines[4];
        // below this line is not initialised by hwdef.h
        enum output_mode current_mode;
        uint16_t frequency_hz;
        // mask of channels that are able to be enabled
        uint32_t ch_mask;
        // mask of channels that are enabled and active
        uint32_t en_mask;
        const stm32_dma_stream_t *dma;
#if AP_HAL_SHARED_DMA_ENABLED
        Shared_DMA *dma_handle;
#endif
        dmar_uint_t *dma_buffer;
        uint16_t dma_buffer_len;
        bool pwm_started;
        uint32_t bit_width_mul;
        uint32_t rc_frequency;
        bool in_serial_dma;
        rcout_timer_t last_dmar_send_us;
        rcout_timer_t dshot_pulse_time_us;
        rcout_timer_t dshot_pulse_send_time_us;
        virtual_timer_t dma_timeout;
#if HAL_SERIALLED_ENABLED
        // serial LED support
        volatile uint8_t serial_nleds;
        uint8_t clock_mask;
        enum output_mode led_mode;
        volatile bool serial_led_pending;
        volatile bool prepared_send;
        HAL_Semaphore serial_led_mutex;
        // structure to hold serial LED data until it can be transferred
        // to the DMA buffer
        SerialLed* serial_led_data[4];
#endif

        eventmask_t dshot_event_mask;
        thread_t* dshot_waiter;
#if HAL_SERIAL_ESC_COMM_ENABLED
        // serial output
        struct {
            // expected time per bit
            uint16_t bit_time_us;

            // channel to output to within group (0 to 3)
            uint8_t chan;

            // thread waiting for byte to be written
            thread_t *waiter;
        } serial;
#endif

        // support for bi-directional dshot
        volatile DshotState dshot_state;
#ifdef HAL_WITH_BIDIR_DSHOT
        struct {
            uint16_t erpm[4];
            volatile bool enabled;
            const stm32_dma_stream_t *ic_dma[4];
            uint16_t dma_tx_size; // save tx value from last read
            Shared_DMA *ic_dma_handle[4];
            uint8_t telem_tim_ch[4];
            uint8_t curr_telem_chan;
            uint8_t prev_telem_chan;
            Shared_DMA *curr_ic_dma_handle; // a shortcut to avoid logic errors involving the wrong lock
            uint16_t telempsc;
            dmar_uint_t dma_buffer_copy[GCR_TELEMETRY_BUFFER_LEN];
#if RCOU_DSHOT_TIMING_DEBUG
            uint16_t telem_rate[4];
            uint16_t telem_err_rate[4];
            rcout_timer_t last_print;  // debug
#endif
        } bdshot;

        // do we have an input capture dma channel
        bool has_ic_dma() const {
          return bdshot.ic_dma_handle[bdshot.curr_telem_chan] != nullptr;
        }

        bool has_shared_ic_up_dma() const {
          return bdshot.ic_dma_handle[bdshot.curr_telem_chan] == dma_handle;
        }

        // is input capture currently enabled
        bool ic_dma_enabled() const {
          return bdshot.enabled && has_ic_dma() && bdshot.ic_dma[bdshot.curr_telem_chan] != nullptr;
        }

        bool has_ic() const {
          return has_ic_dma() || has_shared_ic_up_dma();
        }

        // do we have any kind of input capture
        bool ic_enabled() const {
          return bdshot.enabled && has_ic();
        }
#endif // HAL_WITH_BIDIR_DSHOT
        // are we safe to send another pulse?
        bool can_send_dshot_pulse() const {
          return is_dshot_protocol(current_mode) && AP_HAL::micros64() - last_dmar_send_us > (dshot_pulse_time_us + 50);
        }

        // return whether the group channel is both enabled in the group and for output
        bool is_chan_enabled(uint8_t c) const {
          return chan[c] != CHAN_DISABLED && (en_mask & (1U << chan[c]));
        }
    };
    /*
      timer thread for use by dshot events
     */
    thread_t *rcout_thread_ctx;

#if HAL_SERIALLED_ENABLED
    /*
      timer thread for use by led events
     */
    thread_t *led_thread_ctx;

    /*
      mutex to control LED thread creation
     */
    HAL_Semaphore led_thread_sem;
    bool led_thread_created;
#endif

#if HAL_SERIAL_ESC_COMM_ENABLED
    /*
      structure for IRQ handler for soft-serial input
     */
    static struct irq_state {
        // ioline for port being read
        ioline_t line;

        // time the current byte started
        uint16_t byte_start_tick;

        // number of bits we have read in this byte
        uint8_t nbits;

        // bitmask of bits so far (includes start and stop bits)
        uint16_t bitmask;

        // value of completed byte (includes start and stop bits)
        uint16_t byteval;

        // expected time per bit in micros
        uint16_t bit_time_tick;

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
#endif // HAL_SERIAL_ESC_COMM_ENABLED

    static bool soft_serial_waiting() {
#if HAL_SERIAL_ESC_COMM_ENABLED
      return irq.waiter != nullptr;
#else
      return false;
#endif
    }

    bool in_soft_serial() const {
#if HAL_SERIAL_ESC_COMM_ENABLED
      return serial_group != nullptr;
#else
      return false;
#endif
    }

    static pwm_group pwm_group_list[];
    static const uint8_t NUM_GROUPS;

#if HAL_WITH_IO_MCU
    // cached values of AP_BoardConfig::io_enabled() and AP_BoardConfig::io_dshot()
    // in case the user changes them
    bool iomcu_enabled;
    bool iomcu_dshot;
#endif

    // offset of first local channel
    uint8_t chan_offset;

    // total number of channels on FMU
    uint8_t num_fmu_channels;

    // number of active fmu channels
    uint8_t active_fmu_channels;

#if NUM_SERVO_CHANNELS >= 17
    static const uint8_t max_channels = 32;
#else
    static const uint8_t max_channels = 16;
#endif

    // last sent values are for all channels
    uint16_t last_sent[max_channels];

    // these values are for the local channels. Non-local channels are handled by IOMCU
    uint32_t en_mask;
    uint16_t period[max_channels];

    // handling of bi-directional dshot
    struct {
        uint32_t mask;
        uint32_t disabled_mask; // record of channels that were tried, but failed
        uint16_t erpm[max_channels];
        volatile uint32_t update_mask;
#ifdef HAL_WITH_BIDIR_DSHOT
        uint16_t erpm_errors[max_channels];
        uint16_t erpm_clean_frames[max_channels];
        uint32_t erpm_last_stats_ms[max_channels];
        uint8_t motor_poles;
#endif
    } _bdshot;

    // dshot period
    rcout_timer_t _dshot_period_us = 400;
    // dshot rate as a multiple of loop rate or 0 for 1Khz
    uint8_t _dshot_rate;
    // dshot periods since the last push()
    uint8_t _dshot_cycle;
    // virtual timer for post-push() pulses
    virtual_timer_t _dshot_rate_timer;
    // force triggering of groups
    bool force_trigger;

#if HAL_DSHOT_ENABLED
    // dshot commands
    // RingBuffer to store outgoing request.
    struct DshotCommandPacket {
      uint8_t command;
      uint32_t cycle;
      uint8_t chan;
    };

    ObjectBuffer<DshotCommandPacket> _dshot_command_queue{8};
    DshotCommandPacket _dshot_current_command;

    DshotEscType _dshot_esc_type;

    // control updates to channel masks
    bool _disable_channel_mask_updates;

    bool dshot_command_is_active(const pwm_group& group) const {
      return (_dshot_current_command.chan == RCOutput::ALL_CHANNELS || (group.ch_mask & (1UL << _dshot_current_command.chan)))
                && _dshot_current_command.cycle > 0;
    }
#endif // HAL_DSHOT_ENABLED
    bool corked;
    // mask of channels that are running in high speed
    uint32_t fast_channel_mask;
    uint32_t io_fast_channel_mask;
    // mask of channels that are 3D capable
    uint32_t _reversible_mask;
    // mask of channels that should be reversed at startup
    uint32_t _reversed_mask;
    // mask of active ESCs
    uint32_t _active_escs_mask;

    // last time pulse was triggererd used to prevent overlap
    rcout_timer_t last_pulse_trigger_us;

    // mutex for oneshot triggering
    mutex_t trigger_mutex;

    // which output groups need triggering
    uint8_t trigger_groupmask;

    // widest pulse for oneshot triggering
    uint16_t trigger_widest_pulse;

    bool dshot_timer_setup;

    volatile bool _initialised;

    bool is_bidir_dshot_enabled(const pwm_group& group) const { return (_bdshot.mask & group.ch_mask) != 0; }

    static bool is_dshot_send_allowed(DshotState state) {
      return state == DshotState::IDLE || state == DshotState::RECV_COMPLETE || state == DshotState::RECV_FAILED;
    }

    // are all the ESCs returning data
    bool group_escs_active(const pwm_group& group) const {
      return group.en_mask > 0 && (group.en_mask & _active_escs_mask) == group.en_mask;
    }

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
    uint32_t safety_mask;

    // update safety switch and LED
    void safety_update(void);

    // LED thread
    void led_thread();
    bool start_led_thread();

    uint32_t telem_request_mask;

    /*
      Serial lED handling. Max of 32 LEDs uses max 12k of memory per group
      return true if send was successful
    */
    static const eventmask_t serial_event_mask = EVENT_MASK(10);
    bool serial_led_send(pwm_group &group);
    void serial_led_set_single_rgb_data(pwm_group& group, uint8_t idx, uint8_t led, uint8_t red, uint8_t green, uint8_t blue);
    void fill_DMA_buffer_serial_led(pwm_group& group);
    volatile bool serial_led_pending;

    void dma_allocate(Shared_DMA *ctx);
    void dma_deallocate(Shared_DMA *ctx);
    uint16_t create_dshot_packet(const uint16_t value, bool telem_request, bool bidir_telem);
    void fill_DMA_buffer_dshot(dmar_uint_t *buffer, uint8_t stride, uint16_t packet, uint16_t clockmul);

    // event to allow dshot cascading
#if defined(HAL_TIM_UP_SHARED)
    static const eventmask_t DSHOT_CASCADE = EVENT_MASK(16);
#else
    static const eventmask_t DSHOT_CASCADE = 0;
#endif
    static const eventmask_t EVT_PWM_SEND  = EVENT_MASK(11);
    static const eventmask_t EVT_PWM_SYNTHETIC_SEND  = EVENT_MASK(13);

    void dshot_send_groups(rcout_timer_t cycle_start_us, rcout_timer_t timeout_us);
    void dshot_send(pwm_group &group, rcout_timer_t cycle_start_us, rcout_timer_t timeout_us);
    bool dshot_send_command(pwm_group &group, uint8_t command, uint8_t chan);
    static void dshot_update_tick(virtual_timer_t*, void* p);
    static void dshot_send_next_group(void* p);
    // release locks on the groups that are pending in reverse order
    sysinterval_t calc_ticks_remaining(pwm_group &group, rcout_timer_t cycle_start_us, rcout_timer_t timeout_period_us, rcout_timer_t output_period_us);
    void dshot_collect_dma_locks(rcout_timer_t cycle_start_us, rcout_timer_t timeout_period_us, bool led_thread = false);
    static void dma_up_irq_callback(void *p, uint32_t flags);
    static void dma_unlock(virtual_timer_t*, void *p);
    void dma_cancel(pwm_group& group);
    bool mode_requires_dma(enum output_mode mode) const;
    bool setup_group_DMA(pwm_group &group, uint32_t bitrate, uint32_t bit_width, bool active_high,
                         const uint16_t buffer_length, rcout_timer_t pulse_time_us,
                         bool at_least_freq);
    void send_pulses_DMAR(pwm_group &group, uint32_t buffer_length);
    void set_group_mode(pwm_group &group);
    static uint32_t protocol_bitrate(const enum output_mode mode);
    void print_group_setup_error(pwm_group &group, const char* error_string);

    /*
      Support for bi-direction dshot
     */
    void bdshot_ic_dma_allocate(Shared_DMA *ctx);
    void bdshot_ic_dma_deallocate(Shared_DMA *ctx);
    static uint32_t bdshot_decode_telemetry_packet(dmar_uint_t* buffer, uint32_t count);
    static uint32_t bdshot_decode_telemetry_packet_f1(dmar_uint_t* buffer, uint32_t count, bool reversed);
    bool bdshot_decode_telemetry_from_erpm(uint16_t erpm, uint8_t chan);
    bool bdshot_decode_dshot_telemetry(pwm_group& group, uint8_t chan);
    static uint8_t bdshot_find_next_ic_channel(const pwm_group& group);
    static void bdshot_dma_ic_irq_callback(void *p, uint32_t flags);
    static void bdshot_finish_dshot_gcr_transaction(virtual_timer_t* vt, void *p);
    bool bdshot_setup_group_ic_DMA(pwm_group &group);
    void bdshot_prepare_for_next_pulse(pwm_group& group);
    static void bdshot_receive_pulses_DMAR(pwm_group* group);
    static void bdshot_receive_pulses_DMAR_f1(pwm_group* group);
    static void bdshot_reset_pwm(pwm_group& group, uint8_t telem_channel);
    static void bdshot_reset_pwm_f1(pwm_group& group, uint8_t telem_channel);
    static void bdshot_disable_pwm_f1(pwm_group& group);
    static void bdshot_config_icu_dshot(stm32_tim_t* TIMx, uint8_t chan, uint8_t ccr_ch);
    static void bdshot_config_icu_dshot_f1(stm32_tim_t* TIMx, uint8_t chan, uint8_t ccr_ch);
    static uint32_t bdshot_get_output_rate_hz(const enum output_mode mode);

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
#if AP_HAL_SHARED_DMA_ENABLED
    // serial output support
    bool serial_write_byte(uint8_t b);
    bool serial_read_byte(uint8_t &b);
    void fill_DMA_buffer_byte(dmar_uint_t *buffer, uint8_t stride, uint8_t b , uint32_t bitval);
    static void serial_bit_irq(void);
    static void serial_byte_timeout(virtual_timer_t* vt, void *ctx);
#endif
};

#if RCOU_DSHOT_TIMING_DEBUG
#define TOGGLE_PIN_DEBUG(pin) do { palToggleLine(HAL_GPIO_LINE_GPIO ## pin); } while (0)
#else
#define TOGGLE_PIN_DEBUG(pin) do {} while (0)
#endif

#endif // HAL_USE_PWM
