#pragma once

#include "AP_HAL_Namespace.h"
#include <stdint.h>

#define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

/* Define the CH_n names, indexed from 1, if we don't have them already */
#ifndef CH_1
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10
#define CH_12 11
#define CH_13 12
#define CH_14 13
#define CH_15 14
#define CH_16 15
#define CH_17 16
#define CH_18 17
#define CH_19 18
#define CH_20 19
#define CH_21 20
#define CH_22 21
#define CH_23 22
#define CH_24 23
#define CH_25 24
#define CH_26 25
#define CH_27 26
#define CH_28 27
#define CH_29 28
#define CH_30 29
#define CH_31 30
#define CH_32 31
#define CH_NONE 255
#endif

class ByteBuffer;

class ExpandingString;

class AP_HAL::RCOutput {
public:
    virtual void init() = 0;

    /* Output freq (1/period) control */
    virtual void     set_freq(uint32_t chmask, uint16_t freq_hz) = 0;
    virtual uint16_t get_freq(uint8_t chan) = 0;

    /* Output active/highZ control, either by single channel at a time
     * or a mask of channels */
    virtual void     enable_ch(uint8_t chan) = 0;
    virtual void     disable_ch(uint8_t chan) = 0;

    /*
     * Output a single channel, possibly grouped with previous writes if
     * cork() has been called before.
     */
    virtual void     write(uint8_t chan, uint16_t period_us) = 0;

    /*
     * mark the channels in chanmask as reversible. This is needed for some ESC types (such as DShot)
     * so that output scaling can be performed correctly. The chanmask passed is added (ORed) into any existing mask.
     * The mask uses servo channel numbering
     */
    virtual void     set_reversible_mask(uint32_t chanmask) {}
    
    /*
     * mark the channels in chanmask as reversed.
     * The chanmask passed is added (ORed) into any existing mask.
     * The mask uses servo channel numbering
     */
    virtual void     set_reversed_mask(uint32_t chanmask) {}
    virtual uint32_t get_reversed_mask() { return 0; }

    /*
     * Update channel masks at 1Hz allowing for actions such as dshot commands to be sent
     */
    virtual void     update_channel_masks() {}

    /*
     * Allow channel mask updates to be temporarily suspended
     */
    virtual void     disable_channel_mask_updates() {}
    virtual void     enable_channel_mask_updates() {}

    /*
     * Delay subsequent calls to write() going to the underlying hardware in
     * order to group related writes together. When all the needed writes are
     * done, call push() to commit the changes.
     */
    virtual void     cork() = 0;

    /*
     * Push pending changes to the underlying hardware. All changes between a
     * call to cork() and push() are pushed together in a single transaction.
     */
    virtual void     push() = 0;

    /* Read back current output state, as either single channel or
     * array of channels. On boards that have a separate IO controller,
     * this returns the latest output value that the IO controller has
     * reported */
    virtual uint16_t read(uint8_t chan) = 0;
    virtual void     read(uint16_t* period_us, uint8_t len) = 0;

    /* Read the current input state. This returns the last value that was written. */
    virtual uint16_t read_last_sent(uint8_t chan) { return read(chan); }
    virtual void     read_last_sent(uint16_t* period_us, uint8_t len) { read(period_us, len); };

    /*
      set PWM to send to a set of channels if the FMU firmware dies
     */
    virtual void     set_failsafe_pwm(uint32_t chmask, uint16_t period_us) {}

    /*
      force the safety switch on, disabling PWM output from the IO board
      return false (indicating failure) by default so that boards with no safety switch
      do not need to implement this method
     */
    virtual bool     force_safety_on(void) { return false; }

    /*
      force the safety switch off, enabling PWM output from the IO board
     */
    virtual void     force_safety_off(void) {}

    /*
      setup scaling of ESC output for ESCs that can output a
      percentage of power (such as UAVCAN ESCs). The values are in
      microseconds, and represent minimum and maximum PWM values which
      will be used to convert channel writes into a percentage
     */
    virtual void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) {}

    /*
      return ESC scaling value from set_esc_scaling()
     */
    virtual bool     get_esc_scaling(uint16_t &min_pwm, uint16_t &max_pwm) { return false; }
    
    /*
      returns the pwm value scaled to [-1;1] regrading to set_esc_scaling ranges range without constraints.
     */
    virtual float    scale_esc_to_unity(uint16_t pwm) { return 0; }

    /*
      return the erpm and error rate for a channel if available
     */
    virtual uint16_t get_erpm(uint8_t chan) const { return 0; }
    virtual float get_erpm_error_rate(uint8_t chan) const { return 100.0f; }

    /*
      enable PX4IO SBUS out at the given rate
     */
    virtual bool enable_px4io_sbus_out(uint16_t rate_hz) { return false; }

    /*
     * Optional method to control the update of the motors. Derived classes
     * can implement it if their HAL layer requires.
     */
    virtual void timer_tick(void) { }

    /*
      setup for serial output to an ESC using the given
      baudrate. Assumes 1 start bit, 1 stop bit, LSB first and 8
      databits. This is used for passthrough ESC configuration and
      firmware flashing

      While serial output is active normal output to all channels in
      the chanmask is suspended. Output to some other channels (such
      as those in the same channel timer groups) may also be stopped,
      depending on the implementation
     */
    virtual bool serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t chanmask) { return false; }

    /*
      write a set of bytes to an ESC, using settings from
      serial_setup_output. This is a blocking call
     */
    virtual bool serial_write_bytes(const uint8_t *bytes, uint16_t len) { return false; }

    /*
      read a series of bytes from a port, using serial parameters from serial_setup_output()
      return the number of bytes read. This is a blocking call
     */
    virtual uint16_t serial_read_bytes(uint8_t *buf, uint16_t len) { return 0; }
    
    /*
      stop serial output. This restores the previous output mode for
      the channel and any other channels that were stopped by
      serial_setup_output()
     */
    virtual void serial_end(void) {}
    
    /*
      output modes. Allows for support of PWM, oneshot and dshot 
    */
    // this enum is used by BLH_OTYPE and ESC_PWM_TYPE on AP_Periph
    // double check params are still correct when changing
    enum output_mode {
        MODE_PWM_NONE,
        MODE_PWM_NORMAL,
        MODE_PWM_ONESHOT,
        MODE_PWM_ONESHOT125,
        MODE_PWM_BRUSHED,
        MODE_PWM_DSHOT150,
        MODE_PWM_DSHOT300,
        MODE_PWM_DSHOT600,
        MODE_PWM_DSHOT1200,
        MODE_NEOPIXEL,  // same as MODE_PWM_DSHOT at 800kHz but it's an LED
        MODE_PROFILED,  // same as MODE_PWM_DSHOT using separate clock and data
    };
    // true when the output mode is of type dshot
    // static to allow use in the ChibiOS thread stuff
    static bool is_dshot_protocol(const enum output_mode mode);


    // BLHeli32: https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt
    // BLHeli_S: https://github.com/bitdump/BLHeli/blob/master/BLHeli_S%20SiLabs/Dshotprog%20spec%20BLHeli_S.txt
    enum BLHeliDshotCommand : uint8_t {
      DSHOT_RESET = 0,
      DSHOT_BEEP1 = 1,
      DSHOT_BEEP2 = 2,
      DSHOT_BEEP3 = 3,
      DSHOT_BEEP4 = 4,
      DSHOT_BEEP5 = 5,
      DSHOT_ESC_INFO = 6,
      DSHOT_ROTATE = 7,
      DSHOT_ROTATE_ALTERNATE = 8,
      DSHOT_3D_OFF = 9,
      DSHOT_3D_ON = 10,
      DSHOT_SAVE = 12,
      DSHOT_NORMAL = 20,
      DSHOT_REVERSE = 21,
      // The following options are only available on BLHeli32
      DSHOT_LED0_ON = 22,
      DSHOT_LED1_ON = 23,
      DSHOT_LED2_ON = 24,
      DSHOT_LED3_ON = 25,
      DSHOT_LED0_OFF = 26,
      DSHOT_LED1_OFF = 27,
      DSHOT_LED2_OFF = 28,
      DSHOT_LED3_OFF = 29,
    };

    const uint8_t DSHOT_ZERO_THROTTLE = 48;

    enum DshotEscType {
      DSHOT_ESC_NONE = 0,
      DSHOT_ESC_BLHELI = 1,
      DSHOT_ESC_BLHELI_S = 2
    };

    virtual void    set_output_mode(uint32_t mask, enum output_mode mode) {}

    /*
     * get output mode banner to inform user of how outputs are configured
     */
    virtual bool get_output_mode_banner(char banner_msg[], uint8_t banner_msg_len) const { return false; }

    /*
     * return mask of channels that must be disabled because they share a group with a digital channel
     */
    virtual uint32_t get_disabled_channels(uint32_t digital_mask) { return 0; }

    /*
      set default update rate
     */
    virtual void    set_default_rate(uint16_t rate_hz) {}

    /*
      enable telemetry request for a mask of channels. This is used
      with DShot to get telemetry feedback
     */
    virtual void set_telem_request_mask(uint32_t mask) {}

    /*
      enable bi-directional telemetry request for a mask of channels. This is used
      with DShot to get telemetry feedback
     */
    virtual void set_bidir_dshot_mask(uint32_t mask) {}

    /*
      mark escs as active for the purpose of sending dshot commands
     */
    virtual void set_active_escs_mask(uint32_t mask) {}

    /*
      Set the dshot rate as a multiple of the loop rate
     */
    virtual void set_dshot_rate(uint8_t dshot_rate, uint16_t loop_rate_hz) {}

    /*
      Set the dshot ESC type
     */
    virtual void set_dshot_esc_type(DshotEscType esc_type) {}

    virtual DshotEscType get_dshot_esc_type() const { return DSHOT_ESC_NONE; }

    const static uint32_t ALL_CHANNELS = 255;
    /*
      Send a dshot command, if command timout is 0 then 10 commands are sent
      chan is the servo channel to send the command to
     */
    virtual void send_dshot_command(uint8_t command, uint8_t chan = ALL_CHANNELS, uint32_t command_timeout_ms = 0, uint16_t repeat_count = 10, bool priority = false) {}

    /*
      set the number of motor poles to be used in rpm calculations
     */
    virtual void set_motor_poles(uint8_t poles) {}

    /*
      setup serial led output for a given channel number, with
      the given max number of LEDs in the chain.
     */
    virtual bool set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode = MODE_PWM_NONE, uint32_t clock_mask = 0) { return false; }

    /*
      setup serial led output data for a given output channel
      and led number. A led number of -1 means all LEDs. LED 0 is the first LED
     */
    virtual void set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue) {}
    
    /*
      trigger send of serial led
     */
    virtual void serial_led_send(const uint16_t chan) {}

    virtual void timer_info(ExpandingString &str) {}

    /*
     * calculate the prescaler required to achieve the desire bitrate
     */
    static uint32_t calculate_bitrate_prescaler(uint32_t timer_clock, uint32_t target_frequency, bool is_dshot);

    /*
     * bit width values for different protocols
     */
    /*
     * It seems ESCs are quite sensitive to the DSHOT duty cycle.
     * Options are (ticks, percentage):
     * 20/7/14, 35/70
     * 11/4/8, 36/72
     * 8/3/6, 37/75
     */
    // bitwidths: 8/3/6 == 37%/75%
    static constexpr uint32_t DSHOT_BIT_WIDTH_TICKS_DEFAULT = 8;
    static constexpr uint32_t DSHOT_BIT_0_TICKS_DEFAULT = 3;
    static constexpr uint32_t DSHOT_BIT_1_TICKS_DEFAULT = 6;
    // bitwidths: 11/4/8 == 36%/72%
    static constexpr uint32_t DSHOT_BIT_WIDTH_TICKS_S = 11;
    static constexpr uint32_t DSHOT_BIT_0_TICKS_S = 4;
    static constexpr uint32_t DSHOT_BIT_1_TICKS_S = 8;

    static uint32_t DSHOT_BIT_WIDTH_TICKS;
    static uint32_t DSHOT_BIT_0_TICKS;
    static uint32_t DSHOT_BIT_1_TICKS;

    // See WS2812B spec for expected pulse widths
    static constexpr uint32_t NEOP_BIT_WIDTH_TICKS = 20;
    static constexpr uint32_t NEOP_BIT_0_TICKS = 7;
    static constexpr uint32_t NEOP_BIT_1_TICKS = 14;
    // neopixel does not use pulse widths at all
    static constexpr uint32_t PROFI_BIT_0_TICKS = 7;
    static constexpr uint32_t PROFI_BIT_1_TICKS = 14;

protected:

    // helper functions for implementation of get_output_mode_banner
    void append_to_banner(char banner_msg[], uint8_t banner_msg_len, output_mode out_mode, uint8_t low_ch, uint8_t high_ch) const;
    const char* get_output_mode_string(enum output_mode out_mode) const;
};
