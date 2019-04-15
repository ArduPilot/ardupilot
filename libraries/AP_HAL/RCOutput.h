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
#define CH_NONE 255
#endif

class ByteBuffer;

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
     * so that output scaling can be performed correctly. The chanmask passed is added (ORed) into
     * any existing mask.
     */
    virtual void     set_reversible_mask(uint16_t chanmask) {}
    
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
      set PWM to send to a set of channels when the safety switch is
      in the safe state
     */
    virtual void     set_safety_pwm(uint32_t chmask, uint16_t period_us) {}

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
      If we support async sends (px4), this will force it to be serviced immediately
     */
    virtual void     force_safety_no_wait(void) {}

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
    virtual bool serial_setup_output(uint8_t chan, uint32_t baudrate, uint16_t chanmask) { return false; }

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
    };
    virtual void    set_output_mode(uint16_t mask, enum output_mode mode) {}

    /*
      set default update rate
     */
    virtual void    set_default_rate(uint16_t rate_hz) {}

    /*
      enable telemetry request for a mask of channels. This is used
      with DShot to get telemetry feedback
     */
    virtual void set_telem_request_mask(uint16_t mask) {}
};
