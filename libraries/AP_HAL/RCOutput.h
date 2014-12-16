
#ifndef __AP_HAL_RC_OUTPUT_H__
#define __AP_HAL_RC_OUTPUT_H__

#include "AP_HAL_Namespace.h"

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
#endif


class AP_HAL::RCOutput {
public:
    virtual void init(void* implspecific) = 0;

    /* Output freq (1/period) control */
    virtual void     set_freq(uint32_t chmask, uint16_t freq_hz) = 0;
    virtual uint16_t get_freq(uint8_t ch) = 0;

    /* Output active/highZ control, either by single channel at a time
     * or a mask of channels */
    virtual void     enable_ch(uint8_t ch) = 0;
    virtual void     disable_ch(uint8_t ch) = 0;

    /* Output, either single channel or bulk array of channels */
    virtual void     write(uint8_t ch, uint16_t period_us) = 0;
    virtual void     write(uint8_t ch, uint16_t* period_us, uint8_t len) = 0;

    /* Read back current output state, as either single channel or
     * array of channels. */
    virtual uint16_t read(uint8_t ch) = 0;
    virtual void     read(uint16_t* period_us, uint8_t len) = 0;

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
      setup scaling of ESC output for ESCs that can output a
      percentage of power (such as UAVCAN ESCs). The values are in
      microseconds, and represent minimum and maximum PWM values which
      will be used to convert channel writes into a percentage
     */
    virtual void     set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) {}
};

#endif // __AP_HAL_RC_OUTPUT_H__

