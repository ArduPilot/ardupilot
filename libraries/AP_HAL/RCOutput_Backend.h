
#ifndef __AP_HAL_RC_OUTPUT_BACKEND_H__
#define __AP_HAL_RC_OUTPUT_BACKEND_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::RCOutput_Backend {
  public:
    virtual bool init() = 0;

    virtual uint8_t  get_num_channels() = 0;

    /* Output freq (1/period) control */
    virtual void     set_freq(uint64_t chmask, uint16_t freq_hz) = 0;
    virtual uint16_t get_freq(uint8_t ch) = 0;

    /* Output active/highZ control, either by single channel at a time
     * or a mask of channels */
    virtual void     enable_ch(uint8_t ch) = 0;
    virtual void     disable_ch(uint8_t ch) = 0;

    /* Write output state */
    virtual void     write(uint8_t ch, uint16_t period_us) = 0;

    /* Read back current output state */
    virtual uint16_t read(uint8_t ch) = 0;

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

#endif // __AP_HAL_RC_OUTPUT_BACKEND_H__

