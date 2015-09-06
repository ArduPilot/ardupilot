
#ifndef __AP_HAL_RC_OUTPUT_H__
#define __AP_HAL_RC_OUTPUT_H__

#include "AP_HAL_Namespace.h"

#define RC_OUTPUT_MIN_PULSEWIDTH        400
#define RC_OUTPUT_MAX_PULSEWIDTH        2100
#define RC_OUTPUT_MAX_BACKENDS          4
#define RC_OUTPUT_MAX_CHANNELS          64

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
#define CH_30 29
#define CH_31 30
#define CH_32 31
#define CH_33 32
#define CH_34 33
#define CH_35 34
#define CH_36 35
#define CH_37 36
#define CH_38 37
#define CH_39 38
#define CH_40 39
#define CH_41 40
#define CH_42 41
#define CH_43 42
#define CH_44 43
#define CH_45 44
#define CH_46 45
#define CH_47 46
#define CH_48 47
#define CH_49 48
#define CH_50 49
#define CH_51 50
#define CH_52 51
#define CH_53 52
#define CH_54 53
#define CH_55 54
#define CH_56 55
#define CH_57 56
#define CH_58 57
#define CH_59 58
#define CH_60 59
#define CH_61 60
#define CH_62 61
#define CH_63 62
#define CH_64 64
#endif

class AP_HAL::RCOutput {
public:
    /* Init and add a new backend */
    void init_backend(RCOutput_Backend *backend);

    /* Output freq (1/period) control */
    void set_freq(uint64_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);

    /* Output active/highZ control, either by single channel at a time
     * or a mask of channels */
    void enable_ch(uint8_t ch);
    void disable_ch(uint8_t ch);

    /* Output a single channel */
    void write(uint8_t ch, uint16_t period_us);

    /* Read back current output state */
    uint16_t read(uint8_t ch);

    /*
      set PWM to send to a set of channels when the safety switch is
      in the safe state
     */
    void set_safety_pwm(uint64_t chmask, uint16_t period_us);

    /*
      set PWM to send to a set of channels if the FMU firmware dies
     */
    void set_failsafe_pwm(uint64_t chmask, uint16_t period_us);

    /*
      force the safety switch on, disabling PWM output from the IO board
      return false (indicating failure) by default so that boards with no safety switch
      do not need to implement this method
     */
    bool force_safety_on();

    /*
      force the safety switch off, enabling PWM output from the IO board
     */
    void force_safety_off();

    /*
      setup scaling of ESC output for ESCs that can output a
      percentage of power (such as UAVCAN ESCs). The values are in
      microseconds, and represent minimum and maximum PWM values which
      will be used to convert channel writes into a percentage
     */
    void set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm);

  private:

    // backend objects
    uint8_t _backend_count;
    RCOutput_Backend *_backends[RC_OUTPUT_MAX_BACKENDS];

    // backends by channel
    uint8_t _backend_by_channel[RC_OUTPUT_MAX_CHANNELS];
    // backend channels by backends
    uint8_t _backend_channel_by_channel[RC_OUTPUT_MAX_CHANNELS];
};

#endif // __AP_HAL_RC_OUTPUT_H__
