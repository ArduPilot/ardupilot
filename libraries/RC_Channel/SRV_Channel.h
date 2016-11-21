/*
  control of servo output ranges, trim and servo reversal. This can
  optionally be used to provide separation of input and output channel
  ranges so that RCn_MIN, RCn_MAX, RCn_TRIM and RCn_REV only apply to
  the input side of RC_Channel

  It works by running servo output calculations as normal, then
  re-mapping the output according to the servo MIN/MAX/TRIM/REV from
  this object

  Only 4 channels of ranges are defined as those match the input
  channels for R/C sticks
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "RC_Channel.h"

#define NUM_SERVO_RANGE_CHANNELS 4

/*
  class	SRV_Channel
*/
class SRV_Channels {
public:
    // constructor
    SRV_Channels(void);

    static const struct AP_Param::GroupInfo var_info[];

    // take current radio_out for first 4 channels and remap using
    // servo ranges if enabled
    void remap_servo_output(void);

    // set and save trim values from current RC input trim
    void set_trim(void);

    // setup output ESC scaling for a channel
    void set_esc_scaling(uint8_t chnum);

    // return true when enabled
    bool enabled(void) const { return enable; }

    // return true when auto_trim enabled
    bool auto_trim_enabled(void) const { return auto_trim; }
    
    // adjust trim of a channel by a small increment
    void adjust_trim(uint8_t ch, float v);

    // save trims
    void save_trim(void);
    
private:
    AP_Int8 enable;

    int8_t esc_cal_chan;
    bool last_enable;
    uint8_t trimmed_mask;

    // PWM values for min/max and trim
    AP_Int16 servo_min[NUM_SERVO_RANGE_CHANNELS];
    AP_Int16 servo_max[NUM_SERVO_RANGE_CHANNELS];
    AP_Int16 servo_trim[NUM_SERVO_RANGE_CHANNELS];

    // reversal, following convention that < 0 means reversed, >= 0 means normal
    AP_Int8 reverse[NUM_SERVO_RANGE_CHANNELS];

    AP_Int8 auto_trim;

    // remap a PWM value from a channel in value
    uint16_t remap_pwm(uint8_t ch, uint16_t pwm) const;
};
