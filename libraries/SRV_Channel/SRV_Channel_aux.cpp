/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  SRV_Channel_aux.cpp - handling of servo auxillary functions
 */
#include "SRV_Channel.h"

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;

/// map a function to a servo channel and output it
void SRV_Channel::output_ch(void)
{
    int8_t passthrough_from = -1;

    // take care of special function cases
    switch(function)
    {
    case k_manual:              // manual
        passthrough_from = ch_num;
        break;
    case k_rcin1 ... k_rcin16: // rc pass-thru
        passthrough_from = int8_t(function - k_rcin1);
        break;
    }
    if (passthrough_from != -1) {
        // we are doing passthrough from input to output for this channel
        RC_Channel *c = rc().channel(passthrough_from);
        if (c) {
            if (SRV_Channels::passthrough_disabled()) {
                output_pwm = c->get_radio_trim();
            } else {
                const int16_t radio_in = c->get_radio_in();
                if (!ign_small_rcin_changes) {
                    output_pwm = radio_in;
                    previous_radio_in = radio_in;
                } else {
                    // check if rc input value has changed by more than the deadzone
                    if (abs(radio_in - previous_radio_in) > c->get_dead_zone()) {
                        output_pwm = radio_in;
                        ign_small_rcin_changes = false;
                    }
                }
            }
        }
    }
    if (!(SRV_Channels::disabled_mask & (1U<<ch_num))) {
        hal.rcout->write(ch_num, output_pwm);
    }
}

/*
  call output_ch() on all channels
 */
void SRV_Channels::output_ch_all(void)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        channels[i].output_ch();
    }
}

/*
  return the current function for a channel
*/
SRV_Channel::Aux_servo_function_t SRV_Channels::channel_function(uint8_t channel)
{
    if (channel < NUM_SERVO_CHANNELS) {
        return (SRV_Channel::Aux_servo_function_t)channels[channel].function.get();
    }
    return SRV_Channel::k_none;
}

/*
   setup a channels aux servo function
*/
void SRV_Channel::aux_servo_function_setup(void)
{
    if (type_setup) {
        return;
    }
    switch (function) {
    case k_flap:
    case k_flap_auto:
    case k_egg_drop:
        set_range(100);
        break;
    case k_heli_rsc:
    case k_heli_tail_rsc:
    case k_motor_tilt:
    case k_boost_throttle:
    case k_thrust_out:
        set_range(1000);
        break;
    case k_aileron_with_input:
    case k_elevator_with_input:
    case k_aileron:
    case k_elevator:
    case k_dspoilerLeft1:
    case k_dspoilerLeft2:
    case k_dspoilerRight1:
    case k_dspoilerRight2:
    case k_rudder:
    case k_steering:
    case k_flaperon_left:
    case k_flaperon_right:
    case k_tiltMotorLeft:
    case k_tiltMotorRight:
    case k_elevon_left:
    case k_elevon_right:
    case k_vtail_left:
    case k_vtail_right:
    case k_roll_out:
    case k_pitch_out:
    case k_yaw_out:
        set_angle(4500);
        break;
    case k_throttle:
    case k_throttleLeft:
    case k_throttleRight:
        // fixed wing throttle
        set_range(100);
        break;
    default:
        break;
    }
}

/// setup the output range types of all functions
void SRV_Channels::update_aux_servo_function(void)
{
    if (!channels) {
        return;
    }
    function_mask.clearall();

    for (uint8_t i = 0; i < SRV_Channel::k_nr_aux_servo_functions; i++) {
        functions[i].channel_mask = 0;
    }

    // set auxiliary ranges
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if ((uint8_t)channels[i].function.get() < SRV_Channel::k_nr_aux_servo_functions) {
            channels[i].aux_servo_function_setup();
            function_mask.set((uint8_t)channels[i].function.get());
            functions[channels[i].function.get()].channel_mask |= 1U<<i;
        }
    }
    initialised = true;
}

/// Should be called after the the servo functions have been initialized
void SRV_Channels::enable_aux_servos()
{
    hal.rcout->set_default_rate(uint16_t(_singleton->default_rate.get()));

    update_aux_servo_function();

    // enable all channels that are set to a valid function. This
    // includes k_none servos, which allows those to get their initial
    // trim value on startup
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        // see if it is a valid function
        if ((uint8_t)c.function.get() < SRV_Channel::k_nr_aux_servo_functions) {
            hal.rcout->enable_ch(c.ch_num);
        }

        /*
          for channels which have been marked as digital output then the
          MIN/MAX/TRIM values have no meaning for controlling output, as
          the HAL handles the scaling. We still need to cope with places
          in the code that may try to set a PWM value however, so to
          ensure consistency we force the MIN/MAX/TRIM to be consistent
          across all digital channels. We use a MIN/MAX of 1000/2000, and
          set TRIM to either 1000 or 1500 depending on whether the channel
          is reversible
        */
        if (digital_mask & (1U<<i)) {
            c.servo_min.set(1000);
            c.servo_max.set(2000);
            if (reversible_mask & (1U<<i)) {
                c.servo_trim.set(1500);
            } else {
                c.servo_trim.set(1000);
            }
        }
    }

#if HAL_SUPPORT_RCOUT_SERIAL
    blheli_ptr->update();
#endif
}

/// enable output channels using a channel mask
void SRV_Channels::enable_by_mask(uint16_t mask)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (mask & (1U<<i)) {
            hal.rcout->enable_ch(i);
        }
    }
}

/*
  set radio_out for all channels matching the given function type
 */
void SRV_Channels::set_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t value)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function.get() == function) {
            channels[i].set_output_pwm(value);
            channels[i].output_ch();
        }
    }
}

/*
  set radio_out for all channels matching the given function type
  trim the output assuming a 1500 center on the given value
  reverses pwm output based on channel reversed property
 */
void
SRV_Channels::set_output_pwm_trimmed(SRV_Channel::Aux_servo_function_t function, int16_t value)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function.get() == function) {
            int16_t value2;
            if (channels[i].get_reversed()) {
                value2 = 1500 - value + channels[i].get_trim();
            } else {
                value2 = value - 1500 + channels[i].get_trim();
            }
            channels[i].set_output_pwm(constrain_int16(value2,channels[i].get_output_min(),channels[i].get_output_max()));
            channels[i].output_ch();
          }
    }
}

/*
  set and save the trim value to current output for all channels matching
  the given function type
 */
void
SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::Aux_servo_function_t function)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function.get() == function) {
            channels[i].servo_trim.set_and_save_ifchanged(channels[i].get_output_pwm());
        }
    }
}

/*
  copy radio_in to radio_out for a given function
 */
void
SRV_Channels::copy_radio_in_out(SRV_Channel::Aux_servo_function_t function, bool do_input_output)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function.get() == function) {
            RC_Channel *c = rc().channel(channels[i].ch_num);
            if (c == nullptr) {
                continue;
            }
            channels[i].set_output_pwm(c->get_radio_in());
            if (do_input_output) {
                channels[i].output_ch();
            }
        }
    }
}

/*
  copy radio_in to radio_out for a channel mask
 */
void
SRV_Channels::copy_radio_in_out_mask(uint16_t mask)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if ((1U<<i) & mask) {
            RC_Channel *c = rc().channel(channels[i].ch_num);
            if (c == nullptr) {
                continue;
            }
            channels[i].set_output_pwm(c->get_radio_in());
        }
    }

}

/*
  setup failsafe value for an auxiliary function type to a Limit
 */
void
SRV_Channels::set_failsafe_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t pwm)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel &c = channels[i];
        if (c.function.get() == function) {
            hal.rcout->set_failsafe_pwm(1U<<c.ch_num, pwm);
        }
    }
}

/*
  setup failsafe value for an auxiliary function type to a Limit
 */
void
SRV_Channels::set_failsafe_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::Limit limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel &c = channels[i];
        if (c.function.get() == function) {
            uint16_t pwm = c.get_limit_pwm(limit);
            hal.rcout->set_failsafe_pwm(1U<<c.ch_num, pwm);
        }
    }
}

/*
  setup safety value for an auxiliary function type to a Limit
 */
void
SRV_Channels::set_safety_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::Limit limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel &c = channels[i];
        if (c.function.get() == function) {
            uint16_t pwm = c.get_limit_pwm(limit);
            hal.rcout->set_safety_pwm(1U<<c.ch_num, pwm);
        }
    }
}

/*
  set radio output value for an auxiliary function type to a Limit
 */
void
SRV_Channels::set_output_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::Limit limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function.get() == function) {
            uint16_t pwm = c.get_limit_pwm(limit);
            c.set_output_pwm(pwm);
            if (c.function.get() == SRV_Channel::k_manual) {
                RC_Channel *cin = rc().channel(c.ch_num);
                if (cin != nullptr) {
                    // in order for output_ch() to work for k_manual we
                    // also have to override radio_in
                    cin->set_radio_in(pwm);
                }
            }
        }
    }
}

/*
  return true if a particular function is assigned to at least one RC channel
 */
bool
SRV_Channels::function_assigned(SRV_Channel::Aux_servo_function_t function)
{
    return function_mask.get(uint16_t(function));
}

/*
  set servo_out and angle_min/max, then calc_pwm and output a
  value. This is used to move a AP_Mount servo
 */
void
SRV_Channels::move_servo(SRV_Channel::Aux_servo_function_t function,
                         int16_t value, int16_t angle_min, int16_t angle_max)
{
    if (!function_assigned(function)) {
        return;
    }
    if (angle_max <= angle_min) {
        return;
    }
    float v = float(value - angle_min) / float(angle_max - angle_min);
    v = constrain_float(v, 0.0f, 1.0f);
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function.get() == function) {
            float v2 = c.get_reversed()? (1-v) : v;
            uint16_t pwm = c.servo_min + v2 * (c.servo_max - c.servo_min);
            c.set_output_pwm(pwm);
        }
    }
}

/*
  set the default channel an auxiliary output function should be on
 */
bool SRV_Channels::set_aux_channel_default(SRV_Channel::Aux_servo_function_t function, uint8_t channel)
{
    if (!initialised) {
        update_aux_servo_function();
    }
    if (function_assigned(function)) {
        // already assigned
        return true;
    }
    if (channels[channel].function != SRV_Channel::k_none) {
        if (channels[channel].function == function) {
            return true;
        }
        hal.console->printf("Channel %u already assigned function %u\n",
                            (unsigned)(channel + 1),
                            (unsigned)channels[channel].function);
        return false;
    }
    channels[channel].type_setup = false;
    channels[channel].function.set(function);
    channels[channel].aux_servo_function_setup();
    function_mask.set((uint8_t)function);
    functions[function].channel_mask |= 1U<<channel;
    return true;
}

// find first channel that a function is assigned to
bool SRV_Channels::find_channel(SRV_Channel::Aux_servo_function_t function, uint8_t &chan)
{
    if (!initialised) {
        update_aux_servo_function();
    }
    if (!function_assigned(function)) {
        return false;
    }
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            chan = channels[i].ch_num;
            return true;
        }
    }
    return false;
}

/*
  get a pointer to first auxillary channel for a channel function
*/
SRV_Channel *SRV_Channels::get_channel_for(SRV_Channel::Aux_servo_function_t function, int8_t default_chan)
{
    uint8_t chan;
    if (default_chan >= 0) {
        set_aux_channel_default(function, default_chan);
    }
    if (!find_channel(function, chan)) {
        return nullptr;
    }
    return &channels[chan];
}

void SRV_Channels::set_output_scaled(SRV_Channel::Aux_servo_function_t function, int16_t value)
{
    if (function < SRV_Channel::k_nr_aux_servo_functions) {
        functions[function].output_scaled = value;
        SRV_Channel::have_pwm_mask &= ~functions[function].channel_mask;
    }
}

int16_t SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t function)
{
    if (function < SRV_Channel::k_nr_aux_servo_functions) {
        return functions[function].output_scaled;
    }
    return 0;
}

/*
  get mask of output channels for a function
 */
uint16_t SRV_Channels::get_output_channel_mask(SRV_Channel::Aux_servo_function_t function)
{
    if (!initialised) {
        update_aux_servo_function();
    }
    if (function < SRV_Channel::k_nr_aux_servo_functions) {
        return functions[function].channel_mask;
    }
    return 0;
}


// set the trim for a function channel to given pwm
void SRV_Channels::set_trim_to_pwm_for(SRV_Channel::Aux_servo_function_t function, int16_t pwm)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].servo_trim.set(pwm);
        }
    }
}

// set the trim for a function channel to min output
void SRV_Channels::set_trim_to_min_for(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].servo_trim.set(channels[i].get_reversed()?channels[i].servo_max:channels[i].servo_min);
        }
    }
}

/*
  set the default function for a channel
*/
void SRV_Channels::set_default_function(uint8_t chan, SRV_Channel::Aux_servo_function_t function)
{
    if (chan < NUM_SERVO_CHANNELS) {
        int8_t old = channels[chan].function;
        channels[chan].function.set_default((uint8_t)function);
        if (old != channels[chan].function && channels[chan].function == function) {
            function_mask.set((uint8_t)function);
        }
    }
}

void SRV_Channels::set_esc_scaling_for(SRV_Channel::Aux_servo_function_t function)
{
    uint8_t chan;
    if (find_channel(function, chan)) {
        hal.rcout->set_esc_scaling(channels[chan].get_output_min(), channels[chan].get_output_max());
    }
}

/*
  auto-adjust channel trim from an integrator value. Positive v means
  adjust trim up. Negative means decrease
 */
void SRV_Channels::adjust_trim(SRV_Channel::Aux_servo_function_t function, float v)
{
    if (is_zero(v)) {
        return;
    }
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (function != (SRV_Channel::Aux_servo_function_t)(c.function.get())) {
            continue;
        }
        float change = c.reversed?-v:v;
        uint16_t new_trim = c.servo_trim;
        float trim_scaled = float(c.servo_trim - c.servo_min) / (c.servo_max - c.servo_min);
        if (change > 0 && trim_scaled < 0.6f) {
            new_trim++;
        } else if (change < 0 && trim_scaled > 0.4f) {
            new_trim--;
        } else {
            return;
        }
        c.servo_trim.set(new_trim);

        trimmed_mask |= 1U<<i;
    }
}

// get pwm output for the first channel of the given function type.
bool SRV_Channels::get_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t &value)
{
    uint8_t chan;
    if (!find_channel(function, chan)) {
        return false;
    }
    channels[chan].calc_pwm(functions[function].output_scaled);
    value = channels[chan].get_output_pwm();
    return true;
}

// set output pwm to trim for the given function
void SRV_Channels::set_output_to_trim(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_output_pwm(channels[i].servo_trim);
        }
    }
}

// set output pwm to for first matching channel
void SRV_Channels::set_output_pwm_first(SRV_Channel::Aux_servo_function_t function, uint16_t pwm)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_output_pwm(pwm);
            break;
        }
    }
}

/*
  get the normalised output for a channel function from the pwm value
  of the first matching channel
 */
float SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t function)
{
    uint8_t chan;
    if (!find_channel(function, chan)) {
        return 0;
    }
    channels[chan].calc_pwm(functions[function].output_scaled);
    return channels[chan].get_output_norm();
}

/*
  limit slew rate for an output function to given rate in percent per
  second. This assumes output has not yet done to the hal
 */
void SRV_Channels::limit_slew_rate(SRV_Channel::Aux_servo_function_t function, float slew_rate, float dt)
{
    if (slew_rate <= 0) {
        // nothing to do
        return;
    }
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function == function) {
            c.calc_pwm(functions[function].output_scaled);
            uint16_t last_pwm = hal.rcout->read_last_sent(c.ch_num);
            if (last_pwm == c.get_output_pwm()) {
                continue;
            }
            uint16_t max_change = (c.get_output_max() - c.get_output_min()) * slew_rate * dt * 0.01f;
            if (max_change == 0 || dt > 1) {
                // always allow some change. If dt > 1 then assume we
                // are just starting out, and only allow a small
                // change for this loop
                max_change = 1;
            }
            c.set_output_pwm(constrain_int16(c.get_output_pwm(), last_pwm-max_change, last_pwm+max_change));
        }
    }
}

// call set_angle() on matching channels
void SRV_Channels::set_angle(SRV_Channel::Aux_servo_function_t function, uint16_t angle)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_angle(angle);
        }
    }    
}

// call set_range() on matching channels
void SRV_Channels::set_range(SRV_Channel::Aux_servo_function_t function, uint16_t range)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_range(range);
        }
    }
}

// set MIN parameter for a function
void SRV_Channels::set_output_min_max(SRV_Channel::Aux_servo_function_t function, uint16_t min_pwm, uint16_t max_pwm)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_output_min(min_pwm);
            channels[i].set_output_max(max_pwm);
        }
    }
}

// constrain to output min/max for function
void SRV_Channels::constrain_pwm(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function == function) {
            c.set_output_pwm(constrain_int16(c.output_pwm, c.servo_min, c.servo_max));
        }
    }
}

/*
  upgrade SERVO* parameters. This does the following:

   - update to 16 bit FUNCTION from AP_Int8
*/
void SRV_Channels::upgrade_parameters(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        // convert from AP_Int8 to AP_Int16
        c.function.convert_parameter_width(AP_PARAM_INT8);
    }
}

// set RC output frequency on a function output
void SRV_Channels::set_rc_frequency(SRV_Channel::Aux_servo_function_t function, uint16_t frequency_hz)
{
    uint16_t mask = 0;
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function == function) {
            mask |= (1U<<c.ch_num);
        }
    }
    if (mask != 0) {
        hal.rcout->set_freq(mask, frequency_hz);
    }
}
