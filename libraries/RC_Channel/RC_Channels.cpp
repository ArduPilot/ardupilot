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
 *       RC_Channels.cpp - class containing an array of RC_Channel objects
 *
 */

#include "RC_Channel_config.h"

#if AP_RC_CHANNEL_ENABLED

#include <stdlib.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#include "RC_Channel.h"

#include <AP_Arming/AP_Arming.h>

/*
  channels group object constructor
 */
RC_Channels::RC_Channels(void) :
    override_start_throttle(-1)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("RC_Channels must be singleton");
    }
    _singleton = this;

}

void RC_Channels::init_channel_numbers()
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        channel(i)->ch_in = i;
    }
}

void RC_Channels::set_control_channel_defaults()
{
    // Plane needs the channel numbers early!
    init_channel_numbers();

    set_control_channel_default(0, RC_Channel::AUX_FUNC::ROLL);
    set_control_channel_default(1, RC_Channel::AUX_FUNC::PITCH);
    set_control_channel_default(2, RC_Channel::AUX_FUNC::THROTTLE);
    set_control_channel_default(3, RC_Channel::AUX_FUNC::YAW);
}

void RC_Channels::set_control_channel_default(uint8_t chan, RC_Channel::AUX_FUNC func)
{
    RC_Channel *c = channel(chan);
    if (c == nullptr) {
        return;
    }
    if (_conversion_stale_do_nothing.get(chan)) {
        // an RCn_OPTION stored as DO_NOTHING before the RCMAP_
        // conversion said nothing about control inputs, but stops
        // set_default() applying.  Apply the default regardless;
        // convert_rcmap_parameters() saves it if it survives
        c->option.set((uint16_t)func);
        return;
    }
    c->option.set_default((uint16_t)func);
}

void RC_Channels::init(void)
{
    // vehicles have done this in set_control_channel_defaults();
    // examples call only init()
    init_channel_numbers();

    init_aux_all();
}

bool RC_Channels::has_valid_input() const
{
    // the vehicles override this method and check many more
    // things, but also call this method:
    if (!has_ever_seen_rc_input()) {
        return false;
    }

    return true;
}

uint8_t RC_Channels::get_radio_in(uint16_t *chans, const uint8_t num_channels)
{
    memset(chans, 0, num_channels*sizeof(*chans));

    const uint8_t read_channels = MIN(num_channels, NUM_RC_CHANNELS);
    for (uint8_t i = 0; i < read_channels; i++) {
        chans[i] = channel(i)->get_radio_in();
    }

    return read_channels;
}

// update all the input channels
bool RC_Channels::read_input(void)
{
    if (hal.rcin->new_input() &&
        !rc().option_is_enabled(RC_Channels::Option::IGNORE_RECEIVER)) {
        _has_had_rc_receiver = true;
    } else if (!has_new_overrides) {
        return false;
    }

    _has_ever_seen_rc_input = true;

    has_new_overrides = false;

    last_update_ms = AP_HAL::millis();

    bool success = false;
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        success |= channel(i)->update();
    }

    if (success) {
        rudder_arm_disarm_check();

        // check if RC overrides should be ignored based on RC_OPTIONS and any pilot input change during active overrides
        if (should_ignore_overrides()) {
            set_gcs_overrides_enabled(false);
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "RC overrides cleared by pilot input");
        }
    }

    return success;
}

bool RC_Channels::should_ignore_overrides(void)
{
    if (!rc().option_is_enabled(Option::CLEAR_OVERRIDES_BY_RC) || !has_active_overrides()) {
        return false;
    }
    return has_pilot_input_for_override_clear();
}

bool RC_Channels::channel_outside_trim_dz(const RC_Channel &ch)
{
    return !ch.in_raw_trim_dz();
}

bool RC_Channels::throttle_moved_since_override_start() const
{
    if (override_start_throttle < 0) {
        return false;
    }

    const RC_Channel &thr = get_throttle_channel();
    return abs(thr.get_raw_radio_in() - override_start_throttle) > thr.get_dead_zone();
}

bool RC_Channels::has_pilot_input_for_override_clear()
{
    if (channel_outside_trim_dz(get_roll_channel()) ||
        channel_outside_trim_dz(get_pitch_channel()) ||
        channel_outside_trim_dz(get_yaw_channel())) {
        return true;
    }

    if (throttle_moved_since_override_start()) {
        return true;
    }

    return false;
}

uint8_t RC_Channels::get_valid_channel_count(void)
{
    return MIN(NUM_RC_CHANNELS, hal.rcin->num_channels());
}

int16_t RC_Channels::get_receiver_rssi(void)
{
    return hal.rcin->get_rssi();
}
int16_t RC_Channels::get_receiver_link_quality(void)
{
    return hal.rcin->get_rx_link_quality();
}
void RC_Channels::clear_overrides(void)
{
    RC_Channels &_rc = rc();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        _rc.channel(i)->clear_override();
    }
    // we really should set has_new_overrides to true, and rerun read_input from
    // the vehicle code however doing so currently breaks the failsafe system on
    // copter and plane, RC_Channels needs to control failsafes to resolve this
}

uint16_t RC_Channels::get_override_mask(void) const
{
    uint16_t ret = 0;
    RC_Channels &_rc = rc();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        if (_rc.channel(i)->has_override()) {
            ret |= (1U << i);
        }
    }
    return ret;
}

void RC_Channels::set_override(const uint8_t chan, const int16_t value, const uint32_t timestamp_ms)
{
    RC_Channels &_rc = rc();
    if (chan < NUM_RC_CHANNELS) {
        _rc.channel(chan)->set_override(value, timestamp_ms);
    }
}

bool RC_Channels::has_active_overrides()
{
    RC_Channels &_rc = rc();
    for (uint8_t i = 0; i < NUM_RC_CHANNELS; i++) {
        if (_rc.channel(i)->has_override()) {
            return true;
        }
    }

    return false;
}


// support for auxiliary switches:
// read_aux_switches - checks aux switch positions and invokes configured actions
void RC_Channels::read_aux_all()
{
    if (!has_valid_input()) {
        // exit immediately when no RC input
        return;
    }
    bool need_log = false;

    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            continue;
        }
        need_log |= c->read_aux();
    }
#if HAL_LOGGING_ENABLED
    if (need_log) {
        // guarantee that we log when a switch changes
        AP::logger().Write_RCIN();
    }
#endif
}

void RC_Channels::init_aux_all()
{
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        RC_Channel *c = channel(i);
        if (c == nullptr) {
            continue;
        }
        c->init_aux();
    }
    // the mode channel is intentionally only looked up at boot;
    // changing which RCn_OPTION is set to Mode requires a reboot
    cached_flight_mode_channel = find_channel_for_option(RC_Channel::AUX_FUNC::MODE);
    reset_mode_switch();
}

// PARAMETER_CONVERSION - Added: Apr-2026 for ArduPilot-4.8
// convert from e.g. FLTMODE_CH=5 to RC5_OPTION=Mode, once.  If the old
// parameter was saved then its RCn_OPTION is set to Mode regardless of
// its current value, as the mode channel used to take precedence.  If
// the old parameter was never saved then default_mode_channel is used;
// this is what gives a fresh install its default mode channel.
void RC_Channels::convert_old_fltmode_ch(uint16_t old_key, uint8_t default_mode_channel)
{
    if (_mode_channel_converted == 1) {
        return;
    }

    const AP_Param::ConversionInfo mode_channel_info{
        old_key,
        0,  // old_group_element
        AP_PARAM_INT8,
        "UNUSED"
    };
    int8_t new_mode_channel = default_mode_channel;
    AP_Int8 mode_channel_old;
    const bool found_old = AP_Param::find_old_parameter(&mode_channel_info, &mode_channel_old);
    if (found_old) {
        new_mode_channel = mode_channel_old.get();
    } else if (find_channel_for_option(RC_Channel::AUX_FUNC::MODE) != nullptr) {
        // not explicitly set and e.g. a defaults file has already
        // nominated a mode channel
        new_mode_channel = 0;
    }

    // a channel number below 1 means no mode channel; an out-of-range
    // one means the old parameter held an invalid value
    RC_Channel *c = nullptr;
    if (new_mode_channel >= 1) {
        c = channel(new_mode_channel - 1);
    }

    bool read_only;
    if (c != nullptr && !found_old &&
        (c->option.configured_in_defaults_file(read_only) ||
         (c->option.configured_in_storage() &&
          RC_Channel::AUX_FUNC(c->option.get()) != RC_Channel::AUX_FUNC::DO_NOTHING))) {
        // the default mode channel is already configured to do
        // something, e.g. by a board's defaults file or by the RCMAP_
        // conversion having put a control input on it.  The vehicle
        // ends up with no mode channel and the user nominates one with
        // RCn_OPTION.  Nothing usable is lost: the old mode-channel
        // pre-arm check refused to arm with an option on the mode
        // channel, and a control stick doubling as the six-position
        // mode switch was never flyable.  A DO_NOTHING stored before
        // this conversion (an option set and then cleared) does not
        // count as configured: that channel was still the mode switch
        c = nullptr;
    }

    if (found_old && new_mode_channel < 1) {
        // mode switching had been disabled.  A defaults file nominating
        // a Mode channel must not re-enable it, and with a single Mode
        // channel there is no duplicate for the pre-arm check to catch,
        // so the default is displaced
        for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
            RC_Channel *other = channel(i);
            if (other == nullptr ||
                RC_Channel::AUX_FUNC(other->option.get()) != RC_Channel::AUX_FUNC::MODE ||
                other->option.configured_in_storage()) {
                continue;
            }
            // force the save as DO_NOTHING is the parameter default
            other->option.set((uint16_t)RC_Channel::AUX_FUNC::DO_NOTHING);
            other->option.save(true);
        }
    }

    if (c != nullptr) {
        // a stored old parameter takes precedence over whatever option
        // the channel had, as the mode channel used to; such a
        // configuration could not arm, or was never flyable if the
        // option was a control input, see above.  A defaults file
        // nominating a different Mode channel is deliberately left in
        // place; the duplicate-options pre-arm check then reports that
        // the board's defaults and the stored parameter disagree,
        // which the user resolves by clearing one of them
        c->option.set_and_save(int16_t(RC_Channel::AUX_FUNC::MODE));
    }

    // deciding there is no mode channel is also a completed
    // conversion.  The flag is saved last so an interrupted conversion
    // is retried on the next boot
    _mode_channel_converted.set_and_save(1);
}

//
// Support for mode switches
//
RC_Channel *RC_Channels::flight_mode_channel() const
{
    return cached_flight_mode_channel;
}

// returns true if the channel with RCn_OPTION set to Mode is not the
// one found at boot
bool RC_Channels::flight_mode_channel_changed()
{
    return find_channel_for_option(RC_Channel::AUX_FUNC::MODE) != cached_flight_mode_channel;
}

void RC_Channels::reset_mode_switch()
{
    RC_Channel *c = flight_mode_channel();
    if (c == nullptr) {
        return;
    }
    c->reset_mode_switch();
}

void RC_Channels::read_mode_switch()
{
    if (!has_valid_input()) {
        // exit immediately when no RC input
        return;
    }
    RC_Channel *c = flight_mode_channel();
    if (c == nullptr) {
        return;
    }
    c->read_mode_switch();
}

/*
  get the RC input PWM value given a channel number.  Note that
  channel numbers start at 1, as this API is designed for use in
  LUA
*/
bool RC_Channels::get_pwm(uint8_t c, uint16_t &pwm) const
{
    const RC_Channel *chan = channel(c-1);
    if (chan == nullptr) {
        return false;
    }
    int16_t pwm_signed = chan->get_radio_in();
    if (pwm_signed < 0) {
        return false;
    }
    pwm = (uint16_t)pwm_signed;
    return true;
}

// return mask of enabled protocols.
uint32_t RC_Channels::enabled_protocols() const
{
    if (_singleton == nullptr) {
        // for example firmware
        return 1U;
    }
    return uint32_t(_protocols.get());
}

#if AP_SCRIPTING_ENABLED
/*
  implement aux function cache for scripting
 */

/*
  get last aux cached value for scripting. Returns false if never set, otherwise 0,1,2
*/
bool RC_Channels::get_aux_cached(RC_Channel::AUX_FUNC aux_fn, uint8_t &pos)
{
    const uint16_t aux_idx = uint16_t(aux_fn);
    if (aux_idx >= unsigned(RC_Channel::AUX_FUNC::AUX_FUNCTION_MAX)) {
        return false;
    }
    WITH_SEMAPHORE(aux_cache_sem);
    uint8_t v = aux_cached.get(aux_idx*2) | (aux_cached.get(aux_idx*2+1)<<1);
    if (v == 0) {
        // never been set
        return false;
    }
    pos = v-1;
    return true;
}

/*
  set cached value of an aux function
 */
void RC_Channels::set_aux_cached(RC_Channel::AUX_FUNC aux_fn, RC_Channel::AuxSwitchPos pos)
{
    const uint16_t aux_idx = uint16_t(aux_fn);
    if (aux_idx < unsigned(RC_Channel::AUX_FUNC::AUX_FUNCTION_MAX)) {
        WITH_SEMAPHORE(aux_cache_sem);
        uint8_t v = unsigned(pos)+1;
        aux_cached.setonoff(aux_idx*2, v&1);
        aux_cached.setonoff(aux_idx*2+1, v>>1);
    }
}
#endif // AP_SCRIPTING_ENABLED

// these methods return an RC_Channel reference based on which
// channel has been assigned the relevant RCn_OPTION.  The return
// value is guaranteed to be a valid channel to allow use without
// checking for null-ness.  If no channel has been assigned the
// option then the returned channel will be a dummy channel.
static RC_Channel dummy_rcchannel;
const RC_Channel &RC_Channels::get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC func) const
{
    const RC_Channel *ret = find_channel_for_option(func);
    if (ret != nullptr) {
        return *ret;
    }
    return dummy_rcchannel;
}
RC_Channel &RC_Channels::get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC func)
{
    RC_Channel *ret = find_channel_for_option(func);
    if (ret != nullptr) {
        return *ret;
    }
    return dummy_rcchannel;
}
const RC_Channel &RC_Channels::get_roll_channel() const
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::ROLL);
};
RC_Channel &RC_Channels::get_roll_channel()
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::ROLL);
};
const RC_Channel &RC_Channels::get_pitch_channel() const
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::PITCH);
};
RC_Channel &RC_Channels::get_pitch_channel()
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::PITCH);
};
const RC_Channel &RC_Channels::get_throttle_channel() const
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::THROTTLE);
};
RC_Channel &RC_Channels::get_throttle_channel()
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::THROTTLE);
};
const RC_Channel &RC_Channels::get_yaw_channel() const
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::YAW);
};
RC_Channel &RC_Channels::get_yaw_channel()
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::YAW);
};
const RC_Channel &RC_Channels::get_forward_channel() const
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::FWD_THR);
};
RC_Channel &RC_Channels::get_forward_channel()
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::FWD_THR);
};
const RC_Channel &RC_Channels::get_lateral_channel() const
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::LATERAL_THR);
};
RC_Channel &RC_Channels::get_lateral_channel()
{
    return get_rcmap_channel_nonnull(RC_Channel::AUX_FUNC::LATERAL_THR);
};


/*
  check for pilot input on rudder stick for arming/disarming
*/
void RC_Channels::rudder_arm_disarm_check()
{
    // run no more code if arm/disarm via rudder input channel is
    // completely disabled.  Further checks using this parameter are
    // done below.
    if (AP::arming().get_rudder_arming_type() == AP_Arming::RudderArming::IS_DISABLED) {
        return;
    }

    const RC_Channel *channel = get_arming_channel();
    if (channel == nullptr) {
        return;
    }

    const auto control_in = channel->get_control_in();
    const auto abs_control_in = abs(control_in);

    if (abs_control_in == 0) {
        have_seen_neutral_rudder = true;
    }

    if (abs_control_in <= 4000) {
        // not trying to (or no longer trying to) arm or disarm
        rudder_arm_timer = 0;
        return;
    }

    // enforce correct stick gesture for arming (but not disarming):
    if (arming_check_throttle() && control_in > 4000) {
        // only permit arming if the vehicle isn't being commanded to
        // move via RC input
        const auto &c = rc().get_throttle_channel();
        if (c.get_control_in() != 0) {
            rudder_arm_timer = 0;
            return;
        }
    }

    const uint32_t now = AP_HAL::millis();
    if (rudder_arm_timer == 0) {
        // first time we've seen the attempt
        rudder_arm_timer = now;
        return;
    }

    if (now - rudder_arm_timer < 3000) {
        // not time yet....
        return;
    }

    // time to try to arm or disarm:
    rudder_arm_timer = 0;
    if (control_in > 4000) {
        AP::arming().arm(AP_Arming::Method::RUDDER);
        have_seen_neutral_rudder = false;
    } else {
        if (AP::arming().get_rudder_arming_type() == AP_Arming::RudderArming::ARMDISARM) {
            AP::arming().disarm(AP_Arming::Method::RUDDER);
        }
    }
}

// singleton instance
RC_Channels *RC_Channels::_singleton;


RC_Channels &rc()
{
    return *RC_Channels::get_singleton();
}

#endif  // AP_RC_CHANNEL_ENABLED
