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

#include <stdlib.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "RC_Channel.h"

/*
  channels group object constructor
 */
RC_Channels::RC_Channels(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("RC_Channels must be singleton");
    }
    _singleton = this;
}

// if none of the basic control options (as defined by the things
// we set defaults for) have been set, then assume we've just
// upgraded from an RCMAP-using firmware, and set-and-save options
// from the old RCMAP values.
void RC_Channels::populate_channel_options_from_old_rcmap()
{
    uint8_t k_param_rcmap;
    if (!k_param_rcmap_for_conversion(k_param_rcmap)) {
        return;
    }

    // set defaults:
    const RC_Channels::OptionDefault *defaults;
    uint8_t default_count;

    get_option_defaults(defaults, default_count);

    for (uint8_t i=0; i<default_count; i++) {
        const struct RC_Channels::OptionDefault &def = defaults[i];
        for (uint8_t j=0; j<NUM_RC_CHANNELS; j++) {
            if (channel(j)->option.get() == (uint16_t)def.func) {
                // found at least one option set
                return;
            }
        }
    }

    // this array maps from the old RCMAP_ parameters onto the
    // equivalent RCn_OPTION value.  The offset within this list gives
    // the old offset within the RCMAP_ parameter list, so YAW was:
    // AP_GROUPINFO("YAW",         3, RCMapper, _ch_yaw, 4),

    static const RC_Channel::AUX_FUNC mapping[] = {
        RC_Channel::AUX_FUNC::ROLL,
        RC_Channel::AUX_FUNC::PITCH,
        RC_Channel::AUX_FUNC::THROTTLE,
        RC_Channel::AUX_FUNC::YAW,
        RC_Channel::AUX_FUNC::FORWARD,
        RC_Channel::AUX_FUNC::LATERAL
    };
    for (uint8_t i=0; i<ARRAY_SIZE(mapping); i++) {
        const AP_Param::ConversionInfo cinfo_ret {
            k_param_rcmap,
            i,
            AP_PARAM_INT8,
            nullptr
        };
        AP_Int8 old_rcmap_parameter;
        if (!AP_Param::find_old_parameter(&cinfo_ret, &old_rcmap_parameter)) {
            continue;
        }
        const uint8_t old_channel_number = old_rcmap_parameter.get();
        RC_Channel *_ch = channel(old_channel_number-1);
        if (_ch == nullptr) {
            continue;
        }
        _ch->option.set_and_save((uint16_t)mapping[i]);
    }
}

void RC_Channels::init(void)
{
    // setup ch_in on channels
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        channel(i)->ch_in = i;
    }

    // run an upgrade to convert from old RCMAP_ parameters into
    // rc-channel-options:
    populate_channel_options_from_old_rcmap();

    // set defaults:
    set_channel_options_from_defaults();

    init_aux_all();
}

void RC_Channels::set_channel_options_from_defaults()
{
    const RC_Channels::OptionDefault *defaults;
    uint8_t default_count;

    get_option_defaults(defaults, default_count);

    for (uint8_t i=0; i<default_count; i++) {
        const struct RC_Channels::OptionDefault &def = defaults[i];
        // only set the default if this function isn't assigned to
        // another channel.  This may happen if someone has upgraded
        // from an older firmware and was using a non-1-4 channel for
        // RC input.
        bool found = false;
        for (uint8_t j=0; j<NUM_RC_CHANNELS; j++) {
            if (channel(j)->option.get() == (uint16_t)def.func) {
                found = true;
                break;
            }
        }
        if (found) {
            continue;
        }
        AP_Int16 &option = rc_channel(def.channel-1)->option;
        if (!option.configured_in_storage()) {
            option.set((uint16_t)def.func);
        }
    }
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
    if (hal.rcin->new_input()) {
        _has_had_rc_receiver = true;
    } else if (!has_new_overrides) {
        return false;
    }

    has_new_overrides = false;

    last_update_ms = AP_HAL::millis();

    bool success = false;
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        success |= channel(i)->update();
    }

    return success;
}

uint8_t RC_Channels::get_valid_channel_count(void)
{
    return MIN(NUM_RC_CHANNELS, hal.rcin->num_channels());
}

int16_t RC_Channels::get_receiver_rssi(void)
{
    return hal.rcin->get_rssi();
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

uint16_t RC_Channels::get_override_mask(void)
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

bool RC_Channels::receiver_bind(const int dsmMode)
{
    return hal.rcin->rc_bind(dsmMode);
}


// support for auxillary switches:
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
    if (need_log) {
        // guarantee that we log when a switch changes
        AP::logger().Write_RCIN();
    }
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
    reset_mode_switch();
}

//
// Support for mode switches
//
RC_Channel *RC_Channels::flight_mode_channel() const
{
    const int8_t num = flight_mode_channel_number();
    if (num <= 0) {
        return nullptr;
    }
    if (num >= NUM_RC_CHANNELS) {
        return nullptr;
    }
    return rc_channel(num-1);
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

// check if flight mode channel is assigned RC option
// return true if assigned
bool RC_Channels::flight_mode_channel_conflicts_with_rc_option() const
{
    RC_Channel *chan = flight_mode_channel();
    if (chan == nullptr) {
        return false;
    }
    return (RC_Channel::aux_func_t)chan->option.get() != RC_Channel::AUX_FUNC::DO_NOTHING;
}

/*
  get the RC input PWM value given a channel number.  Note that
  channel numbers start at 1, as this API is designed for use in
  LUA
*/
bool RC_Channels::get_pwm(uint8_t c, uint16_t &pwm) const
{
    RC_Channel *chan = rc_channel(c-1);
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

void RC_Channels::init_channel(RC_Channel *&_channel,
                               RC_Channel::AUX_FUNC func,
                               const char *function_name)
{
    _channel = find_channel_for_option(func);
    if (_channel == nullptr) {
        AP_BoardConfig::config_error("RC %s unset; need %u RCn_OPTION", function_name, (unsigned)func);
    }
}

// singleton instance
RC_Channels *RC_Channels::_singleton;


RC_Channels &rc()
{
    return *RC_Channels::get_singleton();
}
