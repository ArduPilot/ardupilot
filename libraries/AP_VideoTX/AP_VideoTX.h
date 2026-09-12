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
#pragma once

#include "AP_VideoTX_config.h"

#if AP_VIDEOTX_ENABLED

#include <AP_Param/AP_Param.h>

#define VTX_MAX_CHANNELS 8
#define VTX_MAX_POWER_LEVELS 10
// number of user-definable power table entries (VTX_PWRTBL1 to VTX_PWRTBL6),
// sized to match the 6 positions of the power aux switch
#define VTX_USER_POWER_LEVELS 6

class AP_VideoTX {
public:
    AP_VideoTX();
    ~AP_VideoTX();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_VideoTX);

    // init - perform required initialisation
    bool init();

    // run any required updates
    void update();

    static AP_VideoTX *get_singleton(void) {
        return singleton;
    }
    static const struct AP_Param::GroupInfo var_info[];

    enum class VideoOptions {
        VTX_PITMODE           = (1 << 0),
        VTX_PITMODE_UNTIL_ARM = (1 << 1),
        VTX_PITMODE_ON_DISARM = (1 << 2),
        VTX_UNLOCKED          = (1 << 3),
        VTX_PULLDOWN          = (1 << 4),
        VTX_SA_ONE_STOP_BIT   = (1 << 5),
        VTX_SA_IGNORE_CRC     = (1 << 6),
        VTX_CRSF_IGNORE_STAT  = (1 << 7),
    };

    static const char *band_names[];

    enum VideoBand {
        BAND_A,
        BAND_B,
        BAND_E,
        FATSHARK,
        RACEBAND,
        LOW_RACEBAND,
        BAND_1G3_A,
        BAND_1G3_B,
        BAND_X,
        BAND_3G3_A,
        BAND_3G3_B,
        MAX_BANDS
    };

    enum class PowerActive {
        Unknown,
        Active,
        Inactive
    };

    enum VTXType {
        CRSF       = 1U<<0,
        SmartAudio = 1U<<1,
        Tramp      = 1U<<2,
        MSP        = 1U<<3
    };

    struct PowerLevel {
        uint8_t level;
        uint16_t mw;
        uint8_t dbm;
        uint8_t dac; // SmartAudio v1 dac value
        PowerActive active;
    };

    static PowerLevel _power_levels[VTX_MAX_POWER_LEVELS];

    static const uint16_t VIDEO_CHANNELS[MAX_BANDS][VTX_MAX_CHANNELS];

    static uint16_t get_frequency_mhz(uint8_t band, uint8_t channel) { return VIDEO_CHANNELS[band][channel]; }
    static bool get_band_and_channel(uint16_t freq, VideoBand& band, uint8_t& channel);

    void set_frequency_mhz(uint16_t freq) { _current_frequency = freq; }
    void set_configured_frequency_mhz(uint16_t freq) { _frequency_mhz.set_and_save_ifchanged(freq); }
    uint16_t get_frequency_mhz() const { return _current_frequency; }
    uint16_t get_configured_frequency_mhz() const { return _frequency_mhz; }
    bool update_frequency() const { return _defaults_set && _frequency_mhz != _current_frequency; }
    void update_configured_frequency();
    // get / set power level
    void set_power_mw(uint16_t power);
    void set_power_level(uint8_t level, PowerActive active=PowerActive::Active);
    void set_power_dbm(uint8_t power, PowerActive active=PowerActive::Active);
    void set_power_dac(uint16_t power, PowerActive active=PowerActive::Active);
    // add a new dbm setting to those supported
    uint8_t update_power_dbm(uint8_t power, PowerActive active=PowerActive::Active);
    void update_all_power_dbm(uint8_t nlevels, const uint8_t levels[]);
    void set_configured_power_mw(uint16_t power);
    uint16_t get_configured_power_mw() const { return _power_mw; }
    uint16_t get_power_mw() const { return _power_levels[_current_power].mw; }
    // VTX-reported actual power; -1 if the provider doesn't report it, 0 is pit mode
    void set_actual_power_mw(uint16_t power) { _actual_power_mw = int32_t(power); }
    int32_t get_actual_power_mw() const { return _actual_power_mw; }
    uint16_t get_max_power_mw() const { return _max_power_mw; }

    // user-defined power table (VTX_PWRTBL_EN, VTX_PWRTBL1 to VTX_PWRTBL6).
    // These parameters are the table: _power_levels describes the SmartAudio
    // levels and is not extended with user values.
    // raw table entry i: a value of -1 ignores the entry, 0 is pit mode,
    // otherwise the power in mW. There is no unsigned parameter type, so this
    // is the one place the signed parameter is read; everything downstream is
    // uint16_t
    int16_t get_table_entry(uint8_t i) const {
        if (i >= VTX_USER_POWER_LEVELS) {
            return -1;
        }
        return _power_table[i].get();
    }
    // true when the user has enabled the user power table
    bool use_power_table() const;
    // highest power the user has authorised: the top table entry when the user
    // table is in use, otherwise VTX_MAX_POWER
    uint16_t get_power_cap_mw() const;

    // get the power in dbm, rounding appropriately
    uint8_t get_configured_power_dbm() const;
    // get the power "level"
    uint8_t get_configured_power_level() const;
    // get the power "dac"
    uint8_t get_configured_power_dac() const;

    // mark the power level matching the given mW as supported, learning it
    // into the custom slot if it is not a standard value
    void update_power_mw(uint16_t power_mw, PowerActive active = PowerActive::Active);
    // a provider's power index is one based and refers to the supported (active)
    // levels in ascending order, so index 1 is the lowest supported level
    uint8_t get_num_power_levels() const;
    // mW for a one based power index, 0 if not known
    uint16_t get_power_mw_for_index(uint8_t index) const;
    // one based power index for a mW value, 0 if not matched
    uint8_t get_power_index_for_mw(uint16_t power_mw) const;

    bool update_power() const;
    // change the video power based on switch input
    void change_power(int8_t position);
    // the power in mW that the power aux switch (RCx_OPTION 94) selects for the
    // given switch position, 0 meaning pitmode. user_table holds the raw
    // VTX_PWRTBL values: a value of -1 ignores the entry, a 0 entry selects pit
    // mode at that position. The used entries in slot order are the switch's
    // only choices; when fewer than six are used the six positions subdivide
    // over them. Returns false when there is nothing to select. Static and
    // side-effect free so the mapping can be unit tested
    static bool switch_power_mw(int8_t position,
                                const int16_t *user_table, uint8_t user_table_len,
                                bool user_table_enabled, uint16_t max_power_mw,
                                uint16_t &power_mw);
    // get / set the frequency band
    void set_band(uint8_t band) { _current_band = band; }
    void set_configured_band(uint8_t band) { _band.set_and_save_ifchanged(band); }
    uint8_t get_configured_band() const { return _band; }
    uint8_t get_band() const { return _current_band; }
    bool update_band() const { return _defaults_set && _band != _current_band; }
    // get / set the frequency channel
    void set_channel(uint8_t channel) { _current_channel = channel; }
    void set_configured_channel(uint8_t channel) { _channel.set_and_save_ifchanged(channel); }
    uint8_t get_configured_channel() const { return _channel; }
    uint8_t get_channel() const { return _current_channel; }
    bool update_channel() const { return _defaults_set && _channel != _current_channel; }
    void update_configured_channel_and_band();
    // get / set vtx option
    void set_options(uint16_t options) { _current_options = options; }
    void set_configured_options(uint16_t options) { _options.set_and_save_ifchanged(options); }
    uint16_t get_configured_options() const { return _options; }
    uint16_t get_options() const { return _current_options; }
    bool has_option(VideoOptions option) const { return _options.get() & uint16_t(option); }
    // set or clear a single configured option bit, leaving the others untouched
    void set_option_enabled(VideoOptions option, bool enabled) {
        set_configured_options(enabled ? (get_configured_options() | uint16_t(option))
                                       : (get_configured_options() & ~uint16_t(option)));
    }
    bool get_configured_pitmode() const { return _options.get() & uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE); }
    bool get_pitmode() const { return _current_options & uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE); }
    bool update_options() const;
    // get / set whether the vtx is enabled
    void set_enabled(bool enabled);
    bool get_enabled() const { return _enabled; }
    bool update_enabled() const { return _defaults_set && _enabled != _current_enabled; }

    // have the parameters been updated
    bool have_params_changed() const;
    // set configured defaults from current settings, return true if defaults were set
    bool set_defaults();
    // display the current VTX settings in the GCS
    void announce_vtx_settings() const;
    // force the current values to reflect the configured values
    void set_power_is_current();
    void set_freq_is_current();
    void set_options_are_current() {  _current_options = _options; }

    void set_configuration_finished(bool configuration_finished) { _configuration_finished = configuration_finished; }
    bool is_configuration_finished() { return _configuration_finished; }

    // manage VTX backends
    bool is_provider_enabled(VTXType type) const { return (_types & type) != 0; }
    // a provider may only register if the user allows its type (VTX_TYPES)
    void set_provider_enabled(VTXType type) { if (is_type_enabled(type)) { _types |= type; } }
    // is this control transport allowed to manage the VTX
    bool is_type_enabled(VTXType type) const { return (uint8_t(_types_allowed) & uint8_t(type)) != 0; }

    static AP_VideoTX *singleton;

private:
    uint8_t find_current_power() const;
    // index of the configured power's SmartAudio level slot: an exact match
    // first, otherwise the nearest lower built-in level, floored at the 25mW
    // slot for any positive power. The custom learned slot is skipped - its
    // level and dac values are Tramp bookkeeping, not SmartAudio levels - so a
    // user-table power always yields a wire-valid level byte
    uint8_t find_nearest_power_level() const;
    // channel frequency
    AP_Int16 _frequency_mhz;
    uint16_t _current_frequency;

    // power output in mw
    AP_Int16 _power_mw;
    uint16_t _current_power {0};
    int32_t _actual_power_mw {-1};
    AP_Int16 _max_power_mw;

    // user-defined power table
    AP_Int8 _power_table_enabled;
    AP_Int16 _power_table[VTX_USER_POWER_LEVELS];

    // frequency band
    AP_Int8 _band;
    uint16_t _current_band;

    // frequency channel
    AP_Int8 _channel;
    uint8_t _current_channel;

    // vtx options
    AP_Int16 _options;
    uint16_t _current_options;

    AP_Int8 _enabled;
    bool _current_enabled;

    // bitmask of VTXType control transports the user permits (VTX_TYPES)
    AP_Int8 _types_allowed;

    bool _initialized;
    // when defaults have been configured
    bool _defaults_set;
    // true when configuration have been applied successfully to the VTX
    bool _configuration_finished;

    // types of VTX providers
    uint8_t _types;
};

namespace AP {
    AP_VideoTX& vtx();
};

#endif  // AP_VIDEOTX_ENABLED
