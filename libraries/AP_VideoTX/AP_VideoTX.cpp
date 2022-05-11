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

#include "AP_VideoTX.h"
#include <AP_RCTelemetry/AP_CRSF_Telem.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_VideoTX *AP_VideoTX::singleton;

const AP_Param::GroupInfo AP_VideoTX::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Is the Video Transmitter enabled or not
    // @Description: Toggles the Video Transmitter on and off
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_VideoTX, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: POWER
    // @DisplayName: Video Transmitter Power Level
    // @Description: Video Transmitter Power Level. Different VTXs support different power levels, the power level chosen will be rounded down to the nearest supported power level
    // @Range: 1 1000
    AP_GROUPINFO("POWER",    2, AP_VideoTX, _power_mw, 0),

    // @Param: CHANNEL
    // @DisplayName: Video Transmitter Channel
    // @Description: Video Transmitter Channel
    // @User: Standard
    // @Range: 0 7
    AP_GROUPINFO("CHANNEL",  3, AP_VideoTX, _channel, 0),

    // @Param: BAND
    // @DisplayName: Video Transmitter Band
    // @Description: Video Transmitter Band
    // @User: Standard
    // @Values: 0:Band A,1:Band B,2:Band E,3:Airwave,4:RaceBand,5:Low RaceBand
    AP_GROUPINFO("BAND",  4, AP_VideoTX, _band, 0),

    // @Param: FREQ
    // @DisplayName: Video Transmitter Frequency
    // @Description: Video Transmitter Frequency. The frequency is derived from the setting of BAND and CHANNEL
    // @User: Standard
    // @ReadOnly: True
    // @Range: 5000 6000
    AP_GROUPINFO("FREQ",  5, AP_VideoTX, _frequency_mhz, 0),

    // @Param: OPTIONS
    // @DisplayName: Video Transmitter Options
    // @Description: Video Transmitter Options. Pitmode puts the VTX in a low power state. Unlocked enables certain restricted frequencies and power levels. Do not enable the Unlocked option unless you have appropriate permissions in your jurisdiction to transmit at high power levels.
    // @User: Advanced
    // @Bitmask: 0:Pitmode,1:Pitmode until armed,2:Pitmode when disarmed,3:Unlocked,4:Add leading zero byte to requests
    AP_GROUPINFO("OPTIONS",  6, AP_VideoTX, _options, 0),

    // @Param: MAX_POWER
    // @DisplayName: Video Transmitter Max Power Level
    // @Description: Video Transmitter Maximum Power Level. Different VTXs support different power levels, this prevents the power aux switch from requesting too high a power level. The switch supports 6 power levels and the selected power will be a subdivision between 0 and this setting.
    // @Range: 25 1000
    AP_GROUPINFO("MAX_POWER", 7, AP_VideoTX, _max_power_mw, 800),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

const char * AP_VideoTX::band_names[] = {"A","B","E","F","R","L"};

const uint16_t AP_VideoTX::VIDEO_CHANNELS[AP_VideoTX::MAX_BANDS][VTX_MAX_CHANNELS] =
{
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, /* Band A */
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, /* Band B */
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, /* Band E */
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, /* Airwave */
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, /* Race */
    { 5621, 5584, 5547, 5510, 5473, 5436, 5399, 5362}  /* LO Race */
};

AP_VideoTX::AP_VideoTX()
{
    if (singleton) {
        AP_HAL::panic("Too many VTXs");
        return;
    }
    singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

AP_VideoTX::~AP_VideoTX(void)
{
    singleton = nullptr;
}

bool AP_VideoTX::init(void)
{
    if (_initialized) {
        return false;
    }

    _current_power = _power_mw;
    _current_band = _band;
    _current_channel = _channel;
    _current_frequency = _frequency_mhz;
    _current_options = _options;
    _current_enabled = _enabled;
    _initialized = true;

    return true;
}

bool AP_VideoTX::get_band_and_channel(uint16_t freq, VideoBand& band, uint8_t& channel)
{
    for (uint8_t i = 0; i < AP_VideoTX::MAX_BANDS; i++) {
        for (uint8_t j = 0; j < VTX_MAX_CHANNELS; j++) {
            if (VIDEO_CHANNELS[i][j] == freq) {
                band = VideoBand(i);
                channel = j;
                return true;
            }
        }
    }
    return false;
}

// set the current power
void AP_VideoTX::set_configured_power_mw(uint16_t power) {
    _power_mw.set_and_save_ifchanged(power);
}

// set the power in dbm, rounding appropriately
void AP_VideoTX::set_power_dbm(uint8_t power) {
    switch (power) {
    case 14:
        _current_power = 25;
        break;
    case 20:
        _current_power = 100;
        break;
    case 23:
        _current_power = 200;
        break;
    case 26:
        _current_power = 400;
        break;
    case 27:
        _current_power = 500;
        break;
    case 29:
        _current_power = 800;
        break;
    default:
        _current_power = uint16_t(roundf(powf(10, power * 0.1f)));
        break;
    }
}

// get the power in dbm, rounding appropriately
uint8_t AP_VideoTX::get_configured_power_dbm() const {
    switch (_power_mw.get()) {
    case 25:    return 14;
    case 100:   return 20;
    case 200:   return 23;
    case 400:   return 26;
    case 500:   return 27;
    case 800:   return 29;
    default:
        return uint8_t(roundf(10.0f * log10f(_power_mw)));
    }
}

// get the power "level"
uint8_t AP_VideoTX::get_configured_power_level() const {
    if (_power_mw < 26) {
        return 0;
    } else if (_power_mw < 201) {
        return 1;
    } else if (_power_mw < 501) {
        return 2;
    } else {    // 800
        return 3;
    }
}

// set the power "level"
void AP_VideoTX::set_power_level(uint8_t level) {
    switch (level) {
    case 1:
        _current_power = 200;
        break;
    case 2:
        _current_power = 500;
        break;
    case 3:
        _current_power = 800;
        break;
    case 0:
    default:
        _current_power = 25;
        break;
    }
}

// set the current channel
void AP_VideoTX::set_enabled(bool enabled) {
    _current_enabled = enabled;
    if (!_enabled.configured()) {
        _enabled.set_and_save(enabled);
    }
}

// periodic update
void AP_VideoTX::update(void)
{
#if HAL_CRSF_TELEM_ENABLED
    AP_CRSF_Telem* crsf = AP::crsf_telem();

    if (crsf != nullptr) {
        crsf->update();
    }
#endif
    // manipulate pitmode if pitmode-on-disarm or power-on-arm is set
    if (has_option(VideoOptions::VTX_PITMODE_ON_DISARM) || has_option(VideoOptions::VTX_PITMODE_UNTIL_ARM)) {
        if (hal.util->get_soft_armed() && has_option(VideoOptions::VTX_PITMODE)) {
            _options &= ~uint8_t(VideoOptions::VTX_PITMODE);
        } else if (!hal.util->get_soft_armed() && !has_option(VideoOptions::VTX_PITMODE)
            && has_option(VideoOptions::VTX_PITMODE_ON_DISARM)) {
            _options |= uint8_t(VideoOptions::VTX_PITMODE);
        }
    }
}

bool AP_VideoTX::update_options() const
{
    if (!_defaults_set) {
        return false;
    }
    // check pitmode
    if ((_options & uint8_t(VideoOptions::VTX_PITMODE))
        != (_current_options & uint8_t(VideoOptions::VTX_PITMODE))) {
        return true;
    }

    // check unlock only
    if ((_options & uint8_t(VideoOptions::VTX_UNLOCKED)) != 0
        && (_current_options & uint8_t(VideoOptions::VTX_UNLOCKED)) == 0) {
        return true;
    }

    // ignore everything else
    return false;
}

bool AP_VideoTX::have_params_changed() const
{
    return _enabled
        && (update_power()
        || update_band()
        || update_channel()
        || update_frequency()
        || update_options());
}

// update the configured frequency to match the channel and band
void AP_VideoTX::update_configured_frequency()
{
    _frequency_mhz.set_and_save(get_frequency_mhz(_band, _channel));
}

// update the configured channel and band to match the frequency
void AP_VideoTX::update_configured_channel_and_band()
{
    VideoBand band;
    uint8_t channel;
    if (get_band_and_channel(_frequency_mhz, band, channel)) {
        _band.set_and_save(band);
        _channel.set_and_save(channel);
    } else {
        update_configured_frequency();
    }
}

// set the current configured values if not currently set in storage
// this is necessary so that the current settings can be seen
bool AP_VideoTX::set_defaults()
{
    if (_defaults_set) {
        return false;
    }

    // check that our current view of freqency matches band/channel
    // if not then force one to be correct
    uint16_t calced_freq = get_frequency_mhz(_current_band, _current_channel);
    if (_current_frequency != calced_freq) {
        if (_current_frequency > 0) {
            VideoBand band;
            uint8_t channel;
            if (get_band_and_channel(_current_frequency, band, channel)) {
                _current_band = band;
                _current_channel = channel;
            } else {
                _current_frequency = calced_freq;
            }
        } else {
            _current_frequency = calced_freq;
        }
    }

    if (!_options.configured()) {
        _options.set_and_save(_current_options);
    }
    if (!_channel.configured()) {
        _channel.set_and_save(_current_channel);
    }
    if (!_band.configured()) {
        _frequency_mhz.set_and_save(_current_band);
    }
    if (!_power_mw.configured()) {
        _power_mw.set_and_save(_current_power);
    }
    if (!_frequency_mhz.configured()) {
        _frequency_mhz.set_and_save(_current_frequency);
    }

    // Now check that the user didn't screw up by selecting incompatible options
    if (_frequency_mhz != get_frequency_mhz(_band, _channel)) {
        if (_frequency_mhz > 0) {
            update_configured_channel_and_band();
        } else {
            update_configured_frequency();
        }
    }

    _defaults_set = true;

    announce_vtx_settings();

    return true;
}

void AP_VideoTX::announce_vtx_settings() const
{
    // Output a friendly message so the user knows the VTX has been detected
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VTX: %s%d %dMHz, PWR: %dmW",
        band_names[_band.get()], _channel.get() + 1, _frequency_mhz.get(),
        has_option(VideoOptions::VTX_PITMODE) ? 0 : _power_mw.get());
}

// change the video power based on switch input
// 6-pos range is in the middle of the available range
void AP_VideoTX::change_power(int8_t position)
{
    if (position < 0 || position > 5) {
        return;
    }

    uint16_t power = 0;
    // 0 or 25
    if (_max_power_mw < 100) {
        switch (position) {
            case 3:
            case 4:
            case 5:
                power = 25;
                break;
            default:
                power = 0;
                break;
        }
    }
    // 0, 25 or 100
    else if (_max_power_mw < 200) {
        switch (position) {
            case 0:
                power = 0;
                break;
            case 5:
                power = 100;
                break;
            default:
                power = 25;
                break;
        }
    }
    // 0, 25, 100 or 200
    else if (_max_power_mw < 500) {
        switch (position) {
            case 1:
            case 2:
                power = 25;
                break;
            case 3:
            case 4:
                power = 100;
                break;
            case 5:
                power = 200;
                break;
            default:
                power = 0;
                break;
        }
    }
    // 0, 25, 100, 200 or 500
    else if (_max_power_mw < 800) {
        switch (position) {
            case 1:
            case 2:
                power = 25;
                break;
            case 3:
                power = 100;
                break;
            case 4:
                power = 200;
                break;
            case 5:
                power = 500;
                break;
            default:
                power = 0;
                break;
        }
    }
    // full range
    else {
        switch (position) {
            case 1:
                power = 25;
                break;
            case 2:
                power = 100;
                break;
            case 3:
                power = 200;
                break;
            case 4:
                power = 500;
                break;
            case 5:
                power = _max_power_mw; // some VTX's support 1000mw
                break;
            default:
                power = 0;
                break;
        }
    }

    if (power == 0) {
        set_configured_options(get_configured_options() | uint8_t(VideoOptions::VTX_PITMODE));
    } else {
        if (has_option(VideoOptions::VTX_PITMODE)) {
            set_configured_options(get_configured_options() & ~uint8_t(VideoOptions::VTX_PITMODE));
        }
        set_configured_power_mw(power);
    }
}

namespace AP {
    AP_VideoTX& vtx() {
        return *AP_VideoTX::get_singleton();
    }
};
