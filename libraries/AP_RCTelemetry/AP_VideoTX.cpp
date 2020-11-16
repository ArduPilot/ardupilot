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
#include "AP_CRSF_Telem.h"
#include <GCS_MAVLink/GCS.h>

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
    // @Description: Video Transmitter Options.
    // @User: Advanced
    // @Bitmask: 0:Pitmode
    AP_GROUPINFO("OPTIONS",  6, AP_VideoTX, _options, 0),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

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
        _current_power = uint16_t(roundf(powf(10, power / 10.0f)));
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

// set the current channel
void AP_VideoTX::set_enabled(bool enabled) {
    _current_enabled = enabled;
    if (!_enabled.configured_in_storage()) {
        _enabled.set_and_save(enabled);
    }
}

// peiodic update
void AP_VideoTX::update(void)
{
#if HAL_CRSF_TELEM_ENABLED
    AP_CRSF_Telem* crsf = AP::crsf_telem();

    if (crsf != nullptr) {
        crsf->update();
    }
#endif
}

bool AP_VideoTX::have_params_changed() const
{
    return update_power()
        || update_band()
        || update_channel()
        || update_options();
}

// set the current configured values if not currently set in storage
// this is necessary so that the current settings can be seen
void AP_VideoTX::set_defaults()
{
    if (_defaults_set) {
        return;
    }

    if (!_options.configured()) {
        _options.set_and_save(_current_options);
    }
    if (!_channel.configured()) {
        _channel.set_and_save(_current_channel);
    }
    if (!_band.configured()) {
        _band.set_and_save(_current_band);
    }
    if (!_power_mw.configured()) {
        _power_mw.set_and_save(_current_power);
    }

    _defaults_set = true;
    // Output a friendly message so the user knows the VTX has been detected
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VTX: Freq: %dMHz, Power: %dmw, Band: %d, Chan: %d",
        _frequency_mhz.get(), _power_mw.get(), _band.get() + 1, _channel.get() + 1);
}

namespace AP {
    AP_VideoTX& vtx() {
        return *AP_VideoTX::get_singleton();
    }
};