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

#include "AP_VideoTX_Table.h"

#if AP_VIDEOTX_ENABLED

#include <string.h>

// the factory bands. These are the historical AP_VideoTX bands/frequencies, so
// band indices remain compatible with AP_VideoTX::VideoBand and behaviour is
// unchanged out of the box.
struct DefaultBand {
    char letter;
    uint16_t freq[AP_VideoTX_Table::MAX_CHANNELS];
};

static const DefaultBand default_bands[AP_VideoTX_Table::NUM_FACTORY_BANDS] = {
    { 'A', { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 } },  // Boscam A
    { 'B', { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 } },  // Boscam B
    { 'E', { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 } },  // Boscam E
    { 'F', { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 } },  // Fatshark
    { 'R', { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 } },  // Raceband
    { 'L', { 5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621 } },  // Low raceband
    { 'U', { 1080, 1120, 1160, 1200, 1240, 1280, 1320, 1360 } },  // 1G3 A
    { 'V', { 1080, 1120, 1160, 1200, 1258, 1280, 1320, 1360 } },  // 1G3 B
    { 'X', { 4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200 } },  // Band X
    { 'C', { 3330, 3350, 3370, 3390, 3410, 3430, 3450, 3470 } },  // 3G3 A
    { 'D', { 3170, 3190, 3210, 3230, 3250, 3270, 3290, 3310 } },  // 3G3 B
};

// letters for the user bands added after the factory bands
static const char user_band_letters[AP_VideoTX_Table::NUM_USER_BANDS] = { 'Y', 'Z' };

void AP_VideoTX_Table::seed_defaults()
{
    memset(_bands, 0, sizeof(_bands));
    for (uint8_t b = 0; b < NUM_FACTORY_BANDS; b++) {
        _bands[b].letter = default_bands[b].letter;
        _bands[b].is_factory = true;  // standard bands: VTX may use its own map
        memcpy(_bands[b].freq, default_bands[b].freq, sizeof(_bands[b].freq));
    }
    for (uint8_t u = 0; u < NUM_USER_BANDS; u++) {
        _bands[NUM_FACTORY_BANDS + u].letter = user_band_letters[u];
    }
}

void AP_VideoTX_Table::set_user_bands(const UserBand bands[NUM_USER_BANDS])
{
    WITH_SEMAPHORE(_sem);
    seed_defaults();
    for (uint8_t u = 0; u < NUM_USER_BANDS; u++) {
        const UserBand &ub = bands[u];
        // a replaced factory band keeps its letter and stays a factory band:
        // it describes what this VTX's own band map holds for that band
        const bool replacing = ub.replace >= 0 && ub.replace < NUM_FACTORY_BANDS;
        Band &dst = _bands[replacing ? ub.replace : NUM_FACTORY_BANDS + u];
        for (uint8_t c = 0; c < MAX_CHANNELS; c++) {
            if (ub.freq[c] == 0) {
                continue;
            }
            // an implausible frequency is never commanded: on SmartAudio a
            // value with bit 14 set would even be read as a pit mode query
            const bool valid = ub.freq[c] >= MIN_FREQ && ub.freq[c] <= MAX_FREQ;
            dst.freq[c] = valid ? ub.freq[c] : 0;
        }
    }
}

uint16_t AP_VideoTX_Table::factory_frequency(uint8_t band, uint8_t channel)
{
    if (band >= NUM_FACTORY_BANDS || channel >= MAX_CHANNELS) {
        return 0;
    }
    return default_bands[band].freq[channel];
}

bool AP_VideoTX_Table::is_factory_frequency(uint16_t freq)
{
    for (const auto &band : default_bands) {
        for (const uint16_t f : band.freq) {
            if (f == freq) {
                return true;
            }
        }
    }
    return false;
}

uint16_t AP_VideoTX_Table::frequency(uint8_t band, uint8_t channel) const
{
    WITH_SEMAPHORE(_sem);
    if (band >= MAX_BANDS || channel >= MAX_CHANNELS) {
        return 0;
    }
    return _bands[band].freq[channel];
}

bool AP_VideoTX_Table::band_and_channel_for_frequency(uint16_t freq, uint8_t &band, uint8_t &channel) const
{
    WITH_SEMAPHORE(_sem);
    if (freq == 0) {
        return false;
    }
    for (uint8_t b = 0; b < MAX_BANDS; b++) {
        for (uint8_t c = 0; c < MAX_CHANNELS; c++) {
            if (_bands[b].freq[c] == freq) {
                band = b;
                channel = c;
                return true;
            }
        }
    }
    return false;
}

char AP_VideoTX_Table::band_letter(uint8_t band) const
{
    WITH_SEMAPHORE(_sem);
    if (band >= MAX_BANDS) {
        return '?';
    }
    return _bands[band].letter;
}

bool AP_VideoTX_Table::band_is_factory(uint8_t band) const
{
    WITH_SEMAPHORE(_sem);
    if (band >= MAX_BANDS) {
        return false;
    }
    return _bands[band].is_factory;
}

#endif  // AP_VIDEOTX_ENABLED
