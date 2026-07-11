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
  VTX band/frequency table, modelled on Betaflight's vtxTable band table:
  bands with a single-letter id, a factory/custom flag and up to MAX_CHANNELS
  channel frequencies in MHz (0 = channel disabled). band+channel resolves to
  a frequency. Power levels are not part of this table; they come from the
  VTX_PWRTBL* parameters.

  The table holds the historical compiled-in factory bands followed by
  NUM_USER_BANDS user bands, which are defined by the VTX_BNDn_* parameters
  (see AP_VideoTX). A user band is either added after the factory bands or
  replaces the frequencies of one of them.
*/
#pragma once

#include "AP_VideoTX_config.h"

#if AP_VIDEOTX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

class AP_VideoTX_Table {
public:
    // the 11 historical bands, so every AP_VideoTX::VideoBand index maps to
    // the same band as before, then the user bands. Channels match
    // Betaflight (8)
    static const uint8_t NUM_FACTORY_BANDS = 11;
    static const uint8_t NUM_USER_BANDS = 2;
    static const uint8_t MAX_BANDS = NUM_FACTORY_BANDS + NUM_USER_BANDS;
    static const uint8_t MAX_CHANNELS = 8;

    struct Band {
        char letter;                 // single-char id shown in the OSD
        bool is_factory;             // factory band: VTX uses its own freq map
        uint16_t freq[MAX_CHANNELS]; // MHz, 0 = channel disabled/unused
    };

    // a user band as given by its parameters
    struct UserBand {
        // factory band to replace, or -1 to add the band after the factory bands
        int8_t replace;
        // MHz; 0 is unset (unused in an added band, the factory frequency in a
        // replaced band), and -1 or any other value outside MIN_FREQ to
        // MAX_FREQ disables the channel
        int16_t freq[MAX_CHANNELS];
    };

    // seeded with the factory bands so band/channel lookups work before the
    // user bands are applied
    AP_VideoTX_Table() { seed_defaults(); }
    CLASS_NO_COPY(AP_VideoTX_Table);

    // rebuild the table from the factory bands and the given user bands
    void set_user_bands(const UserBand bands[NUM_USER_BANDS]);

    // frequency of a band/channel in the compiled-in factory band map, which
    // is what a VTX's own band/channel indices refer to; 0 if out of range
    static uint16_t factory_frequency(uint8_t band, uint8_t channel);
    // true if the frequency is in the compiled-in factory band map
    static bool is_factory_frequency(uint16_t freq);

    // -- band / frequency accessors (band, channel are zero-based) --
    uint8_t num_bands() const { return MAX_BANDS; }
    uint8_t num_channels() const { return MAX_CHANNELS; }
    // frequency for a band/channel, 0 if out of range or channel disabled
    uint16_t frequency(uint8_t band, uint8_t channel) const;
    // reverse lookup: first band/channel whose frequency matches, false if none
    bool band_and_channel_for_frequency(uint16_t freq, uint8_t &band, uint8_t &channel) const;
    // single-letter band id, '?' if out of range
    char band_letter(uint8_t band) const;
    bool band_is_factory(uint8_t band) const;

    // held across a sequence of lookups that must see one consistent table
    HAL_Semaphore &get_semaphore() const { return _sem; }

    // plausible channel frequencies in MHz; anything else disables a user
    // band channel
    static const uint16_t MIN_FREQ = 1000;
    static const uint16_t MAX_FREQ = 6000;

private:
    // fill the model with the factory bands and empty user bands, without
    // locking: also used by the constructor, which runs during static
    // initialisation before the HAL (and so the semaphore) is available
    void seed_defaults();

    Band _bands[MAX_BANDS];
    // the table is rebuilt from the main thread while other threads (e.g.
    // CRSF telemetry) look up frequencies
    mutable HAL_Semaphore _sem;
};

#endif  // AP_VIDEOTX_ENABLED
