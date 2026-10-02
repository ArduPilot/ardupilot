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
  User-definable VTX band/frequency table, modelled on Betaflight's
  vtxTable band table: up to MAX_BANDS bands, each with a name, a
  single-letter id, a factory/custom flag and up to MAX_CHANNELS channel
  frequencies in MHz (0 = channel disabled). band+channel resolves to a
  frequency. Power levels are not part of this table; they come from the
  VTX_PWRTBL* parameters.

  The table is held in RAM here; persistence (StorageManager blob) and the
  MAVLink FTP transport are layered on top. When no table is stored the model
  is seeded from the historical compiled-in bands so behaviour is unchanged.
*/
#pragma once

#include "AP_VideoTX_config.h"

#if AP_VIDEOTX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

class AP_VideoTX_Table {
public:
    // limits. MAX_BANDS is >= the 11 historical bands so every legacy
    // AP_VideoTX::VideoBand index maps to a real default band, with headroom
    // for user-added custom bands. Channels match Betaflight (8).
    static const uint8_t MAX_BANDS = 12;
    static const uint8_t MAX_CHANNELS = 8;
    static const uint8_t BAND_NAME_LEN = 8;    // not NUL terminated in storage

    struct Band {
        char name[BAND_NAME_LEN];    // space/zero padded, not NUL terminated
        char letter;                 // single-char id shown in the OSD
        bool is_factory;             // factory band: VTX uses its own freq map
        uint16_t freq[MAX_CHANNELS]; // MHz, 0 = channel disabled/unused
    };

    // seeded with the compiled defaults so band/channel lookups work before
    // (and without) a stored table
    AP_VideoTX_Table() { seed_defaults(); }
    CLASS_NO_COPY(AP_VideoTX_Table);

#if AP_VIDEOTX_TABLE_ENABLED
    // wire format / storage constants
    static const uint16_t BLOB_MAGIC = 0x5654;   // 'VT'
    static const uint8_t  BLOB_VERSION = 2;
    static const uint8_t  BLOB_HEADER = 5;       // magic, version, bands, channels
    // worst case serialized size (must fit the StorageVTXTable region)
    static const uint16_t BLOB_MAX = BLOB_HEADER + MAX_BANDS*(BAND_NAME_LEN+2+MAX_CHANNELS*2) + 4;

    // load the table from persistent storage; if absent or invalid, use the
    // compiled defaults (not persisted until a user table is written).
    // Call once at startup.
    void init();
    // persist the current table to storage, false if storage is unavailable/full
    bool save();

    // serialize the current table into buf (must be >= BLOB_MAX bytes),
    // returns the number of bytes written. Used by the @VTX FTP transport.
    uint16_t to_blob(uint8_t *buf) const { return serialize(buf); }
    // replace the table from a serialized blob and persist it. Returns false,
    // leaving the table unchanged, if the blob is malformed or the board has
    // no storage region for it (so an upload is never silently lost on reboot)
    bool from_blob(const uint8_t *buf, uint16_t len);

    // true if this board has a storage region for a user table
    static bool storage_available();

    // check a serialized blob (magic, version, dimensions, length, CRC)
    // without applying it
    static bool validate(const uint8_t *buf, uint16_t len);

    // replace the table from a serialized blob without persisting it; returns
    // false, leaving the table unchanged, if the blob is malformed
    bool apply_blob(const uint8_t *buf, uint16_t len) { return deserialize(buf, len); }
#endif  // AP_VIDEOTX_TABLE_ENABLED

    // seed the model from the historical compiled-in bands
    void load_defaults();

    // frequency of a band/channel in the compiled-in factory band map, which
    // is what a VTX's own band/channel indices refer to; 0 if out of range
    static uint16_t factory_frequency(uint8_t band, uint8_t channel);

    // -- band / frequency accessors (band, channel are zero-based) --
    uint8_t num_bands() const { return _num_bands; }
    uint8_t num_channels() const { return _num_channels; }
    // frequency for a band/channel, 0 if out of range or channel disabled
    uint16_t frequency(uint8_t band, uint8_t channel) const;
    // reverse lookup: first band/channel whose frequency matches, false if none
    bool band_and_channel_for_frequency(uint16_t freq, uint8_t &band, uint8_t &channel) const;
    // single-letter band id, '?' if out of range
    char band_letter(uint8_t band) const;
    // copy a NUL-terminated band name into out (size >= BAND_NAME_LEN+1)
    void band_name(uint8_t band, char *out, size_t out_len) const;
    bool band_is_factory(uint8_t band) const;

    // whether a usable table is loaded
    bool is_valid() const { return _num_bands > 0 && _num_channels > 0; }

private:
    // fill the model with the compiled defaults, without locking: also used by
    // the constructor, which runs during static initialisation before the
    // HAL (and so the semaphore) is available
    void seed_defaults();

#if AP_VIDEOTX_TABLE_ENABLED
    // serialize the model into buf (>= BLOB_MAX), returns bytes written
    uint16_t serialize(uint8_t *buf) const;
    // parse a blob into the model, false if magic/version/counts/crc are bad
    bool deserialize(const uint8_t *buf, uint16_t len);
#endif  // AP_VIDEOTX_TABLE_ENABLED

    Band _bands[MAX_BANDS];
    uint8_t _num_bands;
    uint8_t _num_channels;
    // the table is replaced from the FTP thread while other threads (e.g. CRSF
    // telemetry) look up frequencies
    mutable HAL_Semaphore _sem;
};

#endif  // AP_VIDEOTX_ENABLED
