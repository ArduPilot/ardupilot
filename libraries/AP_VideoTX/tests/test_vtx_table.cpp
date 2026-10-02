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

// tests for the user-definable VTX band table (AP_VideoTX_Table)

#include <AP_gtest.h>

#include <AP_VideoTX/AP_VideoTX_config.h>

#if AP_VIDEOTX_ENABLED

#include <AP_VideoTX/AP_VideoTX_Table.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// AP_VideoTX::set_defaults() announces the settings via gcs()
static GCS_Dummy _gcs;

// the historical AP_VideoTX grid, in VideoBand order
static const uint16_t legacy_grid[11][8] = {
    { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 },
    { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 },
    { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 },
    { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 },
    { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 },
    { 5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621 },
    { 1080, 1120, 1160, 1200, 1240, 1280, 1320, 1360 },
    { 1080, 1120, 1160, 1200, 1258, 1280, 1320, 1360 },
    { 4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200 },
    { 3330, 3350, 3370, 3390, 3410, 3430, 3450, 3470 },
    { 3170, 3190, 3210, 3230, 3250, 3270, 3290, 3310 },
};

// the defaults reproduce the historical grid exactly, so VTX_BAND indices
// keep their meaning
TEST(VTXTable, DefaultsMatchLegacyGrid)
{
    AP_VideoTX_Table t;
    t.load_defaults();
    ASSERT_EQ(t.num_bands(), 11);
    ASSERT_EQ(t.num_channels(), 8);
    for (uint8_t b = 0; b < 11; b++) {
        for (uint8_t c = 0; c < 8; c++) {
            EXPECT_EQ(t.frequency(b, c), legacy_grid[b][c]) << "band " << int(b) << " ch " << int(c);
        }
    }
    EXPECT_EQ(t.band_letter(4), 'R');
    EXPECT_TRUE(t.is_valid());
}

TEST(VTXTable, OutOfRangeLookups)
{
    AP_VideoTX_Table t;
    t.load_defaults();
    EXPECT_EQ(t.frequency(11, 0), 0);
    EXPECT_EQ(t.frequency(0, 8), 0);
    EXPECT_EQ(t.band_letter(11), '?');
    uint8_t band, channel;
    EXPECT_FALSE(t.band_and_channel_for_frequency(0, band, channel));
    EXPECT_FALSE(t.band_and_channel_for_frequency(5999, band, channel));
}

// reverse lookup returns the first match, so a frequency shared by two bands
// (1080 in 1G3_A and 1G3_B) resolves to the earlier band as before
TEST(VTXTable, ReverseLookupFirstMatch)
{
    AP_VideoTX_Table t;
    t.load_defaults();
    uint8_t band = 0xFF, channel = 0xFF;
    ASSERT_TRUE(t.band_and_channel_for_frequency(5806, band, channel));
    EXPECT_EQ(band, 4);
    EXPECT_EQ(channel, 4);
    ASSERT_TRUE(t.band_and_channel_for_frequency(1080, band, channel));
    EXPECT_EQ(band, 6);
    EXPECT_EQ(channel, 0);
}

#if AP_VIDEOTX_TABLE_ENABLED

// recompute the trailing CRC after editing a blob body
static void fix_crc(uint8_t *blob, uint16_t len)
{
    const uint32_t crc = crc_crc32(0, blob, len - 4);
    blob[len-4] = crc & 0xFF;
    blob[len-3] = (crc >> 8) & 0xFF;
    blob[len-2] = (crc >> 16) & 0xFF;
    blob[len-1] = (crc >> 24) & 0xFF;
}

TEST(VTXTable, SerializedDefaultsValidate)
{
    AP_VideoTX_Table t;
    t.load_defaults();
    uint8_t blob[AP_VideoTX_Table::BLOB_MAX];
    const uint16_t len = t.to_blob(blob);
    EXPECT_LE(len, uint16_t(AP_VideoTX_Table::BLOB_MAX));
    EXPECT_EQ(len, uint16_t(AP_VideoTX_Table::BLOB_HEADER + 11*(AP_VideoTX_Table::BAND_NAME_LEN+2+8*2) + 4));
    EXPECT_EQ(blob[2], uint8_t(AP_VideoTX_Table::BLOB_VERSION));
    EXPECT_TRUE(AP_VideoTX_Table::validate(blob, len));
}

TEST(VTXTable, ValidateRejectsCorruption)
{
    AP_VideoTX_Table t;
    t.load_defaults();
    uint8_t blob[AP_VideoTX_Table::BLOB_MAX];
    const uint16_t len = t.to_blob(blob);

    uint8_t bad[AP_VideoTX_Table::BLOB_MAX];

    // flipped payload byte: CRC mismatch
    memcpy(bad, blob, len);
    bad[20] ^= 0x01;
    EXPECT_FALSE(AP_VideoTX_Table::validate(bad, len));

    // truncated
    EXPECT_FALSE(AP_VideoTX_Table::validate(blob, len - 1));
    EXPECT_FALSE(AP_VideoTX_Table::validate(blob, 4));

    // wrong magic / version, even with a matching CRC
    memcpy(bad, blob, len);
    bad[0] ^= 0xFF;
    fix_crc(bad, len);
    EXPECT_FALSE(AP_VideoTX_Table::validate(bad, len));
    memcpy(bad, blob, len);
    bad[2] = 1;
    fix_crc(bad, len);
    EXPECT_FALSE(AP_VideoTX_Table::validate(bad, len));
}

TEST(VTXTable, ValidateRejectsBadDimensions)
{
    uint8_t blob[AP_VideoTX_Table::BLOB_MAX + 64] {};
    blob[0] = AP_VideoTX_Table::BLOB_MAGIC & 0xFF;
    blob[1] = AP_VideoTX_Table::BLOB_MAGIC >> 8;
    blob[2] = AP_VideoTX_Table::BLOB_VERSION;

    // empty tables: would resolve every band/channel to 0 MHz
    blob[3] = 0; blob[4] = 8;
    fix_crc(blob, AP_VideoTX_Table::BLOB_HEADER + 4);
    EXPECT_FALSE(AP_VideoTX_Table::validate(blob, AP_VideoTX_Table::BLOB_HEADER + 4));
    blob[3] = 1; blob[4] = 0;
    fix_crc(blob, AP_VideoTX_Table::BLOB_HEADER + 10 + 4);
    EXPECT_FALSE(AP_VideoTX_Table::validate(blob, AP_VideoTX_Table::BLOB_HEADER + 10 + 4));

    // over the limits
    blob[3] = AP_VideoTX_Table::MAX_BANDS + 1; blob[4] = 8;
    EXPECT_FALSE(AP_VideoTX_Table::validate(blob, sizeof(blob)));
    blob[3] = 1; blob[4] = AP_VideoTX_Table::MAX_CHANNELS + 1;
    EXPECT_FALSE(AP_VideoTX_Table::validate(blob, sizeof(blob)));
}

// a table with fewer bands/channels than the defaults is valid, and trailing
// bytes beyond the declared size are ignored
TEST(VTXTable, ValidateAcceptsSmallTable)
{
    const uint16_t len = AP_VideoTX_Table::BLOB_HEADER + 1*(AP_VideoTX_Table::BAND_NAME_LEN+2+4*2) + 4;
    uint8_t blob[64] {};
    blob[0] = AP_VideoTX_Table::BLOB_MAGIC & 0xFF;
    blob[1] = AP_VideoTX_Table::BLOB_MAGIC >> 8;
    blob[2] = AP_VideoTX_Table::BLOB_VERSION;
    blob[3] = 1;
    blob[4] = 4;
    memcpy(&blob[5], "CUSTOM", 6);
    blob[13] = 'Z';
    fix_crc(blob, len);
    EXPECT_TRUE(AP_VideoTX_Table::validate(blob, len));
    EXPECT_TRUE(AP_VideoTX_Table::validate(blob, sizeof(blob)));
}

// SITL emulates 16k flash storage, which has no table region: an upload must
// be refused without touching the table
TEST(VTXTable, UploadRefusedWithoutStorage)
{
    ASSERT_FALSE(AP_VideoTX_Table::storage_available());
    AP_VideoTX_Table t;
    t.load_defaults();
    uint8_t blob[AP_VideoTX_Table::BLOB_MAX];
    const uint16_t len = t.to_blob(blob);
    blob[AP_VideoTX_Table::BLOB_HEADER + 10] = 0x6F;  // band0 ch0 -> 5999
    blob[AP_VideoTX_Table::BLOB_HEADER + 11] = 0x17;
    fix_crc(blob, len);
    ASSERT_TRUE(AP_VideoTX_Table::validate(blob, len));
    EXPECT_FALSE(t.from_blob(blob, len));
    EXPECT_EQ(t.frequency(0, 0), 5865);
}

// ---- placing VTX reports in the table (AP_VideoTX::resolve_reported) ----

// the default table with one band changed or appended, applied without
// persisting. band 11 is an extra custom band when append is true
static bool load_modified(AP_VideoTX_Table &t, bool append, uint8_t band, bool factory,
                          const uint16_t freq[8])
{
    AP_VideoTX_Table d;
    uint8_t blob[AP_VideoTX_Table::BLOB_MAX];
    uint16_t len = d.to_blob(blob) - 4;
    const uint16_t band_len = AP_VideoTX_Table::BAND_NAME_LEN + 2 + 8*2;
    uint16_t o;
    if (append) {
        blob[3]++;
        o = len;
        len += band_len;
        memset(&blob[o], 0, AP_VideoTX_Table::BAND_NAME_LEN);
        memcpy(&blob[o], "CUSTOM", 6);
        blob[o + AP_VideoTX_Table::BAND_NAME_LEN] = 'Z';
    } else {
        o = AP_VideoTX_Table::BLOB_HEADER + band * band_len;
    }
    blob[o + AP_VideoTX_Table::BAND_NAME_LEN + 1] = factory ? 1 : 0;
    for (uint8_t c = 0; c < 8; c++) {
        blob[o + AP_VideoTX_Table::BAND_NAME_LEN + 2 + c*2] = freq[c] & 0xFF;
        blob[o + AP_VideoTX_Table::BAND_NAME_LEN + 3 + c*2] = freq[c] >> 8;
    }
    fix_crc(blob, len + 4);
    return t.apply_blob(blob, len + 4);
}

static const uint16_t custom_freqs[8] = { 5999, 6010, 0, 6030, 6040, 6050, 6060, 6070 };

// factory bands keep the VTX's own indices; a missing frequency is decoded on
// the factory grid
TEST(VTXReport, FactoryReportUnchanged)
{
    AP_VideoTX_Table t;
    uint8_t band = 4, channel = 0;
    uint16_t freq = 5658;
    AP_VideoTX::resolve_reported(t, 4, 0, true, band, channel, freq);
    EXPECT_EQ(band, 4);
    EXPECT_EQ(channel, 0);
    EXPECT_EQ(freq, 5658);

    band = 4; channel = 1; freq = 0;
    AP_VideoTX::resolve_reported(t, 4, 0, true, band, channel, freq);
    EXPECT_EQ(band, 4);
    EXPECT_EQ(channel, 1);
    EXPECT_EQ(freq, 5695);
}

// a VTX tuned to a custom band by frequency reports its own channel byte;
// that must not stop the configured custom slot from matching, or the change
// is re-sent forever
TEST(VTXReport, CustomBandSettlesOnConfiguredSlot)
{
    AP_VideoTX_Table t;
    ASSERT_TRUE(load_modified(t, true, 11, false, custom_freqs));
    uint8_t band = 4, channel = 3;
    uint16_t freq = 5999;
    AP_VideoTX::resolve_reported(t, 11, 0, true, band, channel, freq);
    EXPECT_EQ(band, 11);
    EXPECT_EQ(channel, 0);
    EXPECT_EQ(freq, 5999);
}

// a channel change within a custom band is still seen as pending until the
// VTX reports the new frequency
TEST(VTXReport, CustomBandChannelChangeStillDue)
{
    AP_VideoTX_Table t;
    ASSERT_TRUE(load_modified(t, true, 11, false, custom_freqs));
    uint8_t band = 4, channel = 3;
    uint16_t freq = 5999;
    AP_VideoTX::resolve_reported(t, 11, 1, true, band, channel, freq);
    EXPECT_EQ(band, 11);
    EXPECT_EQ(channel, 0);   // differs from the configured channel 1

    band = 4; channel = 3; freq = 6010;
    AP_VideoTX::resolve_reported(t, 11, 1, true, band, channel, freq);
    EXPECT_EQ(band, 11);
    EXPECT_EQ(channel, 1);   // settled
}

// a VTX in band/channel mode reporting factory A1 is on 5865 MHz even when
// band A has been made custom in the user table
TEST(VTXReport, FactoryIndicesDecodedOnFactoryGrid)
{
    AP_VideoTX_Table t;
    uint16_t edited[8] = { 5999, 5845, 5825, 5805, 5785, 5765, 5745, 5725 };
    ASSERT_TRUE(load_modified(t, false, 0, false, edited));
    uint8_t band = 0, channel = 0;
    uint16_t freq = 0;
    AP_VideoTX::resolve_reported(t, 0, 0, true, band, channel, freq);
    EXPECT_EQ(freq, 5865);
    EXPECT_NE(freq, t.frequency(0, 0));   // so a retune to 5999 is due
}

// a custom band that shares a frequency with a factory channel stays on the
// configured custom slot when the VTX reports that frequency
TEST(VTXReport, CustomBandSharingFactoryFrequency)
{
    AP_VideoTX_Table t;
    uint16_t shared[8] = { 5880, 6010, 0, 6030, 6040, 6050, 6060, 6070 };
    ASSERT_TRUE(load_modified(t, true, 11, false, shared));
    uint8_t band, channel;
    ASSERT_TRUE(t.band_and_channel_for_frequency(5880, band, channel));
    EXPECT_EQ(band, 3);     // the first match is factory F8
    uint16_t freq = 5880;
    AP_VideoTX::resolve_reported(t, 11, 0, true, band, channel, freq);
    EXPECT_EQ(band, 11);
    EXPECT_EQ(channel, 0);
}

// a factory band with edited frequencies is still tuned from the VTX's own
// map when commanded by index, so reaching the configured slot must settle
// on the table's frequency or the change is re-sent forever
TEST(VTXReport, EditedFactoryBandSettlesByIndex)
{
    AP_VideoTX_Table t;
    uint16_t edited[8] = { 5999, 5845, 5825, 5805, 5785, 5765, 5745, 5725 };
    ASSERT_TRUE(load_modified(t, false, 0, true, edited));

    // band/channel report without a frequency (CRSF)
    uint8_t band = 0, channel = 0;
    uint16_t freq = 0;
    AP_VideoTX::resolve_reported(t, 0, 0, true, band, channel, freq);
    EXPECT_EQ(band, 0);
    EXPECT_EQ(channel, 0);
    EXPECT_EQ(freq, 5999);

    // band/channel report with the frequency from the VTX's map (SmartAudio)
    band = 0; channel = 0; freq = 5865;
    AP_VideoTX::resolve_reported(t, 0, 0, true, band, channel, freq);
    EXPECT_EQ(freq, 5999);

    // frequency-only report (CRSF telemetry): not in the table at all
    band = UINT8_MAX; channel = UINT8_MAX; freq = 5865;
    EXPECT_FALSE(t.band_and_channel_for_frequency(freq, band, channel));
    AP_VideoTX::resolve_reported(t, 0, 0, true, band, channel, freq);
    EXPECT_EQ(band, 0);
    EXPECT_EQ(channel, 0);
    EXPECT_EQ(freq, 5999);
}

// commanded by frequency (Tramp, SmartAudio in frequency mode) an edited
// factory band is really retuned, so the map's frequency is still a change
// to make
TEST(VTXReport, EditedFactoryBandRetunedByFrequency)
{
    AP_VideoTX_Table t;
    uint16_t edited[8] = { 5999, 5845, 5825, 5805, 5785, 5765, 5745, 5725 };
    ASSERT_TRUE(load_modified(t, false, 0, true, edited));
    uint8_t band = 0, channel = 0;
    uint16_t freq = 5865;
    AP_VideoTX::resolve_reported(t, 0, 0, false, band, channel, freq);
    EXPECT_EQ(freq, 5865);
    EXPECT_NE(freq, t.frequency(0, 0));
}

// a change to another factory slot is still due until the VTX reports it
TEST(VTXReport, FactoryChannelChangeStillDueByIndex)
{
    AP_VideoTX_Table t;
    uint8_t band = 0, channel = 0;
    uint16_t freq = 0;
    AP_VideoTX::resolve_reported(t, 0, 1, true, band, channel, freq);
    EXPECT_EQ(channel, 0);
    EXPECT_EQ(freq, 5865);
    EXPECT_NE(freq, t.frequency(0, 1));
}

// disabled (0 MHz) and out of range entries are never selectable, so they
// are never commanded
TEST(VTXReport, DisabledEntryNotSelectable)
{
    AP_VideoTX_Table t;
    ASSERT_TRUE(load_modified(t, true, 11, false, custom_freqs));
    EXPECT_TRUE(AP_VideoTX::selectable(t, 11, 0));
    EXPECT_FALSE(AP_VideoTX::selectable(t, 11, 2));
    EXPECT_FALSE(AP_VideoTX::selectable(t, 12, 0));
    EXPECT_TRUE(AP_VideoTX::selectable(t, 4, 0));
}

// a VTX commanded by frequency (Tramp, SmartAudio in frequency mode) that
// is first seen on a factory frequency the table has since edited must still
// be retuned, not shown as already on the table's frequency
TEST(VTXDefaults, FrequencyModeEditedFactoryBandRetuned)
{
    static AP_VideoTX vtx;
    // the table is only replaced through the FTP path in the firmware
    AP_VideoTX_Table &t = const_cast<AP_VideoTX_Table&>(vtx.table());
    uint16_t edited[8] = { 5870, 5845, 5825, 5805, 5785, 5765, 5745, 5725 };
    ASSERT_TRUE(load_modified(t, false, 0, true, edited));
    vtx.set_reported_frequency(5865);
    vtx.set_defaults();
    EXPECT_EQ(vtx.get_frequency_mhz(), 5865);
    EXPECT_EQ(vtx.get_configured_frequency_mhz(), 5870);
    EXPECT_TRUE(vtx.update_frequency());
}

#endif  // AP_VIDEOTX_TABLE_ENABLED

AP_GTEST_MAIN()

#endif  // AP_VIDEOTX_ENABLED
