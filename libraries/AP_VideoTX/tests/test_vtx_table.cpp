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

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_VIDEOTX_ENABLED

#include <AP_VideoTX/AP_VideoTX_Table.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include <GCS_MAVLink/GCS_Dummy.h>

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
    ASSERT_EQ(t.num_bands(), 13);
    ASSERT_EQ(t.num_channels(), 8);
    for (uint8_t b = 0; b < 11; b++) {
        for (uint8_t c = 0; c < 8; c++) {
            EXPECT_EQ(t.frequency(b, c), legacy_grid[b][c]) << "band " << int(b) << " ch " << int(c);
        }
    }
    EXPECT_EQ(t.band_letter(4), 'R');
    // the user bands are empty until set
    for (uint8_t b = 11; b < 13; b++) {
        EXPECT_FALSE(t.band_is_factory(b));
        for (uint8_t c = 0; c < 8; c++) {
            EXPECT_EQ(t.frequency(b, c), 0);
        }
    }
}

TEST(VTXTable, OutOfRangeLookups)
{
    AP_VideoTX_Table t;
    EXPECT_EQ(t.frequency(13, 0), 0);
    EXPECT_EQ(t.frequency(0, 8), 0);
    EXPECT_EQ(t.band_letter(13), '?');
    uint8_t band, channel;
    EXPECT_FALSE(t.band_and_channel_for_frequency(0, band, channel));
    EXPECT_FALSE(t.band_and_channel_for_frequency(5999, band, channel));
}

// reverse lookup returns the first match, so a frequency shared by two bands
// (1080 in 1G3_A and 1G3_B) resolves to the earlier band as before
TEST(VTXTable, ReverseLookupFirstMatch)
{
    AP_VideoTX_Table t;
    uint8_t band = 0xFF, channel = 0xFF;
    ASSERT_TRUE(t.band_and_channel_for_frequency(5806, band, channel));
    EXPECT_EQ(band, 4);
    EXPECT_EQ(channel, 4);
    ASSERT_TRUE(t.band_and_channel_for_frequency(1080, band, channel));
    EXPECT_EQ(band, 6);
    EXPECT_EQ(channel, 0);
}

// ---- user bands from the VTX_BNDn_* parameters ----

// user bands as their parameters would give them: band 1 placed by replace
// (-1 adds it as band 11), band 2 unset
static void set_user_band(AP_VideoTX_Table &t, int8_t replace, const int16_t freq[8])
{
    AP_VideoTX_Table::UserBand bands[AP_VideoTX_Table::NUM_USER_BANDS] {};
    bands[0].replace = replace;
    memcpy(bands[0].freq, freq, sizeof(bands[0].freq));
    bands[1].replace = -1;
    t.set_user_bands(bands);
}

static const int16_t custom_freqs[8] = { 5999, 5990, 0, 5980, 5970, 5960, 5950, 5940 };
// band A with channel 1 edited and channel 8 disabled
static const int16_t edited_a[8] = { 5999, 0, 0, 0, 0, 0, 0, -1 };

// an added user band is a custom band after the factory bands, with unset
// channels disabled
TEST(VTXUserBands, AddedBand)
{
    AP_VideoTX_Table t;
    set_user_band(t, -1, custom_freqs);
    EXPECT_FALSE(t.band_is_factory(11));
    EXPECT_EQ(t.band_letter(11), 'Y');
    EXPECT_EQ(t.frequency(11, 0), 5999);
    EXPECT_EQ(t.frequency(11, 2), 0);
    EXPECT_EQ(t.frequency(11, 7), 5940);
    // the factory bands and the other user band are untouched
    EXPECT_EQ(t.frequency(0, 0), 5865);
    EXPECT_EQ(t.frequency(12, 0), 0);
    uint8_t band, channel;
    ASSERT_TRUE(t.band_and_channel_for_frequency(5980, band, channel));
    EXPECT_EQ(band, 11);
    EXPECT_EQ(channel, 3);
}

// a replacing user band edits a factory band in place: set channels change,
// unset channels keep the factory frequency, -1 disables a channel, and the
// band stays a factory band with its letter
TEST(VTXUserBands, ReplacedFactoryBand)
{
    AP_VideoTX_Table t;
    set_user_band(t, 0, edited_a);
    EXPECT_TRUE(t.band_is_factory(0));
    EXPECT_EQ(t.band_letter(0), 'A');
    EXPECT_EQ(t.frequency(0, 0), 5999);
    EXPECT_EQ(t.frequency(0, 1), 5845);
    EXPECT_EQ(t.frequency(0, 7), 0);
    // the factory map the VTX uses is unchanged
    EXPECT_EQ(AP_VideoTX_Table::factory_frequency(0, 0), 5865);
    // the user band's own slot stays empty
    for (uint8_t c = 0; c < 8; c++) {
        EXPECT_EQ(t.frequency(11, c), 0);
    }
}

// changing the parameters rebuilds the table from the factory bands, so an
// earlier edit does not linger
TEST(VTXUserBands, RebuiltFromFactoryBands)
{
    AP_VideoTX_Table t;
    set_user_band(t, 0, edited_a);
    set_user_band(t, -1, custom_freqs);
    EXPECT_EQ(t.frequency(0, 0), 5865);
    EXPECT_EQ(t.frequency(0, 7), 5725);
    EXPECT_EQ(t.frequency(11, 0), 5999);
}

// out of range placements add the band rather than writing elsewhere
TEST(VTXUserBands, OutOfRangeReplaceAdds)
{
    AP_VideoTX_Table t;
    set_user_band(t, 11, custom_freqs);
    EXPECT_EQ(t.frequency(11, 0), 5999);
    set_user_band(t, -5, custom_freqs);
    EXPECT_EQ(t.frequency(11, 0), 5999);
    EXPECT_EQ(t.frequency(0, 0), 5865);
}

// implausible frequencies are never commanded: on SmartAudio a value with bit
// 14 set would be read as a pit mode query
TEST(VTXUserBands, ImplausibleFrequencyDisabled)
{
    AP_VideoTX_Table t;
    const int16_t freqs[8] = { 58, 16384, 999, 6001, -5, 1000, 6000, 5800 };
    set_user_band(t, -1, freqs);
    for (uint8_t c = 0; c < 5; c++) {
        EXPECT_EQ(t.frequency(11, c), 0) << "channel " << int(c);
    }
    EXPECT_EQ(t.frequency(11, 5), 1000);
    EXPECT_EQ(t.frequency(11, 6), 6000);
    EXPECT_EQ(t.frequency(11, 7), 5800);
}

// both user bands replacing one factory band: band 2's set channels win,
// and neither user slot is used
TEST(VTXUserBands, BothReplaceSameBand)
{
    AP_VideoTX_Table t;
    AP_VideoTX_Table::UserBand bands[AP_VideoTX_Table::NUM_USER_BANDS] {};
    const int16_t b1[8] = { 5600, 5610, 0, 0, 0, 0, 0, 0 };
    const int16_t b2[8] = { 5700, 0, -1, 0, 0, 0, 0, 0 };
    bands[0].replace = 4;
    memcpy(bands[0].freq, b1, sizeof(b1));
    bands[1].replace = 4;
    memcpy(bands[1].freq, b2, sizeof(b2));
    t.set_user_bands(bands);
    EXPECT_EQ(t.frequency(4, 0), 5700);
    EXPECT_EQ(t.frequency(4, 1), 5610);
    EXPECT_EQ(t.frequency(4, 2), 0);
    EXPECT_EQ(t.frequency(4, 3), 5769);
    EXPECT_EQ(t.frequency(11, 0), 0);
    EXPECT_EQ(t.frequency(12, 0), 0);
}

// ---- placing VTX reports in the table (AP_VideoTX::resolve_reported) ----

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
    set_user_band(t, -1, custom_freqs);
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
    set_user_band(t, -1, custom_freqs);
    uint8_t band = 4, channel = 3;
    uint16_t freq = 5999;
    AP_VideoTX::resolve_reported(t, 11, 1, true, band, channel, freq);
    EXPECT_EQ(band, 11);
    EXPECT_EQ(channel, 0);   // differs from the configured channel 1

    band = 4; channel = 3; freq = 5990;
    AP_VideoTX::resolve_reported(t, 11, 1, true, band, channel, freq);
    EXPECT_EQ(band, 11);
    EXPECT_EQ(channel, 1);   // settled
}

// a VTX in band/channel mode reporting factory A1 while configured on a
// custom band is on 5865 MHz, decoded on the factory grid
TEST(VTXReport, FactoryIndicesDecodedOnFactoryGrid)
{
    AP_VideoTX_Table t;
    set_user_band(t, -1, custom_freqs);
    uint8_t band = 0, channel = 0;
    uint16_t freq = 0;
    AP_VideoTX::resolve_reported(t, 11, 0, true, band, channel, freq);
    EXPECT_EQ(freq, 5865);
    EXPECT_EQ(band, 0);
    EXPECT_EQ(channel, 0);
}

// a custom band that shares a frequency with a factory channel stays on the
// configured custom slot when the VTX reports that frequency
TEST(VTXReport, CustomBandSharingFactoryFrequency)
{
    AP_VideoTX_Table t;
    const int16_t shared[8] = { 5880, 5990, 0, 5980, 5970, 5960, 5950, 5940 };
    set_user_band(t, -1, shared);
    uint8_t band, channel;
    ASSERT_TRUE(t.band_and_channel_for_frequency(5880, band, channel));
    EXPECT_EQ(band, 3);     // the first match is factory F8
    uint16_t freq = 5880;
    AP_VideoTX::resolve_reported(t, 11, 0, true, band, channel, freq);
    EXPECT_EQ(band, 11);
    EXPECT_EQ(channel, 0);
}

// a replaced factory channel set to another factory band's frequency stays
// on the configured slot when the VTX reports that frequency, rather than
// moving to the first band with it
TEST(VTXReport, ReplacedBandSharingFactoryFrequency)
{
    AP_VideoTX_Table t;
    const int16_t r1_is_f1[8] = { 5740, 0, 0, 0, 0, 0, 0, 0 };
    set_user_band(t, 4, r1_is_f1);
    for (const bool by_index : { false, true }) {
        uint8_t band = 3, channel = 0;
        uint16_t freq = 5740;
        AP_VideoTX::resolve_reported(t, 4, 0, by_index, band, channel, freq);
        EXPECT_EQ(band, 4);
        EXPECT_EQ(channel, 0);
        EXPECT_EQ(freq, 5740);
    }
}

// a factory band with edited frequencies is still tuned from the VTX's own
// map when commanded by index, so reaching the configured slot must settle
// on the table's frequency or the change is re-sent forever
TEST(VTXReport, EditedFactoryBandSettlesByIndex)
{
    AP_VideoTX_Table t;
    set_user_band(t, 0, edited_a);

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
    set_user_band(t, 0, edited_a);
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
    set_user_band(t, -1, custom_freqs);
    EXPECT_TRUE(AP_VideoTX::selectable(t, 11, 0));
    EXPECT_FALSE(AP_VideoTX::selectable(t, 11, 2));
    EXPECT_FALSE(AP_VideoTX::selectable(t, 12, 0));
    EXPECT_TRUE(AP_VideoTX::selectable(t, 4, 0));
}

// AP_VideoTX is a singleton, so the tests share one. Parameter saves are
// queued for an IO thread that does not run here, so the tests only change
// parameters on their first run
static AP_VideoTX &test_vtx()
{
    static AP_VideoTX vtx;
    return vtx;
}

// a reported frequency that is not in the table is kept, so the VTX is
// retuned, only for a custom band or for a VTX commanded by frequency that is
// on a factory frequency the user bands have edited away. With default
// parameters a VTX first seen off the factory grid is taken to be on the
// configured frequency, as before user bands
TEST(VTXDefaults, KeepReportedFrequency)
{
    EXPECT_TRUE(AP_VideoTX::keep_reported_frequency(false, false, 5865));
    EXPECT_FALSE(AP_VideoTX::keep_reported_frequency(false, false, 5600));
    EXPECT_FALSE(AP_VideoTX::keep_reported_frequency(false, true, 5865));
    EXPECT_TRUE(AP_VideoTX::keep_reported_frequency(true, true, 5600));
}

// band A with A1 edited to F1's frequency, end to end through AP_VideoTX.
// The values are the same on every run, so repeated runs save nothing
TEST(VTXDefaults, EditedFactoryBand)
{
    AP_VideoTX &vtx = test_vtx();
    AP_VideoTX_Table &t = vtx.table();
    const int16_t a1_is_f1[8] = { 5740, 0, 0, 0, 0, 0, 0, 0 };
    set_user_band(t, 0, a1_is_f1);

    // a VTX commanded by frequency (Tramp, SmartAudio in frequency mode)
    // first seen on the factory A1 frequency the table has edited away is
    // retuned, not shown as already on the table's frequency
    vtx.set_reported_frequency(5865);
    vtx.set_defaults();
    EXPECT_EQ(vtx.get_frequency_mhz(), 5865);
    EXPECT_EQ(vtx.get_configured_frequency_mhz(), 5740);
    EXPECT_TRUE(vtx.update_frequency());

    // VTX_FREQ given by the configured slot keeps VTX_BAND/VTX_CHANNEL on it,
    // although F1 has the same frequency
    vtx.set_configured_band(0);
    vtx.set_configured_channel(0);
    vtx.update_configured_channel_and_band();
    EXPECT_EQ(vtx.get_configured_band(), 0);
    EXPECT_EQ(vtx.get_configured_channel(), 0);

    // a disabled channel never sets VTX_FREQ to 0
    const int16_t a1_disabled[8] = { -1, 0, 0, 0, 0, 0, 0, 0 };
    set_user_band(t, 0, a1_disabled);
    vtx.update_configured_frequency();
    EXPECT_EQ(vtx.get_configured_frequency_mhz(), 5740);
}

#endif  // AP_VIDEOTX_ENABLED

AP_GTEST_MAIN()
