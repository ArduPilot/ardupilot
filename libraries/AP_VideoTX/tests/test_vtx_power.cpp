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

// tests for the power aux switch mapping in AP_VideoTX::switch_power_mw()

#include <AP_gtest.h>

#include <AP_VideoTX/AP_VideoTX_config.h>

#if AP_VIDEOTX_ENABLED

#include <AP_VideoTX/AP_VideoTX.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static bool switch_power(int8_t position, const int16_t table[VTX_USER_POWER_LEVELS], bool table_enabled, uint16_t max_power, uint16_t &power)
{
    return AP_VideoTX::switch_power_mw(position, table, VTX_USER_POWER_LEVELS, table_enabled, max_power, power);
}

// six used entries and six switch positions: 1:1 mapping, pit mode is the
// explicit 0 entry at position 0
TEST(VTXPowerSwitch, SixEntriesMapOneToOne)
{
    const int16_t table[VTX_USER_POWER_LEVELS] = { 0, 25, 400, 800, 1500, 2500 };
    uint16_t power = 42;
    for (int8_t pos = 0; pos < 6; pos++) {
        EXPECT_TRUE(switch_power(pos, table, true, 800, power));
        EXPECT_EQ(power, table[pos]);
    }
}

// a value of -1 ignores the entry: five used entries spread over six positions,
// edge positions repeat
TEST(VTXPowerSwitch, NegativeEntriesIgnored)
{
    const int16_t table[VTX_USER_POWER_LEVELS] = { 25, -1, 400, -1, 800, -1 };
    const uint16_t expected[6] = { 25, 25, 400, 400, 800, 800 };
    uint16_t power;
    for (int8_t pos = 0; pos < 6; pos++) {
        EXPECT_TRUE(switch_power(pos, table, true, 800, power));
        EXPECT_EQ(power, expected[pos]);
    }
}

// a table with no used entries leaves the switch with nothing to select
TEST(VTXPowerSwitch, AllNegativeNothingSelectable)
{
    const int16_t table[VTX_USER_POWER_LEVELS] = { -1, -1, -1, -1, -1, -1 };
    uint16_t power = 42;
    for (int8_t pos = 0; pos < 6; pos++) {
        EXPECT_FALSE(switch_power(pos, table, true, 800, power));
        EXPECT_EQ(power, 42);
    }
}

// a table of nothing but 0 entries is pit mode on every position
TEST(VTXPowerSwitch, AllZeroAllPitmode)
{
    const int16_t table[VTX_USER_POWER_LEVELS] = { 0, 0, 0, 0, 0, 0 };
    uint16_t power;
    for (int8_t pos = 0; pos < 6; pos++) {
        EXPECT_TRUE(switch_power(pos, table, true, 800, power));
        EXPECT_EQ(power, 0);
    }
}

// pitmode is an explicit table entry, so it can sit anywhere in the table
TEST(VTXPowerSwitch, MidTablePitmode)
{
    const int16_t table[VTX_USER_POWER_LEVELS] = { 25, 400, 0, 800, -1, -1 };
    const uint16_t expected[6] = { 25, 25, 400, 0, 0, 800 };
    uint16_t power;
    for (int8_t pos = 0; pos < 6; pos++) {
        EXPECT_TRUE(switch_power(pos, table, true, 800, power));
        EXPECT_EQ(power, expected[pos]);
    }
}

// built-in levels: the top position offers VTX_MAX_POWER even when the
// largest active level is below it
TEST(VTXPowerSwitch, BuiltInTopPositionReachesMaxPower)
{
    const int16_t table[VTX_USER_POWER_LEVELS] = { -1, -1, -1, -1, -1, -1 };
    uint16_t power;
    EXPECT_TRUE(switch_power(5, table, false, 750, power));
    EXPECT_EQ(power, 750);
}

// built-in levels: VTX_MAX_POWER of 0 means no cap
TEST(VTXPowerSwitch, BuiltInZeroMaxPowerMeansNoCap)
{
    const int16_t table[VTX_USER_POWER_LEVELS] = { -1, -1, -1, -1, -1, -1 };
    uint16_t power;
    EXPECT_TRUE(switch_power(5, table, false, 0, power));
    EXPECT_EQ(power, 1000);
}

AP_GTEST_MAIN()

#endif  // AP_VIDEOTX_ENABLED
