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

static bool switch_power(int8_t position, const uint16_t table[VTX_USER_POWER_LEVELS], bool table_enabled, uint16_t max_power, uint16_t &power)
{
    return AP_VideoTX::switch_power_mw(position, table, VTX_USER_POWER_LEVELS, table_enabled, max_power, power);
}

// six entries and six switch positions: 1:1 mapping, no pitmode slot
TEST(VTXPowerSwitch, SixEntriesMapOneToOne)
{
    const uint16_t table[VTX_USER_POWER_LEVELS] = { 25, 100, 200, 500, 800, 1000 };
    uint16_t power = 42;
    for (int8_t pos = 0; pos < 6; pos++) {
        EXPECT_TRUE(switch_power(pos, table, true, 800, power));
        EXPECT_EQ(power, table[pos]);
    }
}

// five entries: position 0 is pitmode, the rest map 1:1 onto the entries
TEST(VTXPowerSwitch, FiveEntriesPitmodeFirst)
{
    const uint16_t table[VTX_USER_POWER_LEVELS] = { 25, 100, 200, 500, 800, 0 };
    const uint16_t expected[6] = { 0, 25, 100, 200, 500, 800 };
    uint16_t power;
    for (int8_t pos = 0; pos < 6; pos++) {
        EXPECT_TRUE(switch_power(pos, table, true, 800, power));
        EXPECT_EQ(power, expected[pos]);
    }
}

// entries are taken in slot order; the user is expected to set them ascending,
// zero entries are skipped
TEST(VTXPowerSwitch, SlotOrderPreserved)
{
    const uint16_t table[VTX_USER_POWER_LEVELS] = { 500, 0, 25, 0, 800, 0 };
    const uint16_t expected[6] = { 0, 0, 500, 25, 25, 800 };
    uint16_t power;
    for (int8_t pos = 0; pos < 6; pos++) {
        EXPECT_TRUE(switch_power(pos, table, true, 800, power));
        EXPECT_EQ(power, expected[pos]);
    }
}

// an enabled but empty table leaves only pitmode on the switch
TEST(VTXPowerSwitch, EmptyEnabledTableIsPitmodeOnly)
{
    const uint16_t table[VTX_USER_POWER_LEVELS] = { 0, 0, 0, 0, 0, 0 };
    uint16_t power;
    for (int8_t pos = 0; pos < 6; pos++) {
        EXPECT_TRUE(switch_power(pos, table, true, 800, power));
        EXPECT_EQ(power, 0);
    }
}

// built-in levels: the top position offers VTX_MAX_POWER even when the
// largest active level is below it
TEST(VTXPowerSwitch, BuiltInTopPositionReachesMaxPower)
{
    const uint16_t table[VTX_USER_POWER_LEVELS] = { 0, 0, 0, 0, 0, 0 };
    uint16_t power;
    EXPECT_TRUE(switch_power(5, table, false, 750, power));
    EXPECT_EQ(power, 750);
}

// built-in levels: VTX_MAX_POWER of 0 means no cap
TEST(VTXPowerSwitch, BuiltInZeroMaxPowerMeansNoCap)
{
    const uint16_t table[VTX_USER_POWER_LEVELS] = { 0, 0, 0, 0, 0, 0 };
    uint16_t power;
    EXPECT_TRUE(switch_power(5, table, false, 0, power));
    EXPECT_EQ(power, 1000);
}

AP_GTEST_MAIN()

#endif  // AP_VIDEOTX_ENABLED
