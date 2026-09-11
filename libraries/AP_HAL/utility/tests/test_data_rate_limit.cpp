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
#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/DataRateLimit.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  DataRateLimit uses AP_HAL::micros() internally; stopping the SITL
  scheduler clock makes time - and thus these tests - deterministic
 */
static uint64_t now_us = 1000000;

static void set_time(uint64_t t_us)
{
    now_us = t_us;
    hal.scheduler->stop_clock(now_us);
}

static void advance_time(uint64_t dt_us)
{
    set_time(now_us + dt_us);
}

TEST(DataRateLimit, Basic)
{
    set_time(1000000);
    DataRateLimit limit {};

    // first call has seen 1 second elapse since construction (time
    // zero), so a 1000 byte/s limit yields 1000 bytes
    EXPECT_EQ(limit.max_bytes(1000.0f), 1000U);

    // no time elapsed, no bytes allowed
    EXPECT_EQ(limit.max_bytes(1000.0f), 0U);

    // half a second at 1000 bytes/s
    advance_time(500000);
    EXPECT_EQ(limit.max_bytes(1000.0f), 500U);

    // 1ms at 1000 bytes/s is a single byte
    advance_time(1000);
    EXPECT_EQ(limit.max_bytes(1000.0f), 1U);
}

TEST(DataRateLimit, ZeroRate)
{
    set_time(2000000);
    DataRateLimit limit {};

    EXPECT_EQ(limit.max_bytes(0.0f), 0U);
    advance_time(1000000);
    EXPECT_EQ(limit.max_bytes(0.0f), 0U);
}

TEST(DataRateLimit, RemainderAccumulates)
{
    set_time(3000000);
    DataRateLimit limit {};
    // flush the time since construction
    limit.max_bytes(5.0f);

    // 5 bytes/s polled every 100ms is 0.5 bytes per call; the
    // fractional part must carry over rather than being lost
    uint32_t total = 0;
    for (uint8_t i=0; i<10; i++) {
        advance_time(100000);
        const uint32_t n = limit.max_bytes(5.0f);
        EXPECT_LE(n, 1U);
        total += n;
    }
    EXPECT_EQ(total, 5U);
}

TEST(DataRateLimit, HighRate)
{
    set_time(5000000);
    DataRateLimit limit {};
    limit.max_bytes(1.0e6f);

    advance_time(1000000);
    EXPECT_EQ(limit.max_bytes(1.0e6f), 1000000U);
}

AP_GTEST_MAIN()
