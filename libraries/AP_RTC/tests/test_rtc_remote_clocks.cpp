/*
  Unit tests for AP_RTC_RemoteClocks.
 */

#include <AP_gtest.h>

#include <AP_RTC/AP_RTC_RemoteClocks.h>

#if AP_RTC_REMOTE_CLOCKS_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static constexpr uint8_t TARGET_SYSID = 7;
static constexpr uint8_t TARGET_COMPID = 1;

// local UTC minus local boot time
static constexpr int64_t LOCAL_OFFSET_US = 1760000000000000LL;

// tests start this long after our boot
static constexpr uint32_t START_MS = 2000000;

// a remote component that booted at local time boot_start_ms, whose boot
// clock runs fast by crystal_error, and whose UTC follows GPS time as ours does
struct Remote {
    uint8_t sysid;
    uint8_t compid;
    uint32_t boot_start_ms;
    double crystal_error;
    uint32_t boot_ms(uint32_t local_ms) const {
        return uint32_t(llround((double(local_ms) - double(boot_start_ms)) * (1.0 + crystal_error)));
    }
    uint64_t unix_usec(uint32_t local_ms) const { return uint64_t(int64_t(local_ms) * 1000 + LOCAL_OFFSET_US); }
    // a SYSTEM_TIME sampled at local_ms, received at rx_ms (at once if zero) by a clock
    // following GPS time (unless local_locked is false)
    void send_time(AP_RTC_RemoteClocks &clocks, uint32_t local_ms, int64_t error_us=0, uint32_t rx_ms=0, bool local_locked=true) const {
        clocks.handle_system_time_at(sysid, compid, unix_usec(local_ms) + error_us, boot_ms(local_ms),
                                     rx_ms != 0 ? rx_ms : local_ms, local_locked, LOCAL_OFFSET_US);
    }
};

// the target booted 1234.567 s before the tests start
static const Remote target { TARGET_SYSID, TARGET_COMPID, START_MS - 1234567, 0.0 };

static bool convert(AP_RTC_RemoteClocks &clocks, uint32_t remote_boot_ms, uint32_t now_ms, uint32_t &local_ms)
{
    return clocks.remote_to_local_ms_at(TARGET_SYSID, TARGET_COMPID, remote_boot_ms, now_ms, LOCAL_OFFSET_US, local_ms);
}

// three consistent samples and a GPS fix make the conversion available
static void acquire(AP_RTC_RemoteClocks &clocks, uint32_t &now_ms)
{
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<3; i++) {
        target.send_time(clocks, now_ms);
        now_ms += 5000;
    }
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
}

TEST(AP_RTC_RemoteClocks, unregistered_source_ignored)
{
    AP_RTC_RemoteClocks clocks;
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

TEST(AP_RTC_RemoteClocks, acquire_and_convert)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));

    uint32_t now_ms = START_MS;
    uint32_t local_ms;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    target.send_time(clocks, now_ms);
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    now_ms += 5000;
    target.send_time(clocks, now_ms);
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    now_ms += 5000;
    target.send_time(clocks, now_ms);

    // a sample stamped 800ms ago converts to 800ms ago
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms - 800), now_ms, local_ms));
    EXPECT_EQ(now_ms - 800, local_ms);
}

TEST(AP_RTC_RemoteClocks, inconsistent_offset_reacquires)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    // small drift is accepted
    target.send_time(clocks, now_ms, 2000);
    EXPECT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    // a reboot or clock step must be confirmed again before use
    target.send_time(clocks, now_ms, 10000);
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    now_ms += 5000;
    target.send_time(clocks, now_ms, 10000);
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    now_ms += 5000;
    target.send_time(clocks, now_ms, 10000);
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    EXPECT_EQ(now_ms + 10, local_ms);
}

TEST(AP_RTC_RemoteClocks, unknown_remote_time_ignored)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<5; i++) {
        clocks.handle_system_time_at(TARGET_SYSID, TARGET_COMPID, 0, target.boot_ms(now_ms), now_ms, true, LOCAL_OFFSET_US);
        now_ms += 1000;
    }
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

TEST(AP_RTC_RemoteClocks, acquisition_requires_gps_fix)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    uint32_t local_ms;

    // no fix reported
    for (uint8_t i=0; i<3; i++) {
        target.send_time(clocks, now_ms);
        now_ms += 1000;
    }
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    // a 2D fix is not enough
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 2, now_ms);
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    EXPECT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    // once acquired, a lost or missing fix report does not drop the offset
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 0, now_ms);
    EXPECT_TRUE(convert(clocks, target.boot_ms(now_ms + 50000), now_ms + 50000, local_ms));
}

TEST(AP_RTC_RemoteClocks, acquisition_rejects_stale_gps_fix)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    now_ms += 15001;
    for (uint8_t i=0; i<3; i++) {
        target.send_time(clocks, now_ms);
        now_ms += 1000;
    }
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

// once the drift is fitted over two minutes the offset is held for 10 minutes
TEST(AP_RTC_RemoteClocks, offset_held_for_10_minutes)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<25; i++) {
        target.send_time(clocks, now_ms);
        now_ms += 5000;
    }
    const uint32_t last_rx_ms = now_ms - 5000;
    uint32_t local_ms;
    for (uint32_t t = now_ms; t <= last_rx_ms + 600000; t += 60000) {
        ASSERT_TRUE(convert(clocks, target.boot_ms(t), t, local_ms));
        EXPECT_EQ(t, local_ms);
    }
    EXPECT_TRUE(convert(clocks, target.boot_ms(last_rx_ms + 600000), last_rx_ms + 600000, local_ms));
    EXPECT_FALSE(convert(clocks, target.boot_ms(last_rx_ms + 600001), last_rx_ms + 600001, local_ms));
}

// before the drift is fitted the newest sample is held without drift, for a minute
TEST(AP_RTC_RemoteClocks, short_fit_held_for_a_minute)
{
    const Remote drifting { TARGET_SYSID, TARGET_COMPID, target.boot_start_ms, 50.0e-6 };
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<3; i++) {
        drifting.send_time(clocks, now_ms);
        now_ms += 5000;
    }
    const uint32_t last_rx_ms = now_ms - 5000;
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, drifting.boot_ms(last_rx_ms + 59000), last_rx_ms + 59000, local_ms));
    EXPECT_NEAR(double(last_rx_ms + 59000), double(local_ms), 3.0);
    EXPECT_TRUE(convert(clocks, drifting.boot_ms(last_rx_ms + 60000), last_rx_ms + 60000, local_ms));
    EXPECT_FALSE(convert(clocks, drifting.boot_ms(last_rx_ms + 60001), last_rx_ms + 60001, local_ms));
}

// copies of samples arriving late over a second link don't count towards acquisition
TEST(AP_RTC_RemoteClocks, late_copies_do_not_acquire)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    target.send_time(clocks, now_ms);
    target.send_time(clocks, now_ms + 5000);
    target.send_time(clocks, now_ms, 0, now_ms + 7000);
    target.send_time(clocks, now_ms + 5000, 0, now_ms + 8000);
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms + 8000), now_ms + 8000, local_ms));
    target.send_time(clocks, now_ms + 10000);
    EXPECT_TRUE(convert(clocks, target.boot_ms(now_ms + 10000), now_ms + 10000, local_ms));
}

TEST(AP_RTC_RemoteClocks, reboot_resets_until_reacquired)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    // the target rebooted a second ago, so its boot time goes back by its uptime
    const Remote rebooted { TARGET_SYSID, TARGET_COMPID, now_ms - 1000, 0.0 };
    EXPECT_FALSE(convert(clocks, rebooted.boot_ms(now_ms), now_ms, local_ms));

    // consistent samples alone are not enough: the fix must be reported again
    for (uint8_t i=0; i<3; i++) {
        rebooted.send_time(clocks, now_ms);
        now_ms += 5000;
    }
    EXPECT_FALSE(convert(clocks, rebooted.boot_ms(now_ms), now_ms, local_ms));
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    ASSERT_TRUE(convert(clocks, rebooted.boot_ms(now_ms - 800), now_ms, local_ms));
    EXPECT_EQ(now_ms - 800, local_ms);
}

// messages can arrive over two links a couple of seconds apart
TEST(AP_RTC_RemoteClocks, late_messages_are_not_a_reboot)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms - 2500), now_ms, local_ms));
    EXPECT_EQ(now_ms - 2500, local_ms);
    target.send_time(clocks, now_ms - 9000, 0, now_ms);
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms - 9000), now_ms, local_ms));
    EXPECT_EQ(now_ms - 9000, local_ms);
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    EXPECT_EQ(now_ms, local_ms);
}

// the held offset follows the drift of a 50ppm boot clock crystal, also
// with every SYSTEM_TIME arriving again late over a second link
TEST(AP_RTC_RemoteClocks, held_offset_follows_boot_clock_drift)
{
    for (const bool duplicates : { false, true }) {
        SCOPED_TRACE(duplicates);
        const Remote drifting { TARGET_SYSID, TARGET_COMPID, target.boot_start_ms, 50.0e-6 };
        AP_RTC_RemoteClocks clocks;
        ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
        uint32_t now_ms = START_MS;
        clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
        // five minutes of 0.2Hz SYSTEM_TIME
        for (uint8_t i=0; i<60; i++) {
            drifting.send_time(clocks, now_ms);
            if (duplicates && i > 0) {
                drifting.send_time(clocks, now_ms - 5000, 0, now_ms);
            }
            now_ms += 5000;
        }
        // then ten minutes without
        const uint32_t check_ms = now_ms - 5000 + 599000;
        uint32_t local_ms;
        ASSERT_TRUE(convert(clocks, drifting.boot_ms(check_ms), check_ms, local_ms));
        EXPECT_NEAR(double(check_ms), double(local_ms), 3.0);
    }
}

// a message can't arrive before it was sent, so a remote clock ahead of GPS time is never acquired
TEST(AP_RTC_RemoteClocks, clock_ahead_never_acquired)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<10; i++) {
        target.send_time(clocks, now_ms, 200000);
        now_ms += 5000;
    }
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

// a remote clock jumping ahead after lock is never acquired again
TEST(AP_RTC_RemoteClocks, clock_ahead_after_lock_not_reacquired)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    for (uint8_t i=0; i<10; i++) {
        target.send_time(clocks, now_ms, 200000);
        clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
        now_ms += 5000;
    }
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

// a timestamp converting into the future shows the offset is wrong, so it is dropped
TEST(AP_RTC_RemoteClocks, conversion_into_future_drops_offset)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms + 100), now_ms, local_ms));
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    // and is acquired again from new consistent samples
    acquire(clocks, now_ms);
    EXPECT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

// clock errors within the tolerance are accepted
TEST(AP_RTC_RemoteClocks, small_lead_within_tolerance)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<3; i++) {
        target.send_time(clocks, now_ms, 15000);
        now_ms += 5000;
    }
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms - 5000), now_ms - 5000, local_ms));
    EXPECT_EQ(now_ms - 5000 + 15, local_ms);
}

// causality can only be judged by a clock that follows GPS time
TEST(AP_RTC_RemoteClocks, ignored_while_not_locked)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<5; i++) {
        target.send_time(clocks, now_ms, 0, 0, false);
        now_ms += 1000;
    }
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

TEST(AP_RTC_RemoteClocks, registrations_counted_and_limited)
{
    AP_RTC_RemoteClocks clocks;

    // the same source registered twice needs two unregistrations
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    clocks.unregister_source(TARGET_SYSID, TARGET_COMPID);
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    EXPECT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    clocks.unregister_source(TARGET_SYSID, TARGET_COMPID);
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    // the table holds AP_RTC_REMOTE_CLOCKS_MAX sources
    for (uint8_t i=0; i<AP_RTC_REMOTE_CLOCKS_MAX; i++) {
        EXPECT_TRUE(clocks.register_source(100 + i, TARGET_COMPID));
    }
    EXPECT_FALSE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    clocks.unregister_source(100, TARGET_COMPID);
    EXPECT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));

    // a re-registered source starts with no offset
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

// SYSTEM_TIME from every other system in a full MAVLink network must not
// disturb the registered target
TEST(AP_RTC_RemoteClocks, swarm_does_not_disturb_target)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<3; i++) {
        for (uint16_t sysid=1; sysid<=255; sysid++) {
            if (sysid == TARGET_SYSID) {
                continue;
            }
            const Remote other { uint8_t(sysid), TARGET_COMPID, target.boot_start_ms + sysid * 1000U, 0.0 };
            other.send_time(clocks, now_ms);
            clocks.handle_gps_fix(uint8_t(sysid), TARGET_COMPID, 0, now_ms);
        }
        target.send_time(clocks, now_ms);
        now_ms += 5000;
    }
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms - 5000), now_ms - 5000, local_ms));
    EXPECT_EQ(now_ms - 5000, local_ms);
}

// each component has its own boot clock, so another component of the
// target system must not affect the registered one
TEST(AP_RTC_RemoteClocks, keyed_by_component)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    const Remote companion { TARGET_SYSID, 191, target.boot_start_ms + 99000U, 0.0 };
    for (uint8_t i=0; i<3; i++) {
        target.send_time(clocks, now_ms);
        companion.send_time(clocks, now_ms);
        clocks.handle_gps_fix(TARGET_SYSID, 191, 0, now_ms);
        now_ms += 5000;
    }
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms - 5000), now_ms - 5000, local_ms));
    EXPECT_EQ(now_ms - 5000, local_ms);
}

#endif  // AP_RTC_REMOTE_CLOCKS_ENABLED

AP_GTEST_MAIN()
