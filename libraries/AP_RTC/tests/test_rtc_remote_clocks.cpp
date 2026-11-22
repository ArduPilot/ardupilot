/*
  Unit tests for AP_RTC_RemoteClocks.
 */

#include <AP_gtest.h>

#include <AP_RTC/AP_RTC_RemoteClocks.h>

#if AP_RTC_REMOTE_CLOCKS_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static constexpr uint32_t TARGET_SYSID = 7;
static constexpr uint8_t TARGET_COMPID = 1;

// local UTC minus local boot time
static constexpr int64_t LOCAL_OFFSET_US = 1760000000000000LL;

// our UTC at local time now_ms
static uint64_t local_utc_us(uint32_t now_ms)
{
    return uint64_t(int64_t(now_ms) * 1000 + LOCAL_OFFSET_US);
}

// tests start this long after our boot
static constexpr uint32_t START_MS = 2000000;

// a remote component that booted at local time boot_start_ms, whose boot
// clock runs fast by crystal_error, and whose UTC follows GPS time as ours does
struct Remote {
    uint32_t sysid;
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
                                     rx_ms != 0 ? rx_ms : local_ms, local_locked, local_utc_us(rx_ms != 0 ? rx_ms : local_ms));
    }
};

// the target booted 1234.567 s before the tests start
static const Remote target { TARGET_SYSID, TARGET_COMPID, START_MS - 1234567, 0.0 };

static bool convert(AP_RTC_RemoteClocks &clocks, uint32_t remote_boot_ms, uint32_t now_ms, uint32_t &local_ms)
{
    return clocks.remote_to_local_ms_at(TARGET_SYSID, TARGET_COMPID, remote_boot_ms, uint64_t(now_ms) * 1000,
                                        local_utc_us(now_ms), local_ms);
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
    AP_RTC_RemoteClocks clocks {};
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

TEST(AP_RTC_RemoteClocks, acquire_and_convert)
{
    AP_RTC_RemoteClocks clocks {};
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
    AP_RTC_RemoteClocks clocks {};
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    // small drift is accepted
    target.send_time(clocks, now_ms, 2000);
    EXPECT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));

    // a reboot or clock step must be confirmed again before use
    target.send_time(clocks, now_ms, 13000);
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    now_ms += 5000;
    target.send_time(clocks, now_ms, 13000);
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    now_ms += 5000;
    target.send_time(clocks, now_ms, 13000);
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    EXPECT_EQ(now_ms + 13, local_ms);
}

TEST(AP_RTC_RemoteClocks, unknown_remote_time_ignored)
{
    AP_RTC_RemoteClocks clocks {};
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<5; i++) {
        clocks.handle_system_time_at(TARGET_SYSID, TARGET_COMPID, 0, target.boot_ms(now_ms), now_ms, true, local_utc_us(now_ms));
        now_ms += 1000;
    }
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

TEST(AP_RTC_RemoteClocks, acquisition_requires_gps_fix)
{
    AP_RTC_RemoteClocks clocks {};
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
    AP_RTC_RemoteClocks clocks {};
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

// the newest offset is held for two minutes, which a 50ppm boot clock crystal moves
// by 6ms, and a late copy of it arriving over a second link doesn't extend the hold
TEST(AP_RTC_RemoteClocks, offset_held_two_minutes)
{
    for (const double crystal_error : { 50.0e-6, -50.0e-6 }) {
        for (const uint8_t num_samples : { 3, 60 }) {
            SCOPED_TRACE(crystal_error * 1e6);
            SCOPED_TRACE(int(num_samples));
            const Remote drifting { TARGET_SYSID, TARGET_COMPID, target.boot_start_ms, crystal_error };
            AP_RTC_RemoteClocks clocks {};
            ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
            uint32_t now_ms = START_MS;
            clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
            for (uint8_t i=0; i<num_samples; i++) {
                clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
                drifting.send_time(clocks, now_ms);
                if (i > 0) {
                    drifting.send_time(clocks, now_ms - 5000, 0, now_ms);
                }
                now_ms += 5000;
            }
            const uint32_t last_rx_ms = now_ms - 5000;
            drifting.send_time(clocks, last_rx_ms, 0, last_rx_ms + 7000);

            const uint32_t end_ms = last_rx_ms + 120000;
            uint32_t local_ms;
            ASSERT_TRUE(convert(clocks, drifting.boot_ms(end_ms), end_ms, local_ms));
            EXPECT_NEAR(double(end_ms), double(local_ms), 8.0);
            EXPECT_FALSE(convert(clocks, drifting.boot_ms(end_ms + 1), end_ms + 1, local_ms));
        }
    }
}

// a component that has lost its fix keeps sending a consistent SYSTEM_TIME from a
// clock drifting away from GPS time, so that alone doesn't hold the offset
TEST(AP_RTC_RemoteClocks, lost_fix_expires)
{
    AP_RTC_RemoteClocks clocks {};
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    const uint32_t last_fix_ms = now_ms;
    uint32_t local_ms;
    while (now_ms < last_fix_ms + 120000) {
        now_ms += 5000;
        clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 0, now_ms);
        target.send_time(clocks, now_ms);
        EXPECT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    }
    now_ms += 5000;
    target.send_time(clocks, now_ms);
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

// the offset may move as far as crystal drift takes it over the hold
TEST(AP_RTC_RemoteClocks, drift_over_hold_is_consistent)
{
    AP_RTC_RemoteClocks clocks {};
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);
    now_ms += 110000;
    target.send_time(clocks, now_ms, 8000);
    uint32_t local_ms;
    ASSERT_TRUE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
    EXPECT_EQ(now_ms + 8, local_ms);
}

// copies of samples arriving late over a second link don't count towards acquisition
TEST(AP_RTC_RemoteClocks, late_copies_do_not_acquire)
{
    AP_RTC_RemoteClocks clocks {};
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
    AP_RTC_RemoteClocks clocks {};
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
    AP_RTC_RemoteClocks clocks {};
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

// a message can't arrive before it was sent, so a remote clock ahead of GPS time is never acquired
TEST(AP_RTC_RemoteClocks, clock_ahead_never_acquired)
{
    AP_RTC_RemoteClocks clocks {};
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
    AP_RTC_RemoteClocks clocks {};
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
    AP_RTC_RemoteClocks clocks {};
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
    AP_RTC_RemoteClocks clocks {};
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
    AP_RTC_RemoteClocks clocks {};
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
    AP_RTC_RemoteClocks clocks {};

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
    AP_RTC_RemoteClocks clocks {};
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<3; i++) {
        for (uint16_t sysid=1; sysid<=255; sysid++) {
            if (sysid == TARGET_SYSID) {
                continue;
            }
            const Remote other { sysid, TARGET_COMPID, target.boot_start_ms + sysid * 1000U, 0.0 };
            other.send_time(clocks, now_ms);
            clocks.handle_gps_fix(sysid, TARGET_COMPID, 0, now_ms);
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
    AP_RTC_RemoteClocks clocks {};
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

// MAVLink system IDs are 32 bits, so a system sharing only the low byte of
// the registered one must not affect it
TEST(AP_RTC_RemoteClocks, keyed_by_full_sysid)
{
    AP_RTC_RemoteClocks clocks {};
    const Remote wide { 0xFFFFFFFE, TARGET_COMPID, target.boot_start_ms, 0.0 };
    const Remote alias { 0xFE, TARGET_COMPID, target.boot_start_ms + 99000U, 0.0 };
    ASSERT_TRUE(clocks.register_source(wide.sysid, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    clocks.handle_gps_fix(wide.sysid, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<3; i++) {
        wide.send_time(clocks, now_ms);
        alias.send_time(clocks, now_ms);
        clocks.handle_gps_fix(alias.sysid, TARGET_COMPID, 0, now_ms);
        now_ms += 5000;
    }
    now_ms -= 5000;
    uint32_t local_ms;
    ASSERT_TRUE(clocks.remote_to_local_ms_at(wide.sysid, TARGET_COMPID, wide.boot_ms(now_ms), uint64_t(now_ms) * 1000,
                                             local_utc_us(now_ms), local_ms));
    EXPECT_EQ(now_ms, local_ms);
    EXPECT_FALSE(clocks.remote_to_local_ms_at(alias.sysid, TARGET_COMPID, alias.boot_ms(now_ms), uint64_t(now_ms) * 1000,
                                              local_utc_us(now_ms), local_ms));
}

// the conversion uses the local time to the microsecond, so a local time part
// way through a millisecond does not round the result a millisecond early
TEST(AP_RTC_RemoteClocks, conversion_exact_within_a_millisecond)
{
    AP_RTC_RemoteClocks clocks {};
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = START_MS;
    acquire(clocks, now_ms);

    // a message sent exactly 9 ms before a local time 0.7 ms into now_ms
    const uint64_t local_us = uint64_t(now_ms) * 1000 + 700;
    uint32_t local_ms;
    ASSERT_TRUE(clocks.remote_to_local_ms_at(TARGET_SYSID, TARGET_COMPID, target.boot_ms(now_ms - 9),
                                             local_us, local_utc_us(now_ms) + 700, local_ms));
    EXPECT_EQ(now_ms - 9, local_ms);
}

// our 32 bit boot time in milliseconds wraps after 49.7 days of uptime, and
// samples and conversions either side of the wrap must still be consistent
TEST(AP_RTC_RemoteClocks, local_boot_time_wrap)
{
    AP_RTC_RemoteClocks clocks {};
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));

    // local times here are 64 bit; the target booted an hour before them
    const uint64_t wrap_ms = uint64_t(1) << 32;
    const uint64_t target_boot_start_ms = wrap_ms - 3600000;
    const auto utc_us = [](uint64_t t_ms) { return t_ms * 1000 + uint64_t(LOCAL_OFFSET_US); };
    const auto target_boot_ms = [&](uint64_t t_ms) { return uint32_t(t_ms - target_boot_start_ms); };

    // acquire before the wrap and keep receiving samples past it; the
    // samples are close enough that no late message looks like a reboot
    uint64_t t_ms = wrap_ms - 12000;
    for (uint8_t i=0; i<6; i++) {
        clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, uint32_t(t_ms));
        clocks.handle_system_time_at(TARGET_SYSID, TARGET_COMPID, utc_us(t_ms), target_boot_ms(t_ms),
                                     uint32_t(t_ms), true, utc_us(t_ms));
        t_ms += 3000;
    }

    // a message sent a second ago, after the wrap
    uint32_t local_ms;
    ASSERT_TRUE(clocks.remote_to_local_ms_at(TARGET_SYSID, TARGET_COMPID, target_boot_ms(t_ms - 1000),
                                             t_ms * 1000, utc_us(t_ms), local_ms));
    EXPECT_EQ(uint32_t(t_ms - 1000), local_ms);

    // a message sent just before the wrap
    ASSERT_TRUE(clocks.remote_to_local_ms_at(TARGET_SYSID, TARGET_COMPID, target_boot_ms(wrap_ms - 500),
                                             t_ms * 1000, utc_us(t_ms), local_ms));
    EXPECT_EQ(uint32_t(wrap_ms - 500), local_ms);
}

#endif  // AP_RTC_REMOTE_CLOCKS_ENABLED

AP_GTEST_MAIN()
