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

// a remote component whose boot clock started 1234.5678 s after ours
struct Remote {
    uint8_t sysid;
    uint8_t compid;
    int64_t offset_us;      // remote UTC minus remote boot time
    uint32_t boot_ms(uint32_t local_ms) const { return uint32_t(local_ms - 1234567); }
    uint64_t unix_usec(uint32_t local_ms) const { return uint64_t(int64_t(boot_ms(local_ms)) * 1000 + offset_us); }
    void send_time(AP_RTC_RemoteClocks &clocks, uint32_t local_ms, int64_t error_us=0) const {
        clocks.handle_system_time(sysid, compid, unix_usec(local_ms) + error_us, boot_ms(local_ms), local_ms);
    }
};

// the remote's offset when both clocks follow GPS time exactly
static const Remote target { TARGET_SYSID, TARGET_COMPID, LOCAL_OFFSET_US + 1234567000LL };

static bool convert(const AP_RTC_RemoteClocks &clocks, uint32_t remote_boot_ms, uint32_t now_ms, uint32_t &local_ms)
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
    uint32_t now_ms = 100000;
    acquire(clocks, now_ms);
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

TEST(AP_RTC_RemoteClocks, acquire_and_convert)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));

    uint32_t now_ms = 100000;
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
    uint32_t now_ms = 100000;
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
    uint32_t now_ms = 100000;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<5; i++) {
        clocks.handle_system_time(TARGET_SYSID, TARGET_COMPID, 0, target.boot_ms(now_ms), now_ms);
        now_ms += 1000;
    }
    uint32_t local_ms;
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms), now_ms, local_ms));
}

TEST(AP_RTC_RemoteClocks, requires_recent_gps_fix)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = 100000;
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

    // the fix report goes stale after 15s
    target.send_time(clocks, now_ms + 15000);
    EXPECT_TRUE(convert(clocks, target.boot_ms(now_ms + 15000), now_ms + 15000, local_ms));
    EXPECT_FALSE(convert(clocks, target.boot_ms(now_ms + 15001), now_ms + 15001, local_ms));
}

TEST(AP_RTC_RemoteClocks, offset_held_for_60s)
{
    AP_RTC_RemoteClocks clocks;
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    uint32_t now_ms = 100000;
    acquire(clocks, now_ms);
    const uint32_t last_rx_ms = now_ms - 5000;
    uint32_t local_ms;
    for (uint32_t t = now_ms; t <= last_rx_ms + 60000; t += 10000) {
        clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, t);
        EXPECT_TRUE(convert(clocks, target.boot_ms(t), t, local_ms));
    }
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, last_rx_ms + 60001);
    EXPECT_FALSE(convert(clocks, target.boot_ms(last_rx_ms + 60001), last_rx_ms + 60001, local_ms));
}

TEST(AP_RTC_RemoteClocks, registrations_counted_and_limited)
{
    AP_RTC_RemoteClocks clocks;

    // the same source registered twice needs two unregistrations
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    ASSERT_TRUE(clocks.register_source(TARGET_SYSID, TARGET_COMPID));
    clocks.unregister_source(TARGET_SYSID, TARGET_COMPID);
    uint32_t now_ms = 100000;
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
    uint32_t now_ms = 100000;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    for (uint8_t i=0; i<3; i++) {
        for (uint16_t sysid=1; sysid<=255; sysid++) {
            if (sysid == TARGET_SYSID) {
                continue;
            }
            const Remote other { uint8_t(sysid), TARGET_COMPID, LOCAL_OFFSET_US + sysid * 1000000LL };
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
    uint32_t now_ms = 100000;
    clocks.handle_gps_fix(TARGET_SYSID, TARGET_COMPID, 3, now_ms);
    const Remote companion { TARGET_SYSID, 191, LOCAL_OFFSET_US + 99000000LL };
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
