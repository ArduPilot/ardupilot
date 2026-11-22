#include "AP_RTC_RemoteClocks.h"

#if AP_RTC_REMOTE_CLOCKS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_RTC.h"

// largest change between offset samples from a consistent clock; a 50ppm
// crystal error moves the offset 6ms over OFFSET_HOLD_MS
static constexpr int64_t OFFSET_CONSISTENT_US = 10000;
// consecutive consistent offset samples before the offset is acquired
static constexpr uint8_t OFFSET_ACQUIRE_SAMPLES = 3;
// an acquired offset is held this long after its newest sample, and after the
// component last reported a 3D fix, as its clock then drifts from GPS time
static constexpr uint32_t OFFSET_HOLD_MS = 120000;
// at acquisition a GPS fix report must be this recent, allowing 0.2Hz reports
static constexpr uint32_t GPS_FIX_TIMEOUT_MS = 15000;

AP_RTC_RemoteClocks::Source *AP_RTC_RemoteClocks::find(uint32_t sysid, uint8_t compid)
{
    for (auto &s : sources) {
        if (s.refs > 0 && s.sysid == sysid && s.compid == compid) {
            return &s;
        }
    }
    return nullptr;
}

bool AP_RTC_RemoteClocks::register_source(uint32_t sysid, uint8_t compid)
{
    WITH_SEMAPHORE(sem);

    Source *s = find(sysid, compid);
    if (s != nullptr) {
        if (s->refs == UINT8_MAX) {
            return false;
        }
        s->refs++;
        return true;
    }
    for (auto &free_source : sources) {
        if (free_source.refs == 0) {
            free_source = Source {};
            free_source.sysid = sysid;
            free_source.compid = compid;
            free_source.refs = 1;
            return true;
        }
    }
    return false;
}

void AP_RTC_RemoteClocks::unregister_source(uint32_t sysid, uint8_t compid)
{
    WITH_SEMAPHORE(sem);

    Source *s = find(sysid, compid);
    if (s != nullptr) {
        s->refs--;
    }
}

// forget everything learned about a source except its registration
void AP_RTC_RemoteClocks::reset(Source &s)
{
    const Source registration = s;
    s = Source {};
    s.sysid = registration.sysid;
    s.compid = registration.compid;
    s.refs = registration.refs;
}

// track the latest remote boot time, resetting the source if it rebooted
void AP_RTC_RemoteClocks::note_boot(Source &s, uint32_t boot_ms)
{
    if (s.have_boot) {
        const int32_t change_ms = int32_t(boot_ms - s.latest_boot_ms);
        if (change_ms < -int32_t(AP_RTC_REMOTE_REBOOT_ALLOWANCE_MS)) {
            reset(s);
        } else if (change_ms <= 0) {
            return;
        }
    }
    s.have_boot = true;
    s.latest_boot_ms = boot_ms;
}

void AP_RTC_RemoteClocks::set_offset(Source &s, int64_t sample_us, uint32_t boot_ms, uint32_t now_ms)
{
    s.offset_us = sample_us;
    s.offset_boot_ms = boot_ms;
    s.last_rx_ms = now_ms;
}

void AP_RTC_RemoteClocks::try_acquire(Source &s, uint32_t now_ms)
{
    if (!s.acquired &&
        (s.consistent >= OFFSET_ACQUIRE_SAMPLES) &&
        s.gps_ok && (now_ms - s.last_gps_ms) <= GPS_FIX_TIMEOUT_MS) {
        s.acquired = true;
    }
}

// the offset must be acquired again from new samples
void AP_RTC_RemoteClocks::restart_acquisition(Source &s)
{
    s.consistent = 0;
    s.acquired = false;
}

void AP_RTC_RemoteClocks::handle_system_time(uint32_t sysid, uint8_t compid, uint64_t time_unix_usec, uint32_t time_boot_ms, uint32_t now_ms)
{
    const AP_RTC &rtc = AP::rtc();
    uint64_t utc_us, local_us;
    const bool locked = rtc.gps_locked() && rtc.get_utc_and_local_usec(utc_us, local_us);
    handle_system_time_at(sysid, compid, time_unix_usec, time_boot_ms, now_ms, locked, locked ? utc_us : 0);
}

void AP_RTC_RemoteClocks::handle_system_time_at(uint32_t sysid, uint8_t compid, uint64_t time_unix_usec, uint32_t time_boot_ms,
                                                uint32_t now_ms, bool local_locked, uint64_t local_utc_us)
{
    WITH_SEMAPHORE(sem);

    Source *s = find(sysid, compid);
    if (s == nullptr) {
        return;
    }

    note_boot(*s, time_boot_ms);

    // the remote system does not know the time
    if (time_unix_usec == 0) {
        return;
    }

    // a sample can only be judged against a clock that follows GPS time
    if (!local_locked) {
        return;
    }

    // a sample sent after it arrived comes from a remote clock running ahead of GPS time
    const int64_t ahead_us = int64_t(time_unix_usec) - int64_t(local_utc_us);
    if (ahead_us > int64_t(AP_RTC_REMOTE_CLOCK_TOLERANCE_MS) * 1000) {
        restart_acquisition(*s);
        return;
    }

    const int64_t sample_us = int64_t(time_unix_usec) - int64_t(time_boot_ms) * 1000;

    if (s->consistent == 0) {
        set_offset(*s, sample_us, time_boot_ms, now_ms);
        s->consistent = 1;
    } else {
        const int64_t change_us = sample_us - s->offset_us;
        if ((change_us > OFFSET_CONSISTENT_US) || (change_us < -OFFSET_CONSISTENT_US)) {
            // the remote clock stepped, so the offset must be acquired again
            set_offset(*s, sample_us, time_boot_ms, now_ms);
            s->consistent = 1;
            s->acquired = false;
        } else if (int32_t(time_boot_ms - s->offset_boot_ms) > 0) {
            // only a newer sample counts, not a copy arriving late over a slower link
            set_offset(*s, sample_us, time_boot_ms, now_ms);
            if (s->consistent < UINT8_MAX) {
                s->consistent++;
            }
        }
    }

    try_acquire(*s, now_ms);
}

void AP_RTC_RemoteClocks::handle_gps_fix(uint32_t sysid, uint8_t compid, uint8_t fix_type, uint32_t now_ms)
{
    WITH_SEMAPHORE(sem);

    Source *s = find(sysid, compid);
    if (s == nullptr) {
        return;
    }

    // GPS_FIX_TYPE_3D_FIX or better; only acquisition depends on it
    s->gps_ok = (fix_type >= 3);
    if (s->gps_ok) {
        s->last_gps_ms = now_ms;
    }

    try_acquire(*s, now_ms);
}

bool AP_RTC_RemoteClocks::remote_to_local_ms(uint32_t sysid, uint8_t compid, uint32_t remote_boot_ms, uint32_t &local_ms)
{
    const AP_RTC &rtc = AP::rtc();
    uint64_t utc_us, local_us;
    if (!rtc.gps_locked() || !rtc.get_utc_and_local_usec(utc_us, local_us)) {
        // still check the boot time for a reboot
        WITH_SEMAPHORE(sem);
        Source *s = find(sysid, compid);
        if (s != nullptr) {
            note_boot(*s, remote_boot_ms);
        }
        return false;
    }
    return remote_to_local_ms_at(sysid, compid, remote_boot_ms, local_us, utc_us, local_ms);
}

bool AP_RTC_RemoteClocks::remote_to_local_ms_at(uint32_t sysid, uint8_t compid, uint32_t remote_boot_ms,
                                                uint64_t local_us, uint64_t local_utc_us, uint32_t &local_ms)
{
    WITH_SEMAPHORE(sem);

    const uint32_t now_ms = uint32_t(local_us / 1000U);

    Source *s = find(sysid, compid);
    if (s == nullptr) {
        return false;
    }

    note_boot(*s, remote_boot_ms);

    if (!s->acquired ||
        (now_ms - s->last_rx_ms) > OFFSET_HOLD_MS ||
        (now_ms - s->last_gps_ms) > OFFSET_HOLD_MS) {
        return false;
    }

    // when the message was sent relative to now, by the two GPS-disciplined clocks
    const int64_t sent_after_now_us = int64_t(remote_boot_ms) * 1000 + s->offset_us - int64_t(local_utc_us);

    // a message can't arrive before it was sent, so an offset putting it in the future is wrong
    if (sent_after_now_us > int64_t(AP_RTC_REMOTE_CLOCK_TOLERANCE_MS) * 1000) {
        restart_acquisition(*s);
        return false;
    }

    // a message sent before this system booted can't be converted
    const int64_t sent_local_us = int64_t(local_us) + sent_after_now_us;
    if (sent_local_us < 0) {
        return false;
    }

    // in milliseconds, wrapping like AP_HAL::millis()
    local_ms = uint32_t(uint64_t(sent_local_us) / 1000U);
    return true;
}

#endif  // AP_RTC_REMOTE_CLOCKS_ENABLED
