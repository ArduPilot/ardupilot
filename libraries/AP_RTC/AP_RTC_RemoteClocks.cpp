#include "AP_RTC_RemoteClocks.h"

#if AP_RTC_REMOTE_CLOCKS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_RTC.h"

// largest change between offset samples; a remote clock that follows GPS
// time drifts from its boot time by well under this between samples
static constexpr int64_t OFFSET_CONSISTENT_US = 5000;
// consecutive consistent offset samples before the offset is used
static constexpr uint8_t OFFSET_ACQUIRE_SAMPLES = 3;
// an offset is held this long after its last sample; remote crystal
// drift moves it by a few milliseconds at most in this time
static constexpr uint32_t OFFSET_TIMEOUT_MS = 60000;
// a GPS fix report is trusted for this long, allowing 0.2Hz reports
static constexpr uint32_t GPS_FIX_TIMEOUT_MS = 15000;

AP_RTC_RemoteClocks::Source *AP_RTC_RemoteClocks::find(uint8_t sysid, uint8_t compid)
{
    for (auto &s : sources) {
        if (s.refs > 0 && s.sysid == sysid && s.compid == compid) {
            return &s;
        }
    }
    return nullptr;
}

const AP_RTC_RemoteClocks::Source *AP_RTC_RemoteClocks::find(uint8_t sysid, uint8_t compid) const
{
    for (const auto &s : sources) {
        if (s.refs > 0 && s.sysid == sysid && s.compid == compid) {
            return &s;
        }
    }
    return nullptr;
}

bool AP_RTC_RemoteClocks::register_source(uint8_t sysid, uint8_t compid)
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

void AP_RTC_RemoteClocks::unregister_source(uint8_t sysid, uint8_t compid)
{
    WITH_SEMAPHORE(sem);

    Source *s = find(sysid, compid);
    if (s != nullptr) {
        s->refs--;
    }
}

void AP_RTC_RemoteClocks::handle_system_time(uint8_t sysid, uint8_t compid, uint64_t time_unix_usec, uint32_t time_boot_ms, uint32_t now_ms)
{
    WITH_SEMAPHORE(sem);

    Source *s = find(sysid, compid);
    if (s == nullptr) {
        return;
    }

    // the remote system does not know the time
    if (time_unix_usec == 0) {
        return;
    }

    const int64_t sample_us = int64_t(time_unix_usec) - int64_t(time_boot_ms) * 1000;

    // a reboot or a step of the remote clock starts the offset again
    const int64_t change_us = sample_us - s->offset_us;
    if ((s->consistent > 0) && ((change_us > OFFSET_CONSISTENT_US) || (change_us < -OFFSET_CONSISTENT_US))) {
        s->consistent = 0;
    }

    s->offset_us = sample_us;
    s->last_rx_ms = now_ms;
    if (s->consistent < UINT8_MAX) {
        s->consistent++;
    }
}

void AP_RTC_RemoteClocks::handle_gps_fix(uint8_t sysid, uint8_t compid, uint8_t fix_type, uint32_t now_ms)
{
    WITH_SEMAPHORE(sem);

    Source *s = find(sysid, compid);
    if (s == nullptr) {
        return;
    }

    // GPS_FIX_TYPE_3D_FIX or better
    s->gps_ok = (fix_type >= 3);
    s->last_gps_ms = now_ms;
}

bool AP_RTC_RemoteClocks::remote_to_local_ms(uint8_t sysid, uint8_t compid, uint32_t remote_boot_ms, uint32_t &local_ms) const
{
    const AP_RTC &rtc = AP::rtc();
    uint64_t utc_us, local_us;
    if (!rtc.gps_locked() || !rtc.get_utc_and_local_usec(utc_us, local_us)) {
        return false;
    }
    return remote_to_local_ms_at(sysid, compid, remote_boot_ms, uint32_t(local_us / 1000U),
                                 int64_t(utc_us) - int64_t(local_us), local_ms);
}

bool AP_RTC_RemoteClocks::remote_to_local_ms_at(uint8_t sysid, uint8_t compid, uint32_t remote_boot_ms,
                                                uint32_t now_ms, int64_t local_offset_us, uint32_t &local_ms) const
{
    WITH_SEMAPHORE(sem);

    const Source *s = find(sysid, compid);
    if (s == nullptr) {
        return false;
    }
    if (s->consistent < OFFSET_ACQUIRE_SAMPLES || (now_ms - s->last_rx_ms) > OFFSET_TIMEOUT_MS) {
        return false;
    }
    if (!s->gps_ok || (now_ms - s->last_gps_ms) > GPS_FIX_TIMEOUT_MS) {
        return false;
    }

    const int64_t local_us = int64_t(remote_boot_ms) * 1000 + s->offset_us - local_offset_us;
    if (local_us < 0) {
        return false;
    }
    local_ms = uint32_t(local_us / 1000);
    return true;
}

#endif  // AP_RTC_REMOTE_CLOCKS_ENABLED
