#include "AP_RTC_RemoteClocks.h"

#if AP_RTC_REMOTE_CLOCKS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_RTC.h"

// largest offset sample residual; a remote clock that follows GPS time
// stays far closer than this to the fitted drift of its boot clock
static constexpr int64_t OFFSET_CONSISTENT_US = 5000;
// consecutive consistent offset samples before the offset is acquired
static constexpr uint8_t OFFSET_ACQUIRE_SAMPLES = 3;
// an acquired offset is held this long after its last sample once its drift
// is fitted; the fitted drift keeps the extrapolated offset within a few milliseconds
static constexpr uint32_t OFFSET_HOLD_MS = 600000;
// until then the newest sample is held without drift for this long, which
// crystal drift of up to 50ppm moves by at most 3 milliseconds
static constexpr uint32_t OFFSET_SHORT_HOLD_MS = 60000;
// samples spanning this long fit the drift well enough to extrapolate it
static constexpr int32_t FIT_MIN_SPAN_MS = 120000;
// at acquisition a GPS fix report must be this recent, allowing 0.2Hz reports
static constexpr uint32_t GPS_FIX_TIMEOUT_MS = 15000;
// time constant of the offset drift fit
static constexpr double FIT_TC_S = 600.0;
// boot clock crystal error limit, microseconds per second
static constexpr double FIT_RATE_MAX = 200.0;

AP_RTC_RemoteClocks::Source *AP_RTC_RemoteClocks::find(uint8_t sysid, uint8_t compid)
{
    for (auto &s : sources) {
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

void AP_RTC_RemoteClocks::restart_fit(Source &s, int64_t sample_us, uint32_t boot_ms)
{
    s.newest_offset_us = sample_us;
    s.anchor_offset_us = sample_us;
    s.anchor_boot_ms = boot_ms;
    s.fit_boot_ms = boot_ms;
    s.sw = 1;
    s.st = 0;
    s.sy = 0;
    s.stt = 0;
    s.sty = 0;
    s.fit_y0_us = 0;
    s.fit_rate = 0;
}

void AP_RTC_RemoteClocks::add_to_fit(Source &s, int64_t sample_us, uint32_t boot_ms)
{
    const double t = double(int32_t(boot_ms - s.anchor_boot_ms)) / 1000;
    const double y = double(sample_us - s.anchor_offset_us);
    const double dt = double(int32_t(boot_ms - s.fit_boot_ms)) / 1000;

    const double decay = exp(-dt / FIT_TC_S);
    s.sw = s.sw * decay + 1;
    s.st = s.st * decay + t;
    s.sy = s.sy * decay + y;
    s.stt = s.stt * decay + t * t;
    s.sty = s.sty * decay + t * y;
    s.fit_boot_ms = boot_ms;
    s.newest_offset_us = sample_us;

    const double det = s.sw * s.stt - s.st * s.st;
    if (det > 1.0e-6 * s.sw * s.stt) {
        const double rate = (s.sw * s.sty - s.st * s.sy) / det;
        s.fit_rate = MAX(-FIT_RATE_MAX, MIN(FIT_RATE_MAX, rate));
    }
    s.fit_y0_us = (s.sy - s.fit_rate * s.st) / s.sw;
}

int64_t AP_RTC_RemoteClocks::offset_at(const Source &s, uint32_t boot_ms) const
{
    const double t = double(int32_t(boot_ms - s.anchor_boot_ms)) / 1000;
    return s.anchor_offset_us + int64_t(s.fit_y0_us + s.fit_rate * t);
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

void AP_RTC_RemoteClocks::handle_system_time(uint8_t sysid, uint8_t compid, uint64_t time_unix_usec, uint32_t time_boot_ms, uint32_t now_ms)
{
    // UTC minus boot time doesn't depend on when it is read
    const AP_RTC &rtc = AP::rtc();
    uint64_t utc_us, local_us;
    const bool locked = rtc.gps_locked() && rtc.get_utc_and_local_usec(utc_us, local_us);
    handle_system_time_at(sysid, compid, time_unix_usec, time_boot_ms, now_ms,
                          locked, locked ? int64_t(utc_us) - int64_t(local_us) : 0);
}

void AP_RTC_RemoteClocks::handle_system_time_at(uint8_t sysid, uint8_t compid, uint64_t time_unix_usec, uint32_t time_boot_ms,
                                                uint32_t now_ms, bool local_locked, int64_t local_offset_us)
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
    const int64_t ahead_us = int64_t(time_unix_usec) - (int64_t(now_ms) * 1000 + local_offset_us);
    if (ahead_us > int64_t(AP_RTC_REMOTE_CLOCK_TOLERANCE_MS) * 1000) {
        restart_acquisition(*s);
        return;
    }

    const int64_t sample_us = int64_t(time_unix_usec) - int64_t(time_boot_ms) * 1000;

    if (s->consistent == 0) {
        restart_fit(*s, sample_us, time_boot_ms);
        s->consistent = 1;
    } else {
        const int64_t residual_us = sample_us - offset_at(*s, time_boot_ms);
        if ((residual_us > OFFSET_CONSISTENT_US) || (residual_us < -OFFSET_CONSISTENT_US)) {
            // the remote clock stepped, so the offset must be acquired again
            restart_fit(*s, sample_us, time_boot_ms);
            s->consistent = 1;
            s->acquired = false;
        } else {
            // a sample no newer than the fit, arriving late over a slower link, adds nothing
            if (int32_t(time_boot_ms - s->fit_boot_ms) > 0) {
                add_to_fit(*s, sample_us, time_boot_ms);
                if (s->consistent < UINT8_MAX) {
                    s->consistent++;
                }
            }
        }
    }
    s->last_rx_ms = now_ms;

    try_acquire(*s, now_ms);
}

void AP_RTC_RemoteClocks::handle_gps_fix(uint8_t sysid, uint8_t compid, uint8_t fix_type, uint32_t now_ms)
{
    WITH_SEMAPHORE(sem);

    Source *s = find(sysid, compid);
    if (s == nullptr) {
        return;
    }

    // GPS_FIX_TYPE_3D_FIX or better; only acquisition depends on it
    s->gps_ok = (fix_type >= 3);
    s->last_gps_ms = now_ms;

    try_acquire(*s, now_ms);
}

bool AP_RTC_RemoteClocks::remote_to_local_ms(uint8_t sysid, uint8_t compid, uint32_t remote_boot_ms, uint32_t &local_ms)
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
    return remote_to_local_ms_at(sysid, compid, remote_boot_ms, uint32_t(local_us / 1000U),
                                 int64_t(utc_us) - int64_t(local_us), local_ms);
}

bool AP_RTC_RemoteClocks::remote_to_local_ms_at(uint8_t sysid, uint8_t compid, uint32_t remote_boot_ms,
                                                uint32_t now_ms, int64_t local_offset_us, uint32_t &local_ms)
{
    WITH_SEMAPHORE(sem);

    Source *s = find(sysid, compid);
    if (s == nullptr) {
        return false;
    }

    note_boot(*s, remote_boot_ms);

    // the drift is only extrapolated once it is fitted over a long enough span
    const bool drift_fitted = int32_t(s->fit_boot_ms - s->anchor_boot_ms) >= FIT_MIN_SPAN_MS;
    const uint32_t hold_ms = drift_fitted ? OFFSET_HOLD_MS : OFFSET_SHORT_HOLD_MS;
    if (!s->acquired || (now_ms - s->last_rx_ms) > hold_ms) {
        return false;
    }
    const int64_t offset_us = drift_fitted ? offset_at(*s, remote_boot_ms) : s->newest_offset_us;

    const int64_t local_us = int64_t(remote_boot_ms) * 1000 + offset_us - local_offset_us;
    if (local_us < 0) {
        return false;
    }

    // a message can't arrive before it was sent, so an offset putting it in the future is wrong
    if (local_us - int64_t(now_ms) * 1000 > int64_t(AP_RTC_REMOTE_CLOCK_TOLERANCE_MS) * 1000) {
        restart_acquisition(*s);
        return false;
    }

    local_ms = uint32_t(local_us / 1000);
    return true;
}

#endif  // AP_RTC_REMOTE_CLOCKS_ENABLED
