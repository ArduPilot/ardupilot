#pragma once

#include "AP_RTC_config.h"

#if AP_RTC_REMOTE_CLOCKS_ENABLED

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <stdint.h>

/*
  clock offsets of other MAVLink components, learned from their
  SYSTEM_TIME messages, used to convert the boot-time timestamps they
  send into local boot time.  Both clocks follow GPS time, so the
  conversion includes the transport delay.

  Only registered components are tracked, so a large swarm cannot push
  a component of interest out of the table.

  An offset is acquired from consistent SYSTEM_TIME samples while the
  component reports a 3D GPS fix.  The newest is then held until the
  component reboots, or has gone two minutes without sending SYSTEM_TIME
  or without reporting a 3D fix.  SYSTEM_TIME can't show whether the
  component's clock is still slewing to GPS time, so its error is then
  included in the conversion.
 */
class AP_RTC_RemoteClocks {
public:

    // start tracking a component's clock.  Registrations are counted, so
    // each must be matched by an unregister_source().  Returns false if
    // the table is full
    bool register_source(uint32_t sysid, uint8_t compid);
    void unregister_source(uint32_t sysid, uint8_t compid);

    // feed received SYSTEM_TIME and GPS_RAW_INT messages
    void handle_system_time(uint32_t sysid, uint8_t compid, uint64_t time_unix_usec, uint32_t time_boot_ms, uint32_t now_ms);
    void handle_gps_fix(uint32_t sysid, uint8_t compid, uint8_t fix_type, uint32_t now_ms);

    // convert a component's boot time in milliseconds to local boot time.
    // False unless this clock follows GPS time and the component's clock
    // offset is acquired and held.  The boot time is also checked for a reboot
    bool remote_to_local_ms(uint32_t sysid, uint8_t compid, uint32_t remote_boot_ms, uint32_t &local_ms);

    // as above, given the local time, whether this clock follows GPS time and
    // the local UTC at now_ms; used by the unit tests
    void handle_system_time_at(uint32_t sysid, uint8_t compid, uint64_t time_unix_usec, uint32_t time_boot_ms,
                               uint32_t now_ms, bool local_locked, uint64_t local_utc_us);

    // as above, given the local boot time in microseconds and the local UTC at that
    // time; used by the unit tests
    bool remote_to_local_ms_at(uint32_t sysid, uint8_t compid, uint32_t remote_boot_ms,
                               uint64_t local_us, uint64_t local_utc_us, uint32_t &local_ms);

private:

    struct Source {
        uint32_t sysid;
        uint8_t compid;
        uint8_t refs;               // registrations, zero for a free entry
        uint8_t consistent;         // consecutive consistent offset samples, saturating
        bool acquired;              // offset usable, until a reboot or the hold expires
        bool gps_ok;                // latest GPS fix report was 3D or better
        bool have_boot;             // latest_boot_ms is valid
        uint32_t latest_boot_ms;    // latest remote boot time seen
        uint32_t last_rx_ms;        // local time of the newest offset sample
        uint32_t last_gps_ms;       // local time of the last report of a 3D fix

        int64_t offset_us;          // remote UTC minus remote boot time, from the newest sample
        uint32_t offset_boot_ms;    // remote boot time of the newest sample
    };
    Source sources[AP_RTC_REMOTE_CLOCKS_MAX] {};

    HAL_Semaphore sem;

    Source *find(uint32_t sysid, uint8_t compid);
    void reset(Source &s);
    void note_boot(Source &s, uint32_t boot_ms);
    void set_offset(Source &s, int64_t sample_us, uint32_t boot_ms, uint32_t now_ms);
    void try_acquire(Source &s, uint32_t now_ms);
    void restart_acquisition(Source &s);
};

#endif  // AP_RTC_REMOTE_CLOCKS_ENABLED
