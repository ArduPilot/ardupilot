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
 */
class AP_RTC_RemoteClocks {
public:

    // start tracking a component's clock.  Registrations are counted, so
    // each must be matched by an unregister_source().  Returns false if
    // the table is full
    bool register_source(uint8_t sysid, uint8_t compid);
    void unregister_source(uint8_t sysid, uint8_t compid);

    // feed received SYSTEM_TIME and GPS_RAW_INT messages
    void handle_system_time(uint8_t sysid, uint8_t compid, uint64_t time_unix_usec, uint32_t time_boot_ms, uint32_t now_ms);
    void handle_gps_fix(uint8_t sysid, uint8_t compid, uint8_t fix_type, uint32_t now_ms);

    // convert a component's boot time in milliseconds to local boot time.
    // False unless this clock follows GPS time, the component's clock
    // offset is known and fresh, and it recently reported a 3D GPS fix
    bool remote_to_local_ms(uint8_t sysid, uint8_t compid, uint32_t remote_boot_ms, uint32_t &local_ms) const;

    // as above, given the local time and the local UTC minus boot time; used by the unit tests
    bool remote_to_local_ms_at(uint8_t sysid, uint8_t compid, uint32_t remote_boot_ms,
                               uint32_t now_ms, int64_t local_offset_us, uint32_t &local_ms) const;

private:

    struct Source {
        uint8_t sysid;
        uint8_t compid;
        uint8_t refs;               // registrations, zero for a free entry
        uint8_t consistent;         // consecutive consistent offset samples, saturating
        bool gps_ok;
        int64_t offset_us;          // remote UTC minus remote boot time
        uint32_t last_rx_ms;        // local time of the last offset sample
        uint32_t last_gps_ms;       // local time of the last GPS fix report
    };
    Source sources[AP_RTC_REMOTE_CLOCKS_MAX] {};

    mutable HAL_Semaphore sem;

    Source *find(uint8_t sysid, uint8_t compid);
    const Source *find(uint8_t sysid, uint8_t compid) const;
};

#endif  // AP_RTC_REMOTE_CLOCKS_ENABLED
