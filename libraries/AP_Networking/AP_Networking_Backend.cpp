#include "AP_Networking_Backend.h"

#if AP_NETWORKING_ENABLED

#if AP_NETWORKING_CAPTURE_ENABLED
#include <AP_RTC/AP_RTC.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <lwip/tcpip.h>
#endif

// add a new route for an interface
// Returns true if the route is added or the route already exists
bool AP_Networking_Backend::add_route(uint8_t iface_idx, uint32_t dest_ip, uint8_t mask_len)
{
    const uint32_t netmask = frontend.convert_netmask_bitcount_to_ip(mask_len);
    for (uint8_t i=0; i<AP_NETWORKING_MAX_ROUTES; i++) {
        auto &r = routes[i];
        if (r.enabled && r.iface_idx == iface_idx && r.dest_ip == dest_ip && r.netmask == netmask) {
            // already have it
            return true;
        }
        if (!r.enabled) {
            r.iface_idx = iface_idx;
            r.dest_ip = dest_ip;
            r.netmask = netmask;
            r.enabled = true;
            return true;
        }
    }
    return false;
}

#if AP_NETWORKING_CAPTURE_ENABLED

/*
  write a header for the start of a new capture packet
  capture.sem should be taken before the header is output
  and held for the whole packet
  note that len should include the direction byte for PPP data

  caller must have checked that capture.fd != -1
*/
void AP_Networking_Backend::capture_header(int fd, uint32_t len)
{
    uint64_t utc_usec = 0;
#if AP_RTC_ENABLED
    AP::rtc().get_utc_usec(utc_usec);
#endif
    if (utc_usec == 0) {
        utc_usec = AP_HAL::micros64();
    }
    const struct pcaprec_hdr {
        uint32_t ts_sec;
        uint32_t ts_usec;
        uint32_t incl_len;
        uint32_t orig_len;
    } rec {
        .ts_sec = uint32_t(utc_usec / 1000000ULL),
        .ts_usec = uint32_t(utc_usec % 1000000ULL),
        .incl_len = len,
        .orig_len = len
    };

    AP::FS().write(fd, (const void *)&rec, sizeof(rec));
}
#endif // AP_NETWORKING_CAPTURE_ENABLED

#endif // AP_NETWORKING_ENABLED
