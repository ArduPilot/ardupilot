#include "AP_Networking_Backend.h"

#if AP_NETWORKING_ENABLED

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

#endif // AP_NETWORKING_ENABLED
