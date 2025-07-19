#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED

#include "AP_Networking.h"

class AP_Networking;

#ifndef AP_NETWORKING_MAX_ROUTES
#define AP_NETWORKING_MAX_ROUTES 4
#endif

class AP_Networking_Backend
{
public:
    friend class AP_Networking;

    AP_Networking_Backend(AP_Networking &_frontend) : frontend(_frontend) {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_Backend);

    virtual bool init() = 0;
    virtual void update() {};

    // add a new route for an interface
    // Returns true if the route is added or the route already exists
    bool add_route(uint8_t iface_idx, uint32_t dest_ip, uint8_t mask_len);

    // hook for custom routes
    virtual struct netif *routing_hook(uint32_t dest) { return nullptr; }

#if AP_NETWORKING_CAPTURE_ENABLED
    /*
      write a header for the start of a new capture packet
      a capture semaphore should be taken before the header is output
      and held for the whole packet
     */
    void capture_header(int fd, uint32_t len);
#endif

protected:
    AP_Networking &frontend;

    struct {
        uint32_t ip;
        uint32_t nm;
        uint32_t gw;
        uint32_t announce_ms;
        uint8_t macaddr[6];
        uint32_t last_change_ms;
    } activeSettings;

    struct {
        bool enabled;
        uint8_t iface_idx;
        uint32_t dest_ip;
        uint32_t netmask;
    } routes[AP_NETWORKING_MAX_ROUTES];
};

#endif // AP_NETWORKING_ENABLED
