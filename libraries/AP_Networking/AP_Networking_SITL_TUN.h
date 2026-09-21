#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SITL_TUN

#include "AP_Networking_Backend.h"

struct netif;
struct pbuf;

/*
  SITL networking backend that bridges the lwIP stack to a Linux TAP device.
  Lets developer tools on the host (browsers, curl, ...) reach lwIP-served
  sockets in the SITL process - typically the PPPGW web interface on a
  sitl_periph_PPP build, where this backend takes the place of the real
  Ethernet interface in PPP_ETHERNET_GATEWAY mode.

  The TAP device is expected to have been pre-created with
  `Tools/scripts/Networking/sitl_network.sh up` so it is owned by the current user;
  the SITL process itself does not need any special privileges.
 */
class AP_Networking_SITL_TUN : public AP_Networking_Backend
{
public:
    using AP_Networking_Backend::AP_Networking_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_SITL_TUN);

    bool init() override;
    void update() override;

private:
    void thread(void);
    static int8_t tap_netif_init(struct netif *netif);
    static int8_t low_level_output(struct netif *netif, struct pbuf *p);

    int tap_fd = -1;
    char ifname[16] {};
    uint8_t macaddr[6] {};
    struct netif *thisif = nullptr;
};

#endif // AP_NETWORKING_BACKEND_SITL_TUN
