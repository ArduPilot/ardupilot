#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_USB_ECM
#include "AP_Networking_Backend.h"

/*
 * Experimental USB CDC-ECM netif backend (Phase 2 spike).
 * Requires HAL_WITH_USB_CDC_ECM and usbcfg_cdc_ecm frame I/O.
 */
class AP_Networking_USB_ECM : public AP_Networking_Backend
{
public:
    using AP_Networking_Backend::AP_Networking_Backend;

    CLASS_NO_COPY(AP_Networking_USB_ECM);

    bool init() override;
    void update() override;

private:
    void thread(void);
    static int8_t ethernetif_init(struct netif *netif);
    static int8_t low_level_output(struct netif *netif, struct pbuf *p);

    struct netif *thisif;
    bool link_was_up;
};

#endif // AP_NETWORKING_BACKEND_USB_ECM
