
#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SLIP

#include "AP_Networking_SLIP.h"
#include <GCS_MAVLink/GCS.h>

#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <lwip/sio.h>
#include <lwip/tcpip.h>

#include <netif/slipif.h>

extern const AP_HAL::HAL& hal;
AP_HAL::UARTDriver *AP_Networking_SLIP::uart;

/*
  initialise SLIP network backend using LWIP
 */
bool AP_Networking_SLIP::init()
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_SLIP, 0);
    if (uart == nullptr) {
        return false;
    }
    uart->set_unbuffered_writes(true);
    
    slipif = new netif;
    if (slipif == nullptr) {
        return false;
    }

    // no dynamic IPs for SLIP
    activeSettings.ip = frontend.get_ip_param();
    activeSettings.gw = frontend.get_gateway_param();
    activeSettings.nm = frontend.get_netmask_param();
    activeSettings.last_change_ms = AP_HAL::millis();
    
    const ip4_addr_t ipaddr {htonl(activeSettings.ip)};
    const ip4_addr_t netmask {htonl(activeSettings.nm)};
    const ip4_addr_t gw {htonl(activeSettings.gw)};

    tcpip_init(NULL, NULL);
    hal.scheduler->delay(100);
    
    netif_add(slipif,
              &ipaddr, &netmask, &gw,
              &num_slip, slipif_init, ip_input);
    netif_set_default(slipif);
    slipif->mtu = 296;
    netif_set_link_up(slipif);
    netif_set_up(slipif);
    
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_SLIP::slip_loop, void),
                                 "slip",
                                 8192, AP_HAL::Scheduler::PRIORITY_UART, -1);
    
    return true;
}

/*
  main loop for SLIP
 */
void AP_Networking_SLIP::slip_loop(void)
{
    uart->begin(0);
    hal.scheduler->delay(100);
    while (true) {
        slipif_poll(slipif);
        hal.scheduler->delay_microseconds(100);
    }
}

void sio_send(uint8_t c, void *fd)
{
    auto *uart = (AP_HAL::UARTDriver *)fd;
    uart->write(c);
}

sio_fd_t sio_open(uint8_t dev)
{
    return AP_Networking_SLIP::uart;
}

u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
    auto *uart = (AP_HAL::UARTDriver *)fd;
    uart->begin(0);
    const auto ret = uart->read(data, len);
    if (ret <= 0) {
        return 0;
    }
    return ret;
}

#endif // AP_NETWORKING_BACKEND_SLIP
