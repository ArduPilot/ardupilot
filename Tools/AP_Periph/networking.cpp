/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Periph.h"

#if AP_PERIPH_NETWORKING_ENABLED

#if AP_NETWORKING_BACKEND_CHIBIOS
#include <AP_Networking/AP_Networking_SwitchPort_Ethernet_ChibiOS.h>
#include <AP_Networking/AP_Networking_SwitchPort_lwIP.h>
#include <AP_Networking/AP_Networking_SwitchPort_COBS.h>
#include <AP_Networking/AP_Networking_Switch.h>
#endif

const AP_Param::GroupInfo Networking_Periph::var_info[] {
    // @Group:
    // @Path: ../../libraries/AP_Networking/AP_Networking.cpp
    AP_SUBGROUPINFO(networking_lib, "", 1, Networking_Periph, AP_Networking),

    /*
      the NET_Pn_ parameters need to be here as otherwise we
      are too deep in the parameter tree
     */

#if AP_NETWORKING_NUM_PORTS > 0
    // @Group: P1_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[0], "P1_", 2, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 1
    // @Group: P2_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[1], "P2_", 3, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 2
    // @Group: P3_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[2], "P3_", 4, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 3
    // @Group: P4_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[3], "P4_", 5, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 4
    // @Group: P5_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[4], "P5_", 6, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 5
    // @Group: P6_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[5], "P6_", 7, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 6
    // @Group: P7_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[6], "P7_", 8, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 7
    // @Group: P8_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[7], "P8_", 9, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 8
    // @Group: P9_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[8], "P9_", 10, Networking_Periph, AP_Networking::Port),
#endif



#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    // @Group: PASS1_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[0], "PASS1_", 11, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 1
    // @Group: PASS2_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[1], "PASS2_", 12, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 2
    // @Group: PASS3_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[2], "PASS3_", 13, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 3
    // @Group: PASS4_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[3], "PASS4_", 14, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 4
    // @Group: PASS5_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[4], "PASS5_", 15, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 5
    // @Group: PASS6_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[5], "PASS6_", 16, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 6
    // @Group: PASS7_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[6], "PASS7_", 17, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 7
    // @Group: PASS8_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[7], "PASS8_", 18, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 8
    // @Group: PASS9_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[8], "PASS9_", 19, Networking_Periph, Passthru),
#endif

#if AP_NETWORKING_BACKEND_PPP
    // @Param: PPP_PORT
    // @DisplayName: PPP serial port
    // @Description: PPP serial port
    // @Range: -1 10
    AP_GROUPINFO("PPP_PORT", 20, Networking_Periph, ppp_port, AP_PERIPH_NET_PPP_PORT_DEFAULT),

    // @Param: PPP_BAUD
    // @DisplayName: PPP serial baudrate
    // @Description: PPP serial baudrate
    // @CopyFieldsFrom: SERIAL1_BAUD
    AP_GROUPINFO("PPP_BAUD", 21, Networking_Periph, ppp_baud, AP_PERIPH_NET_PPP_BAUD_DEFAULT),
#endif

    AP_GROUPEND
};


void Networking_Periph::init(void)
{
#if AP_NETWORKING_BACKEND_PPP
    if (ppp_port >= 0) {
        AP::serialmanager().set_protocol_and_baud(ppp_port, AP_SerialManager::SerialProtocol_PPP, ppp_baud.get());
    }
#endif

    networking_lib.init();

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    for (auto &p : passthru) {
        p.init();
    }
#endif
}

void Networking_Periph::update(void)
{
    networking_lib.update();

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    for (auto &p : passthru) {
        p.update();
    }
#endif

#if HAL_RAM_RESERVE_START >= 256
    if (!got_addresses && networking_lib.get_ip_active() != 0) {
        got_addresses = true;
        auto *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
        if (comms->magic != APP_BOOTLOADER_COMMS_MAGIC) {
            memset(comms, 0, sizeof(*comms));
        }
        comms->magic = APP_BOOTLOADER_COMMS_MAGIC;
        comms->ip = networking_lib.get_ip_active();
        comms->netmask = networking_lib.get_netmask_active();
        comms->gateway = networking_lib.get_gateway_active();
    }
#endif // HAL_RAM_RESERVE_START

#if AP_NETWORKING_BACKEND_SWITCH
    // Periodic stats over CAN (10s period)
    static uint32_t last_stats_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_stats_ms >= 10000U && periph.debug_option_is_set(AP_Periph_FW::DebugOptions::NETWORK_STATS)) {
        last_stats_ms = now_ms;
        auto *hub = networking_lib.get_hub();
        if (hub != nullptr) {
            can_printf("NET: SW routed=%lu dropped=%lu",
                       (unsigned long)hub->get_frames_routed(),
                       (unsigned long)hub->get_frames_dropped());
        }
#if AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET
        auto *eth = networking_lib.get_port_eth();
        if (eth != nullptr) {
            can_printf("NET: ETH rx=%lu tx=%lu rxerr=%lu txerr=%lu link=%u",
                       (unsigned long)eth->get_rx_count(),
                       (unsigned long)eth->get_tx_count(),
                       (unsigned long)eth->get_rx_errors(),
                       (unsigned long)eth->get_tx_errors(),
                       eth->is_link_up() ? 1U : 0U);
        }
#endif // AP_NETWORKING_BACKEND_SWITCHPORT_ETHERNET

#if AP_NETWORKING_BACKEND_SWITCHPORT_LWIP
        auto *lwip = networking_lib.get_port_lwip();
        if (lwip != nullptr) {
            can_printf("NET: LWIP rx=%lu tx=%lu rxerr=%lu txerr=%lu",
                       (unsigned long)lwip->get_rx_count(),
                       (unsigned long)lwip->get_tx_count(),
                       (unsigned long)lwip->get_rx_errors(),
                       (unsigned long)lwip->get_tx_errors());
        }
#endif // AP_NETWORKING_BACKEND_SWITCHPORT_LWIP

#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS
        // COBS bonds
        const uint8_t n_bonds = networking_lib.get_num_cobs_bonds();
        for (uint8_t i = 0; i < n_bonds; i++) {
            auto *g = networking_lib.get_cobs_bond(i);
            if (g == nullptr) {
                continue;
            }
            can_printf("NET: COBS_G%u(%u) up=%u rx=%lu tx=%lu ka=%lu/%lu",
                       (unsigned)g->get_bond_id(),
                       (unsigned)g->get_num_members(),
                       (unsigned)(g->is_link_up() ? 1 : 0),
                       (unsigned long)g->get_rx_count(),
                       (unsigned long)g->get_tx_count(),
                       (unsigned long)g->get_keepalive_rx(),
                       (unsigned long)g->get_keepalive_tx());
        }
#endif // AP_NETWORKING_BACKEND_SWITCHPORT_COBS
    }
#endif // AP_NETWORKING_BACKEND_SWITCH
}

// Check if hub packet debug (ARP/ICMP) is enabled via DEBUG parameter bit 4
bool periph_debug_switch_pkt_enabled()
{
    return (periph.g.debug & (1U << 4)) != 0;
}

#endif  // AP_PERIPH_NETWORKING_ENABLED

