#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB

#include "AP_Networking_Hub.h"
#include "AP_Networking.h"
#include <AP_HAL/AP_HAL.h>
#include <string.h>
#include <GCS_MAVLink/GCS.h>

#if AP_NETWORKING_BACKEND_HUB_PORT_COBS
#include "AP_Networking_Port_COBS.h"
#endif

extern const AP_HAL::HAL& hal;

AP_Networking_Hub::AP_Networking_Hub()
{
#if AP_NETWORKING_HUB_SWITCHING_ENABLED
    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        mac_table[i].port_idx = -1;
    }
#endif
}

int8_t AP_Networking_Hub::register_port(AP_Networking_HubPort *port)
{
    if (port == nullptr) {
        return -1;
    }
    WITH_SEMAPHORE(route_sem);
    if (num_ports >= ARRAY_SIZE(ports)) {
        return -1;
    }
    ports[num_ports] = port;
    return (int8_t)num_ports++;
}

void AP_Networking_Hub::unregister_port(AP_Networking_HubPort *port)
{
    if (port == nullptr) {
        return;
    }
    WITH_SEMAPHORE(route_sem);
    if (num_ports == 0) {
        return;
    }
    for (uint8_t i = 0; i < num_ports; i++) {
        if (ports[i] == port) {
#if AP_NETWORKING_HUB_SWITCHING_ENABLED
            fixup_mac_entries_for_removed_port(i);
#endif
            for (uint8_t j = i; j + 1 < num_ports; j++) {
                ports[j] = ports[j + 1];
            }
            ports[num_ports - 1] = nullptr;
            num_ports--;
            break;
        }
    }
}

void AP_Networking_Hub::route_frame(AP_Networking_HubPort *source, const uint8_t *frame, size_t len)
{
    WITH_SEMAPHORE(route_sem);
    if (frame == nullptr || len < 14 || len > MAX_ETH_FRAME) {
        frames_dropped++;
        return;
    }

    // Ethernet header: dest MAC (6) + src MAC (6) + ethertype (2)
    const uint8_t *dst_mac = frame;
    const uint8_t *src_mac = frame + 6;

#if AP_NETWORKING_HUB_SWITCHING_ENABLED
    // Find source port index
    int8_t src_port_idx = -1;
    for (uint8_t i = 0; i < num_ports; i++) {
        if (ports[i] == source) {
            src_port_idx = (int8_t)i;
            break;
        }
    }

    // Learn source MAC
    if (src_port_idx >= 0 && !is_broadcast_or_multicast(src_mac)) {
        learn_mac(src_mac, src_port_idx);
    }

    // Lookup destination
    int8_t dst_port_idx = -1;
    if (!is_broadcast_or_multicast(dst_mac)) {
        dst_port_idx = lookup_mac(dst_mac);
    }

    if (dst_port_idx >= 0 && dst_port_idx < (int8_t)num_ports) {
        // Unicast to known port
        AP_Networking_HubPort *port = ports[dst_port_idx];
        if (port != nullptr && port != source && port->can_receive()) {
            port->deliver_frame(frame, len);
        }
    } else {
        // Flood: broadcast, multicast, or unknown unicast
        for (uint8_t i = 0; i < num_ports; i++) {
            AP_Networking_HubPort *port = ports[i];
            if (port == nullptr || port == source) {
                continue;
            }
            if (!port->can_receive()) {
                continue;
            }
            port->deliver_frame(frame, len);
        }
    }
#else
    // Pure hub: flood to all ports except source
    (void)dst_mac;
    (void)src_mac;
    for (uint8_t i = 0; i < num_ports; i++) {
        AP_Networking_HubPort *port = ports[i];
        if (port == nullptr || port == source) {
            continue;
        }
        if (!port->can_receive()) {
            continue;
        }
        port->deliver_frame(frame, len);
    }
#endif // AP_NETWORKING_HUB_SWITCHING_ENABLED

    frames_routed++;
}

void AP_Networking_Hub::update()
{
    for (uint8_t i = 0; i < num_ports; i++) {
        if (ports[i] != nullptr) {
            ports[i]->update();
        }
    }
#if AP_NETWORKING_HUB_SWITCHING_ENABLED
    {
        WITH_SEMAPHORE(route_sem);
        age_mac_table();
    }
#endif
#if AP_NETWORKING_BACKEND_HUB_PORT_COBS
    // Periodically check for COBS ports that should be merged
    uint32_t now = AP_HAL::millis();
    if (now - last_gang_check_ms >= 1000) {
        last_gang_check_ms = now;
        check_cobs_ganging();
    }
#endif
}

uint8_t AP_Networking_Hub::get_num_ports_link_up() const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < num_ports; i++) {
        const AP_Networking_HubPort *port = ports[i];
        if (port != nullptr && port->is_link_up()) {
            count++;
        }
    }
    return count;
}

#if AP_NETWORKING_HUB_SWITCHING_ENABLED
void AP_Networking_Hub::learn_mac(const uint8_t *mac, int8_t port_idx)
{
    uint32_t now = AP_HAL::millis();
    int16_t oldest_idx = -1;
    uint32_t oldest_age = 0;
    int16_t empty_idx = -1;

    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        MACEntry &e = mac_table[i];
        if (e.port_idx < 0) {
            if (empty_idx < 0) {
                empty_idx = (int16_t)i;
            }
            continue;
        }
        if (memcmp(e.mac, mac, 6) == 0) {
            // Update existing entry
            e.port_idx = port_idx;
            e.last_seen_ms = now;
            return;
        }
        uint32_t age = now - e.last_seen_ms;
        if (age > oldest_age) {
            oldest_age = age;
            oldest_idx = (int16_t)i;
        }
    }

    // Insert new entry
    int16_t idx = (empty_idx >= 0) ? empty_idx : oldest_idx;
    if (idx >= 0) {
        MACEntry &e = mac_table[idx];
        memcpy(e.mac, mac, 6);
        e.port_idx = port_idx;
        e.last_seen_ms = now;
    }
}

int8_t AP_Networking_Hub::lookup_mac(const uint8_t *mac) const
{
    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        const MACEntry &e = mac_table[i];
        if (e.port_idx >= 0 && memcmp(e.mac, mac, 6) == 0) {
            return e.port_idx;
        }
    }
    return -1;
}

void AP_Networking_Hub::age_mac_table()
{
    uint32_t now = AP_HAL::millis();
    if ((now - last_age_ms) < 1000) {
        return;
    }
    last_age_ms = now;

    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        MACEntry &e = mac_table[i];
        if (e.port_idx >= 0 && (now - e.last_seen_ms) > AP_NETWORKING_HUB_MAC_AGE_MS) {
            e.port_idx = -1;
        }
    }
}

void AP_Networking_Hub::fixup_mac_entries_for_removed_port(uint8_t removed_idx)
{
    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        MACEntry &e = mac_table[i];
        if (e.port_idx < 0) {
            continue;
        }
        if ((uint8_t)e.port_idx == removed_idx) {
            e.port_idx = -1;
        } else if ((uint8_t)e.port_idx > removed_idx) {
            e.port_idx--;
        }
    }
}
#endif // AP_NETWORKING_HUB_SWITCHING_ENABLED

#if AP_NETWORKING_BACKEND_HUB_PORT_COBS

void AP_Networking_Hub::register_cobs_port(AP_Networking_Port_COBS *port)
{
    if (port == nullptr || num_cobs_ports >= MAX_COBS_PORTS) {
        return;
    }
    cobs_ports[num_cobs_ports++] = port;
}

void AP_Networking_Hub::unregister_cobs_port(AP_Networking_Port_COBS *port)
{
    if (port == nullptr) return;
    
    for (uint8_t i = 0; i < num_cobs_ports; i++) {
        if (cobs_ports[i] == port) {
            for (uint8_t j = i; j + 1 < num_cobs_ports; j++) {
                cobs_ports[j] = cobs_ports[j + 1];
            }
            cobs_ports[num_cobs_ports - 1] = nullptr;
            num_cobs_ports--;
            break;
        }
    }
}

void AP_Networking_Hub::request_cobs_split(AP_Networking_Port_COBS *port, uint8_t uart_idx,
                                            const uint8_t new_device_id[6])
{
    // Extract UART from existing port
    AP_HAL::UARTDriver *uart;
    uint32_t baud;
    if (!port->extract_uart(uart_idx, uart, baud)) {
        return;
    }
    
    // Get local device ID from AP_Networking
    uint8_t local_device_id[6];
    auto *net = AP_Networking::get_singleton();
    if (net == nullptr) {
        return;
    }
    net->get_macaddr(local_device_id);
    
    // Create new single-mode port
    auto *new_port = NEW_NOTHROW AP_Networking_Port_COBS(this, uart, baud, local_device_id);
    if (new_port == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: COBS split failed - alloc");
        return;
    }
    
    if (!new_port->init()) {
        delete new_port;
        return;
    }
    
    if (register_port(new_port) < 0) {
        delete new_port;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: COBS split failed - hub full");
        return;
    }
    
    register_cobs_port(new_port);
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS split - new device %02X:%02X:%02X:%02X:%02X:%02X",
                  new_device_id[0], new_device_id[1], new_device_id[2],
                  new_device_id[3], new_device_id[4], new_device_id[5]);
}

void AP_Networking_Hub::check_cobs_ganging()
{
    for (uint8_t i = 0; i < num_cobs_ports; i++) {
        auto *port_i = cobs_ports[i];
        if (port_i == nullptr || !port_i->has_remote_device_id()) {
            continue;
        }
        
        uint8_t id_i[6];
        port_i->get_remote_device_id(id_i);
        
        for (uint8_t j = i + 1; j < num_cobs_ports; j++) {
            auto *port_j = cobs_ports[j];
            if (port_j == nullptr || !port_j->has_remote_device_id()) {
                continue;
            }
            
            uint8_t id_j[6];
            port_j->get_remote_device_id(id_j);
            
            if (memcmp(id_i, id_j, 6) == 0) {
                // Same remote device - merge port j into port i
                while (port_j->get_num_uarts() > 0) {
                    AP_HAL::UARTDriver *uart;
                    uint32_t baud;
                    if (!port_j->extract_uart(0, uart, baud)) {
                        break;
                    }
                    if (!port_i->add_uart(uart, baud)) {
                        break;
                    }
                }
                
                // Remove port_j from hub and COBS tracking
                unregister_port(port_j);
                unregister_cobs_port(port_j);
                delete port_j;
                j--;  // Re-check this index
                
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: COBS ports merged (%u UARTs)", 
                              port_i->get_num_uarts());
            }
        }
    }
}

#endif // AP_NETWORKING_BACKEND_HUB_PORT_COBS

#endif // AP_NETWORKING_BACKEND_HUB


