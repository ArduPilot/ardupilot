#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SWITCH

#include "AP_Networking_Switch.h"
#include "AP_Networking.h"
#include <AP_HAL/AP_HAL.h>
#include <string.h>
#include <GCS_MAVLink/GCS.h>

#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS
#include "AP_Networking_SwitchPort_COBS.h"
#endif

// Debug output - GCS_SEND_TEXT works for both main builds and periph (routes via CAN)
#if defined(HAL_BUILD_AP_PERIPH)
extern bool periph_debug_switch_pkt_enabled();
#define SWITCH_DEBUG(fmt, args...) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SW: " fmt, ##args)
#define SWITCH_PKT_DEBUG(fmt, args...) do { if (periph_debug_switch_pkt_enabled()) GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "SW: " fmt, ##args); } while(0)
#else
#define SWITCH_DEBUG(fmt, args...) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SW: " fmt, ##args)
#define SWITCH_PKT_DEBUG(fmt, args...) do { if (AP::network().option_is_set(AP_Networking::OPTION::DEBUG_SWITCH_PKT)) GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "SW: " fmt, ##args); } while(0)
#endif

extern const AP_HAL::HAL& hal;

AP_Networking_Switch::AP_Networking_Switch()
{
    // mac_table is allocated lazily in register_port() when more than 2 ports
}

AP_Networking_Switch::~AP_Networking_Switch()
{
#if AP_NETWORKING_SWITCH_SWITCHING_ENABLED
    delete[] mac_table;
#endif
}

int8_t AP_Networking_Switch::register_port(AP_Networking_SwitchPort *port)
{
    if (port == nullptr) {
        return -1;
    }
    WITH_SEMAPHORE(route_sem);
    if (num_ports >= ARRAY_SIZE(ports)) {
        return -1;
    }
#if AP_NETWORKING_SWITCH_SWITCHING_ENABLED
    // Allocate MAC table when transitioning from 2 to 3 ports
    // With 2 ports, we use bridge mode (no MAC learning needed)
    // If allocation fails, we fall back to flooding (hub mode)
    if (num_ports == 2 && mac_table == nullptr) {
        mac_table = NEW_NOTHROW MACEntry[AP_NETWORKING_SWITCH_MAC_TABLE_SIZE];
        if (mac_table != nullptr) {
            for (uint16_t i = 0; i < AP_NETWORKING_SWITCH_MAC_TABLE_SIZE; i++) {
                mac_table[i].port_idx = -1;
            }
        }
    }
#endif
    ports[num_ports] = port;
    return (int8_t)num_ports++;
}

void AP_Networking_Switch::unregister_port(AP_Networking_SwitchPort *port)
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
#if AP_NETWORKING_SWITCH_SWITCHING_ENABLED
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

void AP_Networking_Switch::route_frame(AP_Networking_SwitchPort *source, const uint8_t *frame, size_t len)
{
    WITH_SEMAPHORE(route_sem);
    if (frame == nullptr || len < 14 || len > MAX_ETH_FRAME) {
        frames_dropped++;
        return;
    }

    // Fast path: 2-port bridge mode
    // When exactly 2 ports, forward to the other port without MAC learning/lookup
    if (num_ports == 2) {
        AP_Networking_SwitchPort *other = (ports[0] == source) ? ports[1] : ports[0];
        if (other != nullptr && other->can_receive()) {
            other->deliver_frame(frame, len);
        }
        frames_routed++;
        return;
    }

    // Ethernet header: dest MAC (6) + src MAC (6) + ethertype (2)
    const uint8_t *dst_mac = frame;
    const uint8_t *src_mac = frame + 6;
    const uint16_t ethertype = (frame[12] << 8) | frame[13];

    // Find source port index (needed for both switching and debug)
    int8_t src_port_idx = -1;
    for (uint8_t i = 0; i < num_ports; i++) {
        if (ports[i] == source) {
            src_port_idx = (int8_t)i;
            break;
        }
    }

#if AP_NETWORKING_SWITCH_SWITCHING_ENABLED
    // Learn source MAC (only when MAC table is allocated)
    if (src_port_idx >= 0 && !is_broadcast_or_multicast(src_mac)) {
        learn_mac(src_mac, src_port_idx);
    }

    // Lookup destination
    int8_t dst_port_idx = -1;
    if (!is_broadcast_or_multicast(dst_mac)) {
        dst_port_idx = lookup_mac(dst_mac);
    }

    // Debug: sniff ARP packets
    // Ethertype 0x0806 = ARP
    // ARP header starts at offset 14, operation at offset 20-21
    if (ethertype == 0x0806 && len >= 42) {  // 14 eth + 28 ARP minimum
        const uint16_t arp_oper = (frame[20] << 8) | frame[21];  // 1=request, 2=reply
        const uint8_t *sender_ip = &frame[28];  // Sender protocol address
        const uint8_t *target_ip = &frame[38];  // Target protocol address
        
        const char *src_name = (src_port_idx >= 0 && ports[src_port_idx]) ? ports[src_port_idx]->get_name() : "?";
        const char *dst_name;
        const char *dst_action;
        
        if (dst_port_idx >= 0 && dst_port_idx < (int8_t)num_ports && ports[dst_port_idx]) {
            dst_name = ports[dst_port_idx]->get_name();
            if (ports[dst_port_idx]->can_receive()) {
                dst_action = "";
            } else {
                dst_action = " DROPPED:can_rx=0";
            }
        } else {
            dst_name = "FLOOD";
            dst_action = "";
        }
        
        const char *arp_type = (arp_oper == 1) ? "REQ" : ((arp_oper == 2) ? "REP" : "???");
        SWITCH_PKT_DEBUG("ARP %s %u.%u.%u.%u->%u.%u.%u.%u: %s(%d)->%s(%d)%s",
                  arp_type,
                  sender_ip[0], sender_ip[1], sender_ip[2], sender_ip[3],
                  target_ip[0], target_ip[1], target_ip[2], target_ip[3],
                  src_name, src_port_idx,
                  dst_name, dst_port_idx,
                  dst_action);
    }

    // Debug: sniff ICMP packets
    // Ethertype 0x0800 = IPv4, IP protocol 1 = ICMP
    // IPv4 header starts at offset 14, protocol field at offset 14+9=23
    if (ethertype == 0x0800 && len >= 34) {  // 14 eth + 20 IP minimum
        const uint8_t ip_proto = frame[23];
        if (ip_proto == 1) {  // ICMP
            // IP addresses: src at 14+12=26, dst at 14+16=30
            // ICMP type at 14+20=34 (assuming no IP options)
            const uint8_t ip_hdr_len = (frame[14] & 0x0F) * 4;
            const uint8_t *ip_src = &frame[26];
            const uint8_t *ip_dst = &frame[30];
            const uint8_t icmp_type = (len > size_t(14 + ip_hdr_len)) ? frame[14 + ip_hdr_len] : 0;
            const uint8_t icmp_code = (len > size_t(14 + ip_hdr_len + 1)) ? frame[14 + ip_hdr_len + 1] : 0;
            
            const char *src_name = (src_port_idx >= 0 && ports[src_port_idx]) ? ports[src_port_idx]->get_name() : "?";
            const char *dst_name;
            const char *dst_action;
            bool delivered = false;
            
            if (dst_port_idx >= 0 && dst_port_idx < (int8_t)num_ports && ports[dst_port_idx]) {
                dst_name = ports[dst_port_idx]->get_name();
                if (ports[dst_port_idx]->can_receive()) {
                    dst_action = "";
                    delivered = true;
                } else {
                    dst_action = " DROPPED:can_rx=0";
                }
            } else {
                dst_name = "FLOOD";
                dst_action = "";
                delivered = true;
            }
            
            SWITCH_PKT_DEBUG("ICMP %u.%u.%u.%u->%u.%u.%u.%u type=%u code=%u: %s(%d)->%s(%d)%s",
                      ip_src[0], ip_src[1], ip_src[2], ip_src[3],
                      ip_dst[0], ip_dst[1], ip_dst[2], ip_dst[3],
                      icmp_type, icmp_code,
                      src_name, src_port_idx,
                      dst_name, dst_port_idx,
                      dst_action);
            (void)delivered;
        }
    }

    if (dst_port_idx >= 0 && dst_port_idx < (int8_t)num_ports) {
        // Unicast to known port
        AP_Networking_SwitchPort *port = ports[dst_port_idx];
        if (port != nullptr && port != source && port->can_receive()) {
            port->deliver_frame(frame, len);
        }
    } else {
        // Flood: broadcast, multicast, or unknown unicast
        for (uint8_t i = 0; i < num_ports; i++) {
            AP_Networking_SwitchPort *port = ports[i];
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
    (void)src_port_idx;
    (void)ethertype;
    for (uint8_t i = 0; i < num_ports; i++) {
        AP_Networking_SwitchPort *port = ports[i];
        if (port == nullptr || port == source) {
            continue;
        }
        if (!port->can_receive()) {
            continue;
        }
        port->deliver_frame(frame, len);
    }
#endif // AP_NETWORKING_SWITCH_SWITCHING_ENABLED

    frames_routed++;
}

void AP_Networking_Switch::update()
{
    // Take snapshot of ports under lock to avoid race with register_port
    AP_Networking_SwitchPort *ports_snapshot[AP_NETWORKING_SWITCHPORT_MAX_INSTANCES];
    uint8_t num_ports_snapshot;
    {
        WITH_SEMAPHORE(route_sem);
        num_ports_snapshot = num_ports;
        memcpy(ports_snapshot, ports, sizeof(ports_snapshot));
#if AP_NETWORKING_SWITCH_SWITCHING_ENABLED
        age_mac_table();
#endif
    }
    
    // Update ports outside the lock to avoid holding it during potentially slow operations
    for (uint8_t i = 0; i < num_ports_snapshot; i++) {
        if (ports_snapshot[i] != nullptr) {
            ports_snapshot[i]->update();
        }
    }

#if AP_NETWORKING_SWITCH_SWITCHING_ENABLED
    // Debug: print MAC table every 10 seconds
    static uint32_t last_mac_table_print_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_mac_table_print_ms >= 10000) {
        last_mac_table_print_ms = now_ms;
        
        SWITCH_DEBUG("Switch (%u ports, %s)", num_ports_snapshot, 
                     mac_table != nullptr ? "switching" : "bridge");
        for (uint8_t i = 0; i < num_ports_snapshot; i++) {
            if (ports_snapshot[i] != nullptr) {
                SWITCH_DEBUG("Port %u: %s link=%u can_rx=%u",
                          i, ports_snapshot[i]->get_name(),
                          (unsigned)ports_snapshot[i]->is_link_up(),
                          (unsigned)ports_snapshot[i]->can_receive());
            }
        }
        
        if (mac_table != nullptr) {
            uint8_t valid_entries = 0;
            for (uint16_t i = 0; i < AP_NETWORKING_SWITCH_MAC_TABLE_SIZE; i++) {
                const MACEntry &e = mac_table[i];
                if (e.port_idx >= 0) {
                    uint32_t age_ms = now_ms - e.last_seen_ms;
                    const char *port_name = (e.port_idx < num_ports_snapshot && ports_snapshot[e.port_idx] != nullptr) 
                                            ? ports_snapshot[e.port_idx]->get_name() : "???";
                    SWITCH_DEBUG("MAC %02X:%02X:%02X:%02X:%02X:%02X -> port %d (%s) age=%lums",
                              e.mac[0], e.mac[1], e.mac[2], e.mac[3], e.mac[4], e.mac[5],
                              (int)e.port_idx, port_name, (unsigned long)age_ms);
                    valid_entries++;
                }
            }
            if (valid_entries == 0) {
                SWITCH_DEBUG("(no MAC entries)");
            }
        }
    }
#endif
}

uint8_t AP_Networking_Switch::get_num_ports_link_up() const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < num_ports; i++) {
        const AP_Networking_SwitchPort *port = ports[i];
        if (port != nullptr && port->is_link_up()) {
            count++;
        }
    }
    return count;
}

uint8_t AP_Networking_Switch::get_num_external_ports_link_up() const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < num_ports; i++) {
        const AP_Networking_SwitchPort *port = ports[i];
        if (port != nullptr && port->is_link_up()) {
            // Exclude lwIP (local stack) - only count external bridged ports
            if (strcmp(port->get_name(), "lwIP") != 0) {
                count++;
            }
        }
    }
    return count;
}

bool AP_Networking_Switch::any_port_can_receive(AP_Networking_SwitchPort *exclude_port) const
{
    for (uint8_t i = 0; i < num_ports; i++) {
        const AP_Networking_SwitchPort *port = ports[i];
        if (port != nullptr && port != exclude_port && port->can_receive()) {
            return true;
        }
    }
    return false;
}

#if AP_NETWORKING_SWITCH_SWITCHING_ENABLED
void AP_Networking_Switch::learn_mac(const uint8_t *mac, int8_t port_idx)
{
    if (mac_table == nullptr) {
        return;  // No MAC table (bridge mode)
    }
    
    const uint32_t now_ms = AP_HAL::millis();
    int16_t oldest_idx = -1;
    uint32_t oldest_age_ms = 0;
    int16_t empty_idx = -1;

    for (uint16_t i = 0; i < AP_NETWORKING_SWITCH_MAC_TABLE_SIZE; i++) {
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
            e.last_seen_ms = now_ms;
            return;
        }
        uint32_t age_ms = now_ms - e.last_seen_ms;
        if (age_ms > oldest_age_ms) {
            oldest_age_ms = age_ms;
            oldest_idx = (int16_t)i;
        }
    }

    // Insert new entry
    int16_t idx = (empty_idx >= 0) ? empty_idx : oldest_idx;
    if (idx >= 0) {
        MACEntry &e = mac_table[idx];
        memcpy(e.mac, mac, 6);
        e.port_idx = port_idx;
        e.last_seen_ms = now_ms;
    }
}

int8_t AP_Networking_Switch::lookup_mac(const uint8_t *mac) const
{
    if (mac_table == nullptr) {
        return -1;  // No MAC table (bridge mode)
    }
    for (uint16_t i = 0; i < AP_NETWORKING_SWITCH_MAC_TABLE_SIZE; i++) {
        const MACEntry &e = mac_table[i];
        if (e.port_idx >= 0 && memcmp(e.mac, mac, 6) == 0) {
            return e.port_idx;
        }
    }
    return -1;
}

void AP_Networking_Switch::age_mac_table()
{
    if (mac_table == nullptr) {
        return;  // No MAC table (bridge mode)
    }
    const uint32_t now_ms = AP_HAL::millis();
    if ((now_ms - last_age_ms) < 1000) {
        return;
    }
    last_age_ms = now_ms;

    for (uint16_t i = 0; i < AP_NETWORKING_SWITCH_MAC_TABLE_SIZE; i++) {
        MACEntry &e = mac_table[i];
        if (e.port_idx >= 0 && (now_ms - e.last_seen_ms) > AP_NETWORKING_SWITCH_MAC_AGE_MS) {
            e.port_idx = -1;
        }
    }
}

void AP_Networking_Switch::fixup_mac_entries_for_removed_port(uint8_t removed_idx)
{
    if (mac_table == nullptr) {
        return;  // No MAC table (bridge mode)
    }
    for (uint16_t i = 0; i < AP_NETWORKING_SWITCH_MAC_TABLE_SIZE; i++) {
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
#endif // AP_NETWORKING_SWITCH_SWITCHING_ENABLED

#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS

bool AP_Networking_Switch::init_cobs()
{
    if (cobs_thread_started) {
        return true;
    }
    
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_Networking_Switch::cobs_thread_run, void),
            "cobs",
            3072, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: COBS thread failed");
        return false;
    }
    
    cobs_thread_started = true;
    return true;
}

void AP_Networking_Switch::cobs_thread_run()
{
    // Wait for system to be ready
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }
    
    while (true) {
        // Snapshot bonds to process (avoids holding lock during thread_update)
        AP_Networking_SwitchPort_COBS *bonds_to_process[AP_NETWORKING_BACKEND_SWITCHPORT_COBS_BOND_INSTANCE_MAX];
        uint8_t num_to_process;
        
        {
            WITH_SEMAPHORE(cobs_sem);
            num_to_process = num_cobs_bonds;
            memcpy(bonds_to_process, cobs_bonds, sizeof(bonds_to_process));
        }
        
        // Process all COBS bonds outside the lock
        for (uint8_t i = 0; i < num_to_process; i++) {
            if (bonds_to_process[i] != nullptr) {
                bonds_to_process[i]->thread_update();
            }
        }
        
        // Small delay - 100us polling rate
        hal.scheduler->delay_microseconds(100);
    }
}

void AP_Networking_Switch::register_cobs_bond(AP_Networking_SwitchPort_COBS *bond)
{
    if (bond == nullptr) {
        return;
    }
    
    WITH_SEMAPHORE(cobs_sem);
    
    if (num_cobs_bonds >= AP_NETWORKING_BACKEND_SWITCHPORT_COBS_BOND_INSTANCE_MAX) {
        return;
    }
    cobs_bonds[num_cobs_bonds++] = bond;
}

#endif // AP_NETWORKING_BACKEND_SWITCHPORT_COBS

#endif // AP_NETWORKING_BACKEND_SWITCH


