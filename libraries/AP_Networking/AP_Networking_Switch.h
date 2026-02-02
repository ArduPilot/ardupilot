#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_SWITCH

#include <stdint.h>
#include <stddef.h>
#include <AP_HAL/Semaphores.h>

class AP_Networking_SwitchPort;
class AP_Networking_COBS_Link;
class AP_Networking_SwitchPort_COBS;

/*
  Abstract hub port interface
*/
class AP_Networking_SwitchPort
{
public:
    virtual ~AP_Networking_SwitchPort() {}

    // Called by hub to deliver a frame to this port
    virtual void deliver_frame(const uint8_t *frame, size_t len) = 0;

    // Called by hub to check if port is ready to receive
    virtual bool can_receive() const
    {
        return true;
    }

    // Link state reporting for switch aggregation
    // Ports should override to reflect their own link availability
    virtual bool is_link_up() const
    {
        return false;
    }

    // Optional update hook, called by hub->update()
    virtual void update() {}

    // Port identifier (for debugging)
    virtual const char *get_name() const = 0;
};

/*
  Layer-2 switch/hub that routes frames between registered ports.
  When switching is enabled, learns source MACs and forwards unicast
  to the learned port. Broadcasts/multicasts and unknown unicasts flood.
  
  Lock ordering (must acquire in this order to avoid deadlock):
    route_sem -> cobs_sem -> tx_sem
  
  Key constraints:
  - route_frame() takes route_sem, then may call port methods
  - Port methods that call route_frame() must not hold tx_sem
*/
class AP_Networking_Switch
{
public:
    AP_Networking_Switch();
    ~AP_Networking_Switch();

    // Register a port with the hub
    // Returns: port ID (0-based index) or -1 on failure
    int8_t register_port(AP_Networking_SwitchPort *port);

    // Unregister a port
    void unregister_port(AP_Networking_SwitchPort *port);

    // Route a frame from source port to destination port(s)
    void route_frame(AP_Networking_SwitchPort *source, const uint8_t *frame, size_t len);

    // Update all ports and age MAC table
    void update();

    // Statistics
    uint32_t get_frames_routed() const
    {
        return frames_routed;
    }
    uint32_t get_frames_dropped() const
    {
        return frames_dropped;
    }

    // Number of ports currently reporting link up (including lwIP if enabled)
    uint8_t get_num_ports_link_up() const;

    // Number of external (non-lwIP) ports with link up
    // Used to determine if promiscuous mode is needed on Ethernet
    uint8_t get_num_external_ports_link_up() const;

    // Check if any port (other than exclude_port) can receive frames
    // Used for backpressure signaling
    bool any_port_can_receive(AP_Networking_SwitchPort *exclude_port) const;

    // Maximum supported Ethernet frame size that hub will route (includes VLAN)
    static constexpr size_t MAX_ETH_FRAME = 1522;

#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS
    // Initialize COBS subsystem (creates shared thread)
    bool init_cobs();
    
    // Register a COBS bond for thread processing
    void register_cobs_bond(AP_Networking_SwitchPort_COBS *bond);
#endif

private:
    AP_Networking_SwitchPort *ports[AP_NETWORKING_SWITCHPORT_MAX_INSTANCES];
    uint8_t num_ports;
    uint32_t frames_routed;
    uint32_t frames_dropped;
    HAL_Semaphore route_sem;

#if AP_NETWORKING_BACKEND_SWITCHPORT_COBS
    // COBS bond tracking for thread processing
    static constexpr uint8_t MAX_COBS_BONDS = 3;  // Bonds 1, 2, 3
    AP_Networking_SwitchPort_COBS *cobs_bonds[MAX_COBS_BONDS];
    uint8_t num_cobs_bonds = 0;
    HAL_Semaphore cobs_sem;  // Protects cobs_bonds[] and num_cobs_bonds
    
    // Shared COBS thread (services all COBS bonds)
    void cobs_thread_run();
    bool cobs_thread_started = false;
#endif

#if AP_NETWORKING_SWITCH_SWITCHING_ENABLED
    struct MACEntry {
        uint8_t mac[6];
        int8_t port_idx;
        uint32_t last_seen_ms;
    };
    // MAC table - only allocated when needed (more than 2 ports)
    // With exactly 2 ports, we use bridge mode: forward to the other port
    MACEntry *mac_table = nullptr;

    // Learn source MAC on ingress
    void learn_mac(const uint8_t *mac, int8_t port_idx);

    // Lookup destination MAC, returns port index or -1 if not found
    int8_t lookup_mac(const uint8_t *mac) const;

    // Invalidate/adjust MAC entries when port at given index is removed
    void fixup_mac_entries_for_removed_port(uint8_t removed_idx);

    // Age out stale entries
    void age_mac_table();

    // Check if MAC is broadcast or multicast
    static bool is_broadcast_or_multicast(const uint8_t *mac)
    {
        return (mac[0] & 0x01) != 0;
    }

    uint32_t last_age_ms;
#endif // AP_NETWORKING_SWITCH_SWITCHING_ENABLED
};

#endif // AP_NETWORKING_BACKEND_SWITCH


