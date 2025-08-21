
#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED
#include <AP_Param/AP_Param.h>

#include "AP_Networking_address.h"
#include "AP_Networking_Backend.h"
#include "AP_Networking_CAN.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>

/*
  Note! all uint32_t IPv4 addresses are in host byte order
*/

// declare backend classes
class AP_Networking_Backend;
class AP_Networking_ChibiOS;

class SocketAPM;

class AP_Networking
{
public:
    friend class AP_Networking_Backend;
    friend class AP_Networking_ChibiOS;
    friend class AP_Networking_PPP;
    friend class AP_Vehicle;
    friend class Networking_Periph;

    AP_Networking();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking);

    // initialize the library. This should only be run once
    void init();

    // update task, called at 10Hz
    void update();

    static AP_Networking *get_singleton(void)
    {
        return singleton;
    }

    HAL_Semaphore &get_semaphore(void)
    {
        return sem;
    }
    
    // Networking interface is enabled and initialized
    bool is_healthy() const
    {
        return param.enabled && backend != nullptr;
    }

    // returns true if DHCP is enabled
    bool get_dhcp_enabled() const
    {
#if AP_NETWORKING_DHCP_AVAILABLE
        return param.dhcp;
#else
        // DHCP is not available from our scope but could be enabled/controlled
        // by the OS which is the case on Linux builds, including SITL
        // TODO: ask the OS if DHCP is enabled
        return false;
#endif
    }

    // Sets DHCP to be enabled or disabled
    void set_dhcp_enable(const bool enable)
    {
#if AP_NETWORKING_DHCP_AVAILABLE
        param.dhcp.set(enable);
#endif
    }

    // returns the 32bit value of the active IP address that is currently in use
    uint32_t get_ip_active() const;

    // returns the 32bit value of the user-parameter static IP address
    uint32_t get_ip_param() const
    {
#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
        return param.ipaddr.get_uint32();
#else
        // TODO: ask the OS for the IP address
        return 0;
#endif
    }

    // sets the user-parameter static IP address from a 32bit value
    void set_ip_param(const uint32_t ip)
    {
#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
        param.ipaddr.set_uint32(ip);
#endif
    }

    // returns the 32bit value of the active Netmask that is currently in use
    uint32_t get_netmask_active() const;

    // returns the 32bit value of the of the user-parameter static Netmask
    uint32_t get_netmask_param() const
    {
#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
        return convert_netmask_bitcount_to_ip(param.netmask.get());
#else
        // TODO: ask the OS for the Netmask
        return 0;
#endif
    }

    uint32_t get_gateway_active() const;

    uint32_t get_gateway_param() const
    {
#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
        return param.gwaddr.get_uint32();
#else
        // TODO: ask the OS for the Gateway
        return 0;
#endif
    }

    void set_gateway_param(const uint32_t gw)
    {
#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
        param.gwaddr.set_uint32(gw);
#endif
    }

    // wait in a thread for network startup
    void startup_wait(void) const;

    // convert string to ethernet mac address
    static bool convert_str_to_macaddr(const char *mac_str, uint8_t addr[6]);

    // address to string using a static return buffer for scripting
    static const char *address_to_str(uint32_t addr);
    
    // helper functions to convert between 32bit Netmask and counting consecutive bits and back
    static uint32_t convert_netmask_bitcount_to_ip(const uint32_t netmask_bitcount);
    static uint8_t convert_netmask_ip_to_bitcount(const uint32_t netmask_ip);

    // add new routes for an interface.
    // Returns true if the route is added or the route already exists
    bool add_route(uint8_t backend_idx, uint8_t iface_idx, uint32_t dest_ip, uint8_t mask_len);

    // hook for custom routes
    struct netif *routing_hook(uint32_t dest);

    /*
      send contents of a file to a socket then close both socket and file
     */
    bool sendfile(SocketAPM *sock, int fd);

    static const struct AP_Param::GroupInfo var_info[];

    enum class OPTION {
        PPP_ETHERNET_GATEWAY=(1U<<0),
#if AP_NETWORKING_CAN_MCAST_ENABLED
        CAN1_MCAST_ENDPOINT=(1U<<1),
        CAN2_MCAST_ENDPOINT=(1U<<2),
#if AP_NETWORKING_CAN_MCAST_BRIDGING_ENABLED
        CAN1_MCAST_BRIDGED=(1U<<3),
        CAN2_MCAST_BRIDGED=(1U<<4),
#endif
#endif
        PPP_TIMEOUT_DISABLE=(1U<<5),
        PPP_ECHO_LIMIT_DISABLE=(1U<<6),
#if AP_NETWORKING_CAPTURE_ENABLED
        CAPTURE_PACKETS=(1U<<7),
#endif
    };
    bool option_is_set(OPTION option) const {
        return (param.options.get() & int32_t(option)) != 0;
    }

#if AP_NETWORKING_CAN_MCAST_BRIDGING_ENABLED
    bool is_can_mcast_ep_bridged(uint8_t bus) const {
        return (option_is_set(OPTION::CAN1_MCAST_BRIDGED) && bus == 0) ||
               (option_is_set(OPTION::CAN2_MCAST_BRIDGED) && bus == 1);
    }
#endif

private:
    static AP_Networking *singleton;

    void announce_address_changes();

    struct {
#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
        AP_Networking_IPV4 ipaddr{AP_NETWORKING_DEFAULT_STATIC_IP_ADDR};
        AP_Int8 netmask;    // bits to mask. example: (16 == 255.255.0.0) and (24 == 255.255.255.0)
        AP_Networking_IPV4 gwaddr{AP_NETWORKING_DEFAULT_STATIC_GW_ADDR};
        AP_Networking_MAC macaddr{AP_NETWORKING_DEFAULT_MAC_ADDR};
#if AP_NETWORKING_DHCP_AVAILABLE
        AP_Int8 dhcp;
#endif
#endif

        AP_Int8 enabled;
        AP_Int32 options;

#if AP_NETWORKING_TESTS_ENABLED
        AP_Int32 tests;
        AP_Networking_IPV4 test_ipaddr{AP_NETWORKING_TEST_IP};
#endif

#if AP_NETWORKING_PPP_GATEWAY_ENABLED
        AP_Networking_IPV4 remote_ppp_ip{AP_NETWORKING_REMOTE_PPP_IP};
#endif
    } param;

    AP_Networking_Backend *backend;

#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    AP_Networking_Backend *backend_PPP;
#endif

    HAL_Semaphore sem;

    enum class NetworkPortType {
        NONE = 0,
        UDP_CLIENT = 1,
        UDP_SERVER = 2,
        TCP_CLIENT = 3,
        TCP_SERVER = 4,
    };

#if AP_NETWORKING_REGISTER_PORT_ENABLED
    // class for NET_Pn_* parameters
    class Port : public AP_SerialManager::RegisteredPort {
    public:
        /* Do not allow copies */
        CLASS_NO_COPY(Port);

        Port() {}

        static const struct AP_Param::GroupInfo var_info[];
        AP_Enum<NetworkPortType> type_param;
        NetworkPortType type;
        AP_Networking_IPV4 ip {"0.0.0.0"};
        AP_Int32 port;
        SocketAPM *sock;
        SocketAPM *listen_sock;

        bool is_initialized() override {
            return true;
        }
        bool tx_pending() override {
            return false;
        }

        void udp_client_init(void);
        void udp_server_init(void);
        void tcp_server_init(void);
        void tcp_client_init(void);

        void udp_client_loop(void);
        void udp_server_loop(void);
        void tcp_client_loop(void);
        void tcp_server_loop(void);

        bool send_receive(void);

    private:
        bool init_buffers(const uint32_t size_rx, const uint32_t size_tx);
        void thread_create(AP_HAL::MemberProc);

        uint32_t txspace() override;
        void _begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
        size_t _write(const uint8_t *buffer, size_t size) override;
        ssize_t _read(uint8_t *buffer, uint16_t count) override;
        uint32_t _available() override;
        void _end() override {}
        void _flush() override {}
        bool _discard_input() override;

        enum flow_control get_flow_control(void) override;

        uint32_t bw_in_bytes_per_second() const override {
            return 1000000UL;
        }

        ByteBuffer *readbuffer;
        ByteBuffer *writebuffer;
        char thread_name[10];
        uint32_t last_size_tx;
        uint32_t last_size_rx;
        bool packetise;
        bool connected;
        uint32_t last_udp_connect_address;
        uint16_t last_udp_connect_port;
        bool have_received;
        bool close_on_recv_error;
        uint32_t last_udp_srv_recv_time_ms;

        // statistics
        uint32_t tx_stats_bytes;
        uint32_t rx_stats_bytes;

        HAL_Semaphore sem;

    protected:
#if HAL_UART_STATS_ENABLED
        // Getters for cumulative tx and rx counts
        uint32_t get_total_tx_bytes() const override { return tx_stats_bytes; }
        uint32_t get_total_rx_bytes() const override { return rx_stats_bytes; }
#endif
    };
#endif // AP_NETWORKING_REGISTER_PORT_ENABLED

private:
    uint32_t announce_ms;

#if AP_NETWORKING_TESTS_ENABLED
    enum {
        TEST_UDP_CLIENT = (1U<<0),
        TEST_TCP_CLIENT = (1U<<1),
        TEST_TCP_DISCARD = (1U<<2),
        TEST_TCP_REFLECT = (1U<<3),
        TEST_CONNECTOR_LOOPBACK = (1U<<4),
    };
    void start_tests(void);
    void test_UDP_client(void);
    void test_TCP_client(void);
    void test_TCP_discard(void);
    void test_TCP_reflect(void);
    void test_connector_loopback(void);
#endif // AP_NETWORKING_TESTS_ENABLED

#if AP_NETWORKING_REGISTER_PORT_ENABLED
    // ports for registration with serial manager
    Port ports[AP_NETWORKING_NUM_PORTS];
#endif

#if AP_NETWORKING_CAN_MCAST_ENABLED
    AP_Networking_CAN mcast_server;
#endif

    // support for sendfile()
    struct SendFile {
        SocketAPM *sock;
        int fd;
        void close(void);
    } sendfiles[AP_NETWORKING_NUM_SENDFILES];

    uint8_t *sendfile_buf;
    uint32_t sendfile_bufsize;
    void sendfile_check(void);
    bool sendfile_thread_started;

    void ports_init(void);
};

namespace AP
{
    AP_Networking &network();
};

extern "C" {
int ap_networking_printf(const char *fmt, ...);
}

#endif // AP_NETWORKING_ENABLED
