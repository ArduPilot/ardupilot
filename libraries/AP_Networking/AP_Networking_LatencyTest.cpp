
#include "AP_Networking_LatencyTest.h"

#if AP_NETWORKING_LATENCYTEST_ENABLED
#include <GCS_MAVLink/GCS.h>

#ifndef AP_NETWORKING_LATENCYTEST_DEFAULT_PORT
#define AP_NETWORKING_LATENCYTEST_DEFAULT_PORT   5555
#endif

const AP_Param::GroupInfo AP_Networking_LatencyTest::var_info[] = {

    // @Param: DST_PORT
    // @DisplayName: Destination Port
    // @Description: Destination Port
    // @Range: 1 65535
    // @User: Standard
    AP_GROUPINFO("DST_PORT", 1, AP_Networking_LatencyTest, _eth.port, AP_NETWORKING_LATENCYTEST_DEFAULT_PORT),

    // @Param: DST_ADDR0
    // @DisplayName: IPv4 Dest Addr MSB
    // @Description: IPv4 Destination Address MSB. Example: 192.xxx.xxx.xxx. Default IP is Broadcast: 255.255.255.255
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("DST_ADDR0", 2,  AP_Networking_LatencyTest,    _eth.ip[0],   255),

    // @Param: DST_ADDR1
    // @DisplayName: IPv4 Dest Addr 2nd byte
    // @Description: IPv4 Destination Address 2nd byte. Example: xxx.168.xxx.xxx. Default IP is Broadcast: 255.255.255.255
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("DST_ADDR1", 3,  AP_Networking_LatencyTest,    _eth.ip[1],   255),

    // @Param: DST_ADDR2
    // @DisplayName: IPv4 Dest Addr 3rd byte
    // @Description: IPv4 Destination Address 3rd byte. Example: xxx.xxx.13.xxx. Default IP is Broadcast: 255.255.255.255
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("DST_ADDR2", 4,  AP_Networking_LatencyTest,    _eth.ip[2],   255),

    // @Param: DST_ADDR3
    // @DisplayName: IPv4 Dest Addr LSB
    // @Description: IPv4 Destination Address LSB. Example: xxx.xxx.xxx.123. Default IP is Broadcast: 255.255.255.255
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("DST_ADDR3", 5,  AP_Networking_LatencyTest,    _eth.ip[3],   255),
    
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/// Constructor
AP_Networking_LatencyTest::AP_Networking_LatencyTest(AP_Networking &front, AP_Networking::AP_Networking_State &state,  AP_Networking_Params &params) :
    AP_Networking_Backend(front, state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_Networking_LatencyTest::init()
{
    if (_eth.pcb == nullptr) {
        _eth.pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
        if (_eth.pcb != nullptr) {
            // set up RX callback
            udp_recv(_eth.pcb, AP_Networking_LatencyTest::latencytest_recv_callback, this);
        }
    }
}

void AP_Networking_LatencyTest::update()
{
    if (_eth.pcb == nullptr) {
        // init failure or disabled
        return;
    }

    uint32_t last_rx_data = 0;
    {
        // TODO: do we need to wrap this WITH_SEMAPHORE?
        WITH_SEMAPHORE(_stats.sem);
        last_rx_data = _stats.last_rx_data;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if ((last_rx_data == 0 && now_ms <= 5000) || _stats.update_last_ms == 0) {
        // wait for 5 seconds before starting in case someone 
        // is already trying to connect to us
        _stats.update_last_ms = now_ms;
        return;
    }
    if (now_ms - _stats.update_last_ms < 1000) {
        // run this update() at 1Hz
        return;
    }
    _stats.update_last_ms = now_ms;

    if (last_rx_data == 0) {
        // We've never received a packet. Start the process by sending "1" at 1Hz
        
        ip_addr_t dest_ip;
        IP_ADDR_FROM_ARRAY(&dest_ip, _eth.ip);

        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: LatencyTest START");

        // send initial packet
        LatencyTestPacket_t pkt {};
        pkt.magic = MAGIC_VALUE;
        pkt.data = 1;

        AP_Networking::send_udp(_eth.pcb, dest_ip, _eth.port.get(), (uint8_t*)&pkt, sizeof(pkt));
        
    } else {
        // TODO: handle (last_rx_data < _stats.last_rx_data)
        const uint32_t cnt_per_sec = last_rx_data - _stats.last_rx_data_1Hz;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: cnt:%u cnt/s:%u lag:%.2fms",
            (unsigned)last_rx_data,
            (unsigned)cnt_per_sec,
            (double)(1000.0f / (float)cnt_per_sec));

        _stats.last_rx_data_1Hz = last_rx_data;
    }
}

void AP_Networking_LatencyTest::latencytest_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p == nullptr) {
        return;
    }

    struct LatencyTestPacket_t *p_payload = (struct LatencyTestPacket_t *)p->payload;
    
    if (p_payload->magic == AP_Networking_LatencyTest::MAGIC_VALUE && p->len == sizeof(LatencyTestPacket_t))
  //if (p_payload->magic == AP_Networking_LatencyTest::MAGIC_VALUE)
    {
        AP_Networking_LatencyTest* driver = (AP_Networking_LatencyTest*)arg;
        if (driver != nullptr) {

            if (port != driver->_eth.port.get()) {
                // reject packets that are not for our port
                pbuf_free(p);
                return;
            }

            WITH_SEMAPHORE(driver->_stats.sem);
            driver->_stats.last_rx_data = p_payload->data;
        }
        p_payload->data = p_payload->data + 1;
        udp_sendto(pcb, p, &pcb->remote_ip, port);
    }
    pbuf_free(p);
}

#endif // AP_NETWORKING_LATENCYTEST_ENABLED

