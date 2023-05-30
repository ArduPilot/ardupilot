
#include "AP_Networking_SpeedTest.h"

#if AP_NETWORKING_SPEEDTEST_ENABLED
#include <AP_Math/AP_Math.h> // for MIN()
#include <GCS_MAVLink/GCS.h>

#ifndef AP_NETWORKING_SPEEDTEST_DEFAULT_PORT
#define AP_NETWORKING_SPEEDTEST_DEFAULT_PORT   5555
#endif

#ifndef AP_NETWORKING_SPEEDTEST_SEND_ATTEMPT_COUNT_PER_UPDATE_TICK
#define AP_NETWORKING_SPEEDTEST_SEND_ATTEMPT_COUNT_PER_UPDATE_TICK 200
#endif


const AP_Param::GroupInfo AP_Networking_SpeedTest::var_info[] = {

    // @Param: DST_PORT
    // @DisplayName: Destination Port
    // @Description: Destination Port
    // @Range: 1 65535
    // @User: Standard
    AP_GROUPINFO("DST_PORT", 1, AP_Networking_SpeedTest, _params.port, AP_NETWORKING_SPEEDTEST_DEFAULT_PORT),

    // @Param: DST_ADDR0
    // @DisplayName: IPv4 Dest Addr MSB
    // @Description: IPv4 Destination Address MSB. Example: 192.xxx.xxx.xxx. Default IP is Broadcast: 255.255.255.255
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("DST_ADDR0", 2,  AP_Networking_SpeedTest,    _params.ip[0],   255),

    // @Param: DST_ADDR1
    // @DisplayName: IPv4 Dest Addr 2nd byte
    // @Description: IPv4 Destination Address 2nd byte. Example: xxx.168.xxx.xxx. Default IP is Broadcast: 255.255.255.255
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("DST_ADDR1", 3,  AP_Networking_SpeedTest,    _params.ip[1],   255),

    // @Param: DST_ADDR2
    // @DisplayName: IPv4 Dest Addr 3rd byte
    // @Description: IPv4 Destination Address 3rd byte. Example: xxx.xxx.13.xxx. Default IP is Broadcast: 255.255.255.255
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("DST_ADDR2", 4,  AP_Networking_SpeedTest,    _params.ip[2],   255),

    // @Param: DST_ADDR3
    // @DisplayName: IPv4 Dest Addr LSB
    // @Description: IPv4 Destination Address LSB. Example: xxx.xxx.xxx.123. Default IP is Broadcast: 255.255.255.255
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("DST_ADDR3", 5,  AP_Networking_SpeedTest,    _params.ip[3],   255),

    // @Param: SIZE
    // @DisplayName: SpeedTest payload size
    // @Description: SpeedTest payload size per packet. The largest UDP payload capacity for Ethernet is 1472. That's 1500 minus UDP header. Going above this will cause packet fragmentation. Jumbo frames are not supported.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("SIZE", 7, AP_Networking_SpeedTest, _params.payload_size, AP_NETWORKING_ETHERNET_UDP_PAYLOAD_MAX_SIZE),

    // @Param: CONTENT
    // @DisplayName: SpeedTest payload content
    // @Description: SpeedTest payload content. This controls the content of the packet. It doesn't not change any behavior, it's used for debugging if you want to capture and/or view the data in tools such as Wireshark. A value of -1 will fill the packet with the string "ArduPilot Networking SpeedTest". Values 0-255 will populate the packet with that number.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("CONTENT", 8, AP_Networking_SpeedTest, _params.content, -1),

    // @Param: DURATION
    // @DisplayName: SpeedTest duration
    // @Description: SpeedTest duration in seconds. Setting the value > 0 will start the speedtest. To protect the network, this value resets to zero on boot and after the Speedtest duration has elapsed.
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("DURATION", 9, AP_Networking_SpeedTest, _params.duration_seconds, 0),
    
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/// Constructor
AP_Networking_SpeedTest::AP_Networking_SpeedTest(AP_Networking &front, AP_Networking::AP_Networking_State &state,  AP_Networking_Params &params) :
    AP_Networking_Backend(front, state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

void AP_Networking_SpeedTest::init()
{
    // always set this to 0 so we don't spam on boot
    _params.duration_seconds.set_and_save(0);

    if (_eth.pcb == nullptr) {
        _eth.pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
        if (_eth.pcb != nullptr) {
            // allow to sending broadcast packets
            ip_set_option(_eth.pcb, SOF_BROADCAST);
        }
    }
}

void AP_Networking_SpeedTest::update()
{
    if (_eth.pcb == nullptr || _params.duration_seconds.get() <= 0) {
        // init failure or disabled
        _stats.start_ms = 0;
        return;
    }
    
    if (_stats.start_ms == 0) {
        memset(&_stats, 0, sizeof(_stats));
        _stats.start_ms = AP_HAL::millis();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: SpeedTest START, Duration %.2fs", (double)_params.duration_seconds.get());
    }

    send_data();

    if (report_stats()) {
        _params.duration_seconds.set_and_save(0);
    }
}

void AP_Networking_SpeedTest::send_data()
{
    const uint32_t payload_size = constrain_int32(_params.payload_size.get(), 0, 0xFFFF);
    uint8_t payload[payload_size];

    if (payload_size > 0) {
        // fill the payload
        if (_params.content.get() == -1) {
            uint16_t offset = 0;
            const char* str = "ArduPilot Networking SpeedTest";
            const uint16_t str_len = strlen(str);
            while (offset < payload_size-1) {
                const uint16_t offset_len = MIN(str_len, payload_size-offset-1);
                memcpy(&payload[offset], str, offset_len);
                offset += offset_len;
            }

        } else if (_params.content.get() >= 0 && _params.content.get() <= 255) {
            memset(payload, (uint8_t)_params.content.get(), payload_size);

        } else {
            // do not initialize payload[], just send whatever crap/noise that's on the stack
        }
    }

    ip_addr_t dest_ip;
    IP_ADDR_FROM_ARRAY(&dest_ip, _params.ip);
    for (int32_t i=0; i<AP_NETWORKING_SPEEDTEST_SEND_ATTEMPT_COUNT_PER_UPDATE_TICK; i++) {
        // if we were able to queue a packet to send, try to queue one more to keep the queue full
        // the result is negative on error, else how many payload bytes were sent
        const int32_t payload_bytes_sent = AP_Networking::send_udp(_eth.pcb, dest_ip, _params.port, payload, payload_size);

        if (payload_bytes_sent < 0) {
            // Wasn't able to send, don't bother trying to send any more
            // This is expected, the result is likely a memory error such
            // as err_enum_t::ERR_MEM or err_enum_t::ERR_BUF. We're intentionally
            // shoving as many packets into the tx queue until it throws some sort
            // of out-of-memory error.
            break;            
        }

        // Packet successfully queued! Lets keep track of some stats
        _stats.packets_total++;
        _stats.bytes_total += payload_bytes_sent;

        _stats.packets_per_sec++;
        _stats.bytes_per_sec += payload_bytes_sent;
    }
}

bool AP_Networking_SpeedTest::report_stats()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float elapsed_seconds = (now_ms - _stats.start_ms) * 0.001f;
    const bool is_done = (elapsed_seconds >= _params.duration_seconds.get());

    if (is_done) {
        // Post final stats
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: Total data:%.3f MB, wire:%.3f Mb",
            (double)(_stats.bytes_total * 1.0E-6f),
            (double)(((_stats.packets_total*AP_NETWORKING_ETH_IP_UDP_OVERHEAD) + _stats.bytes_total) * 8 * 1.0E-6f));

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: Avg/s data:%.3f MB, wire:%.3f Mb",
            (double)((_stats.bytes_total * 1.0E-6f) / elapsed_seconds),
            (double)((((_stats.packets_total*AP_NETWORKING_ETH_IP_UDP_OVERHEAD) + _stats.bytes_total) * 8 * 1.0E-6f) / elapsed_seconds));

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: Peak/s data:%.3f MB, wire:%.3f Mb",
            (double)(_stats.bytes_per_sec_max * 1.0E-6f),
            (double)(((_stats.packets_per_sec_max*AP_NETWORKING_ETH_IP_UDP_OVERHEAD) + _stats.bytes_per_sec_max) * 8 * 1.0E-6f));

    } else if (now_ms - _stats.gcs_send_ms >= 1000) {
        _stats.gcs_send_ms = now_ms;

        // Post stats at 1Hz
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"NET: %.3f MB/s, wire %.3f Mb/s",
            (double)_stats.bytes_per_sec * 1.0E-6f,
            (double)(((_stats.packets_per_sec*AP_NETWORKING_ETH_IP_UDP_OVERHEAD) + _stats.bytes_per_sec) * 8 * 1.0E-6f));

        if (_stats.packets_per_sec_max < _stats.packets_per_sec) {
            _stats.packets_per_sec_max = _stats.packets_per_sec;
        }
        _stats.packets_per_sec = 0;
        
        if (_stats.bytes_per_sec_max < _stats.bytes_per_sec) {
            _stats.bytes_per_sec_max = _stats.bytes_per_sec;
        }
        _stats.bytes_per_sec = 0;
    }

    return is_done;
}

#endif // AP_NETWORKING_SPEEDTEST_ENABLED

