/*
  Class to handle RC over netowrk and convert it to CAN
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/Socket.h>
#include <AP_Networking/AP_Networking_address.h>

#ifndef AP_PERIPH_RC_NETWORKING_ENABLED
#define AP_PERIPH_RC_NETWORKING_ENABLED 1
#endif


#if AP_PERIPH_RC_NETWORKING_ENABLED

#define SBUS_INPUT_CHANNELS	16

// Lightweight, self-contained UDP channel abstraction.
// Works as client (connect) or server (bind).
class UDPChannel {
public:
    UDPChannel() = default;
    ~UDPChannel() { close(); }

    // --- Lifecycle ----------------------------------------------------------
    bool open_client(const char* ip_str, uint16_t port);
    bool open_server(uint16_t port);
    void close();

    //---Connectivity --------------------------------------------------------
    void last_recv_address(const char *&ip_addr, uint16_t &port) const {
        if (sock != nullptr) {
            sock->last_recv_address(ip_addr, port);
        } else {
            ip_addr = nullptr;
            port = 0;
        }
    }

    // --- I/O ---------------------------------------------------------------
    size_t write(const uint8_t* buf, size_t len, uint8_t* error = nullptr);
    size_t read(uint8_t* buf, size_t len, int timeout_ms = 0, uint8_t* error = nullptr);

    // --- Status -------------------------------------------------------------
    bool is_open() const { return sock != nullptr; }
    bool is_connected() const { return connected; }

    uint32_t tx_bytes() const { return tx_bytes_total; }
    uint32_t rx_bytes() const { return rx_bytes_total; }

private:
    SocketAPM* sock = nullptr;
    bool connected = false;

    uint32_t tx_bytes_total = 0;
    uint32_t rx_bytes_total = 0;
};

class AirSide_RC_Networking {
public:
    AirSide_RC_Networking();
    ~AirSide_RC_Networking() = default;

    bool init();

    void update();            // telemetry sender
    void send_debug(const char* msg);

    void read_sbus_packet();

    bool get_latest_can_frame(uint16_t decoded_rc_vals[SBUS_INPUT_CHANNELS], uint16_t &num_values, bool &failsafe) {
        if (new_sbus_can_frame == false) {
            return false;
        }
        memcpy(decoded_rc_vals, decoded_rc_values, sizeof(decoded_rc_values));
        num_values = rc_num_values;
        failsafe = _sbus_failsafe;
        new_sbus_can_frame = false;
        return true;
    }

private:

    UDPChannel rc_in;
    bool telem_active = false;

    void process_sbus_buffer(const uint8_t *buf, uint32_t nbytes);
    uint8_t sbus_crc8(const uint8_t *data, size_t len);
    bool sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values,
                                     bool &sbus_failsafe, uint16_t max_values);
    void decode_11bit_channels(const uint8_t* data, uint8_t nchannels, uint16_t *values, uint16_t mult, uint16_t div, uint16_t offset);


    uint16_t compute_crc16(const uint8_t *data, size_t len);

    struct Channels11Bit_8Chan {
        uint32_t ch0 : 11;
        uint32_t ch1 : 11;
        uint32_t ch2 : 11;
        uint32_t ch3 : 11;
        uint32_t ch4 : 11;
        uint32_t ch5 : 11;
        uint32_t ch6 : 11;
        uint32_t ch7 : 11;
    } PACKED;

    uint8_t latest_sbus_frame[25];  // Latest SBUS frame
    bool new_sbus_frame = false;  // Flag to indicate if a new SBUS frame is available
    bool new_sbus_can_frame = false;

    uint16_t decoded_rc_values[SBUS_INPUT_CHANNELS];
    uint16_t rc_num_values=0;
    bool _sbus_failsafe = false;

    uint8_t carry_buffer[50];  // Carry buffer for partial SBUS frames
    uint32_t carry_buffer_len = 0;

    uint32_t valid_sbus_packets = 0;
    uint32_t invalid_sbus_packets = 0;
    uint32_t total_sbus_bytes = 0;
    uint32_t last_sbus_timestamp = 0;
    uint32_t lost_frames = 0;  // Count frames lost due to timing
};


#endif // AP_PERIPH_AIRBOSS_NETWORKING_JOYSTICK_ENABLED

