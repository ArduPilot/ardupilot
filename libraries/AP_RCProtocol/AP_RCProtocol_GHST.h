/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_GHST_ENABLED

#include "AP_RCProtocol.h"
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "SoftSerial.h"

#define GHST_MAX_CHANNELS   16U      // Maximum number of channels from GHST datastream
#define GHST_FRAMELEN_MAX   14U      // maximum possible framelength
#define GHST_HEADER_LEN     2U       // header length
#define GHST_FRAME_PAYLOAD_MAX (GHST_FRAMELEN_MAX - GHST_HEADER_LEN)     // maximum size of the frame length field in a packet
#define GHST_BAUDRATE      420000U
#define GHST_TX_TIMEOUT    500000U   // the period after which the transmitter is considered disconnected (matches copters failsafe)
#define GHST_RX_TIMEOUT    150000U   // the period after which the receiver is considered disconnected (>ping frequency)

class AP_RCProtocol_GHST : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_GHST(AP_RCProtocol &_frontend);
    virtual ~AP_RCProtocol_GHST();
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    void process_handshake(uint32_t baudrate) override;
    void update(void) override;

    // is the receiver active, used to detect power loss and baudrate changes
    bool is_rx_active() const override {
        return AP_HAL::micros() < _last_rx_frame_time_us + GHST_RX_TIMEOUT;
    }

    // is the transmitter active, used to adjust telemetry data
    bool is_tx_active() const {
        // this is the same as the Copter failsafe timeout
        return AP_HAL::micros() < _last_tx_frame_time_us + GHST_TX_TIMEOUT;
    }

    // get singleton instance
    static AP_RCProtocol_GHST* get_singleton() {
        return _singleton;
    }

    enum FrameType {
        GHST_UL_RC_CHANS_HS4_5TO8 = 0x10, // Control packet with 4 primary channels + CH5-8
        GHST_UL_RC_CHANS_HS4_9TO12 = 0x11, // Control packet with 4 primary channels + CH9-12
        GHST_UL_RC_CHANS_HS4_13TO16 = 0x12, // Control packet with 4 primary channels + CH13-16
        GHST_UL_RC_CHANS_RSSI = 0x13, // Control packet with RSSI and LQ data
        GHST_UL_RC_VTX_CTRL = 0x14, // Goggle/FC channel changing
        // -> 0x1F reserved
        GHST_UL_VTX_SETUP = 0x20, // vTx Setup w/o 4 primary channels (GECO Only)
        GHST_UL_MSP_REQ = 0x21, // MSP frame, Request
        GHST_UL_MSP_WRITE = 0x22, // MSP frame, Write

        GHST_DL_PACK_STAT = 0x23, // Battery Status
        GHST_DL_GPS_PRIMARY = 0x25, // Primary GPS Data
        GHST_DL_GPS_SECONDARY = 0x26, // Secondary GPS Data
        GHST_DL_MAGBARO = 0x27, // Magnetometer, Barometer (and Vario) Data
        GHST_DL_MSP_RESP = 0x28, // MSP Response

        GHST_UL_RC_CHANS_HS4_12_5TO8 = 0x30, // Control packet with 4 primary channels + CH5-8
        GHST_UL_RC_CHANS_HS4_12_9TO12 = 0x31, // Control packet with 4 primary channels + CH9-12
        GHST_UL_RC_CHANS_HS4_12_13TO16 = 0x32, // Control packet with 4 primary channels + CH13-16
        GHST_UL_RC_CHANS_12_RSSI = 0x33, // Control packet with RSSI and LQ data
        // 0x30 -> 0x3f - raw 12 bit packets
    };

    enum DeviceAddress {
        GHST_ADDRESS_FLIGHT_CONTROLLER = 0x82,
        GHST_ADDRESS_GOGGLES = 0x83,
        GHST_ADDRESS_GHST_RECEIVER = 0x89,
    };

    struct Frame {
        uint8_t device_address;
        uint8_t length;
        uint8_t type;
        uint8_t payload[GHST_FRAME_PAYLOAD_MAX - 1]; // type is already accounted for
    } PACKED;

    struct Channels12Bit_4Chan {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        uint32_t ch0 : 12;
        uint32_t ch1 : 12;
        uint32_t ch2 : 12;
        uint32_t ch3 : 12;
    } PACKED;

    struct RadioFrame {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        Channels12Bit_4Chan channels;   // high-res channels
        uint8_t lowres_channels[4];     // low-res channels
    } PACKED;

    struct LinkStatisticsFrame {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        Channels12Bit_4Chan channels;
        uint8_t link_quality; // ( 0 - 100)
        uint8_t rssi_dbm; // ( dBm * -1 )
        uint8_t protocol : 5;
        uint8_t telemetry : 1;
        uint8_t alt_scale : 1;
        uint8_t reserved : 1;
        int8_t tx_power;
    } PACKED;

    enum RFMode {
        GHST_RF_MODE_NORMAL = 5,    // 55Hz
        GHST_RF_MODE_RACE = 6,      // 160Hz
        GHST_RF_MODE_PURERACE = 7,  // 250Hz
        GHST_RF_MODE_LR = 8,        // 19Hz
        GHST_RF_MODE_RACE250 = 10,  // 250Hz
        GHST_RF_MODE_RACE500 = 11,  // 500Hz
        GHTS_RF_MODE_SOLID150 = 12, // 150Hz
        GHST_RF_MODE_SOLID250 = 13, // 250Hz
        RF_MODE_MAX_MODES,
        RF_MODE_UNKNOWN,
    };

    struct LinkStatus {
        int16_t rssi = -1;
        int16_t link_quality = -1;
        uint8_t rf_mode;
    };

    bool is_telemetry_supported() const;

    // this will be used by AP_GHST_Telem to access link status data
    // from within AP_RCProtocol_GHST thread so no need for cross-thread synch
    const volatile LinkStatus& get_link_status() const {
        return _link_status;
    }

    // return the link rate as defined by the LinkStatistics
    uint16_t get_link_rate()  const;

    // return the protocol string
    const char* get_protocol_string() const;

private:
    struct Frame _frame;
    struct Frame _telemetry_frame;
    uint8_t _frame_ofs;
    uint8_t _frame_crc;

    const uint8_t MAX_CHANNELS = MIN((uint8_t)GHST_MAX_CHANNELS, (uint8_t)MAX_RCIN_CHANNELS);

    static AP_RCProtocol_GHST* _singleton;

    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    bool decode_ghost_packet();
    bool process_telemetry(bool check_constraint = true);
    void process_link_stats_frame(const void* data);
    bool write_frame(Frame* frame);
    AP_HAL::UARTDriver* get_current_UART() { return get_available_UART(); }

    uint16_t _channels[GHST_MAX_CHANNELS];    /* buffer for extracted RC channel data as pulsewidth in microseconds */

    void add_to_buffer(uint8_t index, uint8_t b) { ((uint8_t*)&_frame)[index] = b; }

    uint32_t _last_frame_time_us;
    uint32_t _last_tx_frame_time_us;
    uint32_t _last_rx_frame_time_us;
    uint32_t _start_frame_time_us;
    bool telem_available;
    bool _use_lq_for_rssi;
    int16_t derive_scaled_lq_value(uint8_t uplink_lq);

    volatile struct LinkStatus _link_status;

    static const uint16_t RF_MODE_RATES[RFMode::RF_MODE_MAX_MODES];
};

namespace AP {
    AP_RCProtocol_GHST* ghost();
};

#endif  // AP_RCPROTOCOL_GHST_ENABLED
