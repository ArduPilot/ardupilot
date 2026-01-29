#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB_PORT_COBS || AP_NETWORKING_BACKEND_HUB_PORT_MAVLINK_COBS

#include <stdint.h>
#include <stddef.h>
#include <AP_Math/crc.h>
#include <string.h>
#include "AP_Networking_COBS.h"

/*
  Shared COBS protocol definitions for both UART and MAVLink transports.
  
  Frame types (detected via CRC):
  - Keepalive: "KA" + device_id[6] + rx_good[2] + CRC32
  - Data (single mode): ethernet_frame + CRC32
  - Data (ganged mode): seq[2] + ethernet_frame + CRC32(data + "GANG")
  
  The CRC suffix distinguishes frame types without explicit headers.
*/
namespace AP_Networking_COBS_Protocol
{
    // Keepalive format
    static constexpr uint8_t KA_MARKER[2] = {'K', 'A'};
    static constexpr size_t KA_MARKER_LEN = 2;
    static constexpr size_t KA_DEVICE_ID_LEN = 6;
    static constexpr size_t KA_RX_GOOD_LEN = 2;
    static constexpr size_t KA_DATA_LEN = KA_MARKER_LEN + KA_DEVICE_ID_LEN + KA_RX_GOOD_LEN;  // 10 bytes
    static constexpr size_t KA_TOTAL_LEN = KA_DATA_LEN + 4;  // + CRC32
    
    // CRC suffix for ganged mode (distinguishes from single mode)
    static constexpr uint8_t GANG_CRC_SUFFIX[4] = {'G', 'A', 'N', 'G'};
    
    // Frame size limits
    static constexpr size_t MAX_FRAME = 1522;
    static constexpr size_t MIN_ETH_FRAME = 14;  // Minimum = Ethernet header only
    
    // Timing
    static constexpr uint16_t KEEPALIVE_INTERVAL_MS = 500;
    static constexpr uint16_t KEEPALIVE_TIMEOUT_MS = 2000;
    static constexpr uint16_t LINK_TIMEOUT_MS = KEEPALIVE_INTERVAL_MS * 4;
    
    // Build a keepalive frame (without COBS encoding)
    // Returns total length including CRC
    inline size_t build_keepalive(uint8_t *buf, size_t buf_len,
                                   const uint8_t device_id[6], uint16_t rx_good)
    {
        if (buf_len < KA_TOTAL_LEN) {
            return 0;
        }
        
        // "KA" + device_id[6] + rx_good[2]
        memcpy(buf, KA_MARKER, KA_MARKER_LEN);
        memcpy(&buf[KA_MARKER_LEN], device_id, KA_DEVICE_ID_LEN);
        memcpy(&buf[KA_MARKER_LEN + KA_DEVICE_ID_LEN], &rx_good, KA_RX_GOOD_LEN);
        
        // CRC32
        uint32_t crc = crc_crc32(0, buf, KA_DATA_LEN);
        memcpy(&buf[KA_DATA_LEN], &crc, 4);
        
        return KA_TOTAL_LEN;
    }
    
    // Build a data frame (without COBS encoding)
    // Returns total length including CRC
    inline size_t build_data_frame(uint8_t *buf, size_t buf_len,
                                    const uint8_t *frame, size_t frame_len)
    {
        if (buf_len < frame_len + 4) {
            return 0;
        }
        
        memcpy(buf, frame, frame_len);
        uint32_t crc = crc_crc32(0, frame, frame_len);
        memcpy(&buf[frame_len], &crc, 4);
        
        return frame_len + 4;
    }
    
    // Frame type enumeration
    enum class FrameType {
        INVALID,
        KEEPALIVE,
        DATA_SINGLE,
        DATA_GANGED
    };
    
    // Identify frame type from decoded COBS data (includes CRC)
    // Returns frame type and populates payload/payload_len with data portion
    inline FrameType identify_frame(const uint8_t *data, size_t len,
                                     const uint8_t **payload_out, size_t *payload_len_out)
    {
        if (len < 4) {
            return FrameType::INVALID;
        }
        
        const size_t data_len = len - 4;
        uint32_t rx_crc;
        memcpy(&rx_crc, &data[data_len], 4);
        
        // Check for keepalive: exactly 10 bytes data, starts with "KA"
        if (data_len == KA_DATA_LEN && 
            data[0] == KA_MARKER[0] && data[1] == KA_MARKER[1]) {
            uint32_t calc_crc = crc_crc32(0, data, data_len);
            if (rx_crc == calc_crc) {
                *payload_out = data;
                *payload_len_out = data_len;
                return FrameType::KEEPALIVE;
            }
            return FrameType::INVALID;
        }
        
        // Check for data frame (need minimum ethernet frame size)
        if (data_len >= MIN_ETH_FRAME) {
            // Calculate both possible CRCs
            uint32_t single_crc = crc_crc32(0, data, data_len);
            uint32_t gang_crc = crc_crc32(single_crc, GANG_CRC_SUFFIX, 4);
            
            if (rx_crc == single_crc) {
                *payload_out = data;
                *payload_len_out = data_len;
                return FrameType::DATA_SINGLE;
            }
            
            if (rx_crc == gang_crc && data_len >= 2 + MIN_ETH_FRAME) {
                // Ganged: seq[2] + frame
                *payload_out = &data[2];
                *payload_len_out = data_len - 2;
                return FrameType::DATA_GANGED;
            }
        }
        
        return FrameType::INVALID;
    }
    
    // Extract keepalive fields
    inline void parse_keepalive(const uint8_t *data, uint8_t device_id_out[6], uint16_t *rx_good_out)
    {
        memcpy(device_id_out, &data[KA_MARKER_LEN], KA_DEVICE_ID_LEN);
        memcpy(rx_good_out, &data[KA_MARKER_LEN + KA_DEVICE_ID_LEN], KA_RX_GOOD_LEN);
    }
    
    // Extract ganged frame sequence number
    inline uint16_t get_ganged_seq(const uint8_t *data)
    {
        return data[0] | (data[1] << 8);
    }
    
    // COBS encode a protocol frame (with CRC) and add 0x00 delimiter
    // Returns total encoded length including delimiter, or 0 on error
    inline size_t encode_frame(const uint8_t *frame, size_t frame_len,
                                uint8_t *out_buf, size_t out_buf_len)
    {
        // Need space for COBS overhead + delimiter
        if (out_buf_len < frame_len + (frame_len / 254) + 2) {
            return 0;
        }
        
        size_t enc_len = AP_Networking_COBS::encode(frame, frame_len, 
                                                     out_buf, out_buf_len - 1);
        if (enc_len == 0) {
            return 0;
        }
        
        // Add 0x00 frame delimiter
        out_buf[enc_len] = 0x00;
        return enc_len + 1;
    }
}

#endif // AP_NETWORKING_BACKEND_HUB_PORT_COBS || AP_NETWORKING_BACKEND_HUB_PORT_MAVLINK_COBS
