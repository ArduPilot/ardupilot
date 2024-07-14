/*
  ESC Telemetry for HobbyWing DataLink
 */

#pragma once

#include "AP_ESC_Telem_config.h"

#if AP_HOBBYWING_DATALINK_ENABLED

#include "AP_ESC_Telem_Backend.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_Param/AP_Param.h>

class AP_HobbyWing_DataLink : AP_ESC_Telem_Backend {
public:

    static const struct AP_Param::GroupInfo var_info[];

    AP_HobbyWing_DataLink(AP_HAL::UARTDriver &_uart)
        : uart(_uart) {
        AP_Param::setup_object_defaults(this, var_info);
    }

    void init();
    void update();

    struct ESCInfo {
        bool seen;
    };

    // thread-safe method to get telemetry.  This "consumes" the
    // current telemetry packet, so unless another packet is received
    // subsequent calls will return false.
    bool get_telem(HWESC hwesc[8], ESCInfo info[8]);

    void update_telemetry();

private:

    constexpr static uint32_t PACKAGE_SIZE = 160;
    constexpr static uint32_t HEADER_SIZE = 4;
    constexpr static uint8_t HEADER_START_BYTE_VALUE = 0x9B;
    constexpr static uint8_t HEADER_PACKAGE_LENGTH_BYTE_VALUE = 158;
    constexpr static uint8_t HEADER_PACKAGE_PROTOCOL_BYTE_VALUE = 1;
    constexpr static uint8_t HEADER_PACKAGE_REAL_DATA_BYTE_VALUE = 2;

    struct HWESC decoded[8];
    HAL_Semaphore decoded_sem;
    uint32_t decoded_received_us;

    struct PACKED ESCData {
        uint8_t channel_number;
        uint16_t counter;

        // the following fields look very much like the ones in the XRotor-V4
        uint16_t throttle_req;
        uint16_t throttle;
        uint16_t rpm;
        uint16_t voltage;
        int16_t current;
        int16_t phase_current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
    };

    struct PACKED Packet {
        uint8_t header; // 0x9B
        uint8_t length;  // 160
        uint8_t communication_protocol_version;
        uint8_t command_word;
        uint16_t counter;
        ESCData esc_data[8];
        uint16_t crc;

        uint16_t calc_checksum() const {
            return crc_xmodem((uint8_t*)this, sizeof(*this)-2);
        };
    } packet;

    uint8_t header_first_chunk[HEADER_SIZE];
    uint8_t header_second_chunk[HEADER_SIZE];
    bool first_chunk_filled = false; //if first chunk filled
    int header_read = false;

    struct ESCInfo esc_info[8];  // FIXME: static_assert this to ARRAY_SIZE(packet.ESCData)

    AP_HAL::UARTDriver &uart;
    uint64_t last_frame_us;

    void check_seq(uint16_t this_seq);

    uint16_t last_seq;
    uint32_t lost_count;

    AP_Int8 offset;
    AP_Int8 motor_poles;

    //how many bytes readed
    int readHeader();

    void resetChunks();

};

#endif  // AP_HOBBYWING_DATALINK_ENABLED
