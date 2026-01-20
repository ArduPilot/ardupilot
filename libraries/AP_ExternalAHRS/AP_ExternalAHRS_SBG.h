/*
   Copyright (C) 2025 Kraus Hamdani Aerospace Inc. All rights reserved.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for serial connected SBG INS system
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SBG_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include "AP_ExternalAHRS_SBG_structs.h"

class AP_ExternalAHRS_SBG : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_SBG(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override { return (uart == nullptr) ? -1 : port_num; }

    // accessors for AP_AHRS
    bool healthy(void) const override { return last_received_ms > 0 && (AP_HAL::millis() - last_received_ms < 500); }
    bool initialised(void) const override { return setup_complete; };
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

    void get_filter_status(nav_filter_status &status) const override;

    // TODO: implement this
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override { return false; };

    // check for new data
    void update() override { }

    // Get model/type name
    const char* get_name() const override { return "SBG"; }

protected:

    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:

    static constexpr uint8_t SBG_PACKET_SYNC1 = 0xFF;
    static constexpr uint8_t SBG_PACKET_SYNC2 = 0x5A;
    static constexpr uint8_t SBG_PACKET_ETX = 0x33;
    static constexpr uint16_t SBG_PACKET_PAYLOAD_SIZE_MAX = 100; // real sbgCom limit is 4086 but the largest packet we parse is SbgLogEkfNavData=72 bytes
    static constexpr uint16_t SBG_PACKET_OVERHEAD = 9; // sync1, sync2, id, class, lenLSB, lenMSB, crcLSB, crcMSB, etx

    struct Cached {
        struct {
            AP_ExternalAHRS::gps_data_message_t gps_data;
            AP_ExternalAHRS::mag_data_message_t mag_data;
            AP_ExternalAHRS::baro_data_message_t baro_data;
            AP_ExternalAHRS::ins_data_message_t ins_data;
            AP_ExternalAHRS::airspeed_data_message_t airspeed_data;

            float baro_height;
            
            uint32_t gps_ms;
            uint32_t mag_ms;
            uint32_t baro_ms;
            uint32_t ins_ms;
            uint32_t airspeed_ms;
        } sensors;
    
        struct {
            SbgLogUtcData sbgLogUtcData;
            SbgLogGpsVel sbgLogGpsVel;
            SbgLogGpsPos sbgLogGpsPos;
            SbgLogImuData sbgLogImuData;
            SbgLogFastImuData sbgLogFastImuData;
            SbgEComLogImuShort sbgEComLogImuShort;
            SbgLogEkfEulerData sbgLogEkfEulerData;
            SbgLogEkfQuatData sbgLogEkfQuatData;
            SbgLogEkfNavData sbgLogEkfNavData;      // biggest msg we care about, 72 bytes
            SbgLogAirData sbgLogAirData;
            SbgLogMag sbgLogMag;
            SbgEComDeviceInfo sbgEComDeviceInfo;
       } sbg;
    } cached;

    struct PACKED sbgMessage {

        uint8_t msgid = 0;
        uint8_t msgclass = 0;
        uint16_t len = 0;
        uint8_t data[SBG_PACKET_PAYLOAD_SIZE_MAX];

        sbgMessage() {};

        sbgMessage(const uint8_t msgClass_, const uint8_t msgId_) {
            msgid = msgId_;
            msgclass = msgClass_;
        };

        sbgMessage(const uint8_t msgClass_, const uint8_t msgId_, const uint8_t* payload, const uint16_t payload_len) {
            msgid = msgId_;
            msgclass = msgClass_;

            if (payload_len > 0 && payload_len <= sizeof(data)) {
                memcpy(&data, payload, payload_len);
                len = payload_len;
            }
        };
    };

    enum class SBG_PACKET_PARSE_STATE : uint8_t {
        SYNC1,
        SYNC2,
        MSG,
        CLASS,
        LEN1,
        LEN2,
        DATA,
        CRC1,
        CRC2,
        ETX,
        DROP_THIS_PACKET
    };


    struct SBG_PACKET_INBOUND_STATE {
        SBG_PACKET_PARSE_STATE parser;
        uint16_t data_count;
        uint16_t crc;
        sbgMessage msg;
        uint16_t data_count_skip; // if we are parsing for a packet larger than we can accept, just stop parsing and wait for this many bytes to pass on by
    } _inbound_state;

    void handle_msg(const sbgMessage &msg);
    static bool parse_byte(const uint8_t data, sbgMessage &msg, SBG_PACKET_INBOUND_STATE &inbound_state);
    void update_thread();
    bool check_uart();
    static uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize);
    static bool send_sbgMessage(AP_HAL::UARTDriver *_uart, const sbgMessage &msg);
    static void safe_copy_msg_to_object(uint8_t* dest, const uint16_t dest_len, const uint8_t* src, const uint16_t src_len);
    static bool SbgEkfStatus_is_fullNav(const uint32_t ekfStatus);

    static AP_GPS_FixType SbgGpsPosStatus_to_GpsFixType(const uint32_t gpsPosStatus);

    uint32_t send_MagData_ms;
    uint32_t send_AirData_ms;
    uint32_t send_mag_error_last_ms;
    uint32_t send_air_error_last_ms;
    static bool send_MagData(AP_HAL::UARTDriver *_uart);
    static bool send_AirData(AP_HAL::UARTDriver *_uart);

    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;
    bool setup_complete;
    uint32_t version_check_ms;
    uint32_t last_received_ms;
};

#endif  // AP_EXTERNAL_AHRS_SBG_ENABLED

