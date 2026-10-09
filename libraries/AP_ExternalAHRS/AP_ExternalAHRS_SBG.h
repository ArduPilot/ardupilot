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
#include <AP_HAL/utility/RingBuffer.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <atomic>

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

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // check for new data. All device I/O runs in our own thread; on the main loop
    // we only pump any pending sbgInsRestApi reply back out over MAVLink.
    void update() override {
        service_rest_tx();
    }

    // Get model/type name
    const char* get_name() const override { return "SBG"; }

    // consume a MAVLink TUNNEL message forwarded from the ExternalAHRS frontend
    void handle_tunnel(const mavlink_channel_t chan, const mavlink_message_t &msg) override;

protected:

    uint8_t num_gps_sensors(void) const override {
        return 1;
    }

private:

    static constexpr uint8_t SBG_PACKET_SYNC1 = 0xFF;
    static constexpr uint8_t SBG_PACKET_SYNC2 = 0x5A;
    static constexpr uint8_t SBG_PACKET_ETX = 0x33;
    static constexpr uint16_t SBG_PACKET_PAYLOAD_SIZE_MAX = 100; // real sbgCom limit is 4086 but the largest packet we parse is SbgEComLogEkfNav=72 bytes
    static constexpr uint16_t SBG_PACKET_OVERHEAD = 9; // sync1, sync2, id, class, lenLSB, lenMSB, crcLSB, crcMSB, etx

    // sbgECom "large transfer" (paged) frames set the MSB of the message class and
    // prepend a 5-byte sub-header (txId(1) + pageIndex(2) + nrPages(2)) before the data.
    // Only used for API command payloads that exceed a single standard frame.
    static constexpr uint8_t SBG_PACKET_EXT_CLASS_BIT = 0x80;
    static constexpr uint8_t SBG_PACKET_EXT_HEADER_LEN = 5;

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
            SbgEComLogUtc utc;
            SbgEComLogGnssVel gnssVel;
            SbgEComLogGnssPos gnssPos;
            SbgEComLogImuLegacy imuLegacy;
            SbgEComLogImuFastLegacy imuFastLegacy;
            SbgEComLogImuShort imuShort;
            SbgEComLogEkfEuler ekfEuler;
            SbgEComLogEkfQuat ekfQuat;
            SbgEComLogEkfNav ekfNav;      // biggest msg we care about, 72 bytes
            SbgEComLogAirData airData;
            SbgEComLogMag mag;
            SbgEComDeviceInfo deviceInfo;
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
        // Optional oversized-frame buffer. When provided, frames too big for msg.data
        // are captured here as raw [id, class, len(2), data...] (so the struct-based CRC
        // still applies) instead of being dropped. The parser stays generic - it routes
        // purely by frame size and knows nothing about REST/API; the caller inspects the
        // returned frame. Stays nullptr unless the owner supplies a buffer.
        uint8_t *big_buf = nullptr;
        uint16_t big_buf_cap = 0;
    } _inbound_state;

    void handle_msg(const sbgMessage &msg);
    static bool parse_byte(const uint8_t data, sbgMessage &msg, SBG_PACKET_INBOUND_STATE &inbound_state);
    void update_thread();
    bool check_uart();
    static uint16_t calcCRC(const void *pBuffer, uint16_t bufferSize);
    static bool send_sbgMessage(AP_HAL::UARTDriver *_uart, const sbgMessage &msg);
    static void safe_copy_msg_to_object(uint8_t* dest, const uint16_t dest_len, const uint8_t* src, const uint16_t src_len);
    static uint16_t make_gps_week(const SbgEComLogUtc *utc_data);

    bool ekf_is_full_nav;
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

    // ==== sbgInsRestApi command passthrough over MAVLink TUNNEL ====
    // Everything below is self-contained to this backend; the ExternalAHRS
    // frontend and base class know nothing about it.

    // TUNNEL payload_type identifying an SBG REST request/reply. Chosen from the
    // "local experiment" range (>32767) so other TUNNEL users are left alone.
    static constexpr uint16_t REST_TUNNEL_PAYLOAD_TYPE = 0xD347;

    // TUNNEL app-layer segmentation: 8-byte little-endian header then data.
    //   [0] flags (REPLY/POST/ERROR/BUSY)  [1] session  [2..3] status(u16)
    //   [4..5] seq(u16)  [6..7] total(u16)
    static constexpr uint8_t REST_SEG_HDR   = 8;
    static constexpr uint8_t REST_SEG_DATA  = 128 - REST_SEG_HDR;  // 120 bytes/segment
    static constexpr uint8_t REST_FLAG_REPLY = 1U << 0;
    static constexpr uint8_t REST_FLAG_POST  = 1U << 1;
    static constexpr uint8_t REST_FLAG_ERROR = 1U << 2;
    static constexpr uint8_t REST_FLAG_BUSY  = 1U << 3;

    // max reassembled REST request (single sbgECom standard frame). Kept
    // <= 16*REST_SEG_DATA so the 16-bit reassembly mask covers every segment.
    static constexpr uint16_t REST_REQ_MAX = 1024;

    static constexpr uint16_t SBG_API_FRAME_MAX  = 4090;   // 4-byte [id,class,len] header + up to 4086 data
    static constexpr uint32_t SBG_API_REPLY_CAP  = 65535;  // safety ceiling; reply_buf is right-sized to the actual reply
    static constexpr uint32_t SBG_API_TIMEOUT_MS = 3000;

    // reassembled REST request, main thread (reassembly) -> SBG thread (transmit)
    struct rest_request_t {
        uint8_t  chan;
        uint8_t  target_system;
        uint8_t  target_component;
        uint8_t  session;
        bool     is_post;
        uint16_t len;
        uint8_t  data[REST_REQ_MAX];   // "path\0query\0[body\0]"
    };

    // finished reply, SBG thread -> main thread (pump). data is heap-owned.
    struct rest_reply_t {
        uint8_t  chan;
        uint8_t  target_system;
        uint8_t  target_component;
        uint8_t  session;
        uint16_t status;
        bool     error;
        uint16_t len;
        uint8_t *data;
    };

    // All REST RAM lives here and is allocated only when the first REST command
    // arrives (rest_alloc()). An SBG instance that never receives one pays just
    // the atomic pointer below.
    struct RestState {
        // main -> SBG handoff (own internal semaphore); cap 1 = one outstanding txn
        ObjectBuffer_TS<rest_request_t> req_queue{1};

        uint8_t  rx_frame[SBG_API_FRAME_MAX];                  // parser paged-frame capture
        uint8_t  tx_frame[SBG_PACKET_OVERHEAD + REST_REQ_MAX]; // command TX scratch (SBG thread)

        // reply reassembly (SBG thread). reply_buf is right-sized per reply and,
        // once the status prefix is stripped, becomes the buffer handed to the
        // pump - no second copy.
        uint8_t *reply_buf = nullptr;
        uint16_t reply_len = 0;
        uint16_t reply_cap = 0;
        uint16_t reply_pages_expected = 0;
        uint16_t reply_pages_got = 0;
        uint8_t  reply_txid = 0;         // sbgECom transfer id of a paged reply (page 0)

        HAL_Semaphore reply_sem;
        bool         reply_ready = false;
        rest_reply_t pending_reply {};   // finished reply awaiting the pump

        bool     api_cmd_active = false; // SBG-thread owned
        uint32_t api_cmd_sent_ms = 0;
        uint8_t  api_cmd_msgid = 0;      // command we sent; reply msgid must match
        uint8_t  api_cmd_session = 0;
        uint8_t  api_cmd_chan = 0;
        uint8_t  api_cmd_sysid = 0;
        uint8_t  api_cmd_compid = 0;

        // request reassembly (main thread; single outstanding transaction)
        struct {
            bool     active = false;
            uint8_t  sysid, compid, chan, session;
            bool     is_post;
            uint16_t total, mask, len;
            uint32_t last_ms;
            uint8_t  buf[REST_REQ_MAX];
        } rx;

        // outbound reply pump (main thread)
        rest_reply_t tx {};
        bool     tx_active = false;
        uint16_t tx_total = 0;
        uint16_t tx_seq = 0;

        // main-thread: true from accepting a request until its reply is fully sent.
        // Enforces one end-to-end transaction so a later completion can't overwrite an
        // earlier, still-unsent reply.
        bool     busy = false;
    };

    // nullptr until the first REST command. Written once by the main thread
    // (rest_alloc), read by the SBG thread; atomic release/acquire publishes the
    // fully-constructed RestState with no lock on the hot path.
    std::atomic<RestState*> rest {nullptr};
    RestState *rest_sbg;      // SBG-thread cached view of rest (acquired once)

    RestState *rest_alloc();  // main thread: lazily create RestState

    // Is this frame an sbgInsRestApi command reply (dispatch by id/class, any size)?
    static bool is_api_frame(const uint8_t *frame);

    // main thread
    void rest_send_status(uint8_t chan, uint8_t sysid, uint8_t compid,
                          uint8_t session, uint8_t flags, uint16_t status);
    void service_rest_tx();
    // SBG thread
    void service_rest(uint32_t now_ms);
    bool build_and_send_api_cmd(const rest_request_t &req);
    void handle_api_reply(const uint8_t *frame);
    void finish_api_reply(uint16_t status, bool error);
};

#endif  // AP_EXTERNAL_AHRS_SBG_ENABLED

