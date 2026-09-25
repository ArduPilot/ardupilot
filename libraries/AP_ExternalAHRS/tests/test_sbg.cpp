/*
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
  unit tests for the SBG ExternalAHRS driver
 */
#include <AP_gtest.h>

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_EXTERNAL_AHRS_SBG_ENABLED

#include <AP_ExternalAHRS/AP_ExternalAHRS_SBG.h>
#include <AP_Math/crc.h>

// test access wrapper, see friend declaration in AP_ExternalAHRS_SBG.h
class AP_ExternalAHRS_SBG_Test {
public:
    using sbgMessage = AP_ExternalAHRS_SBG::sbgMessage;
    using InboundState = AP_ExternalAHRS_SBG::SBG_PACKET_INBOUND_STATE;

    static constexpr uint16_t PAYLOAD_MAX = AP_ExternalAHRS_SBG::SBG_PACKET_PAYLOAD_SIZE_MAX;
    static constexpr uint16_t OVERHEAD = AP_ExternalAHRS_SBG::SBG_PACKET_OVERHEAD;

    static bool parse_byte(const uint8_t data, sbgMessage &msg, InboundState &state) {
        return AP_ExternalAHRS_SBG::parse_byte(data, msg, state);
    }
    static bool send(AP_HAL::UARTDriver *uart, const sbgMessage &msg) {
        return AP_ExternalAHRS_SBG::send_sbgMessage(uart, msg);
    }
    static uint16_t make_gps_week(const SbgEComLogUtc *utc) {
        return AP_ExternalAHRS_SBG::make_gps_week(utc);
    }
    static bool is_full_nav(const uint32_t ekfStatus) {
        return AP_ExternalAHRS_SBG::SbgEkfStatus_is_fullNav(ekfStatus);
    }
    static AP_GPS_FixType fix_type(const uint32_t gpsPosStatus) {
        return AP_ExternalAHRS_SBG::SbgGpsPosStatus_to_GpsFixType(gpsPosStatus);
    }
    static void safe_copy(uint8_t *dest, const uint16_t dest_len, const uint8_t *src, const uint16_t src_len) {
        AP_ExternalAHRS_SBG::safe_copy_msg_to_object(dest, dest_len, src, src_len);
    }
};

using SBG = AP_ExternalAHRS_SBG_Test;

// UART mock that captures everything written to it
class CaptureUART : public AP_HAL::UARTDriver {
public:
    uint8_t buf[512];
    uint32_t len;
    size_t write_limit = sizeof(buf); // lower this to simulate a short write

    bool is_initialized() override { return true; }
    bool tx_pending() override { return false; }
    uint32_t txspace() override { return sizeof(buf) - len; }

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {}
    size_t _write(const uint8_t *buffer, size_t size) override {
        const size_t n = MIN(size, write_limit - MIN((size_t)len, write_limit));
        memcpy(&buf[len], buffer, n);
        len += n;
        return n;
    }
    ssize_t _read(uint8_t *buffer, uint16_t count) override { return -1; }
    void _end() override {}
    void _flush() override {}
    uint32_t _available() override { return 0; }
    bool _discard_input() override { return true; }
};

// build a wire frame independently of the driver code:
// SYNC1 SYNC2 MSG CLASS LEN_LSB LEN_MSB DATA[len] CRC_LSB CRC_MSB ETX
// CRC is computed over the [MSG, CLASS, LEN, DATA] fields
static uint16_t make_frame(uint8_t *out, const uint8_t msgclass, const uint8_t msgid, const uint8_t *payload, const uint16_t len)
{
    out[0] = 0xFF;
    out[1] = 0x5A;
    out[2] = msgid;
    out[3] = msgclass;
    out[4] = len & 0xFF;
    out[5] = len >> 8;
    memcpy(&out[6], payload, len);
    const uint16_t crc = crc16_ccitt_r(&out[2], len + 4, 0, 0);
    out[6 + len] = crc & 0xFF;
    out[7 + len] = crc >> 8;
    out[8 + len] = 0x33;
    return len + 9;
}

// feed a buffer byte-by-byte, return how many complete packets were parsed
static uint32_t feed(const uint8_t *buf, const uint32_t len, SBG::sbgMessage &msg, SBG::InboundState &state)
{
    uint32_t count = 0;
    for (uint32_t i = 0; i < len; i++) {
        if (SBG::parse_byte(buf[i], msg, state)) {
            count++;
        }
    }
    return count;
}

// ---------------------------------------------------------------------------
// parse_byte
// ---------------------------------------------------------------------------

TEST(AP_ExternalAHRS_SBG, Parse_HappyPath)
{
    uint8_t payload[8];
    for (uint8_t i = 0; i < sizeof(payload); i++) {
        payload[i] = i + 1;
    }
    uint8_t frame[64];
    const uint16_t flen = make_frame(frame, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, payload, sizeof(payload));

    SBG::sbgMessage msg;
    SBG::InboundState state {};
    EXPECT_EQ(feed(frame, flen, msg, state), 1u);
    EXPECT_EQ(msg.msgclass, SBG_ECOM_CLASS_LOG_ECOM_0);
    EXPECT_EQ(msg.msgid, SBG_ECOM_LOG_UTC_TIME);
    EXPECT_EQ(msg.len, sizeof(payload));
    EXPECT_EQ(memcmp(msg.data, payload, sizeof(payload)), 0);
}

TEST(AP_ExternalAHRS_SBG, Parse_ZeroLengthPayload)
{
    // e.g. a SBG_ECOM_CMD_INFO request has no payload
    uint8_t frame[16];
    const uint16_t flen = make_frame(frame, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO, nullptr, 0);

    SBG::sbgMessage msg;
    SBG::InboundState state {};
    EXPECT_EQ(feed(frame, flen, msg, state), 1u);
    EXPECT_EQ(msg.msgclass, SBG_ECOM_CLASS_LOG_CMD_0);
    EXPECT_EQ(msg.msgid, SBG_ECOM_CMD_INFO);
    EXPECT_EQ(msg.len, 0);
}

TEST(AP_ExternalAHRS_SBG, Parse_RejectsBadCRC_ThenRecovers)
{
    uint8_t payload[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    uint8_t frame[32];
    const uint16_t flen = make_frame(frame, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, payload, sizeof(payload));

    SBG::sbgMessage msg;
    SBG::InboundState state {};

    // corrupt the CRC LSB
    frame[flen - 3] ^= 0xFF;
    EXPECT_EQ(feed(frame, flen, msg, state), 0u);

    // parser must accept a good frame afterwards
    frame[flen - 3] ^= 0xFF;
    EXPECT_EQ(feed(frame, flen, msg, state), 1u);
    EXPECT_EQ(msg.msgid, SBG_ECOM_LOG_MAG);
}

TEST(AP_ExternalAHRS_SBG, Parse_LeadingGarbageAndFalseSync)
{
    uint8_t payload[4] = { 1, 2, 3, 4 };
    uint8_t stream[64];
    uint32_t slen = 0;

    // noise, including a lone SYNC1 with no SYNC2 after it
    stream[slen++] = 0xEE;
    stream[slen++] = 0xFF;
    stream[slen++] = 0x00;
    stream[slen++] = 0x33;
    slen += make_frame(&stream[slen], SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, payload, sizeof(payload));

    SBG::sbgMessage msg;
    SBG::InboundState state {};
    EXPECT_EQ(feed(stream, slen, msg, state), 1u);
    EXPECT_EQ(msg.msgid, SBG_ECOM_LOG_EKF_QUAT);
}

TEST(AP_ExternalAHRS_SBG, Parse_SyncBytesInsidePayload)
{
    // payload containing the sync sequence and ETX must not confuse the parser
    uint8_t payload[6] = { 0xFF, 0x5A, 0x33, 0xFF, 0x5A, 0x33 };
    uint8_t frame[32];
    const uint16_t flen = make_frame(frame, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, payload, sizeof(payload));

    SBG::sbgMessage msg;
    SBG::InboundState state {};
    EXPECT_EQ(feed(frame, flen, msg, state), 1u);
    EXPECT_EQ(msg.len, sizeof(payload));
    EXPECT_EQ(memcmp(msg.data, payload, sizeof(payload)), 0);
}

TEST(AP_ExternalAHRS_SBG, Parse_BackToBackFrames)
{
    uint8_t payload1[3] = { 0x11, 0x22, 0x33 };
    uint8_t payload2[5] = { 0xA1, 0xA2, 0xA3, 0xA4, 0xA5 };
    uint8_t stream[64];
    uint32_t slen = 0;
    slen += make_frame(&stream[slen], SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, payload1, sizeof(payload1));
    slen += make_frame(&stream[slen], SBG_ECOM_CLASS_LOG_ECOM_1, SBG_ECOM_LOG_FAST_IMU_DATA, payload2, sizeof(payload2));

    SBG::sbgMessage msg;
    SBG::InboundState state {};
    EXPECT_EQ(feed(stream, slen, msg, state), 2u);
    // msg holds the last parsed packet
    EXPECT_EQ(msg.msgclass, SBG_ECOM_CLASS_LOG_ECOM_1);
    EXPECT_EQ(msg.msgid, SBG_ECOM_LOG_FAST_IMU_DATA);
    EXPECT_EQ(msg.len, sizeof(payload2));
    EXPECT_EQ(memcmp(msg.data, payload2, sizeof(payload2)), 0);
}

TEST(AP_ExternalAHRS_SBG, Parse_TruncatedFrame_EventualResync)
{
    uint8_t payload[4] = { 1, 2, 3, 4 };
    uint8_t good[32];
    const uint16_t glen = make_frame(good, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, payload, sizeof(payload));

    uint8_t stream[128];
    uint32_t slen = 0;
    // truncated frame: header claims 20 bytes of payload but the device stopped after 10
    uint8_t big_payload[20] = {};
    uint8_t truncated[40];
    make_frame(truncated, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, big_payload, sizeof(big_payload));
    memcpy(&stream[slen], truncated, 6 + 10);
    slen += 6 + 10;
    // two good frames follow: the first is eaten as payload of the truncated
    // frame, the parser must resync on the second at the latest
    memcpy(&stream[slen], good, glen);
    slen += glen;
    memcpy(&stream[slen], good, glen);
    slen += glen;

    SBG::sbgMessage msg;
    SBG::InboundState state {};
    EXPECT_GE(feed(stream, slen, msg, state), 1u);
    EXPECT_EQ(msg.msgid, SBG_ECOM_LOG_MAG);
}

TEST(AP_ExternalAHRS_SBG, Parse_OversizedPacketSkipped_ThenRecovers)
{
    // a frame larger than the driver's rx buffer must be skipped byte-by-byte
    const uint16_t big_len = 3000;
    uint8_t stream[4096];
    uint32_t slen = 0;
    stream[slen++] = 0xFF;
    stream[slen++] = 0x5A;
    stream[slen++] = SBG_ECOM_LOG_GPS1_RAW;
    stream[slen++] = SBG_ECOM_CLASS_LOG_ECOM_0;
    stream[slen++] = big_len & 0xFF;
    stream[slen++] = big_len >> 8;
    memset(&stream[slen], 0x77, big_len);
    slen += big_len;
    // CRC + ETX of the oversized frame (values irrelevant, they are skipped in SYNC1)
    stream[slen++] = 0x00;
    stream[slen++] = 0x00;
    stream[slen++] = 0x33;

    uint8_t payload[4] = { 9, 8, 7, 6 };
    slen += make_frame(&stream[slen], SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_AIR_DATA, payload, sizeof(payload));

    SBG::sbgMessage msg;
    SBG::InboundState state {};
    EXPECT_EQ(feed(stream, slen, msg, state), 1u);
    EXPECT_EQ(msg.msgid, SBG_ECOM_LOG_AIR_DATA);
    EXPECT_EQ(memcmp(msg.data, payload, sizeof(payload)), 0);
}

TEST(AP_ExternalAHRS_SBG, Parse_PayloadSize99)
{
    uint8_t payload[99];
    for (uint16_t i = 0; i < sizeof(payload); i++) {
        payload[i] = i;
    }
    uint8_t frame[128];
    const uint16_t flen = make_frame(frame, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, payload, sizeof(payload));

    SBG::sbgMessage msg;
    SBG::InboundState state {};
    EXPECT_EQ(feed(frame, flen, msg, state), 1u);
    EXPECT_EQ(msg.len, sizeof(payload));
    EXPECT_EQ(memcmp(msg.data, payload, sizeof(payload)), 0);
}

TEST(AP_ExternalAHRS_SBG, Parse_PayloadSizeMax)
{
    // a payload of exactly SBG_PACKET_PAYLOAD_SIZE_MAX must be accepted, not
    // dropped (regression: the DATA state used to reset on data_count reaching
    // sizeof(msg.data) before the msg.len completion check)
    uint8_t payload[SBG::PAYLOAD_MAX];
    for (uint16_t i = 0; i < sizeof(payload); i++) {
        payload[i] = i;
    }
    uint8_t frame[128];
    const uint16_t flen = make_frame(frame, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, payload, sizeof(payload));

    SBG::sbgMessage msg;
    SBG::InboundState state {};
    EXPECT_EQ(feed(frame, flen, msg, state), 1u);
    EXPECT_EQ(msg.len, sizeof(payload));
    EXPECT_EQ(memcmp(msg.data, payload, sizeof(payload)), 0);

    // and the parser must be left clean enough to accept the next frame
    uint8_t next[4] = { 1, 2, 3, 4 };
    uint8_t nframe[32];
    const uint16_t nlen = make_frame(nframe, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, next, sizeof(next));
    EXPECT_EQ(feed(nframe, nlen, msg, state), 1u);
    EXPECT_EQ(msg.msgid, SBG_ECOM_LOG_MAG);
}

// ---------------------------------------------------------------------------
// send_sbgMessage
// ---------------------------------------------------------------------------

TEST(AP_ExternalAHRS_SBG, Send_WireFormat)
{
    SbgEComLogAirData air {};
    air.timeStamp = 0;
    air.status = SBG_ECOM_AIR_DATA_TIME_IS_DELAY | SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID;
    air.pressureAbs = 101325.0f;
    air.altitude = 123.5f;

    const SBG::sbgMessage msg(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_AIR_DATA, (const uint8_t*)&air, sizeof(air));

    CaptureUART uart {};
    EXPECT_TRUE(SBG::send(&uart, msg));
    ASSERT_EQ(uart.len, sizeof(air) + SBG::OVERHEAD);

    EXPECT_EQ(uart.buf[0], 0xFF);
    EXPECT_EQ(uart.buf[1], 0x5A);
    EXPECT_EQ(uart.buf[2], SBG_ECOM_LOG_AIR_DATA);
    EXPECT_EQ(uart.buf[3], SBG_ECOM_CLASS_LOG_ECOM_0);
    EXPECT_EQ(uart.buf[4], sizeof(air) & 0xFF);
    EXPECT_EQ(uart.buf[5], sizeof(air) >> 8);
    EXPECT_EQ(memcmp(&uart.buf[6], &air, sizeof(air)), 0);

    const uint16_t crc = crc16_ccitt_r(&uart.buf[2], sizeof(air) + 4, 0, 0);
    EXPECT_EQ(uart.buf[uart.len - 3], crc & 0xFF);
    EXPECT_EQ(uart.buf[uart.len - 2], crc >> 8);
    EXPECT_EQ(uart.buf[uart.len - 1], 0x33);
}

TEST(AP_ExternalAHRS_SBG, Send_Parse_RoundTrip)
{
    SbgEComLogEkfNav nav {};
    nav.timeStamp = 123456;
    nav.velocity[0] = 1.5f;
    nav.velocity[1] = -2.5f;
    nav.velocity[2] = 0.25f;
    nav.position[0] = 48.8566;
    nav.position[1] = 2.3522;
    nav.position[2] = 35.0;
    nav.status = SBG_ECOM_SOL_MODE_NAV_POSITION;

    const SBG::sbgMessage out(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, (const uint8_t*)&nav, sizeof(nav));

    CaptureUART uart {};
    EXPECT_TRUE(SBG::send(&uart, out));

    SBG::sbgMessage in;
    SBG::InboundState state {};
    EXPECT_EQ(feed(uart.buf, uart.len, in, state), 1u);
    EXPECT_EQ(in.msgclass, SBG_ECOM_CLASS_LOG_ECOM_0);
    EXPECT_EQ(in.msgid, SBG_ECOM_LOG_EKF_NAV);
    ASSERT_EQ(in.len, sizeof(nav));

    SbgEComLogEkfNav nav_in;
    memcpy(&nav_in, in.data, sizeof(nav_in));
    EXPECT_EQ(nav_in.timeStamp, nav.timeStamp);
    EXPECT_FLOAT_EQ(nav_in.velocity[0], nav.velocity[0]);
    EXPECT_FLOAT_EQ(nav_in.velocity[1], nav.velocity[1]);
    EXPECT_FLOAT_EQ(nav_in.velocity[2], nav.velocity[2]);
    EXPECT_DOUBLE_EQ(nav_in.position[0], nav.position[0]);
    EXPECT_DOUBLE_EQ(nav_in.position[1], nav.position[1]);
    EXPECT_DOUBLE_EQ(nav_in.position[2], nav.position[2]);
    EXPECT_EQ(nav_in.status, nav.status);
}

TEST(AP_ExternalAHRS_SBG, Send_NullUart)
{
    const SBG::sbgMessage msg(SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_INFO);
    EXPECT_FALSE(SBG::send(nullptr, msg));
}

TEST(AP_ExternalAHRS_SBG, Send_OversizedLenRejected)
{
    SBG::sbgMessage msg(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV);
    msg.len = SBG::PAYLOAD_MAX + 1;

    CaptureUART uart {};
    EXPECT_FALSE(SBG::send(&uart, msg));
    EXPECT_EQ(uart.len, 0u);
}

TEST(AP_ExternalAHRS_SBG, Send_ShortWriteReturnsFalse)
{
    uint8_t payload[16] = {};
    const SBG::sbgMessage msg(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, payload, sizeof(payload));

    CaptureUART uart {};
    uart.write_limit = 10; // uart accepts fewer bytes than the frame size
    EXPECT_FALSE(SBG::send(&uart, msg));
}

TEST(AP_ExternalAHRS_SBG, Message_PayloadConstructorRejectsOversized)
{
    uint8_t payload[SBG::PAYLOAD_MAX + 1] = {};
    const SBG::sbgMessage msg(SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, payload, sizeof(payload));
    EXPECT_EQ(msg.len, 0);
}

// ---------------------------------------------------------------------------
// make_gps_week
// ---------------------------------------------------------------------------

static SbgEComLogUtc make_utc(uint16_t year, int8_t month, int8_t day, int8_t hour, uint32_t tow_ms)
{
    SbgEComLogUtc utc {};
    utc.year = year;
    utc.month = month;
    utc.day = day;
    utc.hour = hour;
    utc.gpsTimeOfWeek = tow_ms;
    return utc;
}

TEST(AP_ExternalAHRS_SBG, MakeGpsWeek_KnownDates)
{
    // 2024-02-29 (Thursday) 12:00 UTC, tow = 4.5 days -> GPS week 2303
    SbgEComLogUtc utc = make_utc(2024, 2, 29, 12, (4 * 24 + 12) * 3600UL * 1000UL);
    EXPECT_EQ(SBG::make_gps_week(&utc), 2303);

    // 2025-12-10 (Wednesday) 12:00 UTC, tow = 3.5 days -> GPS week 2396
    utc = make_utc(2025, 12, 10, 12, (3 * 24 + 12) * 3600UL * 1000UL);
    EXPECT_EQ(SBG::make_gps_week(&utc), 2396);

    // 1980-01-09 (Wednesday) 12:00 UTC, first GPS week -> week 0
    utc = make_utc(1980, 1, 9, 12, (3 * 24 + 12) * 3600UL * 1000UL);
    EXPECT_EQ(SBG::make_gps_week(&utc), 0);
}

// ---------------------------------------------------------------------------
// SbgEkfStatus_is_fullNav
// ---------------------------------------------------------------------------

TEST(AP_ExternalAHRS_SBG, EkfStatus_FullNav)
{
    EXPECT_FALSE(SBG::is_full_nav(SBG_ECOM_SOL_MODE_UNINITIALIZED));
    EXPECT_FALSE(SBG::is_full_nav(SBG_ECOM_SOL_MODE_VERTICAL_GYRO));
    EXPECT_FALSE(SBG::is_full_nav(SBG_ECOM_SOL_MODE_AHRS));
    EXPECT_FALSE(SBG::is_full_nav(SBG_ECOM_SOL_MODE_NAV_VELOCITY));
    EXPECT_TRUE(SBG::is_full_nav(SBG_ECOM_SOL_MODE_NAV_POSITION));

    // status bits above the solution mode mask must be ignored
    EXPECT_TRUE(SBG::is_full_nav(SBG_ECOM_SOL_MODE_NAV_POSITION | 0xFFFFFFF0));
    EXPECT_FALSE(SBG::is_full_nav(SBG_ECOM_SOL_MODE_NAV_VELOCITY | 0xFFFFFFF0));
}

// ---------------------------------------------------------------------------
// SbgGpsPosStatus_to_GpsFixType
// ---------------------------------------------------------------------------

static uint32_t gps_pos_status(const SbgEComGpsPosType type)
{
    return ((uint32_t)type & SBG_ECOM_GPS_POS_TYPE_MASK) << SBG_ECOM_GPS_POS_TYPE_SHIFT;
}

TEST(AP_ExternalAHRS_SBG, GpsPosStatus_to_FixType)
{
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_NO_SOLUTION)), AP_GPS_FixType::NONE);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_UNKNOWN_TYPE)), AP_GPS_FixType::NONE);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_SINGLE)), AP_GPS_FixType::FIX_3D);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_FIXED)), AP_GPS_FixType::FIX_3D);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_PSRDIFF)), AP_GPS_FixType::DGPS);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_SBAS)), AP_GPS_FixType::DGPS);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_RTK_FLOAT)), AP_GPS_FixType::RTK_FLOAT);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_PPP_FLOAT)), AP_GPS_FixType::RTK_FLOAT);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_OMNISTAR)), AP_GPS_FixType::RTK_FLOAT);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_RTK_INT)), AP_GPS_FixType::RTK_FIXED);
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_PPP_INT)), AP_GPS_FixType::RTK_FIXED);

    // status bits outside the position type field must be ignored
    EXPECT_EQ(SBG::fix_type(gps_pos_status(SBG_ECOM_POS_SINGLE) | 0x3F), AP_GPS_FixType::FIX_3D);
}

// ---------------------------------------------------------------------------
// safe_copy_msg_to_object
// ---------------------------------------------------------------------------

TEST(AP_ExternalAHRS_SBG, SafeCopy_EqualLength)
{
    uint8_t src[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8_t dest[8];
    memset(dest, 0xAA, sizeof(dest));

    SBG::safe_copy(dest, sizeof(dest), src, sizeof(src));
    EXPECT_EQ(memcmp(dest, src, sizeof(src)), 0);
}

TEST(AP_ExternalAHRS_SBG, SafeCopy_ShorterSource_ZeroFills)
{
    // shorter message than the struct: unsent tail must read as zero
    uint8_t src[4] = { 1, 2, 3, 4 };
    uint8_t dest[8];
    memset(dest, 0xAA, sizeof(dest));

    SBG::safe_copy(dest, sizeof(dest), src, sizeof(src));
    EXPECT_EQ(memcmp(dest, src, sizeof(src)), 0);
    for (uint16_t i = sizeof(src); i < sizeof(dest); i++) {
        EXPECT_EQ(dest[i], 0) << "byte " << i << " not zeroed";
    }
}

TEST(AP_ExternalAHRS_SBG, SafeCopy_LongerSource_Truncates)
{
    // extended (future protocol) message: only dest_len bytes are used
    uint8_t src[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    uint8_t dest[5] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }; // sentinel guards the byte after the real dest
    const uint16_t dest_len = 4;

    SBG::safe_copy(dest, dest_len, src, sizeof(src));
    EXPECT_EQ(memcmp(dest, src, dest_len), 0);
    EXPECT_EQ(dest[4], 0xAA);
}

#endif  // AP_EXTERNAL_AHRS_SBG_ENABLED

AP_GTEST_MAIN()
