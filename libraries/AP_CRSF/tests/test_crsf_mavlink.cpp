/*
 * Unit tests for AP_CRSF_MAVLink chunk reassembly and TX chunking.
 *
 * These tests exercise the protocol-level logic without requiring a
 * running GCS or scheduler.  The friend declaration in AP_CRSF_MAVLink
 * allows direct access to internal state for test setup and verification.
 */
#include <AP_gtest.h>

#include <AP_CRSF/AP_CRSF_MAVLink.h>
#include <AP_CRSF/AP_CRSF_Protocol.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// ---------------------------------------------------------------------------
// Test fixture — bypasses init() (which needs GCS) and sets up the object
// for direct reassembly / chunking tests.
//
// gtest generates subclasses of the fixture for each TEST_F, so private
// member access must go through helpers defined here (the actual friend).
// ---------------------------------------------------------------------------
class TestCRSFMAVLink : public ::testing::Test
{
protected:
    AP_CRSF_MAVLink mavlink;

    void SetUp() override
    {
        mavlink._reassembly.reset();
        mavlink._tx_state.reset();
        mavlink._initialized = true;
        mavlink._uart.begin(0);
    }

    // --- helpers that need friend access ---

    uint32_t uart_available()
    {
        return mavlink._uart.available();
    }

    uint32_t uart_read(uint8_t *buf, uint16_t count)
    {
        ssize_t n = mavlink._uart.read(buf, count);
        return (n > 0) ? static_cast<uint32_t>(n) : 0;
    }

    size_t uart_write(const uint8_t *buf, size_t len)
    {
        return mavlink._uart.write(buf, len);
    }

    bool reassembly_active()
    {
        return mavlink._reassembly.active;
    }

    // --- general helpers ---

    // Encodes a CRSF MAVLink envelope chunk. Per the spec the high nibble on
    // the wire is the zero-based index of the last chunk (count - 1), so we
    // take a 1-based chunk_count for test ergonomics and encode last-index.
    static void build_chunk(uint8_t *payload, uint8_t &len,
                            uint8_t chunk_count, uint8_t current_chunk,
                            const uint8_t *data, uint8_t data_size)
    {
        const uint8_t last_chunk = chunk_count - 1;
        payload[0] = ((last_chunk & 0x0F) << 4) | (current_chunk & 0x0F);
        payload[1] = data_size;
        memcpy(&payload[2], data, data_size);
        len = data_size + 2;
    }

    uint32_t read_all_rx(uint8_t *buf, uint32_t max_len)
    {
        uint32_t avail = uart_available();
        uint32_t to_read = (avail < max_len) ? avail : max_len;
        if (to_read == 0) {
            return 0;
        }
        return uart_read(buf, to_read);
    }

    // construct a minimal MAVLink v1 frame with the given payload size.
    // payload bytes are filled with an increasing pattern; CRC bytes are zero
    // (we don't validate CRCs in these tests). returns total wire size.
    static uint16_t make_v1(uint8_t *buf, uint8_t payload_len, uint8_t fill_seed = 0)
    {
        buf[0] = 0xFE;
        buf[1] = payload_len;
        buf[2] = 0;  // seq
        buf[3] = 1;  // sysid
        buf[4] = 1;  // compid
        buf[5] = 0;  // msgid (HEARTBEAT)
        for (uint8_t i = 0; i < payload_len; i++) {
            buf[6 + i] = fill_seed + i;
        }
        buf[6 + payload_len] = 0;
        buf[7 + payload_len] = 0;
        return 8 + payload_len;
    }

    // construct a minimal unsigned MAVLink v2 frame with the given payload size.
    static uint16_t make_v2(uint8_t *buf, uint16_t payload_len, uint8_t fill_seed = 0)
    {
        buf[0] = 0xFD;
        buf[1] = payload_len & 0xFF;
        buf[2] = 0;  // incompat_flags (no signature)
        buf[3] = 0;  // compat_flags
        buf[4] = 0;  // seq
        buf[5] = 1;  // sysid
        buf[6] = 1;  // compid
        buf[7] = 0; buf[8] = 0; buf[9] = 0;  // msgid
        for (uint16_t i = 0; i < payload_len; i++) {
            buf[10 + i] = fill_seed + (i & 0xFF);
        }
        buf[10 + payload_len] = 0;
        buf[11 + payload_len] = 0;
        return 12 + payload_len;
    }
};

// ===========================================================================
// MavlinkEnvelopeFrame struct parsing
// ===========================================================================

TEST(CRSFProtocol, MavlinkEnvelopeChunkInfoParsing)
{
    // chunk_info nibbles are zero-based indexes per the CRSF spec:
    //   high nibble = last chunk index (count - 1)
    //   low nibble  = current chunk index
    AP_CRSF_Protocol::MavlinkEnvelopeFrame frame {};
    // 4 chunks total, this is chunk 0 -> 0x30
    frame.chunk_info = 0x30;
    EXPECT_EQ(frame.total_chunks(), 3);
    EXPECT_EQ(frame.current_chunk(), 0);

    // 16 chunks total, this is chunk 7 -> 0xF7
    frame.chunk_info = 0xF7;
    EXPECT_EQ(frame.total_chunks(), 15);
    EXPECT_EQ(frame.current_chunk(), 7);

    // single-chunk message -> 0x00
    frame.chunk_info = 0x00;
    EXPECT_EQ(frame.total_chunks(), 0);
    EXPECT_EQ(frame.current_chunk(), 0);
}

// ===========================================================================
// RX reassembly
// ===========================================================================

TEST_F(TestCRSFMAVLink, SingleChunkReassembly)
{
    uint8_t data[10];
    for (uint8_t i = 0; i < sizeof(data); i++) {
        data[i] = i + 0xA0;
    }

    uint8_t payload[62];
    uint8_t len;
    build_chunk(payload, len, 1, 0, data, sizeof(data));

    mavlink.process_frame(payload, len);

    uint8_t rx[10];
    uint32_t n = read_all_rx(rx, sizeof(rx));
    EXPECT_EQ(n, sizeof(data));
    EXPECT_EQ(memcmp(data, rx, sizeof(data)), 0);
}

TEST_F(TestCRSFMAVLink, MultiChunkReassembly)
{
    // 100 bytes split across 2 chunks: 58 + 42
    uint8_t data[100];
    for (uint8_t i = 0; i < sizeof(data); i++) {
        data[i] = i;
    }

    uint8_t payload[62];
    uint8_t len;

    build_chunk(payload, len, 2, 0, &data[0], 58);
    mavlink.process_frame(payload, len);
    EXPECT_EQ(uart_available(), 0u);

    build_chunk(payload, len, 2, 1, &data[58], 42);
    mavlink.process_frame(payload, len);

    uint8_t rx[100];
    uint32_t n = read_all_rx(rx, sizeof(rx));
    EXPECT_EQ(n, sizeof(data));
    EXPECT_EQ(memcmp(data, rx, sizeof(data)), 0);
}

TEST_F(TestCRSFMAVLink, ThreeChunkReassembly)
{
    // 150 bytes across 3 chunks: 58 + 58 + 34
    uint8_t data[150];
    for (uint16_t i = 0; i < sizeof(data); i++) {
        data[i] = static_cast<uint8_t>(i & 0xFF);
    }

    uint8_t payload[62];
    uint8_t len;

    build_chunk(payload, len, 3, 0, &data[0], 58);
    mavlink.process_frame(payload, len);
    EXPECT_EQ(uart_available(), 0u);

    build_chunk(payload, len, 3, 1, &data[58], 58);
    mavlink.process_frame(payload, len);
    EXPECT_EQ(uart_available(), 0u);

    build_chunk(payload, len, 3, 2, &data[116], 34);
    mavlink.process_frame(payload, len);

    uint8_t rx[150];
    uint32_t n = read_all_rx(rx, sizeof(rx));
    EXPECT_EQ(n, sizeof(data));
    EXPECT_EQ(memcmp(data, rx, sizeof(data)), 0);
}

TEST_F(TestCRSFMAVLink, OutOfOrderChunkRejected)
{
    uint8_t data[20];
    memset(data, 0xAB, sizeof(data));

    uint8_t payload[62];
    uint8_t len;

    // send chunk 1 first (should be chunk 0)
    build_chunk(payload, len, 2, 1, data, 20);
    mavlink.process_frame(payload, len);

    EXPECT_EQ(uart_available(), 0u);
    EXPECT_FALSE(reassembly_active());
}

TEST_F(TestCRSFMAVLink, MismatchedTotalChunksRejected)
{
    uint8_t data[10];
    memset(data, 0xCC, sizeof(data));

    uint8_t payload[62];
    uint8_t len;

    // chunk 0 says total=3
    build_chunk(payload, len, 3, 0, data, 10);
    mavlink.process_frame(payload, len);
    EXPECT_TRUE(reassembly_active());

    // chunk 1 says total=2 (mismatch)
    build_chunk(payload, len, 2, 1, data, 10);
    mavlink.process_frame(payload, len);

    EXPECT_FALSE(reassembly_active());
    EXPECT_EQ(uart_available(), 0u);
}

TEST_F(TestCRSFMAVLink, ChunkCurrentExceedsTotalRejected)
{
    uint8_t data[10];
    memset(data, 0xEE, sizeof(data));

    uint8_t payload[62];
    uint8_t len;

    // current_chunk (2) >= total_chunks (2) is invalid
    build_chunk(payload, len, 2, 2, data, 10);
    mavlink.process_frame(payload, len);

    EXPECT_EQ(uart_available(), 0u);
}

TEST_F(TestCRSFMAVLink, OversizedDataRejected)
{
    // data_size claims 59 but max is 58
    uint8_t payload[64];
    payload[0] = 0x10;  // total=1, current=0
    payload[1] = 59;    // data_size > MAX_CHUNK_DATA
    memset(&payload[2], 0xFF, 59);

    mavlink.process_frame(payload, 61);

    EXPECT_EQ(uart_available(), 0u);
}

TEST_F(TestCRSFMAVLink, PayloadTooShortRejected)
{
    // frame claims data_size=10 but payload is only 5 bytes total
    uint8_t payload[5];
    payload[0] = 0x10;  // total=1, current=0
    payload[1] = 10;    // claims 10 data bytes
    memset(&payload[2], 0xAA, 3);

    mavlink.process_frame(payload, 5);

    EXPECT_EQ(uart_available(), 0u);
}

TEST_F(TestCRSFMAVLink, NewMessageResetsStaleReassembly)
{
    uint8_t data_a[10];
    memset(data_a, 0xAA, sizeof(data_a));
    uint8_t data_b[20];
    memset(data_b, 0xBB, sizeof(data_b));

    uint8_t payload[62];
    uint8_t len;

    // start a 2-chunk message
    build_chunk(payload, len, 2, 0, data_a, 10);
    mavlink.process_frame(payload, len);
    EXPECT_TRUE(reassembly_active());

    // new message starts (chunk 0 of a different message) before old completes
    build_chunk(payload, len, 1, 0, data_b, 20);
    mavlink.process_frame(payload, len);

    // should get the new single-chunk message
    uint8_t rx[20];
    uint32_t n = read_all_rx(rx, sizeof(rx));
    EXPECT_EQ(n, 20u);
    EXPECT_EQ(memcmp(data_b, rx, 20), 0);
}

TEST_F(TestCRSFMAVLink, FrameTooShortIgnored)
{
    uint8_t payload[1] = {0x10};
    mavlink.process_frame(payload, 1);
    EXPECT_EQ(uart_available(), 0u);
}

// ===========================================================================
// TX chunking
// ===========================================================================

TEST_F(TestCRSFMAVLink, TxSingleChunk)
{
    // 20-byte MAVLink v1 frame: 8 header/crc + 12 payload
    uint8_t msg[20];
    const uint16_t msg_size = make_v1(msg, 12, 0x10);
    EXPECT_EQ(msg_size, sizeof(msg));
    uart_write(msg, msg_size);

    EXPECT_TRUE(mavlink.tx_pending());

    uint8_t payload[62];
    uint8_t len = 0;
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));

    // single-chunk messages use chunk_info == 0x00 to match the inbound convention
    EXPECT_EQ(payload[0], 0x00);
    EXPECT_EQ(payload[1], msg_size);
    EXPECT_EQ(len, msg_size + 2);
    EXPECT_EQ(memcmp(&payload[2], msg, msg_size), 0);

    EXPECT_FALSE(mavlink.tx_pending());
    EXPECT_FALSE(mavlink.get_telem_frame(payload, len));
}

TEST_F(TestCRSFMAVLink, TxMultipleChunks)
{
    // 100-byte MAVLink v2 frame: 12 header/crc + 88 payload → 2 chunks of 58+42
    uint8_t msg[100];
    const uint16_t msg_size = make_v2(msg, 88, 0x00);
    EXPECT_EQ(msg_size, sizeof(msg));
    uart_write(msg, msg_size);

    // chunk 0 — per spec: last-chunk-index = 1 for a 2-chunk message
    uint8_t payload[62];
    uint8_t len = 0;
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));
    EXPECT_EQ((payload[0] >> 4) & 0x0F, 1);
    EXPECT_EQ(payload[0] & 0x0F, 0);
    EXPECT_EQ(payload[1], 58);
    EXPECT_EQ(len, 60);
    EXPECT_EQ(memcmp(&payload[2], &msg[0], 58), 0);

    // chunk 1
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));
    EXPECT_EQ((payload[0] >> 4) & 0x0F, 1);
    EXPECT_EQ(payload[0] & 0x0F, 1);
    EXPECT_EQ(payload[1], 42);
    EXPECT_EQ(len, 44);
    EXPECT_EQ(memcmp(&payload[2], &msg[58], 42), 0);

    EXPECT_FALSE(mavlink.tx_pending());
}

TEST_F(TestCRSFMAVLink, TxExactlyOneChunkBoundary)
{
    // exactly 58 bytes — one full chunk: v1 frame with 50 payload
    uint8_t msg[58];
    const uint16_t msg_size = make_v1(msg, 50, 0x42);
    EXPECT_EQ(msg_size, sizeof(msg));
    uart_write(msg, msg_size);

    uint8_t payload[62];
    uint8_t len = 0;
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));
    EXPECT_EQ(payload[0], 0x00);
    EXPECT_EQ(payload[1], 58);
    EXPECT_EQ(memcmp(&payload[2], msg, 58), 0);

    EXPECT_FALSE(mavlink.get_telem_frame(payload, len));
}

TEST_F(TestCRSFMAVLink, TxEmitsOneMessagePerEnvelope)
{
    // two back-to-back MAVLink messages in the TX buffer must be emitted as
    // two separate envelopes (not concatenated into a multi-chunk envelope).
    uint8_t msg_a[21];  // v2 HEARTBEAT-sized
    uint8_t msg_b[43];  // v1 SYS_STATUS-sized
    const uint16_t a_size = make_v2(msg_a, 9, 0xA0);
    const uint16_t b_size = make_v1(msg_b, 35, 0xB0);
    EXPECT_EQ(a_size, sizeof(msg_a));
    EXPECT_EQ(b_size, sizeof(msg_b));
    uart_write(msg_a, a_size);
    uart_write(msg_b, b_size);

    uint8_t payload[62];
    uint8_t len = 0;

    // first envelope: msg_a, single chunk
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));
    EXPECT_EQ(payload[0], 0x00);
    EXPECT_EQ(payload[1], a_size);
    EXPECT_EQ(memcmp(&payload[2], msg_a, a_size), 0);

    // second envelope: msg_b, single chunk — not concatenated with msg_a
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));
    EXPECT_EQ(payload[0], 0x00);
    EXPECT_EQ(payload[1], b_size);
    EXPECT_EQ(memcmp(&payload[2], msg_b, b_size), 0);

    EXPECT_FALSE(mavlink.tx_pending());
}

TEST_F(TestCRSFMAVLink, TxResyncsOnGarbage)
{
    // junk bytes in the TX buffer must not stall; they should be discarded
    // until a valid STX is found.
    uint8_t junk[] = {0x00, 0x11, 0x22, 0x33};
    uart_write(junk, sizeof(junk));

    uint8_t msg[21];
    const uint16_t msg_size = make_v2(msg, 9, 0xC0);
    uart_write(msg, msg_size);

    uint8_t payload[62];
    uint8_t len = 0;

    // repeatedly pump: junk should be drained, then the real message emitted
    for (int i = 0; i < 10 && mavlink.tx_pending(); i++) {
        if (mavlink.get_telem_frame(payload, len)) {
            EXPECT_EQ(payload[0], 0x00);
            EXPECT_EQ(payload[1], msg_size);
            EXPECT_EQ(memcmp(&payload[2], msg, msg_size), 0);
            return;
        }
    }
    FAIL() << "did not resync to valid MAVLink frame";
}

TEST_F(TestCRSFMAVLink, TxEmptyProducesFalse)
{
    uint8_t payload[62];
    uint8_t len = 0;
    EXPECT_FALSE(mavlink.tx_pending());
    EXPECT_FALSE(mavlink.get_telem_frame(payload, len));
}

// ===========================================================================
// Round-trip: RX reassembly → TX chunking produces identical data
// ===========================================================================

TEST_F(TestCRSFMAVLink, RoundTripSmallMessage)
{
    // 30-byte MAVLink v1 frame: 8 header/crc + 22 payload
    uint8_t original[30];
    const uint16_t original_size = make_v1(original, 22, 0x50);
    EXPECT_EQ(original_size, sizeof(original));

    // RX: inject as a single 0xAA chunk
    uint8_t payload[62];
    uint8_t len;
    build_chunk(payload, len, 1, 0, original, original_size);
    mavlink.process_frame(payload, len);

    // read from virtual UART
    uint8_t rx[30];
    uint32_t n = read_all_rx(rx, sizeof(rx));
    EXPECT_EQ(n, original_size);
    EXPECT_EQ(memcmp(original, rx, original_size), 0);

    // write back as a response
    uart_write(rx, n);

    // TX: drain and reassemble
    uint8_t reassembled[280];
    uint16_t reassembled_len = 0;
    while (mavlink.tx_pending()) {
        uint8_t tx_payload[62];
        uint8_t tx_len = 0;
        if (mavlink.get_telem_frame(tx_payload, tx_len)) {
            uint8_t data_size = tx_payload[1];
            memcpy(&reassembled[reassembled_len], &tx_payload[2], data_size);
            reassembled_len += data_size;
        }
    }

    EXPECT_EQ(reassembled_len, original_size);
    EXPECT_EQ(memcmp(original, reassembled, original_size), 0);
}

TEST_F(TestCRSFMAVLink, RoundTripLargeMessage)
{
    // MAVLink v2 max unsigned frame: 10 header + 255 payload + 2 CRC = 267 bytes
    uint8_t original[267];
    const uint16_t original_size = make_v2(original, 255, 0x00);
    EXPECT_EQ(original_size, sizeof(original));

    // RX: chunk into 58-byte pieces
    uint8_t total_chunks = (original_size + 57) / 58;  // = 5
    for (uint8_t c = 0; c < total_chunks; c++) {
        uint16_t offset = static_cast<uint16_t>(c) * 58;
        uint8_t chunk_size = (original_size - offset < 58)
                                 ? static_cast<uint8_t>(original_size - offset)
                                 : 58;

        uint8_t payload[62];
        uint8_t len;
        build_chunk(payload, len, total_chunks, c, &original[offset], chunk_size);
        mavlink.process_frame(payload, len);
    }

    // verify full message arrived
    uint8_t rx[280];
    uint32_t n = read_all_rx(rx, sizeof(rx));
    EXPECT_EQ(n, original_size);
    EXPECT_EQ(memcmp(original, rx, original_size), 0);

    // write it back and re-chunk via TX
    uart_write(rx, n);

    uint8_t reassembled[280];
    uint16_t reassembled_len = 0;
    uint8_t chunk_count = 0;
    while (mavlink.tx_pending()) {
        uint8_t tx_payload[62];
        uint8_t tx_len = 0;
        if (mavlink.get_telem_frame(tx_payload, tx_len)) {
            uint8_t data_size = tx_payload[1];
            memcpy(&reassembled[reassembled_len], &tx_payload[2], data_size);
            reassembled_len += data_size;
            chunk_count++;
        }
    }

    EXPECT_EQ(reassembled_len, original_size);
    EXPECT_EQ(chunk_count, total_chunks);
    EXPECT_EQ(memcmp(original, reassembled, original_size), 0);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
