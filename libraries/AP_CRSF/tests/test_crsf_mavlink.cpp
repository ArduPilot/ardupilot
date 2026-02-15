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

    static void build_chunk(uint8_t *payload, uint8_t &len,
                            uint8_t total_chunks, uint8_t current_chunk,
                            const uint8_t *data, uint8_t data_size)
    {
        payload[0] = ((total_chunks & 0x0F) << 4) | (current_chunk & 0x0F);
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
};

// ===========================================================================
// MavlinkEnvelopeFrame struct parsing
// ===========================================================================

TEST(CRSFProtocol, MavlinkEnvelopeChunkInfoParsing)
{
    AP_CRSF_Protocol::MavlinkEnvelopeFrame frame {};
    // total=3, current=0
    frame.chunk_info = 0x30;
    EXPECT_EQ(frame.total_chunks(), 3);
    EXPECT_EQ(frame.current_chunk(), 0);

    // total=15, current=7
    frame.chunk_info = 0xF7;
    EXPECT_EQ(frame.total_chunks(), 15);
    EXPECT_EQ(frame.current_chunk(), 7);

    // total=1, current=0  (single chunk)
    frame.chunk_info = 0x10;
    EXPECT_EQ(frame.total_chunks(), 1);
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

TEST_F(TestCRSFMAVLink, ZeroTotalChunksRejected)
{
    uint8_t data[10];
    memset(data, 0xDD, sizeof(data));

    uint8_t payload[62];
    uint8_t len;

    build_chunk(payload, len, 0, 0, data, 10);
    mavlink.process_frame(payload, len);

    EXPECT_EQ(uart_available(), 0u);
    EXPECT_FALSE(reassembly_active());
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
    uint8_t msg[20];
    for (uint8_t i = 0; i < sizeof(msg); i++) {
        msg[i] = i + 0x10;
    }
    uart_write(msg, sizeof(msg));

    EXPECT_TRUE(mavlink.tx_pending());

    uint8_t payload[62];
    uint8_t len = 0;
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));

    uint8_t total = (payload[0] >> 4) & 0x0F;
    uint8_t current = payload[0] & 0x0F;
    uint8_t data_size = payload[1];

    EXPECT_EQ(total, 1);
    EXPECT_EQ(current, 0);
    EXPECT_EQ(data_size, sizeof(msg));
    EXPECT_EQ(len, sizeof(msg) + 2);
    EXPECT_EQ(memcmp(&payload[2], msg, sizeof(msg)), 0);

    EXPECT_FALSE(mavlink.tx_pending());
    EXPECT_FALSE(mavlink.get_telem_frame(payload, len));
}

TEST_F(TestCRSFMAVLink, TxMultipleChunks)
{
    // 100 bytes — 2 chunks: 58 + 42
    uint8_t msg[100];
    for (uint8_t i = 0; i < sizeof(msg); i++) {
        msg[i] = i;
    }
    uart_write(msg, sizeof(msg));

    // chunk 0
    uint8_t payload[62];
    uint8_t len = 0;
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));
    EXPECT_EQ((payload[0] >> 4) & 0x0F, 2);
    EXPECT_EQ(payload[0] & 0x0F, 0);
    EXPECT_EQ(payload[1], 58);
    EXPECT_EQ(len, 60);
    EXPECT_EQ(memcmp(&payload[2], &msg[0], 58), 0);

    // chunk 1
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));
    EXPECT_EQ((payload[0] >> 4) & 0x0F, 2);
    EXPECT_EQ(payload[0] & 0x0F, 1);
    EXPECT_EQ(payload[1], 42);
    EXPECT_EQ(len, 44);
    EXPECT_EQ(memcmp(&payload[2], &msg[58], 42), 0);

    EXPECT_FALSE(mavlink.tx_pending());
}

TEST_F(TestCRSFMAVLink, TxExactlyOneChunkBoundary)
{
    // exactly 58 bytes — one full chunk
    uint8_t msg[58];
    memset(msg, 0x42, sizeof(msg));
    uart_write(msg, sizeof(msg));

    uint8_t payload[62];
    uint8_t len = 0;
    EXPECT_TRUE(mavlink.get_telem_frame(payload, len));
    EXPECT_EQ((payload[0] >> 4) & 0x0F, 1);
    EXPECT_EQ(payload[0] & 0x0F, 0);
    EXPECT_EQ(payload[1], 58);
    EXPECT_EQ(memcmp(&payload[2], msg, 58), 0);

    EXPECT_FALSE(mavlink.get_telem_frame(payload, len));
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
    uint8_t original[30];
    for (uint8_t i = 0; i < sizeof(original); i++) {
        original[i] = i + 0x50;
    }

    // RX: inject as a single 0xAA chunk
    uint8_t payload[62];
    uint8_t len;
    build_chunk(payload, len, 1, 0, original, sizeof(original));
    mavlink.process_frame(payload, len);

    // read from virtual UART
    uint8_t rx[30];
    uint32_t n = read_all_rx(rx, sizeof(rx));
    EXPECT_EQ(n, sizeof(original));
    EXPECT_EQ(memcmp(original, rx, sizeof(original)), 0);

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

    EXPECT_EQ(reassembled_len, sizeof(original));
    EXPECT_EQ(memcmp(original, reassembled, sizeof(original)), 0);
}

TEST_F(TestCRSFMAVLink, RoundTripLargeMessage)
{
    // 280-byte message (max MAVLink2 frame)
    uint8_t original[280];
    for (uint16_t i = 0; i < sizeof(original); i++) {
        original[i] = static_cast<uint8_t>(i & 0xFF);
    }

    // RX: chunk into 58-byte pieces
    uint8_t total_chunks = (sizeof(original) + 57) / 58;  // = 5
    for (uint8_t c = 0; c < total_chunks; c++) {
        uint16_t offset = static_cast<uint16_t>(c) * 58;
        uint8_t chunk_size = (sizeof(original) - offset < 58)
                                 ? static_cast<uint8_t>(sizeof(original) - offset)
                                 : 58;

        uint8_t payload[62];
        uint8_t len;
        build_chunk(payload, len, total_chunks, c, &original[offset], chunk_size);
        mavlink.process_frame(payload, len);
    }

    // verify full message arrived
    uint8_t rx[280];
    uint32_t n = read_all_rx(rx, sizeof(rx));
    EXPECT_EQ(n, sizeof(original));
    EXPECT_EQ(memcmp(original, rx, sizeof(original)), 0);

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

    EXPECT_EQ(reassembled_len, sizeof(original));
    EXPECT_EQ(chunk_count, total_chunks);
    EXPECT_EQ(memcmp(original, reassembled, sizeof(original)), 0);
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
