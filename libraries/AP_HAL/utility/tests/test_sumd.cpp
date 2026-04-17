/*
 * Test for SUMD decoder channel bounds protection.
 *
 * Verifies that sumd_decode() honours max_chan_count and does not write
 * beyond the end of the caller-supplied channels[] array when the packet
 * claims more channels than the array can hold.
 */

#include <AP_gtest.h>

#include <AP_Math/crc.h>
#include <AP_HAL/utility/sumd.h>

// Constants from sumd.cpp (not exported in the header)
#define SUMD_HEADER_ID  0xA8
#define SUMD_ID_SUMD    0x01

// Build and feed a complete, CRC-valid SUMD packet to the decoder.
// Returns the final return code from sumd_decode().
static int feed_sumd_packet(uint8_t num_channels,
                             uint16_t *channels, uint16_t max_chan_count,
                             uint8_t *rssi, uint8_t *rx_count,
                             uint16_t *channel_count)
{
    // Reset the decoder state machine by feeding a byte that is not a valid
    // header (the decoder returns to UNSYNCED on any unexpected byte).
    sumd_decode(0x00, rssi, rx_count, channel_count, channels, max_chan_count);

    // Packet: header(1) + status(1) + length(1) + channel_data(num_channels*2) + crc16(2)
    const size_t data_len = 3U + num_channels * 2U;
    const size_t pkt_len  = data_len + 2U;
    uint8_t pkt[pkt_len];

    pkt[0] = SUMD_HEADER_ID;
    pkt[1] = SUMD_ID_SUMD;
    pkt[2] = num_channels;

    // Encode a centre value (8800 << 3 = 0x2260 raw) for every channel
    for (uint8_t i = 0; i < num_channels; i++) {
        uint16_t raw = 0x2260U;
        pkt[3 + i * 2]     = (uint8_t)(raw >> 8);
        pkt[3 + i * 2 + 1] = (uint8_t)(raw & 0xFF);
    }

    // Compute CRC-XMODEM over header + status + length + channel data
    uint16_t crc = 0;
    for (size_t i = 0; i < data_len; i++) {
        crc = crc_xmodem_update(crc, pkt[i]);
    }
    pkt[data_len]     = (uint8_t)(crc >> 8);
    pkt[data_len + 1] = (uint8_t)(crc & 0xFF);

    int ret = 1;
    for (size_t i = 0; i < pkt_len; i++) {
        ret = sumd_decode(pkt[i], rssi, rx_count, channel_count, channels, max_chan_count);
    }
    return ret;
}

// Verify that a packet claiming more channels than max_chan_count is decoded
// successfully and does not write beyond channels[max_chan_count - 1].
TEST(SUMD, channel_bounds_not_exceeded)
{
    const uint16_t MAX = 4;
    const uint8_t  PKT_CHANNELS = 8;  // packet claims 8 channels
    const uint16_t SENTINEL = 0xDEAD;

    // Allocate one extra slot after the valid range and fill with sentinel
    uint16_t channels[MAX + 1];
    for (uint16_t i = 0; i <= MAX; i++) {
        channels[i] = SENTINEL;
    }

    uint8_t rssi = 0, rx_count = 0;
    uint16_t channel_count = 0;

    int ret = feed_sumd_packet(PKT_CHANNELS, channels, MAX,
                               &rssi, &rx_count, &channel_count);

    // Decoder should report success (return 0)
    EXPECT_EQ(0, ret);

    // Sentinel beyond the valid range must not have been overwritten
    EXPECT_EQ(SENTINEL, channels[MAX]);
}

// Verify a normal packet where num_channels <= max_chan_count is decoded
// correctly, producing non-sentinel channel values.
TEST(SUMD, normal_packet_decoded)
{
    const uint16_t MAX = 8;
    const uint8_t  PKT_CHANNELS = 8;
    const uint16_t SENTINEL = 0xDEAD;

    uint16_t channels[MAX];
    for (uint16_t i = 0; i < MAX; i++) {
        channels[i] = SENTINEL;
    }

    uint8_t rssi = 0, rx_count = 0;
    uint16_t channel_count = 0;

    int ret = feed_sumd_packet(PKT_CHANNELS, channels, MAX,
                               &rssi, &rx_count, &channel_count);

    EXPECT_EQ(0, ret);
    EXPECT_EQ(PKT_CHANNELS, channel_count);

    // All channels should have been written (no longer sentinel)
    for (uint16_t i = 0; i < MAX; i++) {
        EXPECT_NE(SENTINEL, channels[i]);
    }
}

AP_GTEST_MAIN()
int hal = 0;
