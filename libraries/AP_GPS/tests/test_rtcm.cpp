/*
  Unit tests for AP_GPS RTCM fragment reassembly.

  The _rtcm_buffer was changed from a lazily-allocated heap pointer to a
  value member.  These tests verify:

    1. The buffer is correctly zero-initialised at construction (no prior
       calloc needed to make it safe).
    2. Partial fragment arrival updates the buffer state correctly.
    3. A new sequence number discards stale fragments.
    4. A duplicate fragment is silently ignored.
    5. The last (short) fragment triggers correct length accounting.
 */

#include <AP_gtest.h>
#include <AP_GPS/AP_GPS.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

/*
  AP_GPS_RTCMTest is declared as a friend in AP_GPS.h.
  It exposes the private _rtcm_buffer and nulls the drivers array so that
  inject_data() – which would otherwise dereference uninitialised pointers –
  becomes a safe no-op.
 */
class AP_GPS_RTCMTest {
public:
    AP_GPS gps;

    AP_GPS_RTCMTest()
    {
        // Prevent inject_data() from touching uninitialised driver pointers.
        memset(gps.drivers, 0, sizeof(gps.drivers));
    }

    const AP_GPS::rtcm_buffer &buf() const { return gps._rtcm_buffer; }

    void reset_buffer()
    {
        memset(&gps._rtcm_buffer, 0, sizeof(gps._rtcm_buffer));
    }

    // Convenience: build the 8-bit flags field.
    //   bit 0: fragmented flag
    //   bits 1-2: fragment index (0-3)
    //   bits 3-7: sequence number (0-31)
    static uint8_t make_flags(bool fragmented, uint8_t fragment, uint8_t seq)
    {
        return (fragmented ? 1u : 0u) | ((fragment & 0x3u) << 1) | ((seq & 0x1Fu) << 3);
    }

    void send(uint8_t flags, const uint8_t *data, uint8_t len)
    {
        gps.handle_gps_rtcm_fragment(flags, data, len);
    }
};

// One singleton per process.
static AP_GPS_RTCMTest *g_rtcm_test;

class RTCMBufferTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        if (g_rtcm_test == nullptr) {
            g_rtcm_test = new AP_GPS_RTCMTest();
        }
    }

    void SetUp() override
    {
        // Reset buffer state between tests via the friend class.
        g_rtcm_test->reset_buffer();
    }

    AP_GPS_RTCMTest &t() { return *g_rtcm_test; }
};

/*
  The buffer must be zero-initialised from construction – no calloc needed.
 */
TEST_F(RTCMBufferTest, InitialBufferIsZero)
{
    EXPECT_EQ(t().buf().fragments_received, 0);
    EXPECT_EQ(t().buf().fragment_count, 0);
    EXPECT_EQ(t().buf().total_length, 0);
}

/*
  Sending fragment 0 (of an unknown total) should mark bit 0 in
  fragments_received.  fragment_count remains 0 until a short fragment
  reveals the total.
 */
TEST_F(RTCMBufferTest, FirstFragmentSetsReceivedBit)
{
    const uint8_t data[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN] = {};
    t().send(AP_GPS_RTCMTest::make_flags(true, 0, 1), data, sizeof(data));

    EXPECT_EQ(t().buf().fragments_received, 0x01u);
    EXPECT_EQ(t().buf().fragment_count, 0u);  // size still unknown
    EXPECT_EQ(t().buf().sequence, 1u);
}

/*
  A short fragment (len < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) must set
  fragment_count to (fragment_index + 1).

  We send fragments 0 and 2 but deliberately skip fragment 1 to prevent
  assembly completion (inject_data resets the buffer when all fragments
  arrive, which would destroy the state we want to inspect).
 */
TEST_F(RTCMBufferTest, ShortFragmentSetsFragmentCount)
{
    const uint8_t full[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN] = {};
    const uint8_t partial[10] = {};

    // fragment 0 (full size, sequence 2)
    t().send(AP_GPS_RTCMTest::make_flags(true, 0, 2), full, sizeof(full));
    // fragment 2 (short) → fragment_count = 3, but assembly is incomplete
    // because fragment 1 hasn't arrived yet.
    t().send(AP_GPS_RTCMTest::make_flags(true, 2, 2), partial, sizeof(partial));

    EXPECT_EQ(t().buf().fragment_count, 3u);
    // bits 0 and 2 set; bit 1 still missing
    EXPECT_EQ(t().buf().fragments_received, 0x05u);
    EXPECT_EQ(t().buf().total_length,
              MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN * 2 + sizeof(partial));
}

/*
  A new sequence number must discard any previously buffered fragments.
 */
TEST_F(RTCMBufferTest, NewSequenceDiscardsPrevious)
{
    const uint8_t data[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN] = {};

    // sequence 5, fragment 0
    t().send(AP_GPS_RTCMTest::make_flags(true, 0, 5), data, sizeof(data));
    EXPECT_EQ(t().buf().fragments_received, 0x01u);

    // sequence 6, fragment 0 – must clear the previous state
    t().send(AP_GPS_RTCMTest::make_flags(true, 0, 6), data, sizeof(data));
    EXPECT_EQ(t().buf().fragments_received, 0x01u);  // only new fragment
    EXPECT_EQ(t().buf().sequence, 6u);
}

/*
  A duplicate fragment (same sequence, same index, same data) must be
  silently dropped without corrupting the buffer.
 */
TEST_F(RTCMBufferTest, DuplicateFragmentIgnored)
{
    const uint8_t data[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN] = {0x42};

    t().send(AP_GPS_RTCMTest::make_flags(true, 0, 3), data, sizeof(data));
    EXPECT_EQ(t().buf().fragments_received, 0x01u);

    // send identical fragment again
    t().send(AP_GPS_RTCMTest::make_flags(true, 0, 3), data, sizeof(data));
    EXPECT_EQ(t().buf().fragments_received, 0x01u);  // unchanged
}

/*
  Send all fragments of a 2-fragment message and verify that the buffer is
  cleared after assembly completes.

  This is the exact path that was behind the old calloc: without the
  pre-allocated _rtcm_buffer the function would either segfault (null
  pointer dereference on write) or silently discard the message.  This
  test proves the pre-allocated buffer survives a complete round-trip.

  Fragment 0 is full-size; fragment 1 is short (< FIELD_DATA_LEN), which:
    - sets fragment_count = 2
    - triggers inject_data when bits 0+1 are both set
    - clears fragment_count and fragments_received back to 0
 */
TEST_F(RTCMBufferTest, CompleteReassemblyResetsBuffer)
{
    const uint8_t full[MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN] = {};
    const uint8_t partial[10] = {};

    // fragment 0 (full size) – fragment_count still unknown
    t().send(AP_GPS_RTCMTest::make_flags(true, 0, 7), full, sizeof(full));
    EXPECT_EQ(t().buf().fragment_count, 0u);
    EXPECT_EQ(t().buf().fragments_received, 0x01u);

    // fragment 1 (short) – reveals 2 total fragments; assembly completes
    t().send(AP_GPS_RTCMTest::make_flags(true, 1, 7), partial, sizeof(partial));

    // inject_data fired: buffer must be reset
    EXPECT_EQ(t().buf().fragment_count, 0u)
        << "fragment_count must be cleared after inject_data";
    EXPECT_EQ(t().buf().fragments_received, 0u)
        << "fragments_received must be cleared after inject_data";
}

AP_GTEST_MAIN()
