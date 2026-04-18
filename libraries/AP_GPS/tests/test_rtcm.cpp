/*
  Regression test for AP_GPS RTCM fragment reassembly buffer.

  This first commit DOCUMENTS the bug: the reassembly buffer is a lazily-
  allocated heap pointer (calloc on first fragment).  The test below PASSES
  on the unfixed code, proving the pointer is null at startup.

  The follow-up commit (fix) converts the buffer to a pre-allocated value
  member and replaces this doc test with a full regression suite that
  exercises the reassembly path.

  To see the bug:
    1. Check out this commit only (before the fix).
    2. Build and run:
         waf tests && build/sitl/tests/test_rtcm
    3. The LazyAllocBug test passes (buffer IS null), confirming calloc has
       not yet been called.  On a memory-pressured target calloc can fail
       silently, discarding all fragmented RTCM corrections until reboot.
 */

#include <AP_gtest.h>
#include <AP_GPS/AP_GPS.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

/*
  AP_GPS_RTCMTest is declared as a friend in AP_GPS.h.
  It exposes the private rtcm_buffer pointer to allow the doc test below
  to prove the pointer is null before any fragment has been received.
 */
class AP_GPS_RTCMTest {
public:
    AP_GPS gps;

    AP_GPS_RTCMTest()
    {
        memset(gps.drivers, 0, sizeof(gps.drivers));
    }

    // Return true if the heap buffer has been allocated (old API, pre-fix).
    // The struct type is anonymous so we can only compare the pointer to null.
    // After the fix this field no longer exists; the test file is updated
    // in the same commit that removes it.
    bool rtcm_allocated() const { return gps.rtcm_buffer != nullptr; }
};

static AP_GPS_RTCMTest *g_rtcm_test;

class RTCMBufferTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        if (g_rtcm_test == nullptr) {
            g_rtcm_test = new AP_GPS_RTCMTest();
        }
    }
    AP_GPS_RTCMTest &t() { return *g_rtcm_test; }
};

/*
  DOCUMENTS THE BUG: the RTCM reassembly buffer is null at startup.
  AP_GPS lazily calloc()-s it inside handle_gps_rtcm_fragment() on the
  first fragmented packet.

  Consequence: if calloc() fails during flight (memory pressure), ALL
  fragmented RTCM differential corrections are silently discarded until
  the next reboot – GPS precision degrades without any warning.

  This test PASSES on the unfixed code.  After the fix the rtcm_buffer
  field is replaced by a value member (_rtcm_buffer {}) and this test
  is replaced by the full regression suite in the follow-up commit.
 */
TEST_F(RTCMBufferTest, LazyAllocBug_BufferNullBeforeFirstFragment)
{
    EXPECT_FALSE(t().rtcm_allocated())
        << "BUG: rtcm_buffer is null – proves lazy calloc() allocation; "
           "calloc failure in flight silently drops RTCM corrections";
}

AP_GTEST_MAIN()
