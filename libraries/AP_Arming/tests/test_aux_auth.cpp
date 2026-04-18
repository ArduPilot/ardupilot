/*
  Regression tests for AP_Arming auxiliary authorisation failure message.

  These tests FAIL on the current (unfixed) code in two ways, proving the
  bugs are real:

  1. ResetClearsCountAndMessage CRASHES (null-pointer dereference):
     reset_all_aux_auths() calls free(aux_auth_fail_msg) then sets the
     pointer to nullptr.  The test then reads fail_msg(arm())[0], which
     dereferences the now-null pointer → undefined behaviour / SIGSEGV.

  2. MaxLengthMessageIsNullTerminated FAILS:
     set_aux_auth_failed() uses strncpy(buf, src, n) without a subsequent
     explicit null-terminator.  When src length == n, strncpy fills the
     entire buffer with non-null bytes; the last byte is NOT '\0'.
     A consumer calling strlen() or printing the string reads past the
     end of the allocated region.

  Run on the unfixed tree to observe:
    waf tests && build/sitl/tests/test_aux_auth
  Expected output: crash on ResetClearsCountAndMessage, or a gtest
  failure showing stored[str_len-1] != '\0'.

  The fix (follow-up commit) replaces the heap pointer with a fixed-size
  embedded array and adds an explicit null-terminator after strncpy.
 */

#include <AP_gtest.h>
#include <AP_Arming/AP_Arming.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

/*
  AP_Arming_AuxAuthTest is declared as a friend in AP_Arming.h.
  It exposes private fields for assertion purposes.
 */
class AP_Arming_AuxAuthTest {
public:
    static uint8_t auth_count(const AP_Arming &a) { return a.aux_auth_count; }
    static bool auth_error(const AP_Arming &a)    { return a.aux_auth_error; }
    // Works for both char* (old) and char[] (new) – both decay to const char*.
    static const char *fail_msg(const AP_Arming &a) { return a.aux_auth_fail_msg; }
    static uint8_t count_max() { return AP_Arming::aux_auth_count_max; }
    static uint8_t str_len()   { return AP_Arming::aux_auth_str_len; }
};

// One singleton per process – AP_Arming panics on a second instantiation.
static AP_Arming *g_arming;

class AuxAuthTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        if (g_arming == nullptr) {
            g_arming = new AP_Arming();
        }
    }

    void SetUp() override
    {
        g_arming->reset_all_aux_auths();
    }

    AP_Arming &arm() { return *g_arming; }
};

// ---- Tests that also pass on fixed code ------------------------------------

TEST_F(AuxAuthTest, GetAuxAuthIdAssignsSequentialIds)
{
#if AP_ARMING_AUX_AUTH_ENABLED
    uint8_t id0, id1, id2;
    ASSERT_TRUE(arm().get_aux_auth_id(id0));
    ASSERT_TRUE(arm().get_aux_auth_id(id1));
    ASSERT_TRUE(arm().get_aux_auth_id(id2));
    EXPECT_EQ(id0, 0u);
    EXPECT_EQ(id1, 1u);
    EXPECT_EQ(id2, 2u);
    EXPECT_EQ(AP_Arming_AuxAuthTest::auth_count(arm()), 3u);
#endif
}

TEST_F(AuxAuthTest, GetAuxAuthIdCapFails)
{
#if AP_ARMING_AUX_AUTH_ENABLED
    uint8_t id;
    for (uint8_t i = 0; i < AP_Arming_AuxAuthTest::count_max(); i++) {
        ASSERT_TRUE(arm().get_aux_auth_id(id));
    }
    EXPECT_FALSE(arm().get_aux_auth_id(id));
    EXPECT_TRUE(AP_Arming_AuxAuthTest::auth_error(arm()));
#endif
}

TEST_F(AuxAuthTest, SetFailedStoresMessage)
{
#if AP_ARMING_AUX_AUTH_ENABLED
    uint8_t id;
    ASSERT_TRUE(arm().get_aux_auth_id(id));
    arm().set_aux_auth_failed(id, "test failure");
    EXPECT_STREQ(AP_Arming_AuxAuthTest::fail_msg(arm()), "test failure");
#endif
}

// ---- Tests that FAIL/CRASH on the unfixed code ----------------------------

/*
  BUG 1: null dereference after reset.

  reset_all_aux_auths() calls free(aux_auth_fail_msg) and sets the
  pointer to nullptr.  Reading fail_msg(arm())[0] immediately after
  dereferences that null pointer → undefined behaviour / SIGSEGV.

  After the fix (value member, no free), this is a safe read of '\0'.
 */
TEST_F(AuxAuthTest, ResetClearsCountAndMessage)
{
#if AP_ARMING_AUX_AUTH_ENABLED
    uint8_t id;
    ASSERT_TRUE(arm().get_aux_auth_id(id));
    arm().set_aux_auth_failed(id, "some error");
    EXPECT_STRNE(AP_Arming_AuxAuthTest::fail_msg(arm()), "");

    arm().reset_all_aux_auths();

    EXPECT_EQ(AP_Arming_AuxAuthTest::auth_count(arm()), 0u);
    // BUG: on old code, fail_msg() returns nullptr here → CRASH
    EXPECT_EQ(AP_Arming_AuxAuthTest::fail_msg(arm())[0], '\0');
#endif
}

TEST_F(AuxAuthTest, MultipleGetResetCycles)
{
#if AP_ARMING_AUX_AUTH_ENABLED
    for (int cycle = 0; cycle < 5; cycle++) {
        uint8_t id;
        ASSERT_TRUE(arm().get_aux_auth_id(id)) << "cycle " << cycle;
        arm().set_aux_auth_failed(id, "cycle error");
        EXPECT_STREQ(AP_Arming_AuxAuthTest::fail_msg(arm()), "cycle error");
        arm().reset_all_aux_auths();
        // BUG: old code frees on reset → null dereference on next SetUp() reset
        EXPECT_EQ(AP_Arming_AuxAuthTest::auth_count(arm()), 0u);
    }
#endif
}

/*
  BUG 2: strncpy does not null-terminate when src length == buffer size.

  set_aux_auth_failed() calls strncpy(buf, src, aux_auth_str_len).
  When src is exactly aux_auth_str_len characters long, strncpy fills
  the entire buffer without writing a '\0', leaving the string unterminated.
  Any code reading the message as a C-string reads past the buffer end.

  After the fix, aux_auth_fail_msg[aux_auth_str_len-1] = '\0' is always
  applied after strncpy.
 */
TEST_F(AuxAuthTest, MaxLengthMessageIsNullTerminated)
{
#if AP_ARMING_AUX_AUTH_ENABLED
    const uint8_t str_len = AP_Arming_AuxAuthTest::str_len();
    char long_msg[64] = {};          // large enough; {} zero-inits all bytes
    memset(long_msg, 'X', str_len);  // exactly str_len 'X' bytes; [str_len]=='\0'

    uint8_t id;
    ASSERT_TRUE(arm().get_aux_auth_id(id));
    arm().set_aux_auth_failed(id, long_msg);

    const char *stored = AP_Arming_AuxAuthTest::fail_msg(arm());
    // BUG: old code → stored[str_len-1] == 'X', not '\0'
    EXPECT_EQ(stored[str_len - 1], '\0')
        << "last byte of buffer must be null to prevent overread";
    EXPECT_LT(strlen(stored), (size_t)str_len);
#endif
}

/*
  The lowest auth_id's failure message must take precedence.
 */
TEST_F(AuxAuthTest, FailMessagePriority)
{
#if AP_ARMING_AUX_AUTH_ENABLED
    uint8_t id0, id1;
    ASSERT_TRUE(arm().get_aux_auth_id(id0));
    ASSERT_TRUE(arm().get_aux_auth_id(id1));

    arm().set_aux_auth_failed(id1, "low priority");
    EXPECT_STREQ(AP_Arming_AuxAuthTest::fail_msg(arm()), "low priority");

    arm().set_aux_auth_failed(id0, "high priority");
    EXPECT_STREQ(AP_Arming_AuxAuthTest::fail_msg(arm()), "high priority");

    arm().set_aux_auth_failed(id1, "low priority again");
    EXPECT_STREQ(AP_Arming_AuxAuthTest::fail_msg(arm()), "high priority");
#endif
}

AP_GTEST_MAIN()
