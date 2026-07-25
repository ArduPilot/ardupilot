
#include <gtest/gtest.h>

extern "C"
{
#include <c/core/session/stream/seq_num.c>
#include <c/core/session/stream/input_best_effort_stream.c>
}

class InputBestEffortStreamTest : public testing::Test
{
public:

    InputBestEffortStreamTest()
    {
        uxr_init_input_best_effort_stream(&stream);
        EXPECT_EQ(SEQ_NUM_MAX, stream.last_handled);
    }

    virtual ~InputBestEffortStreamTest()
    {
    }

protected:

    uxrInputBestEffortStream stream;
};

TEST_F(InputBestEffortStreamTest, Reset)
{
    uxr_receive_best_effort_message(&stream, 2);
    uxr_reset_input_best_effort_stream(&stream);
    EXPECT_EQ(SEQ_NUM_MAX, stream.last_handled);
}

TEST_F(InputBestEffortStreamTest, ReceiveExpected)
{
    uxrSeqNum expected_seq_num = uxr_seq_num_add(stream.last_handled, 1);
    bool available_to_read = uxr_receive_best_effort_message(&stream, expected_seq_num);
    EXPECT_TRUE(available_to_read);
    EXPECT_EQ(expected_seq_num, stream.last_handled);
}

TEST_F(InputBestEffortStreamTest, ReceiveNoExpectedLost)
{
    uxrSeqNum expected_seq_num = uxr_seq_num_add(stream.last_handled, 2);
    bool available_to_read = uxr_receive_best_effort_message(&stream, expected_seq_num);
    EXPECT_TRUE(available_to_read);
    EXPECT_EQ(expected_seq_num, stream.last_handled);
}

TEST_F(InputBestEffortStreamTest, ReceiveNoExpected)
{
    uxrSeqNum expected_seq_num = stream.last_handled;
    bool available_to_read = uxr_receive_best_effort_message(&stream, expected_seq_num);
    EXPECT_FALSE(available_to_read);
    EXPECT_EQ(expected_seq_num, stream.last_handled);
}
