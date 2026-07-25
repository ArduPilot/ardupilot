
#include <gtest/gtest.h>

extern "C"
{
#include <c/core/session/stream/seq_num.c>
#include <c/core/session/stream/output_best_effort_stream.c>
#include <c/core/session/submessage.c>
#include <c/core/serialization/xrce_subheader.c>
}

#define BUFFER_SIZE     size_t(32)
#define OFFSET          size_t(8)

class OutputBestEffortStreamTest : public testing::Test
{
public:

    OutputBestEffortStreamTest()
    {
        uxr_init_output_best_effort_stream(&stream, buffer, BUFFER_SIZE, OFFSET);
        EXPECT_EQ(OFFSET, stream.writer);
        EXPECT_EQ(SEQ_NUM_MAX, stream.last_send);
        EXPECT_EQ(buffer, stream.buffer);
        EXPECT_EQ(OFFSET, stream.offset);
        EXPECT_EQ(BUFFER_SIZE, stream.size);
    }

    virtual ~OutputBestEffortStreamTest()
    {
    }

protected:

    uxrOutputBestEffortStream stream;
    uint8_t buffer[BUFFER_SIZE];
};

TEST_F(OutputBestEffortStreamTest, PrepareToWriteOk)
{
    size_t message_size = 16;
    ucdrBuffer ub;
    bool available_to_write = uxr_prepare_best_effort_buffer_to_write(&stream, message_size, &ub);

    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(OFFSET + message_size, stream.writer);
    EXPECT_EQ(OFFSET, size_t(ub.iterator - buffer));
    EXPECT_EQ(buffer + OFFSET + message_size, ub.final);
}

TEST_F(OutputBestEffortStreamTest, PrepareToWriteOk2)
{
    size_t message_size = 8;
    ucdrBuffer ub;
    (void) uxr_prepare_best_effort_buffer_to_write(&stream, message_size, &ub);
    bool available_to_write = uxr_prepare_best_effort_buffer_to_write(&stream, message_size, &ub);

    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(OFFSET + message_size * 2, stream.writer);
    EXPECT_EQ(OFFSET + message_size, size_t(ub.iterator - buffer));
    EXPECT_EQ(buffer + OFFSET + message_size * 2, ub.final);
}

TEST_F(OutputBestEffortStreamTest, PrepareToWriteOkFit)
{
    size_t message_size = BUFFER_SIZE - OFFSET;
    ucdrBuffer ub;
    bool available_to_write = uxr_prepare_best_effort_buffer_to_write(&stream, message_size, &ub);

    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(OFFSET + message_size, stream.writer);
    EXPECT_EQ(OFFSET, size_t(ub.iterator - buffer));
    EXPECT_EQ(buffer + OFFSET + message_size, ub.final);
}

TEST_F(OutputBestEffortStreamTest, PrepareToWriteFails)
{
    size_t message_size = BUFFER_SIZE - OFFSET + 1;
    ucdrBuffer ub;
    bool available_to_write = uxr_prepare_best_effort_buffer_to_write(&stream, message_size, &ub);

    EXPECT_FALSE(available_to_write);
    EXPECT_EQ(OFFSET, stream.writer);
}

TEST_F(OutputBestEffortStreamTest, PrepareToSend)
{
    size_t message_size = 16; ucdrBuffer ub;
    (void) uxr_prepare_best_effort_buffer_to_write(&stream, message_size, &ub);

    uint8_t* message; size_t length; uxrSeqNum seq_num;
    bool data_to_send = uxr_prepare_best_effort_buffer_to_send(&stream, &message, &length, &seq_num);

    ASSERT_TRUE(data_to_send);
    EXPECT_EQ(buffer, message);
    EXPECT_EQ(OFFSET + message_size, length);
    EXPECT_EQ(uxr_seq_num_add(SEQ_NUM_MAX, 1), seq_num);
    EXPECT_EQ(uxr_seq_num_add(SEQ_NUM_MAX, 1), stream.last_send);
    EXPECT_EQ(OFFSET, stream.writer);
}

TEST_F(OutputBestEffortStreamTest, PrepareToSendFails)
{
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    bool data_to_send = uxr_prepare_best_effort_buffer_to_send(&stream, &message, &length, &seq_num);

    ASSERT_FALSE(data_to_send);
    EXPECT_EQ(SEQ_NUM_MAX, stream.last_send);
    EXPECT_EQ(OFFSET, stream.writer);
}

TEST_F(OutputBestEffortStreamTest, Reset)
{
    size_t message_size = 16; ucdrBuffer ub;
    (void) uxr_prepare_best_effort_buffer_to_write(&stream, message_size, &ub);

    uint8_t* message; size_t length; uxrSeqNum seq_num;
    (void) uxr_prepare_best_effort_buffer_to_send(&stream, &message, &length, &seq_num);

    uxr_reset_output_best_effort_stream(&stream);
    EXPECT_EQ(OFFSET, stream.writer);
    EXPECT_EQ(SEQ_NUM_MAX, stream.last_send);
    EXPECT_EQ(buffer, stream.buffer);
    EXPECT_EQ(OFFSET, stream.offset);
    EXPECT_EQ(BUFFER_SIZE, stream.size);
}

