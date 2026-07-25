
#include <gtest/gtest.h>

extern "C"
{
#include <c/core/session/stream/seq_num.c>
#include <c/core/session/stream/input_reliable_stream.c>
}

#define BUFFER_SIZE           size_t(128)
#define HISTORY               size_t(4)
#define MAX_MESSAGE_SIZE      (BUFFER_SIZE / HISTORY - INTERNAL_RELIABLE_BUFFER_OFFSET)
#define FRAGMENT_OFFSET       size_t(4)

bool operator == (
        const uxrInputReliableStream& stream1,
        const uxrInputReliableStream& stream2)
{
    return stream1.base.buffer == stream2.base.buffer
           && stream1.base.size == stream2.base.size
           && stream1.base.history == stream2.base.history
           && stream1.last_handled == stream2.last_handled
           && stream1.last_announced == stream2.last_announced
           && stream1.on_get_fragmentation_info == stream2.on_get_fragmentation_info
           && stream1.cleanup_flag == stream2.cleanup_flag;
}

bool operator != (
        const uxrInputReliableStream& stream1,
        const uxrInputReliableStream& stream2)
{
    return !(stream1 == stream2);
}

class InputReliableStreamTest : public testing::Test
{
public:

    InputReliableStreamTest()
    {
        uxr_init_input_reliable_stream(&stream, buffer, BUFFER_SIZE, HISTORY, on_get_fragmentation_info);
        EXPECT_EQ(buffer, stream.base.buffer);
        EXPECT_EQ(BUFFER_SIZE, stream.base.size);
        EXPECT_EQ(HISTORY, stream.base.history);
        EXPECT_EQ(SEQ_NUM_MAX, stream.last_handled);
        EXPECT_EQ(SEQ_NUM_MAX, stream.last_announced);
        EXPECT_EQ(false, stream.cleanup_flag);

        for (uint16_t i = 0; i < HISTORY; ++i)
        {
            EXPECT_EQ(size_t(0), uxr_get_reliable_buffer_size(&stream.base, i));
        }
    }

    void copy(
            uxrInputReliableStream* dest,
            uxrInputReliableStream* source)
    {
        dest->base.buffer = source->base.buffer;
        dest->base.size = source->base.size;
        dest->base.history = source->base.history;

        dest->last_handled = source->last_handled;
        dest->last_announced = source->last_announced;

        dest->on_get_fragmentation_info = source->on_get_fragmentation_info;

        dest->cleanup_flag = source->cleanup_flag;
    }

    virtual ~InputReliableStreamTest()
    {
    }

protected:

    uxrInputReliableStream stream;
    uint8_t buffer[BUFFER_SIZE];
    uint8_t message[MAX_MESSAGE_SIZE];

    static FragmentationInfo on_get_fragmentation_info(
            uint8_t* buffer)
    {
        (void) buffer;
        return NO_FRAGMENTED;
    }

};

TEST_F(InputReliableStreamTest, UpToDate)
{
    bool up_to_date = uxr_is_input_up_to_date(&stream);
    EXPECT_TRUE(up_to_date);
}

TEST_F(InputReliableStreamTest, NotUpToDate)
{
    //PRE
    bool message_stored;
    (void) uxr_receive_reliable_message(&stream, 1, message, MAX_MESSAGE_SIZE, &message_stored);

    bool up_to_date = uxr_is_input_up_to_date(&stream);
    EXPECT_FALSE(up_to_date);
}

TEST_F(InputReliableStreamTest, ReceiveValidMessage)
{
    uxrSeqNum seq_num = 0;

    bool message_stored;
    bool ready_to_read = uxr_receive_reliable_message(&stream, seq_num, message, MAX_MESSAGE_SIZE, &message_stored);
    ASSERT_TRUE(ready_to_read);
    EXPECT_FALSE(message_stored);
    EXPECT_EQ(seq_num, stream.last_handled);
    EXPECT_EQ(seq_num, stream.last_announced);
}

TEST_F(InputReliableStreamTest, ReceiveValidMessageForStoring)
{
    uxrSeqNum seq_num = 1;

    bool message_stored;
    bool ready_to_read = uxr_receive_reliable_message(&stream, seq_num, message, MAX_MESSAGE_SIZE, &message_stored);
    ASSERT_FALSE(ready_to_read);
    EXPECT_TRUE(message_stored);
    EXPECT_EQ(SEQ_NUM_MAX, stream.last_handled);
    EXPECT_EQ(seq_num, stream.last_announced);
}

TEST_F(InputReliableStreamTest, ReceiveInvalidMessageAlreadyReceived)
{
    uxrSeqNum seq_num = SEQ_NUM_MAX;

    uxrInputReliableStream backup;
    copy(&backup, &stream);
    bool message_stored;
    bool ready_to_read = uxr_receive_reliable_message(&stream, seq_num, message, MAX_MESSAGE_SIZE, &message_stored);
    ASSERT_FALSE(ready_to_read);
    EXPECT_EQ(backup, stream);
}

TEST_F(InputReliableStreamTest, ReceiveInvalidMessageTooHigh)
{
    uxrSeqNum seq_num = HISTORY;

    bool message_stored;
    bool ready_to_read = uxr_receive_reliable_message(&stream, seq_num, message, MAX_MESSAGE_SIZE, &message_stored);
    ASSERT_FALSE(ready_to_read);
    EXPECT_EQ(SEQ_NUM_MAX, stream.last_handled);
    EXPECT_EQ(seq_num, stream.last_announced);
}

TEST_F(InputReliableStreamTest, GetFirstUnacked)
{
    uxrSeqNum first_unknown = uxr_get_first_unacked(&stream);
    EXPECT_EQ(uxr_seq_num_add(stream.last_handled, 1), first_unknown);
}

TEST_F(InputReliableStreamTest, ProcessNewHeartbeat)
{
    uxrSeqNum last_seq_num = HISTORY * 2;
    uxr_process_heartbeat(&stream, SEQ_NUM_MAX, last_seq_num);
    EXPECT_EQ(stream.last_announced, last_seq_num);
}

TEST_F(InputReliableStreamTest, ProcessOldHeartbeat)
{
    uxrInputReliableStream backup;
    copy(&backup, &stream);
    uxr_process_heartbeat(&stream, SEQ_NUM_MAX, SEQ_NUM_MAX);
    EXPECT_EQ(backup, stream);
}

TEST_F(InputReliableStreamTest, NextBufferAvailable)
{
    //PRE
    bool message_stored;
    (void) uxr_receive_reliable_message(&stream, 1, message, MAX_MESSAGE_SIZE, &message_stored);
    (void) uxr_receive_reliable_message(&stream, 0, message, MAX_MESSAGE_SIZE, &message_stored);
    ucdrBuffer ub;

    bool available_to_read = uxr_next_input_reliable_buffer_available(&stream, &ub, 4);
    ASSERT_TRUE(available_to_read);
    EXPECT_EQ(1, stream.last_handled);
}

TEST_F(InputReliableStreamTest, NextBufferUnavailable)
{
    ucdrBuffer ub;

    uxrInputReliableStream backup;
    copy(&backup, &stream);
    bool available_to_read = uxr_next_input_reliable_buffer_available(&stream, &ub, 4);
    ASSERT_FALSE(available_to_read);
    EXPECT_EQ(backup, stream);
}

TEST_F(InputReliableStreamTest, NextBufferUnavailableAfterReceive)
{
    //PRE
    bool message_stored;
    (void) uxr_receive_reliable_message(&stream, 1, message, MAX_MESSAGE_SIZE, &message_stored);
    (void) uxr_receive_reliable_message(&stream, 0, message, MAX_MESSAGE_SIZE, &message_stored);
    ucdrBuffer ub;

    bool available_to_read = uxr_next_input_reliable_buffer_available(&stream, &ub, 4);
    ASSERT_TRUE(available_to_read);
    EXPECT_EQ(1, stream.last_handled);

    uxrInputReliableStream backup;
    copy(&backup, &stream);
    available_to_read = uxr_next_input_reliable_buffer_available(&stream, &ub, 4);
    ASSERT_FALSE(available_to_read);
    EXPECT_EQ(backup, stream);
}

TEST_F(InputReliableStreamTest, Reset)
{
    //PRE
    uxrInputReliableStream backup;
    copy(&backup, &stream);
    bool message_stored;
    (void) uxr_receive_reliable_message(&stream, 1, message, MAX_MESSAGE_SIZE, &message_stored);

    uxr_reset_input_reliable_stream(&stream);
    EXPECT_EQ(backup, stream);

    for (uint16_t i = 0; i < HISTORY; ++i)
    {
        EXPECT_EQ(size_t(0), uxr_get_reliable_buffer_size(&stream.base, i));
    }
}

TEST_F(InputReliableStreamTest, FragmentationJumpToNextBuffer)
{
    size_t capacity = uxr_get_reliable_buffer_capacity(&stream.base);
    uint8_t* slot_0 = uxr_get_reliable_buffer(&stream.base, 0);
    uint8_t* slot_1 = uxr_get_reliable_buffer(&stream.base, 1);
    uxr_set_reliable_buffer_size(&stream.base, 0, capacity);
    uxr_set_reliable_buffer_size(&stream.base, 1, capacity - 1);

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, slot_0, uint32_t(capacity));
    ucdr_set_on_full_buffer_callback(&ub, on_full_input_buffer, &stream);
    stream.cleanup_flag = true;

    uint8_t array[BUFFER_SIZE];
    (void) ucdr_serialize_array_uint8_t(&ub, array,
            uint32_t((capacity + capacity - 1) - 2 * (sizeof(length_t) + SUBHEADER_SIZE)));
    ASSERT_FALSE(ub.error);
    EXPECT_EQ(slot_1 + SUBHEADER_SIZE, ub.init);
    EXPECT_EQ(slot_1 + capacity - 1, ub.final);
    EXPECT_EQ(size_t(0), uxr_get_reliable_buffer_size(&stream.base, 1));
}

TEST_F(InputReliableStreamTest, FragmentationJumpToNextBufferLastPosition)
{
    size_t capacity = uxr_get_reliable_buffer_capacity(&stream.base);
    uint8_t* slot_history_1 = uxr_get_reliable_buffer(&stream.base, HISTORY - 1);
    uint8_t* slot_0 = uxr_get_reliable_buffer(&stream.base, 0);
    uxr_set_reliable_buffer_size(&stream.base, HISTORY - 1, capacity);
    uxr_set_reliable_buffer_size(&stream.base, 0, capacity - 1);

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, slot_history_1, uint32_t(capacity));
    ucdr_set_on_full_buffer_callback(&ub, on_full_input_buffer, &stream);
    stream.cleanup_flag = true;

    uint8_t array[BUFFER_SIZE];
    (void) ucdr_serialize_array_uint8_t(&ub, array,
            uint32_t((capacity + capacity - 1) - 2 * (sizeof(length_t) + SUBHEADER_SIZE)));
    ASSERT_FALSE(ub.error);
    EXPECT_EQ(slot_0 + SUBHEADER_SIZE, ub.init);
    EXPECT_EQ(slot_0 + capacity - 1, ub.final);
    EXPECT_EQ(size_t(0), uxr_get_reliable_buffer_size(&stream.base, 0));
}
