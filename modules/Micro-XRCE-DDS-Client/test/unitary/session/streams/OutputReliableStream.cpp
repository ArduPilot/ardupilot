
#include <gtest/gtest.h>

extern "C"
{
#include <c/core/session/stream/seq_num.c>
#include <c/core/session/stream/output_reliable_stream.c>
#include <c/core/serialization/xrce_subheader.c>
#include <c/core/session/submessage.c>
#include <c/core/serialization/xrce_header.c>
}

#define BUFFER_SIZE           size_t(128)
#define HISTORY               size_t(4)
#define OFFSET                size_t(8)
#define MAX_MESSAGE_SIZE      (BUFFER_SIZE / HISTORY - INTERNAL_RELIABLE_BUFFER_OFFSET)
#define SUBMESSAGE_SIZE       size_t(8)
#define MAX_SUBMESSAGE_SIZE   (MAX_MESSAGE_SIZE - OFFSET)
#define FRAGMENT_OFFSET       size_t(4)
#define MAX_FRAGMENT_SIZE     (MAX_MESSAGE_SIZE - OFFSET - FRAGMENT_OFFSET)

bool operator == (
        const uxrOutputReliableStream& stream1,
        const uxrOutputReliableStream& stream2)
{
    return stream1.base.buffer == stream2.base.buffer
           && stream1.base.size == stream2.base.size
           && stream1.base.history == stream2.base.history
           && stream1.offset == stream2.offset
           && stream1.last_written == stream2.last_written
           && stream1.last_sent == stream2.last_sent
           && stream1.last_acknown == stream2.last_acknown
           && stream1.next_heartbeat_timestamp == stream2.next_heartbeat_timestamp
           && stream1.next_heartbeat_tries == stream2.next_heartbeat_tries
           && stream1.send_lost == stream2.send_lost;
}

bool operator != (
        const uxrOutputReliableStream& stream1,
        const uxrOutputReliableStream& stream2)
{
    return !(stream1 == stream2);
}

class OutputReliableStreamTest : public testing::Test
{
public:

    OutputReliableStreamTest()
    {
        uxr_init_output_reliable_stream(&stream, buffer, BUFFER_SIZE, HISTORY, OFFSET);
        EXPECT_EQ(buffer, stream.base.buffer);
        EXPECT_EQ(BUFFER_SIZE, stream.base.size);
        EXPECT_EQ(HISTORY, stream.base.history);
        EXPECT_EQ(OFFSET, stream.offset);
        EXPECT_EQ(0, stream.last_written);
        EXPECT_EQ(SEQ_NUM_MAX, stream.last_sent);
        EXPECT_EQ(SEQ_NUM_MAX, stream.last_acknown);
        EXPECT_EQ(INT64_MAX, stream.next_heartbeat_timestamp);
        EXPECT_EQ(0, stream.next_heartbeat_tries);
        EXPECT_EQ(false, stream.send_lost);

        for (uint16_t i = 0; i < HISTORY; ++i)
        {
            EXPECT_EQ(OFFSET, uxr_get_reliable_buffer_size(&stream.base, i));
        }
    }

    void copy(
            uxrOutputReliableStream* dest,
            uxrOutputReliableStream* source)
    {
        dest->base.buffer = source->base.buffer;
        dest->base.size = source->base.size;
        dest->base.history = source->base.history;
        dest->offset = source->offset;

        dest->last_written = source->last_written;
        dest->last_sent = source->last_sent;
        dest->last_acknown = source->last_acknown;

        dest->next_heartbeat_timestamp = source->next_heartbeat_timestamp;
        dest->next_heartbeat_tries = source->next_heartbeat_tries;
        dest->send_lost = source->send_lost;
    }

    virtual ~OutputReliableStreamTest()
    {
    }

protected:

    uxrOutputReliableStream stream;
    uint8_t buffer[BUFFER_SIZE];
};


TEST_F(OutputReliableStreamTest, UpToDate)
{
    bool up_to_date = uxr_is_output_up_to_date(&stream);
    EXPECT_TRUE(up_to_date);
}

TEST_F(OutputReliableStreamTest, NotUpToDate)
{
    ucdrBuffer ub;
    (void) uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    (void) uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);

    bool up_to_date = uxr_is_output_up_to_date(&stream);
    EXPECT_FALSE(up_to_date);
}

TEST_F(OutputReliableStreamTest, WriteOneMessageSameSlot)
{
    uint8_t* slot_0 = uxr_get_reliable_buffer(&stream.base, 0);

    ucdrBuffer ub;
    bool available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(0u, stream.last_written);
    EXPECT_EQ(OFFSET + SUBMESSAGE_SIZE, uxr_get_reliable_buffer_size(&stream.base, 0));
    EXPECT_EQ(slot_0, ub.init);
    EXPECT_EQ(slot_0 + OFFSET, ub.iterator);
    EXPECT_EQ(slot_0 + OFFSET + SUBMESSAGE_SIZE, ub.final);
}

TEST_F(OutputReliableStreamTest, WriteTwoMessagesSameSlot)
{
    uint8_t* slot_0 = uxr_get_reliable_buffer(&stream.base, 0);

    ucdrBuffer ub;
    bool available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(0u, stream.last_written);
    EXPECT_EQ(OFFSET + SUBMESSAGE_SIZE, uxr_get_reliable_buffer_size(&stream.base, 0));
    EXPECT_EQ(slot_0, ub.init);
    EXPECT_EQ(slot_0 + OFFSET, ub.iterator);
    EXPECT_EQ(slot_0 + OFFSET + SUBMESSAGE_SIZE, ub.final);

    available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(0u, stream.last_written);
    EXPECT_EQ(OFFSET + SUBMESSAGE_SIZE * 2, uxr_get_reliable_buffer_size(&stream.base, 0));
    EXPECT_EQ(slot_0, ub.init);
    EXPECT_EQ(slot_0 + OFFSET + SUBMESSAGE_SIZE, ub.iterator);
    EXPECT_EQ(slot_0 + OFFSET + SUBMESSAGE_SIZE * 2, ub.final);
}

TEST_F(OutputReliableStreamTest, WriteThreeMessagessTwoSlots)
{
    uint8_t* slot_0 = uxr_get_reliable_buffer(&stream.base, 0);
    uint8_t* slot_1 = uxr_get_reliable_buffer(&stream.base, 1);

    ucdrBuffer ub;
    bool available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(0u, stream.last_written);
    EXPECT_EQ(OFFSET + SUBMESSAGE_SIZE, uxr_get_reliable_buffer_size(&stream.base, 0));
    EXPECT_EQ(slot_0, ub.init);
    EXPECT_EQ(slot_0 + OFFSET, ub.iterator);
    EXPECT_EQ(slot_0 + OFFSET + SUBMESSAGE_SIZE, ub.final);

    available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(0u, stream.last_written);
    EXPECT_EQ(OFFSET + SUBMESSAGE_SIZE * 2, uxr_get_reliable_buffer_size(&stream.base, 0));
    EXPECT_EQ(slot_0, ub.init);
    EXPECT_EQ(slot_0 + OFFSET + SUBMESSAGE_SIZE, ub.iterator);
    EXPECT_EQ(slot_0 + OFFSET + SUBMESSAGE_SIZE * 2, ub.final);

    available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(1u, stream.last_written);
    EXPECT_EQ(OFFSET + SUBMESSAGE_SIZE * 2, uxr_get_reliable_buffer_size(&stream.base, 0));
    EXPECT_EQ(OFFSET + SUBMESSAGE_SIZE, uxr_get_reliable_buffer_size(&stream.base, 1));
    EXPECT_EQ(slot_1, ub.init);
    EXPECT_EQ(slot_1 + OFFSET, ub.iterator);
    EXPECT_EQ(slot_1 + OFFSET + SUBMESSAGE_SIZE, ub.final);
}

TEST_F(OutputReliableStreamTest, WriteFragmentMessage)
{
    size_t capacity = uxr_get_reliable_buffer_capacity(&stream.base);
    const size_t message_length = BUFFER_SIZE / HISTORY + SUBMESSAGE_SIZE;
    uint8_t* slot_0 = uxr_get_reliable_buffer(&stream.base, 0);
    uint8_t* slot_2 = uxr_get_reliable_buffer(&stream.base, 2);

    uxrOutputReliableStream backup;
    copy(&backup, &stream);

    ucdrBuffer ub;
    bool available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, message_length, &ub);
    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(2u, stream.last_written);
    EXPECT_EQ(capacity, uxr_get_reliable_buffer_size(&stream.base, 0));
    EXPECT_EQ(capacity, uxr_get_reliable_buffer_size(&stream.base, 1));
    EXPECT_EQ(OFFSET + FRAGMENT_OFFSET + SUBMESSAGE_SIZE, uxr_get_reliable_buffer_size(&stream.base, 2));
    EXPECT_EQ(slot_0 + OFFSET + FRAGMENT_OFFSET, ub.init);
    EXPECT_EQ(slot_0 + OFFSET + FRAGMENT_OFFSET, ub.iterator);
    EXPECT_EQ(slot_0 + capacity, ub.final);

    uint8_t message_to_write[message_length];
    EXPECT_TRUE(ucdr_serialize_array_uint8_t(&ub, message_to_write, message_length));
    EXPECT_EQ(slot_2 + OFFSET + FRAGMENT_OFFSET + SUBMESSAGE_SIZE, ub.iterator);
    EXPECT_EQ(slot_2 + OFFSET + FRAGMENT_OFFSET + SUBMESSAGE_SIZE, ub.final);
}

TEST_F(OutputReliableStreamTest, WriteTwoFragmentMessage)
{
    ucdrBuffer ub;
    bool available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, MAX_FRAGMENT_SIZE * 2, &ub);
    ASSERT_TRUE(available_to_write);
    available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, MAX_FRAGMENT_SIZE * 2, &ub);
    ASSERT_TRUE(available_to_write);
}


TEST_F(OutputReliableStreamTest, WriteMultipleFragmentsAndCheckSubHeaders)
{
    ucdrBuffer ub;
    uint8_t client_key[4] = {0xAA, 0xAA, 0xAA, 0xAA};

    // Init all message headers
    for (uint16_t i = 0; i < HISTORY; i++)
    {
        uint8_t* slot = uxr_get_reliable_buffer(&stream.base, i);
        ucdr_init_buffer(&ub, slot, MAX_MESSAGE_SIZE);
        uxr_serialize_message_header(&ub, 0, 0, 0, client_key);
    }

    // Writing two fragmented message, 3 slots should be used
    size_t first_message_size = 24; // 1.5 * MAX_FRAGMENT_SIZE;
    bool available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, first_message_size, &ub);
    ASSERT_TRUE(available_to_write);
    available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, first_message_size, &ub);
    ASSERT_TRUE(available_to_write);

    // Deserialize both messages
    size_t buffer_capacity = uxr_get_reliable_buffer_capacity(&stream.base);
    for (uint16_t i = 0; i < stream.last_written; i++)
    {
        uint8_t* slot = uxr_get_reliable_buffer(&stream.base, i);
        ucdr_init_buffer_origin_offset(&ub, slot, buffer_capacity, 0u, 0u);

        uint8_t session_id, stream_id;
        uint16_t seq_no;
        uint8_t output_client_key[4];
        uxr_deserialize_message_header(&ub, &session_id, &stream_id, &seq_no, output_client_key);

        while (ub.iterator < ub.final)
        {
            uint8_t id, flags;
            uint16_t length;
            uxr_deserialize_submessage_header(&ub, &id, &flags, &length);
            uint8_t* fragment = reinterpret_cast<uint8_t*>(malloc(length * sizeof(uint8_t)));
            ASSERT_TRUE(ucdr_deserialize_array_uint8_t(&ub, fragment, length));
            free(fragment);
        }
    }
}

TEST_F(OutputReliableStreamTest, WriteMaxSubmessageSize)
{
    ucdrBuffer ub;
    bool available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, 2 * MAX_FRAGMENT_SIZE, &ub);
    ASSERT_TRUE(available_to_write);
    EXPECT_EQ(MAX_MESSAGE_SIZE, uxr_get_reliable_buffer_size(&stream.base, 0));
    EXPECT_EQ(MAX_MESSAGE_SIZE, uxr_get_reliable_buffer_size(&stream.base, 1));
}

TEST_F(OutputReliableStreamTest, WriteMessagesUntilFullBuffer)
{
    ucdrBuffer ub;
    for (size_t i = 0; i < HISTORY; ++i)
    {
        bool available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, MAX_SUBMESSAGE_SIZE, &ub);
        ASSERT_TRUE(available_to_write);
    }

    uxrOutputReliableStream backup;
    copy(&backup, &stream);
    bool available_to_write = uxr_prepare_reliable_buffer_to_write(&stream, MAX_SUBMESSAGE_SIZE, &ub);
    ASSERT_FALSE(available_to_write);
    EXPECT_EQ(backup, stream);

    for (uint16_t i = 0; i < HISTORY; ++i)
    {
        EXPECT_EQ(MAX_MESSAGE_SIZE, uxr_get_reliable_buffer_size(&stream.base, i));
    }
}

TEST_F(OutputReliableStreamTest, PrepareToSendOneMessage)
{
    ucdrBuffer ub;
    (void) uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);

    uint8_t* slot_0 = uxr_get_reliable_buffer(&stream.base, 0);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    bool data_to_send = uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
    ASSERT_TRUE(data_to_send);
    EXPECT_EQ(slot_0, message);
    EXPECT_EQ(uxr_get_reliable_buffer_size(&stream.base, 0), length);
    EXPECT_EQ(uxr_seq_num_add(SEQ_NUM_MAX, 1), seq_num);
}

TEST_F(OutputReliableStreamTest, PrepareToSendNoMessage)
{
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    uxrOutputReliableStream backup;
    copy(&backup, &stream);
    bool data_to_send = uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
    ASSERT_FALSE(data_to_send);
    EXPECT_EQ(backup, stream);
}

TEST_F(OutputReliableStreamTest, PrepareToSendAllMessages)
{
    ucdrBuffer ub;
    for (size_t i = 0; i < HISTORY; ++i)
    {
        (void) uxr_prepare_reliable_buffer_to_write(&stream, MAX_SUBMESSAGE_SIZE, &ub);
    }

    for (uint16_t i = 0; i < HISTORY; ++i)
    {
        uint8_t* slot_i = uxr_get_reliable_buffer(&stream.base, i);
        uint8_t* message; size_t length; uxrSeqNum seq_num;
        bool data_to_send = uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
        ASSERT_TRUE(data_to_send);
        EXPECT_EQ(slot_i, message);
        EXPECT_EQ(uxr_get_reliable_buffer_size(&stream.base, i), length);
        EXPECT_EQ(uxr_seq_num_add(SEQ_NUM_MAX, uint16_t(i + 1)), seq_num);
    }

    uxrOutputReliableStream backup;
    copy(&backup, &stream);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    bool data_to_send = uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
    ASSERT_FALSE(data_to_send);
    EXPECT_EQ(backup, stream);

    for (uint16_t i = 0; i < HISTORY; ++i)
    {
        EXPECT_EQ(MAX_MESSAGE_SIZE, uxr_get_reliable_buffer_size(&stream.base, i));
    }
}

TEST_F(OutputReliableStreamTest, HeartbeatWithUpToDate)
{
    uxrOutputReliableStream backup;
    copy(&backup, &stream);
    bool must_confirm = uxr_update_output_stream_heartbeat_timestamp(&stream, 0);
    ASSERT_FALSE(must_confirm);
    EXPECT_EQ(backup, stream);
}

TEST_F(OutputReliableStreamTest, HeartbeatFirstTry)
{
    ucdrBuffer ub;
    (void) uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    (void) uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);

    bool must_confirm = uxr_update_output_stream_heartbeat_timestamp(&stream, 0);
    ASSERT_FALSE(must_confirm);
    EXPECT_EQ(MIN_HEARTBEAT_TIME_INTERVAL, stream.next_heartbeat_timestamp);
    EXPECT_EQ(1u, stream.next_heartbeat_tries);
}

TEST_F(OutputReliableStreamTest, HeartbeatSuccessfulTry)
{
    ucdrBuffer ub;
    (void) uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    (void) uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
    (void) uxr_update_output_stream_heartbeat_timestamp(&stream, 0);

    bool must_confirm = uxr_update_output_stream_heartbeat_timestamp(&stream, MIN_HEARTBEAT_TIME_INTERVAL);
    ASSERT_TRUE(must_confirm);
    EXPECT_EQ(MIN_HEARTBEAT_TIME_INTERVAL * 2, stream.next_heartbeat_timestamp);
    EXPECT_EQ(2u, stream.next_heartbeat_tries);
}

TEST_F(OutputReliableStreamTest, HeartbeatTwoSuccessfulTry)
{
    ucdrBuffer ub;
    (void) uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    (void) uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
    (void) uxr_update_output_stream_heartbeat_timestamp(&stream, 0);
    (void) uxr_update_output_stream_heartbeat_timestamp(&stream, MIN_HEARTBEAT_TIME_INTERVAL);

    bool must_confirm = uxr_update_output_stream_heartbeat_timestamp(&stream, MIN_HEARTBEAT_TIME_INTERVAL * 3);
    ASSERT_TRUE(must_confirm);
    EXPECT_EQ(MIN_HEARTBEAT_TIME_INTERVAL * 4, stream.next_heartbeat_timestamp);
    EXPECT_EQ(3, stream.next_heartbeat_tries);
}

TEST_F(OutputReliableStreamTest, HeartbeatUnsuccessfulSecondHeartbeat)
{
    ucdrBuffer ub;
    (void) uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    (void) uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
    (void) uxr_update_output_stream_heartbeat_timestamp(&stream, 0);
    (void) uxr_update_output_stream_heartbeat_timestamp(&stream, MIN_HEARTBEAT_TIME_INTERVAL * 3);

    bool must_confirm = uxr_update_output_stream_heartbeat_timestamp(&stream, MIN_HEARTBEAT_TIME_INTERVAL);
    ASSERT_FALSE(must_confirm);
    EXPECT_EQ(MIN_HEARTBEAT_TIME_INTERVAL * 2, stream.next_heartbeat_timestamp);
    EXPECT_EQ(2u, stream.next_heartbeat_tries);
}

TEST_F(OutputReliableStreamTest, AcknackProcessNoLost)
{
    ucdrBuffer ub;
    (void) uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    (void) uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
    (void) uxr_update_output_stream_heartbeat_timestamp(&stream, 0);

    uxr_process_acknack(&stream, 0, uxrSeqNum(1));
    EXPECT_FALSE(stream.send_lost);
    EXPECT_EQ(0u, stream.last_acknown);
    EXPECT_EQ(0u, stream.next_heartbeat_tries);
    EXPECT_EQ(OFFSET, uxr_get_reliable_buffer_size(&stream.base, 0));
}

TEST_F(OutputReliableStreamTest, AcknackProcessLost)
{
    ucdrBuffer ub;
    (void) uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    (void) uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
    (void) uxr_update_output_stream_heartbeat_timestamp(&stream, 0);
    size_t message_length = uxr_get_reliable_buffer_size(&stream.base, 0);

    uxr_process_acknack(&stream, 1, uxrSeqNum(0));
    EXPECT_TRUE(stream.send_lost);
    EXPECT_EQ(SEQ_NUM_MAX, stream.last_acknown);
    EXPECT_EQ(0u, stream.next_heartbeat_tries);
    EXPECT_EQ(message_length, uxr_get_reliable_buffer_size(&stream.base, 0));
}

TEST_F(OutputReliableStreamTest, SendMessageLostNoLost)
{
    uint8_t* lost_message; size_t lost_length; uxrSeqNum lost_seq_num_it;
    bool must_send = uxr_next_reliable_nack_buffer_to_send(&stream, &lost_message, &lost_length, &lost_seq_num_it);
    ASSERT_FALSE(must_send);
}

TEST_F(OutputReliableStreamTest, SendMessageLost)
{
    uint8_t* slot_0 = uxr_get_reliable_buffer(&stream.base, 0);
    ucdrBuffer ub;
    (void) uxr_prepare_reliable_buffer_to_write(&stream, SUBMESSAGE_SIZE, &ub);
    uint8_t* message; size_t length; uxrSeqNum seq_num;
    (void) uxr_prepare_next_reliable_buffer_to_send(&stream, &message, &length, &seq_num);
    (void) uxr_update_output_stream_heartbeat_timestamp(&stream, 0);
    uxr_process_acknack(&stream, 1, uxrSeqNum(0));
    //at this point, the lost has been confirmed

    uint8_t* lost_message; size_t lost_length;
    uxrSeqNum seq_num_it = uxr_begin_output_nack_buffer_it(&stream);
    bool must_send = uxr_next_reliable_nack_buffer_to_send(&stream, &lost_message, &lost_length, &seq_num_it);
    ASSERT_TRUE(must_send);
    EXPECT_EQ(slot_0, lost_message);
    EXPECT_EQ(OFFSET + SUBMESSAGE_SIZE, lost_length);
    EXPECT_EQ(0u, seq_num_it);

    must_send = uxr_next_reliable_nack_buffer_to_send(&stream, &lost_message, &lost_length, &seq_num_it);
    ASSERT_FALSE(must_send);
}

TEST_F(OutputReliableStreamTest, FragmentedSerialization)
{
    uint8_t* slot_1 = uxr_get_reliable_buffer(&stream.base, 1);

    ucdrBuffer ub;
    uint8_t message[MAX_MESSAGE_SIZE + 4];
    (void) uxr_prepare_reliable_buffer_to_write(&stream, MAX_SUBMESSAGE_SIZE + 4, &ub);
    bool serialize = ucdr_serialize_array_uint8_t(&ub, message, MAX_SUBMESSAGE_SIZE + 4);

    ASSERT_TRUE(serialize);
    EXPECT_EQ(slot_1 + uxr_get_reliable_buffer_size(&stream.base, 1), ub.iterator);
    EXPECT_EQ(slot_1 + uxr_get_reliable_buffer_size(&stream.base, 1), ub.final);
}

TEST_F(OutputReliableStreamTest, GetAvailableSeqNum)
{
    EXPECT_EQ(get_available_free_slots(&stream), HISTORY);

    ucdrBuffer ub;
    for (size_t i = 0; i < HISTORY; i++)
    {
        ASSERT_TRUE(uxr_prepare_reliable_buffer_to_write(&stream, MAX_SUBMESSAGE_SIZE, &ub));
        EXPECT_EQ(get_available_free_slots(&stream), HISTORY - (i + 1));
    }

    ASSERT_FALSE(uxr_prepare_reliable_buffer_to_write(&stream, MAX_SUBMESSAGE_SIZE, &ub));
    EXPECT_EQ(get_available_free_slots(&stream), 0);
}
