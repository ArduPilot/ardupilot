#include <chrono>
#include <thread>

extern "C"
{
#include <c/core/serialization/xrce_types.c>
#include <c/core/serialization/xrce_header.c>
#include <c/core/serialization/xrce_subheader.c>

#include <c/core/session/stream/seq_num.c>
#include <c/core/session/stream/stream_id.c>
#include <c/core/session/stream/stream_storage.c>
#include <c/core/session/stream/input_best_effort_stream.c>
#include <c/core/session/stream/output_best_effort_stream.c>
#include <c/core/session/stream/input_reliable_stream.c>
#include <c/core/session/stream/output_reliable_stream.c>

#include <c/core/session/object_id.c>
#include <c/core/session/submessage.c>
#include <c/core/session/session_info.c>
#include <c/core/session/read_access.c>
#include <c/core/session/write_access.c>

#include <c/util/time.c>

#undef UXR_MESSAGE_LOG
#undef UXR_SERIALIZATION_LOG
#include <c/core/session/session.c>
}

#include <gtest/gtest.h>
#include <string>
#include <array>
#include <vector>
#include <algorithm>

#define MTU                   64
#define HISTORY               4
#define OFFSET                4
#define DATA_PAYLOAD_SIZE     4


class SessionTest : public testing::Test
{
public:

    static SessionTest* current;
    SessionTest()
    {
        //Necessary for the mocks
        current = this;

        comm.instance = this;
        comm.mtu = 32;
        comm.send_msg = send_msg;
        comm.recv_msg = recv_msg;
        comm.comm_error = comm_error;

        uxr_init_session(&session, &comm, 0xAAAABBBB);

        EXPECT_EQ(&comm, session.comm);
        EXPECT_EQ(NULL, session.request_list);
        EXPECT_EQ(NULL, session.status_list);
        EXPECT_EQ(size_t(0), session.request_status_list_size);

        EXPECT_EQ(NULL, session.on_status);
        EXPECT_EQ(NULL, session.on_status_args);
        EXPECT_EQ(NULL, session.on_topic);
        EXPECT_EQ(NULL, session.on_topic_args);


        uxrStreamId id = uxr_create_input_best_effort_stream(&session);
        EXPECT_EQ(0u, id.index);
        EXPECT_EQ(BEST_EFFORT_STREAM_THRESHOLD, id.raw);
        EXPECT_EQ(UXR_BEST_EFFORT_STREAM, id.type);
        EXPECT_EQ(UXR_INPUT_STREAM, id.direction);

        id = uxr_create_output_best_effort_stream(&session, output_best_effort_buffer, MTU);
        EXPECT_EQ(0u, id.index);
        EXPECT_EQ(BEST_EFFORT_STREAM_THRESHOLD, id.raw);
        EXPECT_EQ(UXR_BEST_EFFORT_STREAM, id.type);
        EXPECT_EQ(UXR_OUTPUT_STREAM, id.direction);

        id = uxr_create_input_reliable_stream(&session, input_reliable_buffer, MTU * HISTORY, HISTORY);
        EXPECT_EQ(0u, id.index);
        EXPECT_EQ(RELIABLE_STREAM_THRESHOLD, id.raw);
        EXPECT_EQ(UXR_RELIABLE_STREAM, id.type);
        EXPECT_EQ(UXR_INPUT_STREAM, id.direction);

        id = uxr_create_output_reliable_stream(&session, output_reliable_buffer, MTU * HISTORY, HISTORY);
        EXPECT_EQ(0u, id.index);
        EXPECT_EQ(RELIABLE_STREAM_THRESHOLD, id.raw);
        EXPECT_EQ(UXR_RELIABLE_STREAM, id.type);
        EXPECT_EQ(UXR_OUTPUT_STREAM, id.direction);
    }

public:

    uxrCommunication comm;
    uxrSession session;
    uint8_t output_best_effort_buffer[MTU];
    uint8_t output_reliable_buffer[MTU * HISTORY];
    uint8_t input_reliable_buffer[MTU * HISTORY];

    static int listening_counter;

    static bool send_msg(
            void* instance,
            const uint8_t* buf,
            size_t len)
    {
        (void) buf;
        EXPECT_EQ(SessionTest::current, instance);
        if (std::string("FlashStreams") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            EXPECT_EQ(size_t(OFFSET + SUBHEADER_SIZE + 8), len);
        }
        else if (std::string("SendMessageOk") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            EXPECT_EQ(size_t(MTU), len);
        }
        else if (std::string("SendMessageError") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            EXPECT_EQ(size_t(MTU), len);
            return false;
        }
        else if (std::string("SendHeartbeat") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            EXPECT_EQ(size_t(HEARTBEAT_MAX_MSG_SIZE - (MAX_HEADER_SIZE - OFFSET)), len);
        }
        else if (std::string("SendAcknack") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            EXPECT_EQ(size_t(ACKNACK_MAX_MSG_SIZE - (MAX_HEADER_SIZE - OFFSET)), len);
        }

        return true;
    }

    static bool recv_msg(
            void* instance,
            uint8_t** buf,
            size_t* len,
            int timeout)
    {
        EXPECT_EQ(SessionTest::current, instance);
        (void) timeout;
        static std::array<uint8_t, MTU> input_buffer;

        if (std::string("CreateOk") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            std::vector<uint8_t> message = {0x81, 0x00, 0x00, 0x00, 0x04, 0x01, 0x19, 0x00,
                                            0x00, 0x01, 0xFF, 0xFE, 0x00, 0x00, 0x58, 0x52,
                                            0x43, 0x45, 0x01, 0x00, 0x01, 0x0F, 0x0, 0x00,
                                            0x77, 0x6B, 0x48, 0x5C, 0x43, 0x14, 0x1C, 0x34,
                                            0x00};
            *len = message.size();
            *buf = input_buffer.data();
            std::copy_n(message.begin(), *len, input_buffer.begin());
            return true;
        }
        else if (std::string("CreateNoOk") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            return false;
        }
        else if (std::string("DeleteOk") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            std::vector<uint8_t> message = {0x81, 0x00, 0x00, 0x00, 0x05, 0x01, 0x06, 0x00,
                                            0x00, 0x02, 0xFF, 0xFE, 0x00, 0x00};
            *len = message.size();
            *buf = input_buffer.data();
            std::copy_n(message.begin(), *len, input_buffer.begin());
            return true;
        }
        else if (std::string("DeleteNoOk") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            return false;
        }
        else if (std::string("Listen") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            *len = 0u;
            *buf = NULL;
            return true;
        }
        else if (std::string("ListenTimeout") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            return false;
        }
        else if (std::string("ListenReliably") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            *len = 0u;
            *buf = NULL;
            return true;
        }
        else if (std::string("ListenReliablyTimeout") ==
                ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            return false;
        }
        else if (std::string("WaitSessionStatusBad") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            SessionTest::listening_counter++;
            return false;
        }
        else if (std::string("RecvMessageOk") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            *len = 8;
            return true;
        }
        else if (std::string("RecvMessageError") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            return false;
        }
        return false;
    }

    static uint8_t comm_error(
            void)
    {
        return 0u;
    }

    static void on_status_func (
            struct uxrSession* session,
            uxrObjectId object_id,
            uint16_t request_id,
            uint8_t status,
            void* args)
    {
        (void) session; (void) object_id; (void) request_id; (void) status; (void) args;
        if (std::string("ProcessStatus") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            EXPECT_EQ(&SessionTest::current->session, session);
            EXPECT_EQ(2, object_id.id);
            EXPECT_EQ(2, object_id.type);
            EXPECT_EQ(4, request_id);
            EXPECT_EQ(UXR_STATUS_OK, status);
            EXPECT_EQ(&SessionTest::current->session, args);
        }
    }

    static void on_topic_func (
            struct uxrSession* session,
            uxrObjectId object_id,
            uint16_t request_id,
            uxrStreamId stream_id,
            struct ucdrBuffer* ub,
            uint16_t length,
            void* args)
    {
        (void) session; (void) object_id; (void) request_id; (void) stream_id; (void) length; (void) args;
        if (std::string("ReadUint64") == ::testing::UnitTest::GetInstance()->current_test_info()->name())
        {
            uint64_t data;
            ucdr_deserialize_uint64_t(ub, &data);
            EXPECT_EQ(data, UINT64_MAX);
        }
    }

    static void on_time_func (
            struct uxrSession* session,
            int64_t current_timestamp,
            int64_t transmit_timestamp,
            int64_t received_timestamp,
            int64_t originate_timestamp,
            void* args)
    {
        (void) session; (void) current_timestamp; (void) transmit_timestamp; (void) received_timestamp;
        (void) originate_timestamp; (void) args;
    }

};

SessionTest* SessionTest::current = nullptr;
int SessionTest::listening_counter;

TEST_F(SessionTest, SetStatusCallback)
{
    int user_data;
    uxr_set_status_callback(&session, on_status_func, &user_data);
    EXPECT_EQ(reinterpret_cast<void*>(session.on_status), reinterpret_cast<void*>(on_status_func));
    EXPECT_EQ(session.on_status_args, &user_data);
}

TEST_F(SessionTest, SetTopicCallback)
{
    int user_data;
    uxr_set_topic_callback(&session, on_topic_func, &user_data);
    EXPECT_EQ(reinterpret_cast<void*>(session.on_topic), reinterpret_cast<void*>(on_topic_func));
    EXPECT_EQ(session.on_topic_args, &user_data);
}

TEST_F(SessionTest, SetTimeCallback)
{
    int user_data;
    uxr_set_time_callback(&session, on_time_func, &user_data);
    EXPECT_EQ(reinterpret_cast<void*>(session.on_time), reinterpret_cast<void*>(on_time_func));
    EXPECT_EQ(session.on_time_args, &user_data);
}

TEST_F(SessionTest, CreateOk)
{
    bool created = uxr_create_session(&session);
    ASSERT_TRUE(created);
}

TEST_F(SessionTest, CreateNoOk)
{
    bool created = uxr_create_session(&session);
    ASSERT_FALSE(created);
}

TEST_F(SessionTest, DeleteOk)
{
    bool deleted = uxr_delete_session(&session);
    ASSERT_TRUE(deleted);
}

TEST_F(SessionTest, DeleteNoOk)
{
    bool deleted = uxr_delete_session(&session);
    ASSERT_FALSE(deleted);
}

TEST_F(SessionTest, Listen)
{
    bool must_be_read = listen_message(&session, 1000);
    ASSERT_TRUE(must_be_read);
}

TEST_F(SessionTest, ListenTimeout)
{
    bool must_be_read = listen_message(&session, 1000);
    ASSERT_FALSE(must_be_read);
}

TEST_F(SessionTest, ListenReliably)
{
    bool must_be_read = listen_message_reliably(&session, 1000);
    ASSERT_TRUE(must_be_read);
}

TEST_F(SessionTest, ListenReliablyTimeout)
{
    bool must_be_read = listen_message_reliably(&session, 1000);
    ASSERT_FALSE(must_be_read);
}

TEST_F(SessionTest, FlashStreams)
{
    ucdrBuffer ub;
    uxrStreamId output_best_effort = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    uxrStreamId output_reliable = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);
    (void) uxr_prepare_stream_to_write_submessage(&session, output_reliable, 8, &ub, 1, 0);
    (void) uxr_prepare_stream_to_write_submessage(&session, output_best_effort, 8, &ub, 1, 0);
    uxr_flash_output_streams(&session);
}

TEST_F(SessionTest, WaitSessionStatusBad)
{
    // The OK version is already checked with the CreateOk and DeleteOk test versions
    SessionTest::listening_counter = 0;
    uint8_t buffer[MTU];
    size_t length = 0;
    size_t attempts = 10;
    int64_t start_timestamp = uxr_millis();
    bool found = wait_session_status(&session, buffer, length, attempts);
    EXPECT_FALSE(found);
    EXPECT_GE(size_t(SessionTest::listening_counter), attempts);

    // Check elapsed time with 100 ms tolerance
    int64_t final_time = uxr_millis() - start_timestamp;
    int64_t expected_time = (int64_t) (UXR_CONFIG_MIN_SESSION_CONNECTION_INTERVAL * attempts);
    EXPECT_NEAR((double) final_time, (double) expected_time, 100);
}

TEST_F(SessionTest, SendMessageOk)
{
    uint8_t buffer[MTU];
    bool sent = send_message(&session, buffer, MTU);
    ASSERT_TRUE(sent);
}

TEST_F(SessionTest, SendMessageError)
{
    uint8_t buffer[MTU];
    bool sent = send_message(&session, buffer, MTU);
    ASSERT_FALSE(sent);
}

TEST_F(SessionTest, RecvMessageOk)
{
    uint8_t* buffer; size_t length;
    bool received = recv_message(&session, &buffer, &length, 0);
    ASSERT_TRUE(received);
    EXPECT_EQ(8u, length);
}

TEST_F(SessionTest, RecvMessageError)
{
    uint8_t* buffer; size_t length;
    bool received = recv_message(&session, &buffer, &length, 0);
    ASSERT_FALSE(received);
}

TEST_F(SessionTest, SendHeartbeat)
{
    uxrStreamId output_reliable = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);
    write_submessage_heartbeat(&session, output_reliable);
}

TEST_F(SessionTest, SendAcknack)
{
    uxrStreamId input_reliable = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_INPUT_STREAM);
    write_submessage_acknack(&session, input_reliable);
}

TEST_F(SessionTest, ProcessStatus)
{
    uxr_set_status_callback(&session, on_status_func, &session);
    process_status(&session, uxr_object_id(2, 2), 4, UXR_STATUS_OK);
}

TEST_F(SessionTest, WriteBestEffortOk)
{
    ucdrBuffer ub;
    uxrStreamId output_best_effort = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    bool available = uxr_prepare_stream_to_write_submessage(&session, output_best_effort, 8, &ub, 1, 0);
    ASSERT_TRUE(available);
}

TEST_F(SessionTest, WriteBestEffortTooLong)
{
    ucdrBuffer ub;
    uxrStreamId output_best_effort = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    bool available = uxr_prepare_stream_to_write_submessage(&session, output_best_effort, MTU, &ub, 1, 0);
    ASSERT_FALSE(available);
}

TEST_F(SessionTest, WriteReliableOk)
{
    ucdrBuffer ub;
    uxrStreamId output_reliable = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);
    bool available = uxr_prepare_stream_to_write_submessage(&session, output_reliable, 8, &ub, 1, 0);
    ASSERT_TRUE(available);
}

TEST_F(SessionTest, WriteReliableFragment)
{
    ucdrBuffer ub;
    uxrStreamId output_reliable = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);
    bool available = uxr_prepare_stream_to_write_submessage(&session, output_reliable, MTU, &ub, 1, 0);
    ASSERT_TRUE(available);
}

TEST_F(SessionTest, FragmentationInfoNoFragment)
{
    ucdrBuffer ub;
    std::array<uint8_t, SUBHEADER_SIZE> frag_header;
    ucdr_init_buffer(&ub, frag_header.data(), SUBHEADER_SIZE);
    uxr_buffer_submessage_header(&ub, 0, 0, 0);
    FragmentationInfo info = on_get_fragmentation_info(frag_header.data());
    EXPECT_EQ(NO_FRAGMENTED, info);
}

TEST_F(SessionTest, FragmentationInfoIntermediateFragment)
{
    ucdrBuffer ub;
    std::array<uint8_t, SUBHEADER_SIZE> frag_header;
    ucdr_init_buffer(&ub, frag_header.data(), SUBHEADER_SIZE);
    uxr_buffer_submessage_header(&ub, SUBMESSAGE_ID_FRAGMENT, 0, 0);
    FragmentationInfo info = on_get_fragmentation_info(frag_header.data());
    EXPECT_EQ(INTERMEDIATE_FRAGMENT, info);
}

TEST_F(SessionTest, FragmentationInfoLastFragment)
{
    ucdrBuffer ub;
    std::array<uint8_t, SUBHEADER_SIZE> frag_header;
    ucdr_init_buffer(&ub, frag_header.data(), SUBHEADER_SIZE);
    uxr_buffer_submessage_header(&ub, SUBMESSAGE_ID_FRAGMENT, 0, FLAG_LAST_FRAGMENT);
    FragmentationInfo info = on_get_fragmentation_info(frag_header.data());
    EXPECT_EQ(LAST_FRAGMENT, info);
}

TEST_F(SessionTest, ReadUint64)
{
    uxr_set_topic_callback(&session, on_topic_func, nullptr);

    std::array<uint8_t, 32> buffer;
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer.data(), buffer.size());
    uxr_serialize_message_header(&ub, session.info.id, UXR_NONE_STREAM, 0x00, session.info.key);
    uxr_serialize_submessage_header(
        &ub,
        SUBMESSAGE_ID_DATA,
        UCDR_MACHINE_ENDIANNESS,
        DATA_PAYLOAD_SIZE + sizeof(uint64_t));

    DATA_Payload_Data payload{};
    uxr_serialize_DATA_Payload_Data(&ub, &payload);

    ucdr_init_buffer(&ub, ub.iterator, size_t(ub.final - ub.iterator));
    ucdr_serialize_uint64_t(&ub, UINT64_MAX);

    ucdr_init_buffer(&ub, buffer.data(), size_t(ub.iterator - ub.init));
    read_message(&session, &ub);
}

TEST_F(SessionTest, WriteUint64)
{
    ucdrBuffer written_ub;
    uxrStreamId output_reliable = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);
    uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    uxr_prepare_output_stream(&session, output_reliable, datawriter_id, &written_ub, sizeof(uint64_t));
    ucdr_serialize_uint64_t(&written_ub, UINT64_MAX);

    ucdrBuffer expected_ub;
    ucdr_init_buffer(&expected_ub, written_ub.init, size_t(written_ub.iterator - written_ub.init));

    uxr_run_session_time(&session, 1);

    uint64_t data;
    ucdr_deserialize_uint64_t(&expected_ub, &data);
    EXPECT_EQ(data, UINT64_MAX);
}
