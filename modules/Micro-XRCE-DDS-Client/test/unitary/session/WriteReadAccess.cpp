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
#include <c/core/session/session.c>

#include <c/util/time.c>
}

#include <gtest/gtest.h>

#define MTU     64
#define HISTORY 4
#define TOPIC_FITTED_SIZE   (MTU - (MIN_HEADER_SIZE + SUBHEADER_SIZE + WRITE_DATA_PAYLOAD_SIZE))
#define REQUEST_FITTED_SIZE TOPIC_FITTED_SIZE
#define REPLY_FITTED_SIZE   (MTU - (MIN_HEADER_SIZE + SUBHEADER_SIZE + WRITE_DATA_PAYLOAD_SIZE) - \
    sizeof(SampleIdentity))

class WriteReadAccessTest : public testing::Test
{
public:

    WriteReadAccessTest()
        : session_{}
        , data_writer_id_{uxr_object_id(0, UXR_DATAWRITER_ID)}
        , data_reader_id_{uxr_object_id(0, UXR_DATAREADER_ID)}
        , requester_id_{uxr_object_id(0, UXR_REQUESTER_ID)}
        , replier_id_{uxr_object_id(0, UXR_REPLIER_ID)}
        , topic_{}
        , request_{}
        , reply_{}
        , output_best_effort_buffer_{}
    {
        uxr_init_session(&session_, NULL, 0xAAAABBBB);
        uxr_create_output_best_effort_stream(&session_, output_best_effort_buffer_, MTU);
        uxr_set_topic_callback(&session_, on_topic_func, &topic_);
        uxr_set_reply_callback(&session_, on_reply_func, &reply_);
        uxr_set_request_callback(&session_, on_request_func, &request_);
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
        uint64_t topic_sent = *reinterpret_cast<uint64_t*>(args);
        uint64_t topic_received;
        ucdr_deserialize_uint64_t(ub, &topic_received);
        EXPECT_EQ(topic_sent, topic_received);
    }

    static void on_reply_func (
            struct uxrSession* session,
            uxrObjectId object_id,
            uint16_t request_id,
            uint16_t reply_id,
            struct ucdrBuffer* ub,
            uint16_t length,
            void* args)
    {
        (void) session; (void) object_id; (void) request_id; (void) reply_id; (void) length; (void) args;
        uint64_t reply_sent = *reinterpret_cast<uint64_t*>(args);
        uint64_t reply_received;
        ucdr_deserialize_uint64_t(ub, &reply_received);
        EXPECT_EQ(reply_sent, reply_received);
    }

    static void on_request_func (
            struct uxrSession* session,
            uxrObjectId object_id,
            uint16_t request_id,
            SampleIdentity* sample_id,
            struct ucdrBuffer* ub,
            uint16_t length,
            void* args)
    {
        (void) session; (void) object_id; (void) request_id; (void) sample_id; (void) length; (void) args;
        uint32_t* data_ptr = reinterpret_cast<uint32_t*>(args);
        uint32_t request_sent[2] = {*data_ptr, *(data_ptr + 1)};
        uint32_t request_received[2];
        ucdr_deserialize_array_uint32_t(ub, request_received, 2);
        EXPECT_EQ(request_sent[0], request_received[0]);
        EXPECT_EQ(request_sent[1], request_received[1]);
    }

protected:

    uxrSession session_;
    uxrObjectId data_writer_id_;
    uxrObjectId data_reader_id_;
    uxrObjectId requester_id_;
    uxrObjectId replier_id_;
    uint64_t topic_;
    uint32_t request_[2];
    uint64_t reply_;

private:

    uint8_t output_best_effort_buffer_[MTU];
};

TEST_F(WriteReadAccessTest, PrepareStreamToWriteSubmessage)
{
    ucdrBuffer ub;
    uxrStreamId stream_id;
    uint16_t data_length;

    // stream_id:   no valid
    // data_length: fitted
    // expected:    false
    stream_id = uxr_stream_id(1, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(TOPIC_FITTED_SIZE);
    ASSERT_FALSE(uxr_prepare_output_stream(&session_, stream_id, data_writer_id_, &ub, data_length));

    // stream_id:   valid
    // data_length: no fitted
    // expected:    false
    stream_id = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(1 + TOPIC_FITTED_SIZE);
    ASSERT_FALSE(uxr_prepare_output_stream(&session_, stream_id, data_writer_id_, &ub, data_length));

    // stream_id:   valid
    // data_length: fitted
    // expected:    true
    stream_id = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(TOPIC_FITTED_SIZE);
    ASSERT_TRUE(uxr_prepare_output_stream(&session_, stream_id, data_writer_id_, &ub, data_length));
}

TEST_F(WriteReadAccessTest, BufferRequest)
{
    uint8_t buffer[MTU];
    uxrStreamId stream_id;
    uint16_t data_length;

    // stream_id:   no valid
    // data_length: fitted
    // expected:    false
    stream_id = uxr_stream_id(1, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(REQUEST_FITTED_SIZE);
    ASSERT_FALSE(uxr_buffer_request(&session_, stream_id, requester_id_, buffer, data_length));

    // stream_id:   valid
    // data_length: no fitted
    // expected:    false
    stream_id = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(1 + REQUEST_FITTED_SIZE);
    ASSERT_FALSE(uxr_buffer_request(&session_, stream_id, requester_id_, buffer, data_length));

    // stream_id:   valid
    // data_length: fitted
    // expected:    true
    stream_id = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(REQUEST_FITTED_SIZE);
    ASSERT_TRUE(uxr_buffer_request(&session_, stream_id, requester_id_, buffer, data_length));

}

TEST_F(WriteReadAccessTest, BufferReply)
{
    uint8_t buffer[MTU];
    SampleIdentity sample_id{};
    uxrStreamId stream_id;
    uint16_t data_length;

    // stream_id:   no valid
    // data_length: fitted
    // expected:    false
    stream_id = uxr_stream_id(1, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(REPLY_FITTED_SIZE);
    ASSERT_FALSE(uxr_buffer_reply(&session_, stream_id, requester_id_, &sample_id, buffer, data_length));

    // stream_id:   valid
    // data_length: no fitted
    // expected:    false
    stream_id = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(1 + REPLY_FITTED_SIZE);
    ASSERT_FALSE(uxr_buffer_reply(&session_, stream_id, requester_id_, &sample_id, buffer, data_length));


    // stream_id:   valid
    // data_length: fitted
    // expected:    true
    stream_id = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(REPLY_FITTED_SIZE);
    ASSERT_TRUE(uxr_buffer_reply(&session_, stream_id, replier_id_, &sample_id, buffer, data_length));
}

TEST_F(WriteReadAccessTest, BufferTopic)
{
    uint8_t buffer[MTU];
    uxrStreamId stream_id;
    uint16_t data_length;

    // stream_id:   no valid
    // data_length: fitted
    // expected:    false
    stream_id = uxr_stream_id(1, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(TOPIC_FITTED_SIZE);
    ASSERT_FALSE(uxr_buffer_topic(&session_, stream_id, data_writer_id_, buffer, data_length));

    // stream_id:   valid
    // data_length: no fitted
    // expected:    false
    stream_id = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(1 + TOPIC_FITTED_SIZE);
    ASSERT_FALSE(uxr_buffer_topic(&session_, stream_id, data_writer_id_, buffer, data_length));

    // stream_id:   valid
    // data_length: fitted
    // expected:    true
    stream_id = uxr_stream_id(0, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
    data_length = uint16_t(TOPIC_FITTED_SIZE);
    ASSERT_TRUE(uxr_buffer_topic(&session_, stream_id, data_writer_id_, buffer, data_length));
}

TEST_F(WriteReadAccessTest, ReadFormatData)
{
    uint8_t buffer[MTU] = {0};
    ucdrBuffer ub;
    uxrStreamId stream_id{};
    uint16_t request_id = 0;
    size_t expected_offset;
    BaseObjectRequest base_object_request{};
    SampleIdentity sample_identity{};
    uint16_t length;

    // data:        topic_ = 0x11
    // length:      sizeof(topic_)
    // object_id:   data_reader_id_
    topic_ = 0x11;
    length = sizeof(topic_);
    ucdr_init_buffer(&ub, buffer, sizeof(buffer));
    ucdr_serialize_uint64_t(&ub, topic_);
    expected_offset = ub.offset;
    ucdr_reset_buffer(&ub);
    read_format_data(&session_, &ub, length, stream_id, data_reader_id_, request_id);
    EXPECT_EQ(ub.offset, expected_offset);

    // data:        reply_ = {0x0123456789ABCDEF}
    // length:      sizeof(base_object_request) + sizeof(request_)
    // object_id:   requester_id_
    reply_ = 0x0123456789ABCDEF;
    length = sizeof(base_object_request) + sizeof(request_);
    ucdr_reset_buffer(&ub);
    uxr_serialize_BaseObjectRequest(&ub, &base_object_request);
    expected_offset = ub.offset;
    ucdr_init_buffer(&ub, buffer + ub.offset, sizeof(buffer) - ub.offset);
    ucdr_serialize_uint64_t(&ub, reply_);
    expected_offset += ub.offset;
    ucdr_init_buffer(&ub, buffer, sizeof(buffer));
    read_format_data(&session_, &ub, length, stream_id, requester_id_, request_id);
    EXPECT_EQ(ub.offset, expected_offset);

    // data:        request_ = {0x01234567, 0x89ABCDEF}
    // length:      sizeof(sample_identity) + sizeof(reply_)
    // object_id:   data_writer_id_
    request_[0] = 0x01234567;
    request_[1] = 0x89ABCDEF;
    length = sizeof(sample_identity) + sizeof(reply_);
    ucdr_serialize_array_uint32_t(&ub, request_, 2);
    ucdr_init_buffer(&ub, buffer, sizeof(buffer));
    uxr_serialize_SampleIdentity(&ub, &sample_identity);
    expected_offset = ub.offset;
    ucdr_init_buffer(&ub, buffer + ub.offset, sizeof(buffer) - ub.offset);
    ucdr_serialize_array_uint32_t(&ub, request_, 2);
    expected_offset += ub.offset;
    ucdr_init_buffer(&ub, buffer, sizeof(buffer));
    read_format_data(&session_, &ub, length, stream_id, replier_id_, request_id);
    EXPECT_EQ(ub.offset, expected_offset);
}