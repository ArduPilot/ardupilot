#include <gtest/gtest.h>

extern "C"
{
#include <c/core/serialization/xrce_types.c>
#include <c/core/serialization/xrce_header.c>
#include <c/core/serialization/xrce_subheader.c>
#include <c/core/session/object_id.c>
#include <c/core/session/submessage.c>
#include <c/core/session/session_info.c>
}

#define BUFFER_SIZE    size_t(512)

TEST(SessionInfoTest, CreateSessionSize)
{
    uint8_t buffer[BUFFER_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer, BUFFER_SIZE);

    uxrSessionInfo info;
    uxr_buffer_create_session(&info, &ub, 512);
    EXPECT_EQ(info.last_request_id, UXR_REQUEST_LOGIN);
    EXPECT_EQ(size_t(SUBHEADER_SIZE + CREATE_CLIENT_PAYLOAD_SIZE), ucdr_buffer_length(&ub));
}

TEST(SessionInfoTest, DeleteSessionSize)
{
    uint8_t buffer[BUFFER_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer, BUFFER_SIZE);

    uxrSessionInfo info;

    uxr_buffer_delete_session(&info, &ub);
    EXPECT_EQ(info.last_request_id, UXR_REQUEST_LOGOUT);
    EXPECT_EQ(size_t(SUBHEADER_SIZE + DELETE_CLIENT_PAYLOAD_SIZE), ucdr_buffer_length(&ub));
}

TEST(SessionInfoTest, RequestIdGeneration)
{
    uxrSessionInfo info;
    info.last_request_id = RESERVED_REQUESTS_ID;
    uint16_t request_id = generate_request_id(&info);
    EXPECT_EQ(RESERVED_REQUESTS_ID + 1, request_id);
    EXPECT_EQ(RESERVED_REQUESTS_ID + 1, info.last_request_id);

    info.last_request_id = 20;
    request_id = generate_request_id(&info);
    EXPECT_EQ(21, request_id);
    EXPECT_EQ(21, info.last_request_id);

    info.last_request_id = UINT16_MAX;
    request_id = generate_request_id(&info);
    EXPECT_EQ(RESERVED_REQUESTS_ID + 1, request_id);
    EXPECT_EQ(RESERVED_REQUESTS_ID + 1, info.last_request_id);
}

TEST(SessionInfoTest, BaseObjectRequestConversion)
{
    uxrSessionInfo info;
    info.last_request_id = RESERVED_REQUESTS_ID;

    uxrObjectId object_id = uxr_object_id(0x0AAA, 0x0B);
    BaseObjectRequest base = {};
    uint16_t request_id = uxr_init_base_object_request(&info, object_id, &base);

    uxrObjectId read_object_id = {};
    uint16_t read_request_id {};
    uxr_parse_base_object_request(&base, &read_object_id, &read_request_id);

    EXPECT_EQ(request_id, read_request_id);
    EXPECT_EQ(info.last_request_id, read_request_id);
    EXPECT_EQ(object_id.id, read_object_id.id);
    EXPECT_EQ(object_id.type, read_object_id.type);
}

TEST(SessionInfoTest, SessionHeaderOffset)
{
    uxrSessionInfo info;
    info.id = 0x79;
    EXPECT_EQ(uxr_session_header_offset(&info), MAX_HEADER_SIZE);

    info.id = 0x80;
    EXPECT_EQ(uxr_session_header_offset(&info), MIN_HEADER_SIZE);
}

TEST(SessionInfoTest, WriteReadSessionHeaderWithKey)
{
    uint8_t buffer[BUFFER_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer, BUFFER_SIZE);

    uxrSessionInfo info;
    uint8_t id = 0x01;
    uxr_init_session_info(&info, id, 0xAABBCCDD);
    uint8_t stream_id = 1;
    uxrSeqNum seq_num = 2;
    uxr_stamp_session_header(&info, stream_id, seq_num, buffer);

    uint8_t read_stream_id = 0;
    uxrSeqNum read_seq_num = 0;
    ASSERT_TRUE(uxr_read_session_header(&info, &ub, &read_stream_id, &read_seq_num));
    EXPECT_EQ(info.id, id);
    EXPECT_EQ(stream_id, read_stream_id);
    EXPECT_EQ(seq_num, read_seq_num);
    EXPECT_EQ(size_t(MAX_HEADER_SIZE), ucdr_buffer_length(&ub));
}

TEST(SessionInfoTest, WriteReadSessionHeaderWithoutKey)
{
    uint8_t buffer[BUFFER_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer, BUFFER_SIZE);

    uxrSessionInfo info;
    uint8_t id = 0x81;
    uxr_init_session_info(&info, id, 0xAABBCCDD);
    uint8_t stream_id = 1;
    uxrSeqNum seq_num = 2;
    uxr_stamp_session_header(&info, stream_id, seq_num, buffer);

    uint8_t read_stream_id = 0;
    uxrSeqNum read_seq_num = 0;
    ASSERT_TRUE(uxr_read_session_header(&info, &ub, &read_stream_id, &read_seq_num));
    EXPECT_EQ(info.id, id);
    EXPECT_EQ(stream_id, read_stream_id);
    EXPECT_EQ(seq_num, read_seq_num);
    EXPECT_EQ(size_t(MIN_HEADER_SIZE), ucdr_buffer_length(&ub));
}
