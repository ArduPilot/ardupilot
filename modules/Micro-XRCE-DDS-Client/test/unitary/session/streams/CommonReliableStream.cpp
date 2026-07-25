#include <gtest/gtest.h>

#include <c/core/session/stream/common_reliable_stream_internal.h>

#define BUFFER_SIZE size_t(512)
#define HISTORY     size_t(8)

uint8_t stream_buffer[BUFFER_SIZE];
uxrReliableStream stream{stream_buffer, BUFFER_SIZE, HISTORY};

TEST(CommonReliableStreamTest, GetBufferSize)
{
    size_t buffer_capacity = uxr_get_reliable_buffer_capacity(&stream);
    EXPECT_EQ(BUFFER_SIZE / HISTORY - INTERNAL_RELIABLE_BUFFER_OFFSET, buffer_capacity);
}

TEST(CommonReliableStreamTest, GetBuffer)
{
    uint16_t history_pos = 3;
    uint8_t* buffer = uxr_get_reliable_buffer(&stream, history_pos);

    size_t slot_size = BUFFER_SIZE / HISTORY;
    EXPECT_EQ(history_pos * slot_size + INTERNAL_RELIABLE_BUFFER_OFFSET, size_t(buffer - stream_buffer));
}

TEST(CommonReliableStreamTest, SetGetBufferLength)
{
    size_t input_length = 0xFFFF;
    uxr_set_reliable_buffer_size(&stream, 0, input_length);
    size_t output_length = uxr_get_reliable_buffer_size(&stream, 0);
    EXPECT_EQ(input_length, output_length);
}

