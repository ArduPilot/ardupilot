#include <gtest/gtest.h>

extern "C"
{
#include <c/core/session/stream/stream_id.c>
}

TEST(StreamIdTest, InitializationNone)
{
    uxrStreamId stream_id = uxr_stream_id(0, UXR_NONE_STREAM, UXR_OUTPUT_STREAM);
    EXPECT_EQ(uint8_t(0), stream_id.index);
    EXPECT_EQ(UXR_NONE_STREAM, stream_id.type);
    EXPECT_EQ(UXR_OUTPUT_STREAM, stream_id.direction);
    EXPECT_EQ(uint8_t(0), stream_id.raw);
}

TEST(StreamIdTest, InitializationBestEffort)
{
    uint8_t index = 2;
    uxrStreamId stream_id = uxr_stream_id(index, UXR_BEST_EFFORT_STREAM, UXR_INPUT_STREAM);
    EXPECT_EQ(index, stream_id.index);
    EXPECT_EQ(UXR_BEST_EFFORT_STREAM, stream_id.type);
    EXPECT_EQ(UXR_INPUT_STREAM, stream_id.direction);
    EXPECT_EQ(index + BEST_EFFORT_STREAM_THRESHOLD, stream_id.raw);
}

TEST(StreamIdTest, InitializationReliable)
{
    uint8_t index = 2;
    uxrStreamId stream_id = uxr_stream_id(index, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);
    EXPECT_EQ(index, stream_id.index);
    EXPECT_EQ(UXR_RELIABLE_STREAM, stream_id.type);
    EXPECT_EQ(UXR_OUTPUT_STREAM, stream_id.direction);
    EXPECT_EQ(index + RELIABLE_STREAM_THRESHOLD, stream_id.raw);
}

TEST(StreamIdTest, FromRawNone)
{
    uxrStreamId stream_id = uxr_stream_id_from_raw(0, UXR_INPUT_STREAM);
    EXPECT_EQ(uint8_t(0), stream_id.index);
    EXPECT_EQ(UXR_NONE_STREAM, stream_id.type);
    EXPECT_EQ(UXR_INPUT_STREAM, stream_id.direction);
    EXPECT_EQ(uint8_t(0), stream_id.raw);
}

TEST(StreamIdTest, FromRawBestEffort)
{
    uint8_t raw = 3;
    uxrStreamId stream_id = uxr_stream_id_from_raw(raw, UXR_OUTPUT_STREAM);
    EXPECT_EQ(raw - BEST_EFFORT_STREAM_THRESHOLD, stream_id.index);
    EXPECT_EQ(UXR_BEST_EFFORT_STREAM, stream_id.type);
    EXPECT_EQ(UXR_OUTPUT_STREAM, stream_id.direction);
    EXPECT_EQ(raw, stream_id.raw);
}

TEST(StreamIdTest, FromRawReliable)
{
    uint8_t raw = 130;
    uxrStreamId stream_id = uxr_stream_id_from_raw(raw, UXR_INPUT_STREAM);
    EXPECT_EQ(raw - RELIABLE_STREAM_THRESHOLD, stream_id.index);
    EXPECT_EQ(UXR_RELIABLE_STREAM, stream_id.type);
    EXPECT_EQ(UXR_INPUT_STREAM, stream_id.direction);
    EXPECT_EQ(raw, stream_id.raw);
}
