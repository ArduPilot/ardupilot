
#include <gtest/gtest.h>

extern "C"
{
#include <c/core/session/stream/seq_num.c>
#include <c/core/session/stream/stream_id.c>
#include <c/core/session/stream/stream_storage.c>
#include <uxr/client/profile/multithread/multithread.h>
}

#define BUFFER_SIZE           128
#define HISTORY               4
#define OFFSET                8

class StreamStorageTest : public testing::Test
{
public:

    StreamStorageTest()
    {
        uxr_init_stream_storage(&storage);
        EXPECT_EQ(0, storage.input_best_effort_size);
        EXPECT_EQ(0, storage.output_best_effort_size);
        EXPECT_EQ(0, storage.input_reliable_size);
        EXPECT_EQ(0, storage.output_reliable_size);

    }

    virtual ~StreamStorageTest()
    {
    }

protected:

    uxrStreamStorage storage;
    uint8_t ob_buffer[BUFFER_SIZE / HISTORY];
    uint8_t or_buffer[BUFFER_SIZE];
    uint8_t ir_buffer[BUFFER_SIZE];

    static FragmentationInfo on_get_fragmentation_info(
            uint8_t* buffer)
    {
        (void) buffer;
        return NO_FRAGMENTED;
    }

public:

    static int output_best_effort_reset_times;
    static int input_best_effort_reset_times;
    static int output_reliable_reset_times;
    static int input_reliable_reset_times;

    static bool output_best_effort_initialized;
    static bool input_best_effort_initialized;
    static bool output_reliable_initialized;
    static bool input_reliable_initialized;

    static bool output_reliable_up_to_date;
};

int StreamStorageTest::output_best_effort_reset_times = 0;
int StreamStorageTest::input_best_effort_reset_times = 0;
int StreamStorageTest::output_reliable_reset_times = 0;
int StreamStorageTest::input_reliable_reset_times = 0;

bool StreamStorageTest::output_best_effort_initialized = false;
bool StreamStorageTest::input_best_effort_initialized = false;
bool StreamStorageTest::output_reliable_initialized = false;
bool StreamStorageTest::input_reliable_initialized = false;

bool StreamStorageTest::output_reliable_up_to_date = false;
TEST_F(StreamStorageTest, Reset)
{
    output_best_effort_reset_times = 0;
    input_best_effort_reset_times = 0;
    output_reliable_reset_times = 0;
    input_reliable_reset_times = 0;

    (void) uxr_add_input_best_effort_buffer(&storage);
    (void) uxr_add_output_best_effort_buffer(&storage, ob_buffer, BUFFER_SIZE / HISTORY, OFFSET);
    (void) uxr_add_input_reliable_buffer(&storage, ir_buffer, BUFFER_SIZE, HISTORY, on_get_fragmentation_info);
    (void) uxr_add_output_reliable_buffer(&storage, or_buffer, BUFFER_SIZE, HISTORY, OFFSET);

    uxr_reset_stream_storage(&storage);

    EXPECT_EQ(UXR_CONFIG_MAX_OUTPUT_BEST_EFFORT_STREAMS, output_best_effort_reset_times);
    EXPECT_EQ(UXR_CONFIG_MAX_INPUT_BEST_EFFORT_STREAMS, input_best_effort_reset_times);
    EXPECT_EQ(UXR_CONFIG_MAX_OUTPUT_RELIABLE_STREAMS, output_reliable_reset_times);
    EXPECT_EQ(UXR_CONFIG_MAX_INPUT_RELIABLE_STREAMS, input_reliable_reset_times);
}

TEST_F(StreamStorageTest, InputBestEffortInitialization)
{
    input_best_effort_initialized = false;
    uxrStreamId id = uxr_add_input_best_effort_buffer(&storage);
    EXPECT_TRUE(input_best_effort_initialized);
    EXPECT_EQ(0, id.index);
    EXPECT_EQ(1, id.raw);
    EXPECT_EQ(UXR_BEST_EFFORT_STREAM, id.type);
    EXPECT_EQ(UXR_INPUT_STREAM, id.direction);
}

TEST_F(StreamStorageTest, OutputBestEffortInitialization)
{
    output_best_effort_initialized = false;
    uxrStreamId id = uxr_add_output_best_effort_buffer(&storage, ob_buffer, BUFFER_SIZE / HISTORY, OFFSET);
    EXPECT_TRUE(output_best_effort_initialized);
    EXPECT_EQ(0, id.index);
    EXPECT_EQ(1, id.raw);
    EXPECT_EQ(UXR_BEST_EFFORT_STREAM, id.type);
    EXPECT_EQ(UXR_OUTPUT_STREAM, id.direction);
}

TEST_F(StreamStorageTest, InputReliableInitialization)
{
    input_reliable_initialized = false;
    uxrStreamId id =
            uxr_add_input_reliable_buffer(&storage, ir_buffer, BUFFER_SIZE, HISTORY, on_get_fragmentation_info);
    EXPECT_TRUE(input_reliable_initialized);
    EXPECT_EQ(0, id.index);
    EXPECT_EQ(128, id.raw);
    EXPECT_EQ(UXR_RELIABLE_STREAM, id.type);
    EXPECT_EQ(UXR_INPUT_STREAM, id.direction);
}

TEST_F(StreamStorageTest, OutputReliableInitialization)
{
    output_reliable_initialized = false;
    uxrStreamId id = uxr_add_output_reliable_buffer(&storage, or_buffer, BUFFER_SIZE, HISTORY, OFFSET);
    EXPECT_TRUE(output_reliable_initialized);
    EXPECT_EQ(0, id.index);
    EXPECT_EQ(128, id.raw);
    EXPECT_EQ(UXR_RELIABLE_STREAM, id.type);
    EXPECT_EQ(UXR_OUTPUT_STREAM, id.direction);
}

TEST_F(StreamStorageTest, GetOk)
{
    (void) uxr_add_input_best_effort_buffer(&storage);
    (void) uxr_add_output_best_effort_buffer(&storage, ob_buffer, BUFFER_SIZE / HISTORY, OFFSET);
    (void) uxr_add_input_reliable_buffer(&storage, ir_buffer, BUFFER_SIZE, HISTORY, on_get_fragmentation_info);
    (void) uxr_add_output_reliable_buffer(&storage, or_buffer, BUFFER_SIZE, HISTORY, OFFSET);

    EXPECT_NE(nullptr, uxr_get_input_best_effort_stream(&storage, 0));
    EXPECT_NE(nullptr, uxr_get_output_best_effort_stream(&storage, 0));
    EXPECT_NE(nullptr, uxr_get_input_reliable_stream(&storage, 0));
    EXPECT_NE(nullptr, uxr_get_output_reliable_stream(&storage, 0));
}

TEST_F(StreamStorageTest, GetNoOk)
{
    EXPECT_EQ(nullptr, uxr_get_input_best_effort_stream(&storage, 0));
    EXPECT_EQ(nullptr, uxr_get_output_best_effort_stream(&storage, 0));
    EXPECT_EQ(nullptr, uxr_get_input_reliable_stream(&storage, 0));
    EXPECT_EQ(nullptr, uxr_get_output_reliable_stream(&storage, 0));
}

TEST_F(StreamStorageTest, OutputStreamConfirmed)
{
    output_reliable_up_to_date = true;
    (void) uxr_add_output_reliable_buffer(&storage, or_buffer, BUFFER_SIZE, HISTORY, OFFSET);
    EXPECT_TRUE(uxr_output_streams_confirmed(&storage));
}

TEST_F(StreamStorageTest, OutputStreamNoConfirmed)
{
    output_reliable_up_to_date = false;
    (void) uxr_add_output_reliable_buffer(&storage, or_buffer, BUFFER_SIZE, HISTORY, OFFSET);
    EXPECT_FALSE(uxr_output_streams_confirmed(&storage));
}
// ****************************************************************************Y
//                                  MOCKS
// ****************************************************************************Y
void uxr_reset_output_best_effort_stream(
        uxrOutputBestEffortStream* stream)
{
    (void) stream;
    StreamStorageTest::output_best_effort_reset_times++;
}

void uxr_reset_input_best_effort_stream(
        uxrInputBestEffortStream* stream)
{
    (void) stream;
    StreamStorageTest::input_best_effort_reset_times++;
}

void uxr_reset_output_reliable_stream(
        uxrOutputReliableStream* stream)
{
    (void) stream;
    StreamStorageTest::output_reliable_reset_times++;
}

void uxr_reset_input_reliable_stream(
        uxrInputReliableStream* stream)
{
    (void) stream;
    StreamStorageTest::input_reliable_reset_times++;
}

void uxr_init_input_best_effort_stream(
        uxrInputBestEffortStream* stream)
{
    (void) stream;
    UXR_INIT_LOCK(&stream->mutex);
    StreamStorageTest::input_best_effort_initialized = true;
}

void uxr_init_output_best_effort_stream(
        uxrOutputBestEffortStream* stream,
        uint8_t* buffer,
        size_t size,
        uint8_t offset)
{
    (void) stream; (void) buffer; (void) size; (void) offset;
    UXR_INIT_LOCK(&stream->mutex);
    StreamStorageTest::output_best_effort_initialized = true;
}

void uxr_init_input_reliable_stream(
        uxrInputReliableStream* stream,
        uint8_t* buffer,
        size_t size,
        uint16_t history,
        OnGetFragmentationInfo on_get_fragmentation_info)
{
    (void) stream; (void) buffer; (void) size; (void) history; (void) on_get_fragmentation_info;
    UXR_INIT_LOCK(&stream->mutex);
    StreamStorageTest::input_reliable_initialized = true;
}

void uxr_init_output_reliable_stream(
        uxrOutputReliableStream* stream,
        uint8_t* buffer,
        size_t size,
        uint16_t history,
        uint8_t header_offset)
{
    (void) stream; (void) buffer; (void) size; (void) history; (void) header_offset;
    UXR_INIT_LOCK(&stream->mutex);
    StreamStorageTest::output_reliable_initialized = true;
}

bool uxr_is_output_up_to_date(
        const uxrOutputReliableStream* stream)
{
    (void) stream;
    return StreamStorageTest::output_reliable_up_to_date;
}
