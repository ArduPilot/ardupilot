#include <gtest/gtest.h>

extern "C"
{
#include <c/core/serialization/xrce_subheader.c>
#include <c/core/session/submessage.c>
}

#define BUFFER_SIZE     size_t(8)
#define PAYLOAD_SIZE    size_t(4)

class SubmessageTest : public testing::Test
{
public:

    SubmessageTest()
    {
        ucdr_init_buffer(&ub_write, buffer, BUFFER_SIZE);
        ucdr_init_buffer(&ub_read, buffer, BUFFER_SIZE);
    }

    void write_and_read(
            uint8_t id,
            uint16_t length,
            uint8_t flags,
            bool fit_payload)
    {
        ASSERT_EQ(fit_payload, uxr_buffer_submessage_header(&ub_write, id, length, flags));
        uint8_t read_id; uint16_t read_length; uint8_t read_flags;
        EXPECT_TRUE(uxr_read_submessage_header(&ub_read, &read_id, &read_length, &read_flags));
        EXPECT_EQ(id, read_id);
        EXPECT_EQ(length, read_length);
        EXPECT_EQ(flags, read_flags);
        EXPECT_EQ(ub_write.endianness, UCDR_MACHINE_ENDIANNESS);
        EXPECT_EQ(ub_read.endianness, UCDR_MACHINE_ENDIANNESS);
    }

    virtual ~SubmessageTest()
    {
    }

protected:

    uint8_t buffer[BUFFER_SIZE];
    ucdrBuffer ub_write;
    ucdrBuffer ub_read;
};

TEST_F(SubmessageTest, WriteReadHeader)
{
    write_and_read(SUBMESSAGE_ID_CREATE, PAYLOAD_SIZE, FLAG_LAST_FRAGMENT, true);
}

TEST_F(SubmessageTest, WriteReadHeaderNoFits)
{
    ub_write.iterator += 4;
    ub_read.iterator += 4;
    write_and_read(SUBMESSAGE_ID_CREATE, PAYLOAD_SIZE, FLAG_LAST_FRAGMENT, false);
}

TEST(SubmessageTest_, PaddingHeader)
{
    EXPECT_EQ(size_t(0), uxr_submessage_padding(4));
    EXPECT_EQ(size_t(1), uxr_submessage_padding(3));
    EXPECT_EQ(size_t(2), uxr_submessage_padding(2));
    EXPECT_EQ(size_t(3), uxr_submessage_padding(1));
    EXPECT_EQ(size_t(0), uxr_submessage_padding(0));
}
