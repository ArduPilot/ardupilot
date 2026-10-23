#include <AP_gtest.h>
#include <stdint.h>

#include <AP_HAL/utility/sparse-endian.h>

TEST(EndianTest, PutLe64BasicPattern)
{
    uint8_t buf[8] = {};
    const uint64_t value = 0x1122334455667788ULL;

    put_le64_ptr(buf, value);

    // Expected little-endian layout:
    // LSB first
    EXPECT_EQ(buf[0], 0x88);
    EXPECT_EQ(buf[1], 0x77);
    EXPECT_EQ(buf[2], 0x66);
    EXPECT_EQ(buf[3], 0x55);
    EXPECT_EQ(buf[4], 0x44);
    EXPECT_EQ(buf[5], 0x33);
    EXPECT_EQ(buf[6], 0x22);
    EXPECT_EQ(buf[7], 0x11);
}

TEST(EndianTest, PutLe64RoundTrip)
{
    uint8_t buf[8] = {};
    const uint64_t value = 0xDEADBEEFCAFEBABEULL;

    put_le64_ptr(buf, value);

    // Read it back using existing helper
    uint64_t result = le64toh_ptr(buf);

    EXPECT_EQ(result, value);
}

TEST(EndianTest, PutLe64Zero)
{
    uint8_t buf[8];
    memset(buf, 0xFF, sizeof(buf));  // ensure overwrite

    put_le64_ptr(buf, 0ULL);

    for (uint8_t i = 0; i < 8; i++) {
        EXPECT_EQ(buf[i], 0x00);
    }
}

TEST(EndianTest, PutLe64AllBytesDifferent)
{
    uint8_t buf[8] = {};
    const uint64_t value = 0x0102030405060708ULL;

    put_le64_ptr(buf, value);

    // This test is particularly good at catching shift bugs
    EXPECT_EQ(buf[0], 0x08);
    EXPECT_EQ(buf[1], 0x07);
    EXPECT_EQ(buf[2], 0x06);
    EXPECT_EQ(buf[3], 0x05);
    EXPECT_EQ(buf[4], 0x04);
    EXPECT_EQ(buf[5], 0x03);
    EXPECT_EQ(buf[6], 0x02);
    EXPECT_EQ(buf[7], 0x01);
}

AP_GTEST_MAIN()
