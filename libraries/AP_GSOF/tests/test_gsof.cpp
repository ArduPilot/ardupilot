// Tests for the GSOF parser.
// * ./waf tests
// * ./build/sitl/tests/test_gsof


#include <AP_gtest.h>

#include <AP_GSOF/AP_GSOF.h>

#include <cstdio>
#include <cstdlib>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();


TEST(AP_GSOF, incomplete_packet)
{
    AP_GSOF gsof;
    AP_GSOF::MsgTypes expected;
    EXPECT_FALSE(gsof.parse(0, expected));
}

TEST(AP_GSOF, packet1)
{
    GTEST_SKIP() << "There is not yet a convention for loading in a data file in a cross-platform way in AP for unit tests";
    FILE* fp = std::fopen("libraries/AP_GSOF/tests/gsof_gps.dat", "rb");
    ASSERT_NE(fp, nullptr);
    AP_GSOF gsof;
    char c = 0;
    bool parsed = false;

    AP_GSOF::MsgTypes expected;
    expected.set(1);
    expected.set(2);
    expected.set(8);
    expected.set(9);
    expected.set(12);

    while (c != EOF) {
        c = fgetc (fp);
        parsed |= gsof.parse((uint8_t)c, expected);
    }

    EXPECT_TRUE(parsed);

    fclose(fp);

}

TEST(AP_GSOF, packet1_corrupt_bytes)
{
    GTEST_SKIP() << "There is not yet a convention for loading in a data file in a cross-platform way in AP for unit tests";
    FILE* fp = fopen("libraries/AP_GSOF/tests/gsof_gps.dat", "rb");
    ASSERT_TRUE(fp != NULL);

    uint8_t buf[120];
    int len = 0;
    int ch;

    while ((ch = fgetc(fp)) != EOF && len < (int)sizeof(buf)) {
        buf[len++] = (uint8_t)ch;
    }

    fclose(fp);
    ASSERT_TRUE(len == 120);

    // Corrupt 80th byte (index 79, output_length for last output), originally value 0x26
    buf[79]++;

    // Corrupt second-to-last byte (checksum field), originally value 0x05
    buf[len - 2]++;

    AP_GSOF gsof;
    bool parsed = false;

    AP_GSOF::MsgTypes expected;
    expected.set(1);
    expected.set(2);
    expected.set(8);
    expected.set(9);
    expected.set(12);

    for (int i = 0; i < len; i++) {
        parsed |= gsof.parse(buf[i], expected);
    }

    // We expect the packet to be corrupted due to checksum failure
    EXPECT_FALSE(parsed);
}

AP_GTEST_MAIN()
