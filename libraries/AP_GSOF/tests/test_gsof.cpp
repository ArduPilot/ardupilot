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
    EXPECT_NE(fp, nullptr);
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

AP_GTEST_MAIN()
