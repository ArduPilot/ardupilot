#include <AP_gtest.h>

#include <AP_HAL/utility/ftoa_engine.h>

TEST(FTOAEngine, LargeNegativeExponent)
{
    char buf[255] {};
    ftoa_engine(1e-44f, buf, 7, 7);

    EXPECT_STREQ(buf, "");  // but at least it doesn't loop infinitely
}

AP_GTEST_MAIN()
