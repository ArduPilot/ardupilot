#include <AP_gtest.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include <cerrno>
#include <cmath>
#include <cstdlib>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(AP_PICOLIBC_SITL_ENABLED) && AP_PICOLIBC_SITL_ENABLED

TEST(Picolibc, NumericConversions)
{
    char input[] = " -123.5e-2 trailing";
    char *end = nullptr;
    EXPECT_DOUBLE_EQ(strtod(input, &end), static_cast<double>(-1.235L));
    EXPECT_STREQ(end, " trailing");

    char float_input[] = "149.165230,rest";
    EXPECT_FLOAT_EQ(strtof(float_input, &end), 149.165230f);
    EXPECT_STREQ(end, ",rest");

    double (*atof_function)(const char *) = &atof;
    EXPECT_DOUBLE_EQ(atof_function("584.25"), 584.25);
}

TEST(Picolibc, InvalidInput)
{
    char input[] = "not-a-number";
    char *end = nullptr;
    errno = EDOM;
    EXPECT_DOUBLE_EQ(strtod(input, &end), 0.0);
    EXPECT_EQ(end, input);
    EXPECT_EQ(errno, EDOM);
}

TEST(Picolibc, RangeErrors)
{
    char *end = nullptr;

    errno = 0;
    EXPECT_TRUE(std::isinf(strtod("1e9999", &end)));
    EXPECT_EQ(*end, '\0');
    EXPECT_EQ(errno, ERANGE);

    errno = 0;
    EXPECT_DOUBLE_EQ(strtod("1e-9999", &end), 0.0);
    EXPECT_EQ(*end, '\0');
    EXPECT_EQ(errno, ERANGE);
}

#endif

AP_GTEST_MAIN()
