#include <AP_gtest.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_PICOLIBC_SITL_ENABLED

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

TEST(Picolibc, LongHexSignificand)
{
    char input[580];
    char *end = nullptr;
    memcpy(input, "0x1", 3);
    memset(input + 3, '0', 56);
    strcpy(input + 59, "p-2240");
    errno = 0;
    EXPECT_FLOAT_EQ(strtof(input, &end), 0.0f);
    EXPECT_EQ(*end, '\0');
    EXPECT_EQ(errno, ERANGE);

    memset(input + 3, '0', 560);
    strcpy(input + 563, "p-22400");
    errno = 0;
    EXPECT_DOUBLE_EQ(strtod(input, &end), 0.0);
    EXPECT_EQ(*end, '\0');
    EXPECT_EQ(errno, ERANGE);
}

#endif

AP_GTEST_MAIN()
