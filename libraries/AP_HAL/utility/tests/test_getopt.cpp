/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_gtest.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/utility/getopt_cpp.h>

static int flag_value;

static const GetOptLong::option longopts[] = {
    {"help",    false, nullptr,     'h'},
    {"level",   true,  nullptr,     'L'},
    {"setflag", false, &flag_value, 42},
    {nullptr,   false, nullptr,     0},
};

static const char *optstring = "hvL:";

#define MAKE_ARGS(...) \
    const char *argv[] { "prog", __VA_ARGS__ }; \
    GetOptLong gopt(ARRAY_SIZE(argv), (char* const*)argv, optstring, longopts)

TEST(GetOptLong, ShortOption)
{
    MAKE_ARGS("-h");
    EXPECT_EQ(gopt.getoption(), 'h');
    EXPECT_EQ(gopt.getoption(), -1);
    EXPECT_EQ(gopt.optind, 2);
}

TEST(GetOptLong, ShortOptionArgAttached)
{
    MAKE_ARGS("-Lfive");
    EXPECT_EQ(gopt.getoption(), 'L');
    EXPECT_STREQ(gopt.optarg, "five");
    EXPECT_EQ(gopt.getoption(), -1);
}

TEST(GetOptLong, ShortOptionArgSeparate)
{
    MAKE_ARGS("-L", "five");
    EXPECT_EQ(gopt.getoption(), 'L');
    EXPECT_STREQ(gopt.optarg, "five");
    EXPECT_EQ(gopt.optind, 3);
    EXPECT_EQ(gopt.getoption(), -1);
}

TEST(GetOptLong, CombinedShortOptions)
{
    MAKE_ARGS("-hv");
    EXPECT_EQ(gopt.getoption(), 'h');
    EXPECT_EQ(gopt.getoption(), 'v');
    EXPECT_EQ(gopt.getoption(), -1);
}

TEST(GetOptLong, UnknownShortOption)
{
    MAKE_ARGS("-z");
    EXPECT_EQ(gopt.getoption(), GetOptLong::BADCH);
    EXPECT_EQ(gopt.optopt, 'z');
}

TEST(GetOptLong, ShortOptionMissingArg)
{
    MAKE_ARGS("-L");
    EXPECT_EQ(gopt.getoption(), GetOptLong::BADCH);
}

TEST(GetOptLong, ShortOptionMissingArgColon)
{
    const char *argv[] { "prog", "-L" };
    GetOptLong gopt(ARRAY_SIZE(argv), (char* const*)argv, ":hL:", longopts);
    EXPECT_EQ(gopt.getoption(), GetOptLong::BADARG);
}

TEST(GetOptLong, LongOption)
{
    MAKE_ARGS("--help");
    EXPECT_EQ(gopt.getoption(), 'h');
    EXPECT_EQ(gopt.longindex, 0);
    EXPECT_EQ(gopt.getoption(), -1);
}

TEST(GetOptLong, LongOptionArgEquals)
{
    MAKE_ARGS("--level=5");
    EXPECT_EQ(gopt.getoption(), 'L');
    EXPECT_STREQ(gopt.optarg, "5");
    EXPECT_EQ(gopt.longindex, 1);
    EXPECT_EQ(gopt.getoption(), -1);
}

TEST(GetOptLong, LongOptionArgSeparate)
{
    MAKE_ARGS("--level", "5");
    EXPECT_EQ(gopt.getoption(), 'L');
    EXPECT_STREQ(gopt.optarg, "5");
    EXPECT_EQ(gopt.getoption(), -1);
}

TEST(GetOptLong, LongOptionFlag)
{
    flag_value = 0;
    MAKE_ARGS("--setflag");
    EXPECT_EQ(gopt.getoption(), 0);
    EXPECT_EQ(flag_value, 42);
}

TEST(GetOptLong, UnknownLongOption)
{
    MAKE_ARGS("--bogus");
    EXPECT_EQ(gopt.getoption(), GetOptLong::BADCH);
    EXPECT_EQ(gopt.optind, 2);
}

TEST(GetOptLong, LongOptionMissingArg)
{
    MAKE_ARGS("--level");
    EXPECT_EQ(gopt.getoption(), GetOptLong::BADCH);
}

TEST(GetOptLong, LongOptionMissingArgColon)
{
    const char *argv[] { "prog", "--level" };
    GetOptLong gopt(ARRAY_SIZE(argv), (char* const*)argv, ":hL:", longopts);
    EXPECT_EQ(gopt.getoption(), GetOptLong::BADARG);
}

TEST(GetOptLong, DoubleDashTerminates)
{
    MAKE_ARGS("--", "-h");
    EXPECT_EQ(gopt.getoption(), -1);
    // "--" itself is consumed, leaving optind at the first operand
    EXPECT_EQ(gopt.optind, 2);
}

TEST(GetOptLong, NonOptionStopsParsing)
{
    MAKE_ARGS("foo", "-h");
    EXPECT_EQ(gopt.getoption(), -1);
    EXPECT_EQ(gopt.optind, 1);
}

TEST(GetOptLong, MixedOptions)
{
    MAKE_ARGS("-h", "--level=3", "-L", "x", "rest");
    EXPECT_EQ(gopt.getoption(), 'h');
    EXPECT_EQ(gopt.getoption(), 'L');
    EXPECT_STREQ(gopt.optarg, "3");
    EXPECT_EQ(gopt.getoption(), 'L');
    EXPECT_STREQ(gopt.optarg, "x");
    EXPECT_EQ(gopt.getoption(), -1);
    EXPECT_EQ(gopt.optind, 5);
}

AP_GTEST_MAIN()
