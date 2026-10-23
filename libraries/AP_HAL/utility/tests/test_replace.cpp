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

#include <AP_HAL/AP_HAL.h>

#ifdef HAVE_MEMRCHR
// this build's libc provides memrchr, so replace.cpp compiles to
// nothing; compile the replacement here so it can be tested
#undef HAVE_MEMRCHR
#include <AP_HAL/utility/replace.cpp>
#else
#include <AP_HAL/utility/replace.h>
#endif

TEST(ReplaceMemrchr, LastOccurrence)
{
    char buf[] = "abcabc";
    void *ret = replace_memrchr(buf, 'b', 6);
    EXPECT_EQ(ret, &buf[4]);
}

TEST(ReplaceMemrchr, SingleOccurrence)
{
    char buf[] = "abcdef";
    void *ret = replace_memrchr(buf, 'c', 6);
    EXPECT_EQ(ret, &buf[2]);
}

TEST(ReplaceMemrchr, FirstByte)
{
    char buf[] = "abcdef";
    void *ret = replace_memrchr(buf, 'a', 6);
    EXPECT_EQ(ret, &buf[0]);
}

TEST(ReplaceMemrchr, NotFound)
{
    char buf[] = "abcdef";
    EXPECT_EQ(replace_memrchr(buf, 'z', 6), nullptr);
}

TEST(ReplaceMemrchr, ZeroLength)
{
    char buf[] = "abcdef";
    EXPECT_EQ(replace_memrchr(buf, 'a', 0), nullptr);
}

TEST(ReplaceMemrchr, RespectsLength)
{
    // 'e' is present, but beyond the searched length
    char buf[] = "abcdef";
    EXPECT_EQ(replace_memrchr(buf, 'e', 3), nullptr);
    // last occurrence within the searched length is found
    char buf2[] = "aabaa";
    EXPECT_EQ(replace_memrchr(buf2, 'a', 2), &buf2[1]);
}

TEST(ReplaceMemrchr, CharConversion)
{
    // the int argument must be converted to unsigned char for comparison
    uint8_t buf[] = { 0x00, 0xff, 0x00 };
    EXPECT_EQ(replace_memrchr(buf, 0xff, 3), &buf[1]);
    EXPECT_EQ(replace_memrchr(buf, (int)0xffffffff, 3), &buf[1]);
}

AP_GTEST_MAIN()
