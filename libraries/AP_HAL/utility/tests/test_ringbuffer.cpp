/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_gtest.h>

#include <utility>
#include <AP_HAL/utility/RingBuffer.h>

TEST(ByteBufferTest, Basic)
{
    const uint16_t size = 32;
    ByteBuffer x{size};
    EXPECT_EQ(x.available(), 0U);
    EXPECT_EQ(x.get_size(), unsigned(size));
    EXPECT_EQ(x.space(), unsigned(size-1));
    EXPECT_TRUE(x.is_empty());

    // write a char in, check numbers again:
    EXPECT_EQ(x.write((uint8_t*)"f", 1), 1U);
    EXPECT_EQ(x.available(), 1U);
    EXPECT_EQ(x.get_size(), unsigned(size));
    EXPECT_EQ(x.space(), unsigned(size-2));
    EXPECT_FALSE(x.is_empty());
    // clear that byte, check numbers again
    x.clear();
    EXPECT_EQ(x.available(), 0U);
    EXPECT_EQ(x.get_size(), size);
    EXPECT_EQ(x.space(), unsigned(size-1));
    EXPECT_TRUE(x.is_empty());

    constexpr auto str_size = 3;
    static const char str[str_size] = "fo";
    EXPECT_EQ(x.write((uint8_t*)str, 2), 2U);
    uint8_t buf[str_size+5] {};
    EXPECT_EQ(x.read(buf, sizeof(buf)), 2U);
    EXPECT_STREQ((char*)buf, (char*)str);
}

TEST(ByteBufferTest, SetSize)
{
    const uint16_t size = 32;
    ByteBuffer x{0};
    x.set_size(size);
    EXPECT_EQ(x.available(), 0U);
    EXPECT_EQ(x.get_size(), unsigned(size));
    EXPECT_EQ(x.space(), unsigned(size-1));
    EXPECT_TRUE(x.is_empty());
}

TEST(ObjectBufferTest, Basic)
{
    const uint16_t size = 32;
    class TestData {
        uint32_t test_member;
        uint8_t test_member2;
    };
    ObjectBuffer<TestData> x{size};
    EXPECT_EQ(x.available(), 0U);
    EXPECT_EQ(x.get_size(), unsigned(size));
    EXPECT_EQ(x.space(), unsigned(size));
    EXPECT_TRUE(x.is_empty());

    // write an object in, check numbers again:
    x.push(TestData{});
    EXPECT_EQ(x.available(), 1U);
    EXPECT_EQ(x.get_size(), unsigned(size));
    EXPECT_EQ(x.space(), unsigned(size-1));
    EXPECT_FALSE(x.is_empty());
    // clear that byte, check numbers again
    x.clear();
    EXPECT_EQ(x.available(), 0U);
    EXPECT_EQ(x.get_size(), size);
    EXPECT_EQ(x.space(), unsigned(size));
    EXPECT_TRUE(x.is_empty());
}

TEST(ObjectBufferTest, SetSize)
{
    const uint16_t size = 32;
    class TestData {
        uint32_t test_member;
        uint8_t test_member2;
    };
    ObjectBuffer<TestData> x{1};
    x.set_size(size);
    EXPECT_EQ(x.available(), 0U);
    EXPECT_EQ(x.get_size(), unsigned(size));
    EXPECT_EQ(x.space(), unsigned(size));
    EXPECT_TRUE(x.is_empty());
}

TEST(ObjectBufferTest, PeekTest)
{
    ByteBuffer bb(128);
    uint8_t seven[7] {1,2,3,4,5,6,7};
    for (uint8_t i=0; i<100; i++) {
        bb.write(seven, 7);
        ByteBuffer::IoVec vec[2];
        uint8_t nvec = bb.peekiovec(vec, 7);
        uint32_t got = 0;
        if (nvec > 0) {
            got += vec[0].len;
        }
        if (nvec > 1) {
            got += vec[1].len;
        }
        EXPECT_EQ(got, 7U);
        bb.advance(7);
    }
}

AP_GTEST_MAIN()
