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

#include <string.h>
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

TEST(ByteBufferTest, ReadByte)
{
    ByteBuffer x{32};
    uint8_t b = 0;

    // read of an empty buffer fails
    EXPECT_FALSE(x.read_byte(&b));
    // as does a read into nullptr
    EXPECT_FALSE(x.read_byte(nullptr));

    EXPECT_EQ(x.write((const uint8_t *)"ab", 2), 2U);
    EXPECT_TRUE(x.read_byte(&b));
    EXPECT_EQ(b, 'a');
    EXPECT_TRUE(x.read_byte(&b));
    EXPECT_EQ(b, 'b');
    EXPECT_TRUE(x.is_empty());
}

TEST(ByteBufferTest, Update)
{
    ByteBuffer x{8};

    EXPECT_EQ(x.write((const uint8_t *)"abcd", 4), 4U);

    // can't update more bytes than are stored
    EXPECT_FALSE(x.update((const uint8_t *)"WXYZ!", 5));

    // update the head of the buffer without popping
    EXPECT_TRUE(x.update((const uint8_t *)"XY", 2));
    uint8_t buf[8] {};
    EXPECT_EQ(x.peekbytes(buf, 4), 4U);
    EXPECT_STREQ((char*)buf, "XYcd");

    // move the read pointer near the end of the 8-byte backing
    // buffer, then write and update data wrapping around the end
    EXPECT_EQ(x.read(buf, 4), 4U);
    EXPECT_EQ(x.write((const uint8_t *)"efghij", 6), 6U);
    EXPECT_TRUE(x.update((const uint8_t *)"EFGHIJ", 6));
    memset(buf, 0, sizeof(buf));
    EXPECT_EQ(x.read(buf, 6), 6U);
    EXPECT_STREQ((char*)buf, "EFGHIJ");
}

TEST(ByteBufferTest, SetSizeBest)
{
    ByteBuffer x{32};
    EXPECT_TRUE(x.set_size_best(64));
    EXPECT_EQ(x.get_size(), 64U);

    // an external buffer can't be resized, so every candidate size
    // is refused
    uint8_t backing[16];
    ByteBuffer ext{backing, sizeof(backing)};
    EXPECT_FALSE(ext.set_size_best(64));

    // ... but reads and writes work as normal
    EXPECT_EQ(ext.write((const uint8_t *)"abc", 3), 3U);
    uint8_t buf[4] {};
    EXPECT_EQ(ext.read(buf, sizeof(buf)), 3U);
    EXPECT_STREQ((char*)buf, "abc");
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
