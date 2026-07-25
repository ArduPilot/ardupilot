// Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <ucdr/microcdr.h>
#include <cstring>

#define BUFFER_LENGTH 1024UL

bool operator ==(
        const ucdrBuffer& rhs,
        const ucdrBuffer& lhs)
{
    return
        rhs.init            ==  lhs.init
        && rhs.final           ==  lhs.final
        && rhs.iterator        ==  lhs.iterator
        && rhs.origin          ==  lhs.origin
        && rhs.offset          ==  lhs.offset
        && rhs.endianness      ==  lhs.endianness
        && rhs.last_data_size  ==  lhs.last_data_size
        && rhs.error           ==  lhs.error
        && rhs.on_full_buffer  ==  lhs.on_full_buffer
        && rhs.args            ==  lhs.args;
}

class CommonFunctions : public ::testing::Test
{
public:

    void SetUp() override
    {
        std::memset(buffer, 0, BUFFER_LENGTH);
    }

protected:

    ucdrBuffer ub;
    uint8_t buffer[BUFFER_LENGTH];
};

TEST_F(CommonFunctions, ucdr_init_buffer)
{
    ucdr_init_buffer(&ub, buffer, BUFFER_LENGTH);
    EXPECT_EQ(ub.init, buffer);
    EXPECT_EQ(ub.final, ub.init + BUFFER_LENGTH);
    EXPECT_EQ(ub.iterator, ub.init);
    EXPECT_EQ(ub.origin, 0UL);
    EXPECT_EQ(ub.offset, ub.origin);
    EXPECT_EQ(ub.endianness, UCDR_MACHINE_ENDIANNESS);
    EXPECT_EQ(ub.last_data_size, 0);
    EXPECT_EQ(ub.error, false);
    EXPECT_EQ(ub.on_full_buffer, nullptr);
    EXPECT_EQ(ub.args, nullptr);
}

TEST_F(CommonFunctions, ucdr_init_buffer_origin)
{
    size_t origin = 1;
    ucdrBuffer temp_ub;

    ucdr_init_buffer(&ub, buffer, BUFFER_LENGTH);
    ub.origin = origin;
    ub.offset = ub.origin;
    ucdr_init_buffer_origin(&temp_ub, buffer, BUFFER_LENGTH, origin);
    EXPECT_EQ(ub, temp_ub);
}

TEST_F(CommonFunctions, ucdr_init_buffer_origin_offset)
{
    size_t origin = 1;
    size_t offset = 1;
    ucdrBuffer temp_ub;

    ucdr_init_buffer_origin(&ub, buffer, BUFFER_LENGTH, origin);
    ub.iterator += offset;
    ub.offset += offset;
    ucdr_init_buffer_origin_offset(&temp_ub, buffer, BUFFER_LENGTH, origin, offset);
    EXPECT_EQ(ub, temp_ub);
}

TEST_F(CommonFunctions, ucdr_init_buffer_origin_offset_endian)
{
    size_t origin = 1;
    size_t offset = 1;
    ucdrEndianness endian = UCDR_BIG_ENDIANNESS;
    ucdrBuffer temp_ub;

    ucdr_init_buffer_origin_offset(&ub, buffer, BUFFER_LENGTH, origin, offset);
    ub.endianness = endian;
    ucdr_init_buffer_origin_offset_endian(&temp_ub, buffer, BUFFER_LENGTH, origin, offset, endian);
    EXPECT_EQ(ub, temp_ub);
}

TEST_F(CommonFunctions, ucdr_copy_buffer)
{
    size_t origin = 1;
    size_t offset = 1;
    ucdrEndianness endian = UCDR_BIG_ENDIANNESS;
    ucdrBuffer temp_ub;

    ucdr_init_buffer_origin_offset_endian(&ub, buffer, BUFFER_LENGTH, origin, offset, endian);
    ucdr_copy_buffer(&temp_ub, &ub);
    EXPECT_EQ(ub, temp_ub);
}

TEST_F(CommonFunctions, ucdr_reset_buffer)
{
    ucdrBuffer temp_ub;

    ucdr_init_buffer(&ub, buffer, BUFFER_LENGTH);
    ucdr_init_buffer(&temp_ub, buffer, BUFFER_LENGTH);
    ucdr_serialize_bool(&temp_ub, true);
    ucdr_reset_buffer(&temp_ub);
    EXPECT_EQ(ub, temp_ub);
}

TEST_F(CommonFunctions, ucdr_reset_buffer_offset)
{
    size_t origin = 1;
    size_t offset = 1;
    ucdrBuffer temp_ub;

    ucdr_init_buffer_origin_offset(&ub, buffer, BUFFER_LENGTH, origin, offset);
    ucdr_init_buffer_origin_offset(&temp_ub, buffer, BUFFER_LENGTH, origin, offset);
    ucdr_serialize_bool(&temp_ub, true);
    ucdr_reset_buffer_offset(&temp_ub, offset);
    EXPECT_EQ(ub, temp_ub);
}

TEST_F(CommonFunctions, ucdr_align)
{
    size_t origin = 1;
    size_t offset = 1;

    ucdr_init_buffer_origin_offset(&ub, buffer, BUFFER_LENGTH, origin, offset);
    EXPECT_EQ(ucdr_alignment(offset, 1), 0UL);
    EXPECT_EQ(ucdr_buffer_alignment(&ub, 1), 0UL);
    ucdr_align_to(&ub, 1);
    EXPECT_EQ(ub.iterator, ub.init + 1);
    EXPECT_EQ(ub.offset, ub.origin + 1);

    ucdr_reset_buffer_offset(&ub, offset);
    EXPECT_EQ(ucdr_alignment(offset, 2), 1UL);
    EXPECT_EQ(ucdr_buffer_alignment(&ub, 2), 0UL);
    ucdr_align_to(&ub, 2);
    EXPECT_EQ(ub.iterator, ub.init + 1);
    EXPECT_EQ(ub.offset, ub.origin + 1);

    ucdr_reset_buffer_offset(&ub, offset);
    EXPECT_EQ(ucdr_alignment(offset, 4), 3UL);
    EXPECT_EQ(ucdr_buffer_alignment(&ub, 4), 2UL);
    ucdr_align_to(&ub, 4);
    EXPECT_EQ(ub.iterator, ub.init + 3);
    EXPECT_EQ(ub.offset, ub.origin + 3);

    ucdr_reset_buffer_offset(&ub, offset);
    EXPECT_EQ(ucdr_alignment(offset, 8), 7UL);
    EXPECT_EQ(ucdr_buffer_alignment(&ub, 8), 6UL);
    ucdr_align_to(&ub, 8);
    EXPECT_EQ(ub.iterator, ub.init + 7);
    EXPECT_EQ(ub.offset, ub.origin + 7);
}

TEST_F(CommonFunctions, ucdr_advance_buffer)
{
    const size_t distance = 7;

    ucdr_init_buffer(&ub, buffer, BUFFER_LENGTH);
    ucdr_advance_buffer(&ub, distance);
    EXPECT_EQ(ub.iterator, ub.init + distance);
    EXPECT_EQ(ub.offset, ub.origin + distance);
}

TEST_F(CommonFunctions, ucdr_buffer_size)
{
    ucdr_init_buffer(&ub, buffer, BUFFER_LENGTH);
    EXPECT_EQ(ucdr_buffer_size(&ub), BUFFER_LENGTH);
}

TEST_F(CommonFunctions, ucdr_buffer_length)
{
    const size_t distance = 7;

    ucdr_init_buffer(&ub, buffer, BUFFER_LENGTH);
    ucdr_advance_buffer(&ub, distance);
    EXPECT_EQ(ucdr_buffer_length(&ub), distance);
}

TEST_F(CommonFunctions, ucdr_buffer_remaining)
{
    const size_t distance = 7;

    ucdr_init_buffer(&ub, buffer, BUFFER_LENGTH);
    ucdr_advance_buffer(&ub, distance);
    EXPECT_EQ(ucdr_buffer_remaining(&ub), BUFFER_LENGTH - distance);
}