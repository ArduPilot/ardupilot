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

#include "../serialization/ArraySerialization.hpp"

class ArrayEndianness : public ArraySerialization, public ::testing::WithParamInterface<ucdrEndianness>
{
public:

    ArrayEndianness()
    {
        endianness = GetParam();
    }

    virtual ~ArrayEndianness()
    {
    }

protected:

    ucdrEndianness endianness;
};

TEST_P(ArrayEndianness, Int16)
{
    int16_t input[ARRAY_SIZE];
    std::fill_n(input, ARRAY_SIZE, int16_t(0x0A0B));
    int16_t output[ARRAY_SIZE] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_array_int16_t(&writer, endianness, input, ARRAY_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_array_int16_t(&reader, endianness, output, ARRAY_SIZE));

    EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
}

TEST_P(ArrayEndianness, Uint16)
{
    uint16_t input[ARRAY_SIZE];
    std::fill_n(input, ARRAY_SIZE, uint16_t(0x0A0Bu));
    uint16_t output[ARRAY_SIZE] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_array_uint16_t(&writer, endianness, input, ARRAY_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_array_uint16_t(&reader, endianness, output, ARRAY_SIZE));

    EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
}

TEST_P(ArrayEndianness, Int32)
{
    int32_t input[ARRAY_SIZE];
    std::fill_n(input, ARRAY_SIZE, 0x0C0D0E0F);
    int32_t output[ARRAY_SIZE] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_array_int32_t(&writer, endianness, input, ARRAY_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_array_int32_t(&reader, endianness, output, ARRAY_SIZE));

    EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
}

TEST_P(ArrayEndianness, Uint32)
{
    uint32_t input[ARRAY_SIZE];
    std::fill_n(input, ARRAY_SIZE, 0x0C0D0E0F);
    uint32_t output[ARRAY_SIZE] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_array_uint32_t(&writer, endianness, input, ARRAY_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_array_uint32_t(&reader, endianness, output, ARRAY_SIZE));

    EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
}

TEST_P(ArrayEndianness, Int64)
{
    int64_t input[ARRAY_SIZE];
    std::fill_n(input, ARRAY_SIZE, 0x0102030405060708L);
    int64_t output[ARRAY_SIZE] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_array_int64_t(&writer, endianness, input, ARRAY_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_array_int64_t(&reader, endianness, output, ARRAY_SIZE));

    EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
}

TEST_P(ArrayEndianness, Uint64)
{
    uint64_t input[ARRAY_SIZE];
    std::fill_n(input, ARRAY_SIZE, 0x0102030405060708L);
    uint64_t output[ARRAY_SIZE] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_array_uint64_t(&writer, endianness, input, ARRAY_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_array_uint64_t(&reader, endianness, output, ARRAY_SIZE));

    EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
}

TEST_P(ArrayEndianness, Float)
{
    float input[ARRAY_SIZE];
    std::fill_n(input, ARRAY_SIZE, 3.141592653589793238462f);
    float output[ARRAY_SIZE] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_array_float(&writer, endianness, input, ARRAY_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_array_float(&reader, endianness, output, ARRAY_SIZE));

    EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
}

TEST_P(ArrayEndianness, Double)
{
    double input[ARRAY_SIZE];
    std::fill_n(input, ARRAY_SIZE, 3.141592653589793238462);
    double output[ARRAY_SIZE] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_array_double(&writer, endianness, input, ARRAY_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_array_double(&reader, endianness, output, ARRAY_SIZE));

    EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
}

#ifdef INSTANTIATE_TEST_SUITE_P
#define GTEST_INSTANTIATE_TEST_MACRO(x, y, z) INSTANTIATE_TEST_SUITE_P(x, y, z)
#else
#define GTEST_INSTANTIATE_TEST_MACRO(x, y, z) INSTANTIATE_TEST_CASE_P(x, y, z)
#endif // ifdef INSTANTIATE_TEST_SUITE_P

GTEST_INSTANTIATE_TEST_MACRO(ucdrEndianness, ArrayEndianness,
        ::testing::Values(UCDR_LITTLE_ENDIANNESS, UCDR_BIG_ENDIANNESS));
