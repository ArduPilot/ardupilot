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

#include "../serialization/SequenceSerialization.hpp"

class SequenceEndianness : public SequenceSerialization, public ::testing::WithParamInterface<ucdrEndianness>
{
public:

    SequenceEndianness()
    {
        endianness = GetParam();
    }

    virtual ~SequenceEndianness()
    {
    }

protected:

    ucdrEndianness endianness;
};

TEST_P(SequenceEndianness, Bool)
{
    bool input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, true);
    bool output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_bool(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_bool(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Char)
{
    char input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, 'A');
    char output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_char(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_char(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Int8)
{
    int8_t input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, int8_t(0x09));
    int8_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_int8_t(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_int8_t(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Uint8)
{
    uint8_t input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, uint8_t(0x09));
    uint8_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_uint8_t(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_uint8_t(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Int16)
{
    int16_t input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, int16_t(0x0A0B));
    int16_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_int16_t(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_int16_t(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Uint16)
{
    uint16_t input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, uint16_t(0x0A0B));
    uint16_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_uint16_t(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_uint16_t(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Int32)
{
    int32_t input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, 0x0A0B0C0D);
    int32_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_int32_t(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_int32_t(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Uint32)
{
    uint32_t input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, 0x0A0B0C0D);
    uint32_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_uint32_t(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_uint32_t(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Int64)
{
    int64_t input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, 0x0102030405060708L);
    int64_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_int64_t(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_int64_t(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Uint64)
{
    uint64_t input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, 0x0102030405060708L);
    uint64_t output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_uint64_t(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_uint64_t(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Float)
{
    float input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, 3.141592653589793238462f);
    float output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_float(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_float(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

TEST_P(SequenceEndianness, Double)
{
    double input[ARRAY_CAPACITY];
    std::fill_n(input, SEQUENCE_SIZE, 3.141592653589793238462);
    double output[ARRAY_CAPACITY] = {0};

    EXPECT_TRUE(ucdr_serialize_endian_sequence_double(&writer, endianness, input, SEQUENCE_SIZE));
    EXPECT_TRUE(ucdr_deserialize_endian_sequence_double(&reader, endianness, output, ARRAY_CAPACITY, &output_size));

    EXPECT_TRUE(0 == std::memcmp(input, output, SEQUENCE_SIZE * sizeof(input[0])));
}

#ifdef INSTANTIATE_TEST_SUITE_P
#define GTEST_INSTANTIATE_TEST_MACRO(x, y, z) INSTANTIATE_TEST_SUITE_P(x, y, z)
#else
#define GTEST_INSTANTIATE_TEST_MACRO(x, y, z) INSTANTIATE_TEST_CASE_P(x, y, z)
#endif // ifdef INSTANTIATE_TEST_SUITE_P
       //
GTEST_INSTANTIATE_TEST_MACRO(ucdrEndianness, SequenceEndianness,
        ::testing::Values(UCDR_LITTLE_ENDIANNESS, UCDR_BIG_ENDIANNESS));
