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

#include "../serialization/BasicSerialization.hpp"

class BasicEndianness : public BasicSerialization, public ::testing::WithParamInterface<ucdrEndianness>
{
public:

    BasicEndianness()
    {
        endianness = GetParam();
    }

    virtual ~BasicEndianness()
    {
    }

protected:

    ucdrEndianness endianness;
};

TEST_P(BasicEndianness, Int16)
{
    int16_t input = 0x0A0B;
    int16_t output = 0;

    EXPECT_TRUE(ucdr_serialize_endian_int16_t(&writer, endianness, input));
    EXPECT_TRUE(ucdr_deserialize_endian_int16_t(&reader, endianness, &output));

    EXPECT_EQ(input, output);
}

TEST_P(BasicEndianness, Uint16)
{
    uint16_t input = 0x0A0B;
    uint16_t output = 0;

    EXPECT_TRUE(ucdr_serialize_endian_uint16_t(&writer, endianness, input));
    EXPECT_TRUE(ucdr_deserialize_endian_uint16_t(&reader, endianness, &output));

    EXPECT_EQ(input, output);
}

TEST_P(BasicEndianness, Int32)
{
    int32_t input = 0x0C0D0E0F;
    int32_t output = 0;

    EXPECT_TRUE(ucdr_serialize_endian_int32_t(&writer, endianness, input));
    EXPECT_TRUE(ucdr_deserialize_endian_int32_t(&reader, endianness, &output));

    EXPECT_EQ(input, output);
}

TEST_P(BasicEndianness, Uint32)
{
    uint32_t input = 0x0C0D0E0F;
    uint32_t output = 0;

    EXPECT_TRUE(ucdr_serialize_endian_uint32_t(&writer, endianness, input));
    EXPECT_TRUE(ucdr_deserialize_endian_uint32_t(&reader, endianness, &output));

    EXPECT_EQ(input, output);
}

TEST_P(BasicEndianness, Int64)
{
    int64_t input = 0x0102030405060708L;
    int64_t output = 0;

    EXPECT_TRUE(ucdr_serialize_endian_int64_t(&writer, endianness, input));
    EXPECT_TRUE(ucdr_deserialize_endian_int64_t(&reader, endianness, &output));

    EXPECT_EQ(input, output);
}

TEST_P(BasicEndianness, Uint64)
{
    uint64_t input = 0x0102030405060708L;
    uint64_t output = 0;

    EXPECT_TRUE(ucdr_serialize_endian_uint64_t(&writer, endianness, input));
    EXPECT_TRUE(ucdr_deserialize_endian_uint64_t(&reader, endianness, &output));

    EXPECT_EQ(input, output);
}

TEST_P(BasicEndianness, Float)
{
    float input  = 3.141592653589793238462f;
    float output = 0;

    EXPECT_TRUE(ucdr_serialize_endian_float(&writer, endianness, input));
    EXPECT_TRUE(ucdr_deserialize_endian_float(&reader, endianness, &output));

    EXPECT_EQ(input, output);
}

TEST_P(BasicEndianness, Double)
{
    double input  = 3.141592653589793238462;
    double output = 0;

    EXPECT_TRUE(ucdr_serialize_endian_double(&writer, endianness, input));
    EXPECT_TRUE(ucdr_deserialize_endian_double(&reader, endianness, &output));

    EXPECT_EQ(input, output);
}

#ifdef INSTANTIATE_TEST_SUITE_P
#define GTEST_INSTANTIATE_TEST_MACRO(x, y, z) INSTANTIATE_TEST_SUITE_P(x, y, z)
#else
#define GTEST_INSTANTIATE_TEST_MACRO(x, y, z) INSTANTIATE_TEST_CASE_P(x, y, z)
#endif // ifdef INSTANTIATE_TEST_SUITE_P
       //
GTEST_INSTANTIATE_TEST_MACRO(ucdrEndianness, BasicEndianness,
        ::testing::Values(UCDR_LITTLE_ENDIANNESS, UCDR_BIG_ENDIANNESS));
