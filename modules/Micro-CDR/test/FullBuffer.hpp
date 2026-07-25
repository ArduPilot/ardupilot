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

#include "serialization/BasicSerialization.hpp"

#define ARRAY_CAPACITY 128

class FullBuffer : public BasicSerialization
{
public:

    FullBuffer()
    {
    }

    void fill_buffer_except(
            int gap)
    {
        for (int i = 0; i < BUFFER_LENGTH - gap; ++i)
        {
            uint8_t_serialization();
        }
    }

    void try_block_1()
    {
        uint8_t input = 0xAA;
        uint8_t output = 0;

        EXPECT_FALSE(ucdr_serialize_uint8_t(&writer, input));
        EXPECT_FALSE(ucdr_deserialize_uint8_t(&reader, &output));
    }

    void try_block_2()
    {
        uint16_t input = 0xAABB;
        uint16_t output = 0;

        EXPECT_FALSE(ucdr_serialize_uint16_t(&writer, input));
        EXPECT_FALSE(ucdr_deserialize_uint16_t(&reader, &output));
    }

    void try_block_4()
    {
        uint32_t input = 0xAABBCCDD;
        uint32_t output = 0;

        EXPECT_FALSE(ucdr_serialize_uint32_t(&writer, input));
        EXPECT_FALSE(ucdr_deserialize_uint32_t(&reader, &output));
    }

    void try_block_8()
    {
        uint64_t input = 0x0123456789ABCDEF;
        uint64_t output = 0;

        EXPECT_FALSE(ucdr_serialize_uint64_t(&writer, input));
        EXPECT_FALSE(ucdr_deserialize_uint64_t(&reader, &output));
    }

    void try_array_block_1(
            uint32_t size)
    {
        uint8_t input[ARRAY_CAPACITY];
        std::fill_n(input, size, uint8_t(0x09));
        uint8_t output[ARRAY_CAPACITY] = {0};

        EXPECT_FALSE(ucdr_serialize_array_uint8_t(&writer, input, size));
        EXPECT_FALSE(ucdr_deserialize_array_uint8_t(&reader, output, size));
    }

    void try_array_block_2(
            uint32_t size)
    {
        uint16_t input[ARRAY_CAPACITY];
        std::fill_n(input, size, int16_t(0x0A0B));
        uint16_t output[ARRAY_CAPACITY] = {0};

        EXPECT_FALSE(ucdr_serialize_array_uint16_t(&writer, input, size));
        EXPECT_FALSE(ucdr_deserialize_array_uint16_t(&reader, output, size));
    }

    void try_array_block_4(
            uint32_t size)
    {

        uint32_t input[ARRAY_CAPACITY];
        std::fill_n(input, size, 0x0C0D0E0F);
        uint32_t output[ARRAY_CAPACITY] = {0};

        EXPECT_FALSE(ucdr_serialize_array_uint32_t(&writer, input, size));
        EXPECT_FALSE(ucdr_deserialize_array_uint32_t(&reader, output, size));
    }

    void try_array_block_8(
            uint32_t size)
    {
        uint64_t input[ARRAY_CAPACITY];
        std::fill_n(input, size, 0x0102030405060708L);
        uint64_t output[ARRAY_CAPACITY] = {0};

        EXPECT_FALSE(ucdr_serialize_array_uint64_t(&writer, input, size));
        EXPECT_FALSE(ucdr_deserialize_array_uint64_t(&reader, output, size));
    }

    ~FullBuffer()
    {
        EXPECT_TRUE(writer.error);
        EXPECT_TRUE(reader.error);

        try_block_1(); //a serialization with an error, gives an error

        EXPECT_TRUE(writer.error);
        EXPECT_TRUE(reader.error);

        // To satisfy the base destructor
        writer.error = false;
        reader.error = false;
    }

};
