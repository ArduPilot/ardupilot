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

#ifndef _BASIC_SERIALIZATION_HPP_
#define _BASIC_SERIALIZATION_HPP_

#include <gtest/gtest.h>
#include <ucdr/microcdr.h>
#include <algorithm>
#include <cstring>

#define BUFFER_LENGTH 1024

class BasicSerialization : public ::testing::Test
{
public:

    BasicSerialization()
    {
        std::memset(buffer, 0, BUFFER_LENGTH);
        ucdr_init_buffer(&writer, buffer, BUFFER_LENGTH);
        ucdr_init_buffer(&reader, buffer, BUFFER_LENGTH);
    }

    void check_data_size(
            uint32_t data_size)
    {
        EXPECT_EQ(writer.last_data_size, data_size);
        EXPECT_EQ(reader.last_data_size, data_size);
    }

    virtual ~BasicSerialization()
    {
        EXPECT_EQ(writer.iterator, reader.iterator);
        EXPECT_FALSE(writer.error);
        EXPECT_FALSE(reader.error);
    }

    void bool_serialization()
    {
        bool input = true;
        bool output = 0;

        EXPECT_TRUE(ucdr_serialize_bool(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_bool(&reader, &output));

        EXPECT_EQ(input, output);

        check_data_size(1);
    }

    void char_serialization()
    {
        char input = 'A';
        char output = 0;

        EXPECT_TRUE(ucdr_serialize_char(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_char(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(1);
    }

    void int8_t_serialization()
    {
        int8_t input = 0x09;
        int8_t output = 0;

        EXPECT_TRUE(ucdr_serialize_int8_t(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_int8_t(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(1);
    }

    void uint8_t_serialization()
    {
        uint8_t input = 0x09;
        uint8_t output = 0;

        EXPECT_TRUE(ucdr_serialize_uint8_t(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_uint8_t(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(1);
    }

    void int16_t_serialization()
    {
        int16_t input = 0x0A0B;
        int16_t output = 0;

        EXPECT_TRUE(ucdr_serialize_int16_t(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_int16_t(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(2);
    }

    void uint16_t_serialization()
    {
        uint16_t input = 0x0A0B;
        uint16_t output = 0;

        EXPECT_TRUE(ucdr_serialize_uint16_t(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_uint16_t(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(2);
    }

    void int32_t_serialization()
    {
        int32_t input = 0x0C0D0E0F;
        int32_t output = 0;

        EXPECT_TRUE(ucdr_serialize_int32_t(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_int32_t(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(4);
    }

    void uint32_t_serialization()
    {
        uint32_t input = 0x0C0D0E0F;
        uint32_t output = 0;

        EXPECT_TRUE(ucdr_serialize_uint32_t(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_uint32_t(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(4);
    }

    void int64_t_serialization()
    {
        int64_t input = 0x0102030405060708L;
        int64_t output = 0;

        EXPECT_TRUE(ucdr_serialize_int64_t(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_int64_t(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(8);
    }

    void uint64_t_serialization()
    {
        uint64_t input = 0x0102030405060708L;
        uint64_t output = 0;

        EXPECT_TRUE(ucdr_serialize_uint64_t(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_uint64_t(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(8);
    }

    void float_serialization()
    {
        float input  = 3.141592653589793238462f;
        float output = 0;

        EXPECT_TRUE(ucdr_serialize_float(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_float(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(4);
    }

    void double_serialization()
    {
        double input  = 3.141592653589793238462;
        double output = 0;

        EXPECT_TRUE(ucdr_serialize_double(&writer, input));
        EXPECT_TRUE(ucdr_deserialize_double(&reader, &output));

        EXPECT_EQ(input, output);
        check_data_size(8);
    }

protected:

    ucdrBuffer writer;
    ucdrBuffer reader;
    uint8_t buffer[BUFFER_LENGTH];
};

#endif //_BASIC_SERIALIZATION_HPP_
