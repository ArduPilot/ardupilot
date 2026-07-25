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

#ifndef _ARRAY_SERIALIZATION_HPP_
#define _ARRAY_SERIALIZATION_HPP_

#include "BasicSerialization.hpp"

#define ARRAY_SIZE 32

class ArraySerialization : public BasicSerialization
{
public:

    ArraySerialization()
    {
    }

    virtual ~ArraySerialization()
    {
    }

    void bool_array_serialization()
    {
        bool input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, true);
        bool output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_bool(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_bool(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void char_array_serialization()
    {
        char input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, 'A');
        char output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_char(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_char(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void int8_t_array_serialization()
    {
        int8_t input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, int8_t(0x09));
        int8_t output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_int8_t(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_int8_t(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void uint8_t_array_serialization()
    {
        uint8_t input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, uint8_t(0x09));
        uint8_t output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_uint8_t(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_uint8_t(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void int16_t_array_serialization()
    {
        int16_t input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, int16_t(0x0A0B));
        int16_t output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_int16_t(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_int16_t(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void uint16_t_array_serialization()
    {
        uint16_t input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, uint16_t(0x0A0B));
        uint16_t output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_uint16_t(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_uint16_t(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void int32_t_array_serialization()
    {
        int32_t input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, 0x0C0D0E0F);
        int32_t output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_int32_t(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_int32_t(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void uint32_t_array_serialization()
    {
        uint32_t input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, 0x0C0D0E0F);
        uint32_t output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_uint32_t(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_uint32_t(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void int64_t_array_serialization()
    {
        int64_t input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, 0x0102030405060708L);
        int64_t output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_int64_t(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_int64_t(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void uint64_t_array_serialization()
    {
        uint64_t input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, 0x0102030405060708L);
        uint64_t output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_uint64_t(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_uint64_t(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void float_array_serialization()
    {
        float input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, 3.141592653589793238462f);
        float output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_float(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_float(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

    void double_array_serialization()
    {
        double input[ARRAY_SIZE];
        std::fill_n(input, ARRAY_SIZE, 3.141592653589793238462);
        double output[ARRAY_SIZE] = {0};

        EXPECT_TRUE(ucdr_serialize_array_double(&writer, input, ARRAY_SIZE));
        EXPECT_TRUE(ucdr_deserialize_array_double(&reader, output, ARRAY_SIZE));

        EXPECT_TRUE(0 == std::memcmp(input, output, ARRAY_SIZE * sizeof(input[0])));
    }

};

#endif //_ARRAY_SERIALIZATION_HPP_