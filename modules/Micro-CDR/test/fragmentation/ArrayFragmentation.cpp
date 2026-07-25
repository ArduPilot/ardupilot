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

#define OFFSET 16

class ArrayFragmentation : public ArraySerialization
{
public:

    ArrayFragmentation()
    {
        ucdr_set_on_full_buffer_callback(&writer, on_full_buffer, this);
        ucdr_set_on_full_buffer_callback(&reader, on_full_buffer, this);
        std::memset(buffer2, 0, BUFFER_LENGTH);
        for (int i = 0; i < BUFFER_LENGTH - OFFSET; ++i)
        {
            uint8_t_serialization();
        }
    }

protected:

    uint8_t buffer2[BUFFER_LENGTH];
    uint8_t asymmetric_buffer1[999];
    uint8_t asymmetric_buffer2[1025];

    static bool on_full_buffer(
            ucdrBuffer* ub,
            void* args)
    {
        ArrayFragmentation* obj =  static_cast<ArrayFragmentation*>(args);

        ub->init = obj->buffer2;
        ub->iterator = ub->init;
        ub->final = ub->init + BUFFER_LENGTH;

        return false;
    }

    static bool asymmetric_first_on_full_buffer(
            ucdrBuffer* ub,
            void* args)
    {
        ArrayFragmentation* obj =  static_cast<ArrayFragmentation*>(args);

        ub->init = obj->asymmetric_buffer2;
        ub->iterator = ub->init;
        ub->final = ub->init + sizeof(asymmetric_buffer2);
        ucdr_set_on_full_buffer_callback(ub, asymmetric_last_on_full_buffer, obj);
        return false;
    }

    static bool asymmetric_last_on_full_buffer(
            ucdrBuffer* ub,
            void* args)
    {
        (void) ub;
        (void) args;
        return true;
    }

};

TEST_F(ArrayFragmentation, Bool)
{
    bool_array_serialization();
}

TEST_F(ArrayFragmentation, Char)
{
    char_array_serialization();
}

TEST_F(ArrayFragmentation, Int8)
{
    int8_t_array_serialization();
}

TEST_F(ArrayFragmentation, Uint8)
{
    uint8_t_array_serialization();
}

TEST_F(ArrayFragmentation, Int16)
{
    int16_t_array_serialization();
}

TEST_F(ArrayFragmentation, Uint16)
{
    uint16_t_array_serialization();
}

TEST_F(ArrayFragmentation, Int32)
{
    int32_t_array_serialization();
}

TEST_F(ArrayFragmentation, Uint32)
{
    uint32_t_array_serialization();
}

TEST_F(ArrayFragmentation, Int64)
{
    int64_t_array_serialization();
}

TEST_F(ArrayFragmentation, Uint64)
{
    uint64_t_array_serialization();
}

TEST_F(ArrayFragmentation, Float)
{
    float_array_serialization();
}

TEST_F(ArrayFragmentation, Double)
{
    double_array_serialization();
}

TEST_F(ArrayFragmentation, AsymmetricFragmentation)
{
    constexpr size_t n_elements = (sizeof(asymmetric_buffer1) +  sizeof(asymmetric_buffer2)) / sizeof(double);

    ucdr_init_buffer(&writer, asymmetric_buffer1, sizeof(asymmetric_buffer1));
    ucdr_init_buffer(&reader, asymmetric_buffer1, sizeof(asymmetric_buffer1));

    double input[n_elements];
    std::fill_n(input, n_elements, 3.141592653589793238462);
    double output[n_elements] = {0};
    std::memset(output, 0, n_elements * sizeof(double));

    ucdr_set_on_full_buffer_callback(&writer, asymmetric_first_on_full_buffer, this);
    ucdr_set_on_full_buffer_callback(&reader, asymmetric_first_on_full_buffer, this);

    EXPECT_TRUE(ucdr_serialize_array_double(&writer, input, n_elements));
    EXPECT_TRUE(ucdr_deserialize_array_double(&reader, output, n_elements));

    EXPECT_TRUE(0 == std::memcmp(input, output, n_elements * sizeof(double)));
}
