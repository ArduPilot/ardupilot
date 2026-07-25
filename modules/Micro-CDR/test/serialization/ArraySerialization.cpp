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

#include "ArraySerialization.hpp"

TEST_F(ArraySerialization, Bool)
{
    bool_array_serialization();
}

TEST_F(ArraySerialization, Char)
{
    char_array_serialization();
}

TEST_F(ArraySerialization, Int8)
{
    int8_t_array_serialization();
}

TEST_F(ArraySerialization, Uint8)
{
    uint8_t_array_serialization();
}

TEST_F(ArraySerialization, Int16)
{
    int16_t_array_serialization();
}

TEST_F(ArraySerialization, Uint16)
{
    uint16_t_array_serialization();
}

TEST_F(ArraySerialization, Int32)
{
    int32_t_array_serialization();
}

TEST_F(ArraySerialization, Uint32)
{
    uint32_t_array_serialization();
}

TEST_F(ArraySerialization, Int64)
{
    int64_t_array_serialization();
}

TEST_F(ArraySerialization, Uint64)
{
    uint64_t_array_serialization();
}

TEST_F(ArraySerialization, Float)
{
    float_array_serialization();
}

TEST_F(ArraySerialization, Double)
{
    double_array_serialization();
}
