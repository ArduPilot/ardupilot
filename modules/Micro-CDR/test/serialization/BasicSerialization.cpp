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

#include "BasicSerialization.hpp"

TEST_F(BasicSerialization, Bool)
{
    bool_serialization();
}

TEST_F(BasicSerialization, Char)
{
    char_serialization();
}

TEST_F(BasicSerialization, Int8)
{
    int8_t_serialization();
}

TEST_F(BasicSerialization, Uint8)
{
    uint8_t_serialization();
}

TEST_F(BasicSerialization, Int16)
{
    int16_t_serialization();
}

TEST_F(BasicSerialization, Uint16)
{
    uint16_t_serialization();
}

TEST_F(BasicSerialization, Int32)
{
    int32_t_serialization();
}

TEST_F(BasicSerialization, Uint32)
{
    uint32_t_serialization();
}

TEST_F(BasicSerialization, Int64)
{
    int64_t_serialization();
}

TEST_F(BasicSerialization, Uint64)
{
    uint64_t_serialization();
}

TEST_F(BasicSerialization, Float)
{
    float_serialization();
}

TEST_F(BasicSerialization, Double)
{
    double_serialization();
}

