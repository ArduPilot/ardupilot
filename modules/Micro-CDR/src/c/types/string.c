
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

#include <ucdr/microcdr.h>

#include <string.h>

// -------------------------------------------------------------------
//              PUBLIC SERIALIZATION IMPLEMENTATIONS
// -------------------------------------------------------------------

bool ucdr_serialize_string(
        ucdrBuffer* ub,
        const char* string)
{
    return ucdr_serialize_sequence_char(ub, string, (uint32_t)strlen(string) + 1);
}

bool ucdr_serialize_endian_string(
        ucdrBuffer* ub,
        ucdrEndianness endianness,
        const char* string)
{
    return ucdr_serialize_endian_sequence_char(ub, endianness, string, (uint32_t)strlen(string) + 1);
}

bool ucdr_deserialize_string(
        ucdrBuffer* ub,
        char* string,
        size_t string_capacity)
{
    uint32_t length;
    return ucdr_deserialize_sequence_char(ub, string, string_capacity, &length);
}

bool ucdr_deserialize_endian_string(
        ucdrBuffer* ub,
        ucdrEndianness endianness,
        char* string,
        size_t string_capacity)
{
    uint32_t length;
    return ucdr_deserialize_endian_sequence_char(ub, endianness, string, string_capacity, &length);
}
