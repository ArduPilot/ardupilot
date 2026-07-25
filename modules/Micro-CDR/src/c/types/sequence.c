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

static void ucdr_deserialize_sequence_header(
        ucdrBuffer* ub,
        ucdrEndianness endianness,
        size_t capacity,
        uint32_t* length);

// -------------------------------------------------------------------
//                INTERNAL UTIL IMPLEMENTATION
// -------------------------------------------------------------------
inline void ucdr_deserialize_sequence_header(
        ucdrBuffer* ub,
        ucdrEndianness endianness,
        size_t capacity,
        uint32_t* length)
{
    ucdr_deserialize_endian_uint32_t(ub, endianness, length);
    if (*length > capacity)
    {
        ub->error = true;
    }
}

// -------------------------------------------------------------------
//                    SERIALIZE MACROS
// -------------------------------------------------------------------
#define UCDR_SEQUENCE_SERIALIZE_DEFINITION(SUFFIX, TYPE, SIZE) \
    bool ucdr_serialize_sequence ## SUFFIX(ucdrBuffer * ub, const TYPE * array, uint32_t length) \
    { \
        ucdr_serialize_endian_uint32_t(ub, ub->endianness, length); \
        return length == 0 || ucdr_serialize_endian_array ## SUFFIX(ub, ub->endianness, array, length); \
    } \
    bool ucdr_serialize_endian_sequence ## SUFFIX(ucdrBuffer * ub, ucdrEndianness endianness, const TYPE * array, \
            uint32_t length) \
    { \
        ucdr_serialize_endian_uint32_t(ub, endianness, length); \
        return length == 0 || ucdr_serialize_endian_array ## SUFFIX(ub, endianness, array, length); \
    } \

// -------------------------------------------------------------------
//                    DESERIALIZE MACROS
// -------------------------------------------------------------------
#define UCDR_SEQUENCE_DESERIALIZE_DEFINITION(SUFFIX, TYPE, SIZE) \
    bool ucdr_deserialize_sequence ## SUFFIX(ucdrBuffer * ub, TYPE * array, size_t array_capacity, uint32_t * length) \
    { \
        ucdr_deserialize_sequence_header(ub, ub->endianness, array_capacity, length); \
        return *length == 0 || ucdr_deserialize_endian_array ## SUFFIX(ub, ub->endianness, array, *length); \
    } \
    bool ucdr_deserialize_endian_sequence ## SUFFIX(ucdrBuffer * ub, ucdrEndianness endianness, TYPE * array, \
            size_t array_capacity, uint32_t * length) \
    { \
        ucdr_deserialize_sequence_header(ub, endianness, array_capacity, length); \
        return *length == 0 || ucdr_deserialize_endian_array ## SUFFIX(ub, endianness, array, *length); \
    } \

// -------------------------------------------------------------------
//                         DEFINITION MACRO
// -------------------------------------------------------------------
#define UCDR_SEQUENCE_DEFINITIONS(SUFFIX, TYPE, SIZE) \
    UCDR_SEQUENCE_SERIALIZE_DEFINITION(SUFFIX, TYPE, SIZE) \
    UCDR_SEQUENCE_DESERIALIZE_DEFINITION(SUFFIX, TYPE, SIZE) \

// -------------------------------------------------------------------
//              PUBLIC DE-SERIALIZATION IMPLEMENTATIONS
// -------------------------------------------------------------------
UCDR_SEQUENCE_DEFINITIONS(_char, char, 1)
UCDR_SEQUENCE_DEFINITIONS(_bool, bool, 1)
UCDR_SEQUENCE_DEFINITIONS(_uint8_t, uint8_t, 1)
UCDR_SEQUENCE_DEFINITIONS(_uint16_t, uint16_t, 2)
UCDR_SEQUENCE_DEFINITIONS(_uint32_t, uint32_t, 4)
UCDR_SEQUENCE_DEFINITIONS(_uint64_t, uint64_t, 8)
UCDR_SEQUENCE_DEFINITIONS(_int8_t, int8_t, 1)
UCDR_SEQUENCE_DEFINITIONS(_int16_t, int16_t, 2)
UCDR_SEQUENCE_DEFINITIONS(_int32_t, int32_t, 4)
UCDR_SEQUENCE_DEFINITIONS(_int64_t, int64_t, 8)
UCDR_SEQUENCE_DEFINITIONS(_float, float, 4)
UCDR_SEQUENCE_DEFINITIONS(_double, double, 8)
