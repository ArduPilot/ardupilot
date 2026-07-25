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

#include "../common_internal.h"

#include <string.h>

static void ucdr_array_to_buffer(
        ucdrBuffer* ub,
        const uint8_t* array,
        size_t size,
        size_t data_size);
static void ucdr_buffer_to_array(
        ucdrBuffer* ub,
        uint8_t* array,
        size_t size,
        size_t data_size);

// -------------------------------------------------------------------
//                         SERIALIZE MACROS
// -------------------------------------------------------------------
void ucdr_array_to_buffer(
        ucdrBuffer* ub,
        const uint8_t* array,
        size_t size,
        size_t data_size)
{
    if (ucdr_check_buffer_available_for(ub, size))
    {
        memcpy(ub->iterator, array, size);
        ub->iterator += size;
        ub->offset += size;
    }
    else
    {
        size_t remaining_size = size;
        size_t serialization_size;
        while (0 < (serialization_size = ucdr_check_final_buffer_behavior_array(ub, remaining_size, data_size)))
        {
            memcpy(ub->iterator, array + (size - remaining_size), serialization_size);
            remaining_size -= serialization_size;
            ub->iterator += serialization_size;
            ub->offset += serialization_size;
        }
    }
    ub->last_data_size = (uint8_t)data_size;
}

void ucdr_buffer_to_array(
        ucdrBuffer* ub,
        uint8_t* array,
        size_t size,
        size_t data_size)
{
    if (ucdr_check_buffer_available_for(ub, size))
    {
        memcpy(array, ub->iterator, size);
        ub->iterator += size;
        ub->offset += size;
    }
    else
    {
        size_t remaining_size = size;
        size_t deserialization_size;
        while (0 < (deserialization_size = ucdr_check_final_buffer_behavior_array(ub, remaining_size, data_size)))
        {
            memcpy(array + (size - remaining_size), ub->iterator, deserialization_size);
            remaining_size -= deserialization_size;
            ub->iterator += deserialization_size;
            ub->offset += deserialization_size;
        }
    }
    ub->last_data_size = (uint8_t)data_size;
}

#define UCDR_SERIALIZE_ARRAY_BYTE_1(TYPE, ENDIAN) \
    (void)ENDIAN; \
    ucdr_array_to_buffer(ub, (uint8_t*)array, size, 1); \
    return !ub->error;

#define UCDR_SERIALIZE_ARRAY_BYTE_N(TYPE, TYPE_SIZE, ENDIAN) \
    size_t alignment = ucdr_buffer_alignment(ub, TYPE_SIZE); \
    uint8_t last_data_size = ub->last_data_size; \
    ucdr_advance_buffer(ub, alignment); \
    ub->last_data_size = last_data_size; \
    if (UCDR_MACHINE_ENDIANNESS == ENDIAN) \
    { \
        ucdr_array_to_buffer(ub, (uint8_t*)array, size * TYPE_SIZE, TYPE_SIZE); \
    } \
    else \
    { \
        for (uint32_t i = 0; i < size; ++i) \
        { \
            ucdr_serialize_endian_ ## TYPE(ub, ENDIAN, *(array + i)); \
        } \
    } \
    return !ub->error;

#define UCDR_SERIALIZE_ARRAY_BYTE_2(TYPE, ENDIAN) UCDR_SERIALIZE_ARRAY_BYTE_N(TYPE, 2, ENDIAN)
#define UCDR_SERIALIZE_ARRAY_BYTE_4(TYPE, ENDIAN) UCDR_SERIALIZE_ARRAY_BYTE_N(TYPE, 4, ENDIAN)
#define UCDR_SERIALIZE_ARRAY_BYTE_8(TYPE, ENDIAN) UCDR_SERIALIZE_ARRAY_BYTE_N(TYPE, 8, ENDIAN)

#define UCDR_SERIALIZE_ARRAY_DEFINITION(SUFFIX, TYPE, TYPE_SIZE) \
    bool ucdr_serialize_array ## SUFFIX(ucdrBuffer * ub, const TYPE * array, size_t size) \
    { \
        UCDR_SERIALIZE_ARRAY_BYTE_ ## TYPE_SIZE(TYPE, ub->endianness) \
    } \
    bool ucdr_serialize_endian_array ## SUFFIX(ucdrBuffer * ub, const ucdrEndianness endianness, const TYPE * array, \
            size_t size) \
    { \
        UCDR_SERIALIZE_ARRAY_BYTE_ ## TYPE_SIZE(TYPE, endianness) \
    } \

// -------------------------------------------------------------------
//                         DESERIALIZE MACROS
// -------------------------------------------------------------------
#define UCDR_DESERIALIZE_ARRAY_BYTE_1(TYPE, ENDIAN) \
    (void)ENDIAN; \
    ucdr_buffer_to_array(ub, (uint8_t*)array, size, 1); \
    return !ub->error;

#define UCDR_DESERIALIZE_ARRAY_BYTE_N(TYPE, TYPE_SIZE, ENDIAN) \
    size_t alignment = ucdr_buffer_alignment(ub, TYPE_SIZE); \
    uint8_t last_data_size = ub->last_data_size; \
    ucdr_advance_buffer(ub, alignment); \
    ub->last_data_size = last_data_size; \
    if (UCDR_MACHINE_ENDIANNESS == ENDIAN) \
    { \
        ucdr_buffer_to_array(ub, (uint8_t*)array, size * TYPE_SIZE, TYPE_SIZE); \
    } \
    else \
    { \
        for (uint32_t i = 0; i < size; ++i) \
        { \
            ucdr_deserialize_endian_ ## TYPE(ub, ENDIAN, array + i); \
        } \
    } \
    return !ub->error;

#define UCDR_DESERIALIZE_ARRAY_BYTE_2(TYPE, ENDIAN) UCDR_DESERIALIZE_ARRAY_BYTE_N(TYPE, 2, ENDIAN)
#define UCDR_DESERIALIZE_ARRAY_BYTE_4(TYPE, ENDIAN) UCDR_DESERIALIZE_ARRAY_BYTE_N(TYPE, 4, ENDIAN)
#define UCDR_DESERIALIZE_ARRAY_BYTE_8(TYPE, ENDIAN) UCDR_DESERIALIZE_ARRAY_BYTE_N(TYPE, 8, ENDIAN)

#define UCDR_DESERIALIZE_ARRAY_DEFINITION(SUFFIX, TYPE, TYPE_SIZE) \
    bool ucdr_deserialize_array ## SUFFIX(ucdrBuffer * ub, TYPE * array, size_t size) \
    { \
        UCDR_DESERIALIZE_ARRAY_BYTE_ ## TYPE_SIZE(TYPE, ub->endianness) \
    } \
    bool ucdr_deserialize_endian_array ## SUFFIX(ucdrBuffer * ub, ucdrEndianness endianness, TYPE * array, size_t size) \
    { \
        UCDR_DESERIALIZE_ARRAY_BYTE_ ## TYPE_SIZE(TYPE, endianness) \
    }

// -------------------------------------------------------------------
//                         DEFINITION MACRO
// -------------------------------------------------------------------
#define UCDR_ARRAY_DEFINITIONS(SUFFIX, TYPE, TYPE_SIZE) \
    UCDR_SERIALIZE_ARRAY_DEFINITION(SUFFIX, TYPE, TYPE_SIZE) \
    UCDR_DESERIALIZE_ARRAY_DEFINITION(SUFFIX, TYPE, TYPE_SIZE) \

// -------------------------------------------------------------------
//              PUBLIC SERIALIZATION IMPLEMENTATIONS
// -------------------------------------------------------------------
UCDR_ARRAY_DEFINITIONS(_char, char, 1)
UCDR_ARRAY_DEFINITIONS(_bool, bool, 1)
UCDR_ARRAY_DEFINITIONS(_uint8_t, uint8_t, 1)
UCDR_ARRAY_DEFINITIONS(_uint16_t, uint16_t, 2)
UCDR_ARRAY_DEFINITIONS(_uint32_t, uint32_t, 4)
UCDR_ARRAY_DEFINITIONS(_uint64_t, uint64_t, 8)
UCDR_ARRAY_DEFINITIONS(_int8_t, int8_t, 1)
UCDR_ARRAY_DEFINITIONS(_int16_t, int16_t, 2)
UCDR_ARRAY_DEFINITIONS(_int32_t, int32_t, 4)
UCDR_ARRAY_DEFINITIONS(_int64_t, int64_t, 8)
UCDR_ARRAY_DEFINITIONS(_float, float, 4)
UCDR_ARRAY_DEFINITIONS(_double, double, 8)
