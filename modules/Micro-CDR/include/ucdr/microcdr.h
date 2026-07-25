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

#ifndef _MICROCDR_MICRO_CDR_H_
#define _MICROCDR_MICRO_CDR_H_

#ifdef __cplusplus
extern "C" {
#endif // ifdef __cplusplus

#include <ucdr/visibility.h>
#include <ucdr/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ------------------------------------------------
//                      Types
// ------------------------------------------------

struct ucdrBuffer;
typedef bool (* OnFullBuffer)(
        struct ucdrBuffer* buffer,
        void* args);

typedef enum ucdrEndianness
{
    UCDR_BIG_ENDIANNESS = 0,
    UCDR_LITTLE_ENDIANNESS = 1

} ucdrEndianness;

typedef struct ucdrBuffer
{
    uint8_t* init;
    uint8_t* final;
    uint8_t* iterator;

    size_t origin;
    size_t offset;

    ucdrEndianness endianness;
    uint8_t last_data_size;

    bool error;

    OnFullBuffer on_full_buffer;
    void* args;

} ucdrBuffer;

// ------------------------------------------------
//                 Main functions
// ------------------------------------------------

UCDRDLLAPI void ucdr_init_buffer                        (
        ucdrBuffer* ub,
        uint8_t* data,
        size_t size);
UCDRDLLAPI void ucdr_init_buffer_origin                 (
        ucdrBuffer* ub,
        uint8_t* data,
        size_t size,
        size_t origin);
UCDRDLLAPI void ucdr_init_buffer_origin_offset          (
        ucdrBuffer* ub,
        uint8_t* data,
        size_t size,
        size_t origin,
        size_t offset);
UCDRDLLAPI void ucdr_init_buffer_origin_offset_endian   (
        ucdrBuffer* ub,
        uint8_t* data,
        size_t size,
        size_t origin,
        size_t offset,
        ucdrEndianness endianness);
UCDRDLLAPI void ucdr_copy_buffer                        (
        ucdrBuffer* ub_dest,
        const ucdrBuffer* ub_source);
UCDRDLLAPI void ucdr_set_on_full_buffer_callback        (
        ucdrBuffer* ub,
        OnFullBuffer on_full_buffer,
        void* args);

UCDRDLLAPI void ucdr_reset_buffer        (
        ucdrBuffer* ub);
UCDRDLLAPI void ucdr_reset_buffer_offset (
        ucdrBuffer* ub,
        size_t offset);

UCDRDLLAPI void   ucdr_align_to         (
        ucdrBuffer* ub,
        size_t alignment);
UCDRDLLAPI size_t ucdr_alignment        (
        size_t buffer_position,
        size_t data_size);
UCDRDLLAPI size_t ucdr_buffer_alignment (
        const ucdrBuffer* ub,
        size_t data_size);
UCDRDLLAPI void   ucdr_advance_buffer   (
        ucdrBuffer* ub,
        size_t size);

UCDRDLLAPI size_t         ucdr_buffer_size       (
        const ucdrBuffer* ub);
UCDRDLLAPI size_t         ucdr_buffer_length     (
        const ucdrBuffer* ub);
UCDRDLLAPI size_t         ucdr_buffer_remaining  (
        const ucdrBuffer* ub);
UCDRDLLAPI ucdrEndianness ucdr_buffer_endianness (
        const ucdrBuffer* ub);
UCDRDLLAPI bool           ucdr_buffer_has_error  (
        const ucdrBuffer* ub);

// -------------------------------------------------------------------
//              PUBLIC DE-SERIALIZATION DECLARATIONS
// -------------------------------------------------------------------

#define UCDR_BASIC_TYPE_DECLARATIONS(SUFFIX, TYPE) \
    UCDRDLLAPI bool ucdr_serialize ## SUFFIX(ucdrBuffer * ub, TYPE value); \
    UCDRDLLAPI bool ucdr_serialize_endian ## SUFFIX(ucdrBuffer * ub, ucdrEndianness endianness, TYPE value); \
    UCDRDLLAPI bool ucdr_deserialize ## SUFFIX(ucdrBuffer * ub, TYPE * value); \
    UCDRDLLAPI bool ucdr_deserialize_endian ## SUFFIX(ucdrBuffer * ub, ucdrEndianness endianness, TYPE * value); \

#define UCDR_ARRAY_DECLARATIONS(SUFFIX, TYPE) \
    UCDRDLLAPI bool ucdr_serialize_array ## SUFFIX(ucdrBuffer * ub, const TYPE * array, size_t size); \
    UCDRDLLAPI bool ucdr_serialize_endian_array ## SUFFIX(ucdrBuffer * ub, ucdrEndianness endianness, \
            const TYPE * array, size_t size); \
    UCDRDLLAPI bool ucdr_deserialize_array ## SUFFIX(ucdrBuffer * ub, TYPE * array, size_t size); \
    UCDRDLLAPI bool ucdr_deserialize_endian_array ## SUFFIX(ucdrBuffer * ub, ucdrEndianness endianness, TYPE * array, \
            size_t size); \

#define UCDR_SEQUENCE_DECLARATIONS(SUFFIX, TYPE) \
    UCDRDLLAPI bool ucdr_serialize_sequence ## SUFFIX(ucdrBuffer * ub, const TYPE * array, uint32_t length); \
    UCDRDLLAPI bool ucdr_serialize_endian_sequence ## SUFFIX(ucdrBuffer * ub, ucdrEndianness endianness, \
            const TYPE * array, uint32_t length); \
    UCDRDLLAPI bool ucdr_deserialize_sequence ## SUFFIX(ucdrBuffer * ub, TYPE * array, size_t array_capacity, \
            uint32_t * length); \
    UCDRDLLAPI bool ucdr_deserialize_endian_sequence ## SUFFIX(ucdrBuffer * ub, ucdrEndianness endianness, TYPE * array, \
            size_t array_capacity, uint32_t * length); \

UCDRDLLAPI bool ucdr_serialize_string(
        ucdrBuffer* ub,
        const char* string);
UCDRDLLAPI bool ucdr_serialize_endian_string(
        ucdrBuffer* ub,
        ucdrEndianness endianness,
        const char* string);
UCDRDLLAPI bool ucdr_deserialize_string(
        ucdrBuffer* ub,
        char* string,
        size_t string_capacity);
UCDRDLLAPI bool ucdr_deserialize_endian_string(
        ucdrBuffer* ub,
        ucdrEndianness endianness,
        char* string,
        size_t string_capacity);

// -------------------------------------------------------------------
//               VALID TYPES DECLARATIONS
// -------------------------------------------------------------------

UCDR_BASIC_TYPE_DECLARATIONS(_char, char)
UCDR_BASIC_TYPE_DECLARATIONS(_bool, bool)
UCDR_BASIC_TYPE_DECLARATIONS(_uint8_t, uint8_t)
UCDR_BASIC_TYPE_DECLARATIONS(_uint16_t, uint16_t)
UCDR_BASIC_TYPE_DECLARATIONS(_uint32_t, uint32_t)
UCDR_BASIC_TYPE_DECLARATIONS(_uint64_t, uint64_t)
UCDR_BASIC_TYPE_DECLARATIONS(_int8_t, int8_t)
UCDR_BASIC_TYPE_DECLARATIONS(_int16_t, int16_t)
UCDR_BASIC_TYPE_DECLARATIONS(_int32_t, int32_t)
UCDR_BASIC_TYPE_DECLARATIONS(_int64_t, int64_t)
UCDR_BASIC_TYPE_DECLARATIONS(_float, float)
UCDR_BASIC_TYPE_DECLARATIONS(_double, double)

UCDR_ARRAY_DECLARATIONS(_char, char)
UCDR_ARRAY_DECLARATIONS(_bool, bool)
UCDR_ARRAY_DECLARATIONS(_uint8_t, uint8_t)
UCDR_ARRAY_DECLARATIONS(_uint16_t, uint16_t)
UCDR_ARRAY_DECLARATIONS(_uint32_t, uint32_t)
UCDR_ARRAY_DECLARATIONS(_uint64_t, uint64_t)
UCDR_ARRAY_DECLARATIONS(_int8_t, int8_t)
UCDR_ARRAY_DECLARATIONS(_int16_t, int16_t)
UCDR_ARRAY_DECLARATIONS(_int32_t, int32_t)
UCDR_ARRAY_DECLARATIONS(_int64_t, int64_t)
UCDR_ARRAY_DECLARATIONS(_float, float)
UCDR_ARRAY_DECLARATIONS(_double, double)

UCDR_SEQUENCE_DECLARATIONS(_char, char)
UCDR_SEQUENCE_DECLARATIONS(_bool, bool)
UCDR_SEQUENCE_DECLARATIONS(_uint8_t, uint8_t)
UCDR_SEQUENCE_DECLARATIONS(_uint16_t, uint16_t)
UCDR_SEQUENCE_DECLARATIONS(_uint32_t, uint32_t)
UCDR_SEQUENCE_DECLARATIONS(_uint64_t, uint64_t)
UCDR_SEQUENCE_DECLARATIONS(_int8_t, int8_t)
UCDR_SEQUENCE_DECLARATIONS(_int16_t, int16_t)
UCDR_SEQUENCE_DECLARATIONS(_int32_t, int32_t)
UCDR_SEQUENCE_DECLARATIONS(_int64_t, int64_t)
UCDR_SEQUENCE_DECLARATIONS(_float, float)
UCDR_SEQUENCE_DECLARATIONS(_double, double)

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif //_MICROCDR_MICRO_CDR_H_
