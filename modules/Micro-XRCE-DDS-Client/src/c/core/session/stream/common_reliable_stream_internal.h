// Copyright 2017-present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef SRC__C__CORE__SESSION__STREAM__COMMON_RELIABLE_STREAM_INTERNAL_H_
#define SRC__C__CORE__SESSION__STREAM__COMMON_RELIABLE_STREAM_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/stream/reliable_stream.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef uint32_t length_t;
#define INTERNAL_RELIABLE_BUFFER_OFFSET sizeof(length_t)

static inline uint8_t* uxr_get_reliable_buffer(
        uxrReliableStream const* stream,
        uint16_t seq_num)
{
    return stream->buffer
           + ((seq_num % stream->history) * (stream->size / stream->history))
           + INTERNAL_RELIABLE_BUFFER_OFFSET;
}

static inline size_t uxr_get_reliable_buffer_capacity(
        uxrReliableStream const* stream)
{
    return stream->size / stream->history - INTERNAL_RELIABLE_BUFFER_OFFSET;
}

static inline uint16_t uxr_get_reliable_buffer_history_position(
        uxrReliableStream const* stream,
        uint8_t const* current_position)
{
    return (uint16_t)((size_t)(current_position - stream->buffer) / (stream->size / stream->history));
}

static inline size_t uxr_get_reliable_buffer_size(
        uxrReliableStream const* stream,
        uint16_t seq_num)
{
    length_t length;
    memcpy(
        &length,
        uxr_get_reliable_buffer(stream, (seq_num % stream->history)) - INTERNAL_RELIABLE_BUFFER_OFFSET,
        sizeof(length_t));
    return (size_t)length;
}

static inline void uxr_set_reliable_buffer_size(
        uxrReliableStream const* stream,
        uint16_t seq_num,
        size_t length)
{
    length_t temp_length = (length_t)length;
    memcpy(
        uxr_get_reliable_buffer(stream, (seq_num % stream->history)) - INTERNAL_RELIABLE_BUFFER_OFFSET,
        &temp_length,
        INTERNAL_RELIABLE_BUFFER_OFFSET);
}

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // SRC__C__CORE__SESSION__STREAM__COMMON_RELIABLE_STREAM_INTERNAL_H_
