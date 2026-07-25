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

#ifndef _SRC_C_CORE_SESSION_STREAM_STREAM_STORAGE_INTERNAL_H_
#define _SRC_C_CORE_SESSION_STREAM_STREAM_STORAGE_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/stream/stream_storage.h>
#include <uxr/client/core/session/stream/output_best_effort_stream.h>
#include <uxr/client/core/session/stream/output_reliable_stream.h>
#include <uxr/client/core/session/stream/input_best_effort_stream.h>
#include <uxr/client/core/session/stream/input_reliable_stream.h>
#include <uxr/client/core/session/stream/stream_id.h>
#include <uxr/client/config.h>

void uxr_init_stream_storage(
        uxrStreamStorage* storage);
void uxr_reset_stream_storage(
        uxrStreamStorage* storage);

uxrStreamId uxr_add_output_best_effort_buffer(
        uxrStreamStorage* storage,
        uint8_t* buffer,
        size_t size,
        uint8_t header_offset);
uxrStreamId uxr_add_output_reliable_buffer(
        uxrStreamStorage* storage,
        uint8_t* buffer,
        size_t size,
        uint16_t history,
        uint8_t header_offset);
uxrStreamId uxr_add_input_best_effort_buffer(
        uxrStreamStorage* storage);
uxrStreamId uxr_add_input_reliable_buffer(
        uxrStreamStorage* storage,
        uint8_t* buffer,
        size_t size,
        uint16_t history,
        OnGetFragmentationInfo on_get_fragmentation_info);

uxrOutputBestEffortStream* uxr_get_output_best_effort_stream(
        uxrStreamStorage* storage,
        uint8_t index);
uxrOutputReliableStream* uxr_get_output_reliable_stream(
        uxrStreamStorage* storage,
        uint8_t index);
uxrInputBestEffortStream* uxr_get_input_best_effort_stream(
        uxrStreamStorage* storage,
        uint8_t index);
uxrInputReliableStream* uxr_get_input_reliable_stream(
        uxrStreamStorage* storage,
        uint8_t index);

bool uxr_output_streams_confirmed(
        const uxrStreamStorage* storage);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _SRC_C_CORE_SESSION_STREAM_STREAM_STORAGE_INTERNAL_H_
