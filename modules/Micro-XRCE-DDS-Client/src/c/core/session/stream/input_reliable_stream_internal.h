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

#ifndef _SRC_C_CORE_SESSION_INPUT_STREAM_RELIABLE_STREAM_INTERNAL_H_
#define _SRC_C_CORE_SESSION_INPUT_STREAM_RELIABLE_STREAM_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/stream/input_reliable_stream.h>
#include <uxr/client/core/session/stream/seq_num.h>

#include <stdbool.h>
#include <stddef.h>

#define ACKNACK_PAYLOAD_SIZE  5

struct ucdrBuffer;

void uxr_init_input_reliable_stream(
        uxrInputReliableStream* stream,
        uint8_t* buffer,
        size_t size,
        uint16_t history,
        OnGetFragmentationInfo on_get_fragmentation_info);
void uxr_reset_input_reliable_stream(
        uxrInputReliableStream* stream);
bool uxr_receive_reliable_message(
        uxrInputReliableStream* stream,
        uint16_t seq_num,
        uint8_t* buffer,
        size_t length,
        bool* message_stored);
bool uxr_next_input_reliable_buffer_available(
        uxrInputReliableStream* stream,
        struct ucdrBuffer* ub,
        size_t fragment_offset);

uint16_t uxr_compute_acknack(
        const uxrInputReliableStream* stream,
        uxrSeqNum* from);
void uxr_process_heartbeat(
        uxrInputReliableStream* stream,
        uxrSeqNum first_seq_num,
        uxrSeqNum last_seq_num);

bool uxr_is_input_up_to_date(
        const uxrInputReliableStream* stream);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _SRC_C_CORE_SESSION_INPUT_STREAM_RELIABLE_STREAM_INTERNAL_H_
