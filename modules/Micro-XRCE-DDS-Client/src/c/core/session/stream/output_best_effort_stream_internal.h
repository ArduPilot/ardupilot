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

#ifndef _SRC_C_CORE_SESSION_STREAM_OUTPUT_BEST_EFFORT_STREAM_INTERNAL_H_
#define _SRC_C_CORE_SESSION_STREAM_OUTPUT_BEST_EFFORT_STREAM_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/stream/output_best_effort_stream.h>
#include <uxr/client/core/session/stream/seq_num.h>

#include <stddef.h>
#include <stdbool.h>

void uxr_init_output_best_effort_stream(
        uxrOutputBestEffortStream* stream,
        uint8_t* buffer,
        size_t size,
        uint8_t offset);
void uxr_reset_output_best_effort_stream(
        uxrOutputBestEffortStream* stream);
bool uxr_prepare_best_effort_buffer_to_write(
        uxrOutputBestEffortStream* stream,
        size_t size,
        struct ucdrBuffer* ub);
bool uxr_prepare_best_effort_buffer_to_send(
        uxrOutputBestEffortStream* stream,
        uint8_t** buffer,
        size_t* length,
        uint16_t* seq_num);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _SRC_C_CORE_SESSION_STREAM_OUTPUT_BEST_EFFORT_STREAM_INTERNAL_H_
