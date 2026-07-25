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

#ifndef _UXR_CLIENT_CORE_SESSION_STREAM_STREAM_STORAGE_H_
#define _UXR_CLIENT_CORE_SESSION_STREAM_STREAM_STORAGE_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/stream/output_best_effort_stream.h>
#include <uxr/client/core/session/stream/output_reliable_stream.h>
#include <uxr/client/core/session/stream/input_best_effort_stream.h>
#include <uxr/client/core/session/stream/input_reliable_stream.h>
#include <uxr/client/core/session/stream/stream_id.h>
#include <uxr/client/config.h>

typedef struct uxrStreamStorage
{
    uxrOutputBestEffortStream output_best_effort[UXR_CONFIG_MAX_OUTPUT_BEST_EFFORT_STREAMS];
    uint8_t output_best_effort_size;
    uxrOutputReliableStream output_reliable[UXR_CONFIG_MAX_OUTPUT_RELIABLE_STREAMS];
    uint8_t output_reliable_size;
    uxrInputBestEffortStream input_best_effort[UXR_CONFIG_MAX_INPUT_BEST_EFFORT_STREAMS];
    uint8_t input_best_effort_size;
    uxrInputReliableStream input_reliable[UXR_CONFIG_MAX_INPUT_RELIABLE_STREAMS];
    uint8_t input_reliable_size;

} uxrStreamStorage;

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _UXR_CLIENT_CORE_SESSION_STREAM_STREAM_STORAGE_H
