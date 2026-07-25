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

#ifndef _UXR_CLIENT_CORE_SESSION_STREAM_OUTPUT_BEST_EFFORT_STREAM_H_
#define _UXR_CLIENT_CORE_SESSION_STREAM_OUTPUT_BEST_EFFORT_STREAM_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/config.h>
#include <uxr/client/core/session/stream/seq_num.h>

#ifdef UCLIENT_PROFILE_MULTITHREAD
#include <uxr/client/profile/multithread/multithread.h>
#endif // ifdef UCLIENT_PROFILE_MULTITHREAD

#include <stddef.h>
#include <stdbool.h>

struct ucdrBuffer;
typedef struct uxrOutputBestEffortStream
{
    uint8_t* buffer;
    size_t writer;
    size_t size;
    uint8_t offset;

    uxrSeqNum last_send;

#ifdef UCLIENT_PROFILE_MULTITHREAD
    uxrMutex mutex;
#endif // ifdef UCLIENT_PROFILE_MULTITHREAD

} uxrOutputBestEffortStream;

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _UXR_CLIENT_CORE_SESSION_STREAM_OUTPUT_BEST_EFFORT_STREAM_H
