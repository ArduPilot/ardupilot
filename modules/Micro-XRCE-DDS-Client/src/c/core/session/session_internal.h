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

#ifndef _SRC_C_CORE_SESSION_SESSION_INTERNAL_H_
#define _SRC_C_CORE_SESSION_SESSION_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/session.h>

struct ucdrBuffer;

bool uxr_prepare_stream_to_write_submessage(
        uxrSession* session,
        uxrStreamId stream_id,
        size_t payload_size,
        struct ucdrBuffer* ub,
        uint8_t submessage_id,
        uint8_t mode);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _SRC_C_CORE_SESSION_SESSION_INTERNAL_H_

