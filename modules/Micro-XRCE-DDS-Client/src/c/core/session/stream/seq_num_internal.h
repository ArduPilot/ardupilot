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

#ifndef _SRC_C_CORE_SESSION_STREAM_SEQ_NUM_INTERNAL_H_
#define _SRC_C_CORE_SESSION_STREAM_SEQ_NUM_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus


#include <uxr/client/core/session/stream/seq_num.h>

#define SEQ_NUM_MAX       UINT16_MAX

uxrSeqNum uxr_seq_num_add(
        uxrSeqNum seq_num,
        uint16_t increment);
uxrSeqNum uxr_seq_num_sub(
        uxrSeqNum seq_num,
        uint16_t decrement);
int uxr_seq_num_cmp(
        uxrSeqNum seq_num_1,
        uxrSeqNum seq_num_2);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _SRC_C_CORE_SESSION_STREAM_SEQ_NUM_INTERNAL_H_
