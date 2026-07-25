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

#ifndef _SRC_C_CORE_SESSION_SESSION_INFO_INTERNAL_H_
#define _SRC_C_CORE_SESSION_SESSION_INFO_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/session_info.h>
#include "../serialization/xrce_header_internal.h"

#define CREATE_CLIENT_PAYLOAD_SIZE 16
#define DELETE_CLIENT_PAYLOAD_SIZE 4

#define MIN_HEADER_SIZE 4
#define MAX_HEADER_SIZE (MIN_HEADER_SIZE + CLIENT_KEY_SIZE)

struct ucdrBuffer;
struct BaseObjectRequest;

void uxr_init_session_info(
        uxrSessionInfo* info,
        uint8_t id,
        uint32_t key);

void uxr_buffer_create_session(
        uxrSessionInfo* info,
        struct ucdrBuffer* ub,
        uint16_t mtu);
void uxr_buffer_delete_session(
        uxrSessionInfo* info,
        struct ucdrBuffer* ub);
void uxr_read_create_session_status(
        uxrSessionInfo* info,
        struct ucdrBuffer* ub);
void uxr_read_delete_session_status(
        uxrSessionInfo* info,
        struct ucdrBuffer* ub);

void uxr_stamp_create_session_header(
        const uxrSessionInfo* info,
        uint8_t* buffer);
void uxr_stamp_session_header(
        const uxrSessionInfo* info,
        uint8_t stream_id_raw,
        uxrSeqNum seq_num,
        uint8_t* buffer);
bool uxr_read_session_header(
        const uxrSessionInfo* info,
        struct ucdrBuffer* ub,
        uint8_t* stream_id_raw,
        uxrSeqNum* seq_num);

uint8_t uxr_session_header_offset(
        const uxrSessionInfo* info);

uint16_t uxr_init_base_object_request(
        uxrSessionInfo* info,
        uxrObjectId object_id,
        struct BaseObjectRequest* base);
void uxr_parse_base_object_request(
        const struct BaseObjectRequest* base,
        uxrObjectId* object_id,
        uint16_t* request_id);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _SRC_C_CORE_SESSION_SESSION_INFO_INTERNAL_H_

