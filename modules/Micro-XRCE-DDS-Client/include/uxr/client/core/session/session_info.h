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

#ifndef _UXR_CLIENT_CORE_SESSION_SESSION_INFO_H_
#define _UXR_CLIENT_CORE_SESSION_SESSION_INFO_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/core/session/stream/seq_num.h>
#include <uxr/client/core/session/object_id.h>
#include <stdbool.h>

#define UXR_STATUS_OK                     0x00
#define UXR_STATUS_OK_MATCHED             0x01
#define UXR_STATUS_ERR_DDS_ERROR          0x80
#define UXR_STATUS_ERR_MISMATCH           0x81
#define UXR_STATUS_ERR_ALREADY_EXISTS     0x82
#define UXR_STATUS_ERR_DENIED             0x83
#define UXR_STATUS_ERR_UNKNOWN_REFERENCE  0x84
#define UXR_STATUS_ERR_INVALID_DATA       0x85
#define UXR_STATUS_ERR_INCOMPATIBLE       0x86
#define UXR_STATUS_ERR_RESOURCES          0x87
#define UXR_STATUS_NONE                   0xFF //Never sent or received. It is used for managing an unknown status

#define UXR_REUSE            0x01 << 1
#define UXR_REPLACE          0x01 << 2
#ifdef PERFORMANCE_TESTING
#define UXR_ECHO             0x01 << 7
#endif // ifdef PERFORMANCE_TESTING

#define UXR_INVALID_REQUEST_ID 0

#define UXR_REQUEST_NONE     0x00
#define UXR_REQUEST_LOGIN    0x01
#define UXR_REQUEST_LOGOUT   0x02

typedef struct uxrSessionInfo
{
    uint8_t id;
    uint8_t key[4];
    uint8_t last_requested_status;
    uint16_t last_request_id;

} uxrSessionInfo;

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _UXR_CLIENT_CORE_SESSION_SESSION_INFO_H
