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

#ifndef _SRC_C_CORE_SESSION_SUBMESSAGE_INTERNAL_H_
#define _SRC_C_CORE_SESSION_SUBMESSAGE_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/visibility.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SUBHEADER_SIZE 4

struct ucdrBuffer;

typedef enum SubmessageId
{
    SUBMESSAGE_ID_CREATE_CLIENT     = 0,
    SUBMESSAGE_ID_CREATE            = 1,
    SUBMESSAGE_ID_GET_INFO          = 2,
    SUBMESSAGE_ID_DELETE            = 3,
    SUBMESSAGE_ID_STATUS_AGENT      = 4,
    SUBMESSAGE_ID_STATUS            = 5,
    SUBMESSAGE_ID_INFO              = 6,
    SUBMESSAGE_ID_WRITE_DATA        = 7,
    SUBMESSAGE_ID_READ_DATA         = 8,
    SUBMESSAGE_ID_DATA              = 9,
    SUBMESSAGE_ID_ACKNACK           = 10,
    SUBMESSAGE_ID_HEARTBEAT         = 11,
    SUBMESSAGE_ID_RESET             = 12,
    SUBMESSAGE_ID_FRAGMENT          = 13,
    SUBMESSAGE_ID_TIMESTAMP         = 14,
    SUBMESSAGE_ID_TIMESTAMP_REPLY   = 15
#ifdef PERFORMANCE_TESTING
    ,
    SUBMESSAGE_ID_PERFORMANCE   = 14
#endif // ifdef PERFORMANCE_TESTING

} SubmessageId;

typedef enum SubmessageFlags
{
    FLAG_ENDIANNESS  =           0x01,
    FLAG_LAST_FRAGMENT =         0x01 << 1,
    FLAG_FORMAT_DATA =           0x00,
    FLAG_FORMAT_SAMPLE =         0x02,
    FLAG_FORMAT_DATA_SEQ =       0x08,
    FLAG_FORMAT_SAMPLE_SEQ =     0x0A,
    FLAG_FORMAT_PACKED_SAMPLES = 0x0E

} SubmessageFlags;

bool uxr_buffer_submessage_header(
        struct ucdrBuffer* ub,
        uint8_t submessage_id,
        uint16_t length,
        uint8_t flags);
bool uxr_read_submessage_header(
        struct ucdrBuffer* ub,
        uint8_t* submessage_id,
        uint16_t* length,
        uint8_t* flags);
size_t uxr_submessage_padding(
        size_t length);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // _SRC_C_CORE_SESSION_SUBMESSAGE_INTERNAL_H_
