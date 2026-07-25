// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef _SRC_C_PROFILE_TRANSPORT_FRAMING_PROTOCOL_INTERNAL_H_
#define _SRC_C_PROFILE_TRANSPORT_FRAMING_PROTOCOL_INTERNAL_H_
#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/profile/transport/stream_framing/stream_framing_protocol.h>
#include <stddef.h>
#include <stdbool.h>

#define UXR_FRAMING_BEGIN_FLAG 0x7E
#define UXR_FRAMING_ESC_FLAG 0x7D
#define UXR_FRAMING_XOR_FLAG 0x20

void uxr_init_framing_io(
        uxrFramingIO* framing_io,
        uint8_t local_addr);

void uxr_update_crc(
        uint16_t* crc,
        const uint8_t data);
bool uxr_get_next_octet(
        uxrFramingIO* framing_io,
        uint8_t* octet);
bool uxr_add_next_octet(
        uxrFramingIO* framing_io,
        uint8_t octet);

typedef size_t (* uxr_write_cb)(
        void*,
        const uint8_t*,
        size_t,
        uint8_t*);
typedef size_t (* uxr_read_cb)(
        void*,
        uint8_t*,
        size_t,
        int,
        uint8_t*);

size_t uxr_write_framed_msg(
        uxrFramingIO* framing_io,
        uxr_write_cb write_cb,
        void* cb_arg,
        const uint8_t* buf,
        size_t len,
        uint8_t remote_addr,
        uint8_t* errcode);

size_t uxr_read_framed_msg(
        uxrFramingIO* framing_io,
        uxr_read_cb read_cb,
        void* cb_arg,
        uint8_t* buf,
        size_t len,
        uint8_t* remote_addr,
        int* timeout,
        uint8_t* errcode);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif //_SRC_C_PROFILE_TRANSPORT_FRAMING_PROTOCOL_INTERNAL_H_
