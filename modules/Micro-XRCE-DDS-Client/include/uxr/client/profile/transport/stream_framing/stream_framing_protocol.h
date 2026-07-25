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

#ifndef _UXR_CLIENT_PROFILE_TRANSPORT_STREAM_FRAMING_PROTOCOL_H_
#define _UXR_CLIENT_PROFILE_TRANSPORT_STREAM_FRAMING_PROTOCOL_H_
#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <stdint.h>

#define UXR_FRAMING_BEGIN_FLAG 0x7E
#define UXR_FRAMING_ESC_FLAG 0x7D
#define UXR_FRAMING_XOR_FLAG 0x20

typedef enum uxrFramingInputState
{
    UXR_FRAMING_UNINITIALIZED,
    UXR_FRAMING_READING_SRC_ADDR,
    UXR_FRAMING_READING_DST_ADDR,
    UXR_FRAMING_READING_LEN_LSB,
    UXR_FRAMING_READING_LEN_MSB,
    UXR_FRAMING_READING_PAYLOAD,
    UXR_FRAMING_READING_CRC_LSB,
    UXR_FRAMING_READING_CRC_MSB,

} uxrFramingInputState;

typedef struct uxrFramingIO
{
    uxrFramingInputState state;
    uint8_t local_addr;
    uint8_t rb[42];
    uint8_t rb_head;
    uint8_t rb_tail;
    uint8_t src_addr;
    uint16_t msg_len;
    uint16_t msg_pos;
    uint16_t msg_crc;
    uint16_t cmp_crc;
    uint8_t wb[42];
    uint8_t wb_pos;

} uxrFramingIO;

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif //_UXR_CLIENT_PROFILE_TRANSPORT_STREAM_FRAMING_PROTOCOL_H_
