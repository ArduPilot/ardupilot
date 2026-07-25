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

#ifndef SRC_C_PROFILE_TRANSPORT_CAN_CAN_TRANSPORT_INTERNAL_H_
#define SRC_C_PROFILE_TRANSPORT_CAN_CAN_TRANSPORT_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/profile/transport/can/can_transport.h>

bool uxr_init_can_platform(
        struct uxrCANPlatform* platform,
        const char* dev,
        uint32_t can_id);

bool uxr_close_can_platform(
        struct uxrCANPlatform* platform);

size_t uxr_write_can_data_platform(
        struct uxrCANPlatform* platform,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode);

size_t uxr_read_can_data_platform(
        struct uxrCANPlatform* platform,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // SRC_C_PROFILE_TRANSPORT_CAN_CAN_TRANSPORT_INTERNAL_H_
