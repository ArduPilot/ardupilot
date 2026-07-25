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

#ifndef _SRC_C_PROFILE_TRANSPORT_SERIAL_SERIAL_TRANSPORT_PLATFORM_H_
#define _SRC_C_PROFILE_TRANSPORT_SERIAL_SERIAL_TRANSPORT_PLATFORM_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/profile/transport/serial/serial_transport.h>

bool uxr_init_serial_platform(
        void* args,
        const int fd,
        uint8_t remote_addr,
        uint8_t local_addr);
bool uxr_close_serial_platform(
        void* args);

size_t uxr_write_serial_data_platform(
        void* args,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode);

size_t uxr_read_serial_data_platform(
        void* args,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif //_SRC_C_PROFILE_TRANSPORT_SERIAL_SERIAL_TRANSPORT_PLATFORM_H_
