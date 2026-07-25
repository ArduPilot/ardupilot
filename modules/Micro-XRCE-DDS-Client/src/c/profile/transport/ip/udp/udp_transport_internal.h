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

#ifndef SRC_C_PROFILE_TRANSPORT_UDP_UDP_TRANSPORT_INTERNAL_H_
#define SRC_C_PROFILE_TRANSPORT_UDP_UDP_TRANSPORT_INTERNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/profile/transport/ip/udp/udp_transport.h>

bool uxr_init_udp_platform(
        struct uxrUDPPlatform* platform,
        uxrIpProtocol ip_protocol,
        const char* ip,
        const char* port);

bool uxr_close_udp_platform(
        struct uxrUDPPlatform* platform);

size_t uxr_write_udp_data_platform(
        struct uxrUDPPlatform* platform,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode);

size_t uxr_read_udp_data_platform(
        struct uxrUDPPlatform* platform,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode);

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif // SRC_C_PROFILE_TRANSPORT_UDP_UDP_TRANSPORT_INTERNAL_H_
