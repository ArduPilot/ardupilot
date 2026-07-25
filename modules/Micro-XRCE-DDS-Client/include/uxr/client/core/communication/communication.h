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

#ifndef _UXR_CLIENT_CORE_COMMUNICATION_COMMUNICATION_H_
#define _UXR_CLIENT_CORE_COMMUNICATION_COMMUNICATION_H_

#ifdef __cplusplus
extern "C"
{
#endif // ifdef __cplusplus

#include <uxr/client/config.h>

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef UCLIENT_PROFILE_MULTITHREAD
#include <uxr/client/profile/multithread/multithread.h>
#endif // ifdef UCLIENT_PROFILE_MULTITHREAD

typedef bool (* send_msg_func)(
        void* instance,
        const uint8_t* buf,
        size_t len);
typedef bool (* recv_msg_func)(
        void* instance,
        uint8_t** buf,
        size_t* len,
        int timeout);
typedef uint8_t (* comm_error_func)(
        void);

typedef struct uxrCommunication
{
    void* instance;
    send_msg_func send_msg;
    recv_msg_func recv_msg;
    comm_error_func comm_error;
    uint16_t mtu;

#ifdef UCLIENT_PROFILE_MULTITHREAD
    uxrMutex mutex;
#endif // ifdef UCLIENT_PROFILE_MULTITHREAD

} uxrCommunication;

#ifdef __cplusplus
}
#endif // ifdef __cplusplus

#endif //_UXR_CLIENT_CORE_COMMUNICATION_COMMUNICATION_H_
