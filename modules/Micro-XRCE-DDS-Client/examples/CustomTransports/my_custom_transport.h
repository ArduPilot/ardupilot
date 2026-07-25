
// Copyright 2021 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef _my_custom_transport_H_
#define _my_custom_transport_H_

#include <uxr/client/profile/transport/custom/custom_transport.h>

bool my_custom_transport_open(
        uxrCustomTransport* transport);
bool my_custom_transport_close(
        uxrCustomTransport* transport);
size_t my_custom_transport_write(
        uxrCustomTransport* transport,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode);
size_t my_custom_transport_read(
        uxrCustomTransport* transport,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode);

#endif // ifndef _my_custom_transport_H_
