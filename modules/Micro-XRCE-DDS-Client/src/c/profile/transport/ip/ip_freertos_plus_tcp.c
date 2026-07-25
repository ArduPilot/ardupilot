// Copyright 2017-present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <uxr/client/profile/transport/ip/ip.h>
#include <uxr/client/config.h>

#include "FreeRTOS.h"
#include "FreeRTOS_Sockets.h"

bool uxr_ip_to_locator(
        char const* ip,
        uint16_t port,
        uxrIpProtocol ip_protocol,
        TransportLocator* locator)
{
    bool result = false;
    switch (ip_protocol)
    {
        case UXR_IPv4:
            locator->format = ADDRESS_FORMAT_MEDIUM;
            locator->_.medium_locator.locator_port = port;
            uint32_t addr = FreeRTOS_inet_addr(ip);
            memcpy(&locator->_.medium_locator.address, &addr, sizeof(locator->_.medium_locator.address));
            result = locator->_.medium_locator.address != 0;
            break;
        case UXR_IPv6:
            break;
        default:
            break;
    }
    return result;
}

bool uxr_locator_to_ip(
        TransportLocator const* locator,
        char* ip,
        size_t size,
        uint16_t* port,
        uxrIpProtocol* ip_protocol)
{
    bool result = false;
    (void)size;
    switch (locator->format)
    {
        case ADDRESS_FORMAT_MEDIUM:
            *port = locator->_.medium_locator.locator_port;
            *ip_protocol = UXR_IPv4;
            uint32_t addr;
            memcpy(&addr, &locator->_.medium_locator.address, sizeof(locator->_.medium_locator.address));
            FreeRTOS_inet_ntoa(addr, ip);
            result = true;
            break;
        case ADDRESS_FORMAT_LARGE:
            break;
        default:
            break;
    }
    return result;
}