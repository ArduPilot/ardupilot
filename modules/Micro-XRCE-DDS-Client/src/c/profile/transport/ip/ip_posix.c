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

#include <arpa/inet.h>
#if defined(UCLIENT_PLATFORM_POSIX_NOPOLL)
#include <sys/socket.h>
#endif /* if defined(UCLIENT_PLATFORM_POSIX) */

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
            result = (1 == inet_pton(AF_INET, ip, locator->_.medium_locator.address));
            break;
        case UXR_IPv6:
            locator->format = ADDRESS_FORMAT_LARGE;
            locator->_.large_locator.locator_port = port;
            result = (1 == inet_pton(AF_INET6, ip, locator->_.large_locator.address));
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
    switch (locator->format)
    {
        case ADDRESS_FORMAT_MEDIUM:
            *port = locator->_.medium_locator.locator_port;
            *ip_protocol = UXR_IPv4;
            result = (NULL != inet_ntop(AF_INET, locator->_.medium_locator.address, ip, (socklen_t)size));
            break;
        case ADDRESS_FORMAT_LARGE:
            *port = (uint16_t)locator->_.large_locator.locator_port;
            *ip_protocol = UXR_IPv6;
            result = (NULL != inet_ntop(AF_INET6, locator->_.large_locator.address, ip, (socklen_t)size));
            break;
        default:
            break;
    }
    return result;
}
