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

#include <uxr/client/client.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_AGENTS 10

bool on_agent_found(
        const TransportLocator* locator,
        void* args)
{
    (void) args;
    switch (locator->format)
    {
        case ADDRESS_FORMAT_MEDIUM:
        {
            char ip[16];
            uint16_t port;
            uxrIpProtocol ip_protocol;
            uxr_locator_to_ip(locator, ip, sizeof(ip), &port, &ip_protocol);
            printf("Agent found => ip: %s, port: %d\n", ip, port);
            break;
        }
        case ADDRESS_FORMAT_LARGE:
        {
            char ip[46];
            uint16_t port;
            uxrIpProtocol ip_protocol;
            uxr_locator_to_ip(locator, ip, sizeof(ip), &port, &ip_protocol);
            printf("Agent found => ip: %s, port: %d\n", ip, port);
            break;
        }
        default:
            break;
    }
    return false;
}

int main(
        int args,
        char** argv)
{
    if (args < 1 || (args >= 2 && (0 == strcmp("-h", argv[1]) ||
            0 == strcmp("--help", argv[1]) ||
            0 == args % 2 ||
            MAX_AGENTS * 2 < args + 2)))
    {
        printf("usage: program [ -h | --help | [<ip> <port> ...] ]\n");
        return 0;
    }

    if (args == 1)
    {
        uxr_discovery_agents_default(10, 1000, on_agent_found, NULL);
    }
    else
    {
        size_t size = 0;
        TransportLocator agent_list[MAX_AGENTS];
        for (int i = 1; i < args; i += 2, size++)
        {
            uxr_ip_to_locator(argv[i], (uint16_t)atoi(argv[i + 1]), UXR_IPv4, &agent_list[size]);
        }

        uxr_discovery_agents(10, 1000, on_agent_found, NULL, agent_list, size);
    }

    return 0;
}
