// Copyright 2020 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
#include <uxr/client/util/ping.h>

#include <stdio.h>

int main(
        int argc,
        char** argv)
{
    // CLI
    if (3 != argc)
    {
        printf("usage: %s <ip> <port>\n", argv[0]);
        return 1;
    }

    char* ip = argv[1];
    char* port = argv[2];

    uxrTCPTransport transport;

    if (!uxr_init_tcp_transport(&transport, UXR_IPv4, ip, port))
    {
        printf("Error during transport creation\n");
        return 1;
    }

    // Sending ping without initing a XRCE session
    if (uxr_ping_agent_attempts(&transport.comm, 1000, 10))
    {
        printf("Success! Agent is up on %s:%s\n", ip, port);
    }
    else
    {
        printf("Sorry, no agent available at %s:%s\n", ip, port);
    }

    // Sending ping with initing a XRCE session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, 0xAAAABBBB);
    if (!uxr_create_session(&session))
    {
        printf("Error at create session.\n");
        return 1;
    }

    if (uxr_ping_agent_session(&session, 1000, 1))
    {
        printf("Success! Agent is up on %s:%s within a session\n", ip, port);
    }
    else
    {
        printf("Sorry, no agent available at %s:%s\n", ip, port);
    }

    uxr_close_tcp_transport(&transport);

    return 0;
}
