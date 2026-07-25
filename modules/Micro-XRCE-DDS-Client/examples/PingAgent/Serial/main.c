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

#include <fcntl.h>  // O_RDWR, O_NOCTTY, O_NONBLOCK
#include <stdio.h>

int main(
        int argc,
        char** argv)
{
#ifdef WIN32
    printf("Sorry, this example is not available for Windows platforms.");
    return 0;
#else
    // CLI
    if (2 != argc)
    {
        printf("usage: %s <dev>\n", argv[0]);
        return 1;
    }

    char* dev = argv[1];

    uxrSerialTransport transport;
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);


    if (!uxr_init_serial_transport(&transport, fd, 0, 1))
    {
        printf("Error during transport creation\n");
        return 1;
    }

    // Sending ping without initing a XRCE session
    if (uxr_ping_agent_attempts(&transport.comm, 1000, 10))
    {
        printf("Success! Agent is up on device '%s' without a session\n", dev);
    }
    else
    {
        printf("Sorry, no agent available at device '%s'\n", dev);
        return 1;
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
        printf("Success! Agent is up on device '%s' within a session\n", dev);
    }
    else
    {
        printf("Sorry, no agent available at device '%s'\n", dev);
    }

    uxr_close_serial_transport(&transport);

    return 0;
#endif  // WIN32
}
