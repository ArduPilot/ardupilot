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

#include <uxr/client/client.h>

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif /* ifdef WIN32 */

int64_t avg_time_offset = 0;
size_t sample_id = 0;

void on_time(
        uxrSession* session,
        int64_t current_time,
        int64_t transmit_timestamp,
        int64_t received_timestamp,
        int64_t originate_timestamp,
        void* args)
{
    (void) args;
    int64_t current_time_offset = ((current_time - originate_timestamp) - (transmit_timestamp - received_timestamp)) /
            2;
    avg_time_offset = (avg_time_offset * sample_id + current_time_offset) / (sample_id + 1);
    ++sample_id;
    session->time_offset = avg_time_offset;
}

int main(
        int args,
        char** argv)
{
    // CLI
    if (3 > args || 0 == atoi(argv[2]))
    {
        printf("usage: program [-h | --help] | ip port\n");
        return 0;
    }

    char* ip = argv[1];
    char* port = argv[2];

    // Transport
    uxrUDPTransport transport;
    if (!uxr_init_udp_transport(&transport, UXR_IPv4, ip, port))
    {
        printf("Error at create transport.\n");
        return 1;
    }

    // Session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, 0xCCCCDDDD);
    if (!uxr_create_session(&session))
    {
        printf("Error at create session.\n");
        return 1;
    }

    // Set time-callback.
    uxr_set_time_callback(&session, on_time, NULL);

    // Synchronize with the Agent
    bool synchronized = false;
    do
    {
        synchronized = uxr_sync_session(&session, 1000);
#ifdef WIN32
        printf("synchronized with time offset %-5I64d us\n", session.time_offset / 1000);
        Sleep(1);
#else
        printf("synchronized with time offset %-5" PRId64 "us\n", session.time_offset / 1000);
        sleep(1);
#endif /* ifdef WIN32 */

    } while (synchronized);

    // Delete resources
    uxr_delete_session(&session);
    uxr_close_udp_transport(&transport);

    return 0;
}
