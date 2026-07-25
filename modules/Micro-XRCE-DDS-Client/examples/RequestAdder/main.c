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
#include <inttypes.h>

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_UDP_TRANSPORT_MTU* STREAM_HISTORY

void on_reply(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uint16_t reply_id,
        ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) session;
    (void) object_id;
    (void) request_id;
    (void) length;
    (void) args;

    uint64_t result;
    ucdr_deserialize_uint64_t(ub, &result);

#ifdef WIN32
    printf("Reply received: %I64u [id: %d]\n", result, reply_id);
#else
    printf("Reply received: %" PRIu64 " [id: %d]\n", result, reply_id);
#endif /* ifdef WIN32 */
}

int main(
        int args,
        char** argv)
{
    if (3 > args || 0 == atoi(argv[2]))
    {
        printf("usage: program [-h | --help] | ip port [key]\n");
        return 0;
    }

    char* ip = argv[1];
    char* port = argv[2];
    uint32_t key = (args == 4) ? (uint32_t)atoi(argv[3]) : 0xAAAABBBB;

    // Transport
    uxrUDPTransport transport;
    if (!uxr_init_udp_transport(&transport, UXR_IPv4, ip, port))
    {
        printf("Error at init transport.\n");
        return 1;
    }

    // Session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, key);
    uxr_set_reply_callback(&session, on_reply, false);
    if (!uxr_create_session(&session))
    {
        printf("Error at init session.\n");
        return 1;
    }

    // Streams
    uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
    uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE,
                    STREAM_HISTORY);

    uint8_t input_reliable_stream_buffer[BUFFER_SIZE];
    uxrStreamId reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE,
                    STREAM_HISTORY);

    // Create entities
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
            "<participant>"
            "<rtps>"
            "<name>default_xrce_participant</name>"
            "</rtps>"
            "</participant>"
            "</dds>";
    uint16_t participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0,
                    participant_xml, UXR_REPLACE);

    uxrObjectId requester_id = uxr_object_id(0x01, UXR_REQUESTER_ID);
    const char* requester_xml = "<dds>"
            "<requester profile_name=\"my_requester\""
            "service_name=\"service_name\""
            "request_type=\"request_type\""
            "reply_type=\"reply_type\">"
            "</requester>"
            "</dds>";
    uint16_t requester_req = uxr_buffer_create_requester_xml(&session, reliable_out, requester_id, participant_id,
                    requester_xml, UXR_REPLACE);

    // Send create entities message and wait its status
    uint8_t status[2];
    uint16_t requests[2] = {
        participant_req, requester_req
    };
    if (!uxr_run_session_until_all_status(&session, 1000, requests, status, 2))
    {
        printf("Error at create entities: participant: %i requester: %i\n", status[0], status[1]);
        return 1;
    }

    // Request replies
    uxrDeliveryControl delivery_control = {
        0
    };
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    uxr_buffer_request_data(&session, reliable_out, requester_id, reliable_in,
            &delivery_control);

    // Write requests
    bool connected = true;
    uint32_t count = 0;
    while (connected)
    {
        uint8_t request[2 * 4] = {
            0
        };
        ucdrBuffer ub;

        ucdr_init_buffer(&ub, request, sizeof(request));
        ucdr_serialize_uint32_t(&ub, count);
        ucdr_serialize_uint32_t(&ub, count);

        uint16_t request_id = uxr_buffer_request(&session, reliable_out, requester_id, request, sizeof(request));
        printf("Request sent: (%d + %d) [id: %d]\n", count, count, request_id);
        connected = uxr_run_session_time(&session, 1000);

        ++count;
    }

    return 0;
}
