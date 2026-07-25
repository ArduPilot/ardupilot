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
#include <ucdr/microcdr.h>

#include <stdio.h> //printf
#include <string.h> //strcmp
#include <stdlib.h> //atoi
#include <unistd.h>

#define BUFFER_SIZE             1024
#define HISTORY_SIZE            8
#define SAMPLE_IDENTITY_SIZE    24

uxrObjectId replier_id;
uxrStreamId output_besteffort;

static bool send_msg_empty(
        void* instance,
        const uint8_t* buf,
        size_t len)
{
    (void) instance;
    (void) buf;
    (void) len;

    return true;
}

static bool recv_msg_empty(
        void* instance,
        uint8_t** buf,
        size_t* len,
        int timeout)
{
    (void) instance;
    (void) buf;
    (void) len;
    (void) timeout;

    return false;
}

static void on_request(
        struct uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        SampleIdentity* sample_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    // Process request
    uint32_t in, out;
    ucdr_deserialize_uint32_t(ub, &in);
    out = in * 2;

    // Send reply
    ucdrBuffer replay_ub;
    uxr_prepare_output_stream(session, output_besteffort, replier_id, &replay_ub,
            length + SAMPLE_IDENTITY_SIZE);
    uxr_serialize_SampleIdentity(&replay_ub, sample_id);
    ucdr_serialize_uint32_t(&replay_ub, out);

    printf("Request received: %d. Sending reply: %d\n", in, out);
    uxr_run_session_time(session, 1000);
}

static void on_reply(
        struct uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uint16_t reply_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) session;
    (void) object_id;
    (void) request_id;
    (void) length;
    (void) args;

    // Process reply
    uint32_t out;
    ucdr_deserialize_uint32_t(ub, &out);
    printf("Reply received: %d [id: %d]\n", out, reply_id);
}

int main(
        int args,
        char** argv)
{
    uint32_t count = (args == 2) ? (uint32_t)atoi(argv[1]) : 10;

    // Empty transport
    uxrCommunication comm;
    comm.send_msg = send_msg_empty;
    comm.recv_msg = recv_msg_empty;

    // Session
    uxrSession session;
    uxr_init_session(&session, &comm, 0xAAAABBBB);

    // Streams
    uint8_t output_besteffort_buffer[BUFFER_SIZE];
    output_besteffort = uxr_create_output_best_effort_stream(&session, output_besteffort_buffer, BUFFER_SIZE);

    uint8_t input_reliable_stream_buffer[BUFFER_SIZE * HISTORY_SIZE];
    uxrStreamId reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE,
                    HISTORY_SIZE);

    // Create entities
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);

    // Create requester
    uxrObjectId requester_id = uxr_object_id(0x01, UXR_REQUESTER_ID);
    uxrQoS_t qos = {
        .reliability = UXR_RELIABILITY_RELIABLE, .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
        .history = UXR_HISTORY_KEEP_LAST, .depth = 1
    };

    uxr_buffer_create_requester_bin(&session, output_besteffort, requester_id, participant_id, "shared_memory_reqres",
            "req_type", "res_type", "", "", qos, UXR_REPLACE);
    uxr_set_request_callback(&session, on_request, NULL);

    // Create replier
    replier_id = uxr_object_id(0x01, UXR_REPLIER_ID);
    uxr_buffer_create_replier_bin(&session, reliable_in, replier_id, participant_id, "shared_memory_reqres", "req_type",
            "res_type", "", "", qos, UXR_REPLACE);

    uxr_set_reply_callback(&session, on_reply, NULL);

    // Publish topic
    const size_t data_lenght = sizeof(uint32_t);
    ucdrBuffer ub;

    for (size_t i = 0; i < count; i++)
    {
        uint8_t buffer[data_lenght];
        ucdr_init_buffer(&ub, buffer, sizeof(buffer));
        ucdr_serialize_uint32_t(&ub, i);

        uint16_t req_id = uxr_buffer_request(&session, output_besteffort, requester_id, buffer, data_lenght);

        printf("Sending request: %ld [id: %d]\n", i, req_id);
        uxr_run_session_time(&session, 1000);
    }

    // Delete resources
    uxr_delete_session(&session);

    return 0;
}
