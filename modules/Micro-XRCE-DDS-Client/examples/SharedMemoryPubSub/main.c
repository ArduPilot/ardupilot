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

#define BUFFER_SIZE     1024
#define HISTORY_SIZE    8

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

void on_topic(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) session; (void) object_id; (void) request_id; (void) stream_id; (void) length;

    uint32_t out;
    ucdr_deserialize_uint32_t(ub, &out);
    printf("Received value: %d\n", out);
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
    uxrStreamId output_besteffort =
            uxr_create_output_best_effort_stream(&session, output_besteffort_buffer, BUFFER_SIZE);

    uint8_t input_reliable_stream_buffer[BUFFER_SIZE * HISTORY_SIZE];
    uxrStreamId reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE,
                    HISTORY_SIZE);

    // Create entities
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);

    // Create publisher
    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);

    uxr_buffer_create_topic_bin(&session, output_besteffort, topic_id, participant_id,
            "shared_memory_topic", "shared_memory_type", UXR_REPLACE);
    uxrQoS_t qos = {
        .reliability = UXR_RELIABILITY_RELIABLE, .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
        .history = UXR_HISTORY_KEEP_LAST, .depth = 10
    };
    uxr_buffer_create_datawriter_bin(&session, output_besteffort, datawriter_id, publisher_id, topic_id, qos,
            UXR_REPLACE);

    // Create subscriber
    uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
    uxr_buffer_create_datareader_bin(&session, reliable_in, datareader_id, subscriber_id, topic_id, qos, UXR_REPLACE);

    uxr_set_topic_callback(&session, on_topic, NULL);

    // Publish topic
    const size_t data_lenght = sizeof(uint32_t);
    ucdrBuffer ub;

    for (uint32_t i = 0; i < count; i++)
    {
        uxr_prepare_output_stream(&session, output_besteffort, datawriter_id, &ub, data_lenght);
        ucdr_serialize_uint32_t(&ub, i);

        printf("Publish value: %d\n", i);
        uxr_run_session_time(&session, 1000);
    }

    // Delete resources
    uxr_delete_session(&session);

    return 0;
}
