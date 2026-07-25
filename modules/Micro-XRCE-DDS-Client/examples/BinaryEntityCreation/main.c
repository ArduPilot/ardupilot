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

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_UDP_TRANSPORT_MTU* STREAM_HISTORY

void on_topic(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) session; (void) object_id; (void) request_id; (void) stream_id; (void) length; (void) args;

    int32_t data;
    ucdr_deserialize_int32_t(ub, &data);

    printf("Received topic: %i\n", data);
}

int main(
        int args,
        char** argv)
{
    // CLI
    if (3 > args || 0 == atoi(argv[2]))
    {
        printf("usage: program [-h | --help] | ip port [<max_topics>]\n");
        return 0;
    }

    char* ip = argv[1];
    char* port = argv[2];
    uint32_t max_topics = (args == 4) ? (uint32_t)atoi(argv[3]) : UINT32_MAX;

    // Transport
    uxrUDPTransport transport;
    if (!uxr_init_udp_transport(&transport, UXR_IPv4, ip, port))
    {
        printf("Error at create transport.\n");
        return 1;
    }

    // Session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, 0xAAAABBBB);
    uxr_set_topic_callback(&session, on_topic, NULL);
    if (!uxr_create_session(&session))
    {
        printf("Error at create session.\n");
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
    uint16_t participant_req =
            uxr_buffer_create_participant_bin(&session, reliable_out, participant_id, 0, NULL, UXR_REPLACE);

    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    uint16_t topic_req = uxr_buffer_create_topic_bin(&session, reliable_out, topic_id, participant_id, "ExampleTopic",
                    "ExampleType", UXR_REPLACE);

    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    uint16_t publisher_req = uxr_buffer_create_publisher_bin(&session, reliable_out, publisher_id, participant_id,
                    UXR_REPLACE);

    uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    uxrQoS_t qos = {
        .reliability = UXR_RELIABILITY_RELIABLE, .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
        .history = UXR_HISTORY_KEEP_LAST, .depth = 0
    };
    uint16_t datawriter_req = uxr_buffer_create_datawriter_bin(&session, reliable_out, datawriter_id, publisher_id,
                    topic_id, qos, UXR_REPLACE);

    uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    uint16_t subscriber_req = uxr_buffer_create_subscriber_bin(&session, reliable_out, subscriber_id, participant_id,
                    UXR_REPLACE);

    uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
    uint16_t datareader_req = uxr_buffer_create_datareader_bin(&session, reliable_out, datareader_id, subscriber_id,
                    topic_id, qos, UXR_REPLACE);

    // Send create entities message and wait its status
    uint16_t requests[] = {
        participant_req, topic_req, publisher_req, datawriter_req, subscriber_req, datareader_req
    };
    uint8_t status[sizeof(requests) / 2];
    if (!uxr_run_session_until_all_status(&session, 1000, requests, status, sizeof(status)))
    {
        printf("Error at create entities.\n");
        return 1;
    }

    // Request topics
    uxrDeliveryControl delivery_control = {
        0
    };
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    uxr_buffer_request_data(&session, reliable_out, datareader_id, reliable_in,
            &delivery_control);

    // Write topics
    bool connected = true;
    uint32_t count = 0;
    while (connected && count < max_topics)
    {
        ucdrBuffer ub;
        uxr_prepare_output_stream(&session, reliable_out, datawriter_id, &ub, 4);
        ucdr_serialize_int32_t(&ub, count);
        printf("Send topic: %i\n", count);
        count++;

        connected = uxr_run_session_time(&session, 1000);
    }

    // Delete resources
    uxr_delete_session(&session);
    uxr_close_udp_transport(&transport);

    return 0;
}
