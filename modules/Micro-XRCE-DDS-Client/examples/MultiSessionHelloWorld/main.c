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

#include "HelloWorld.h"

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
    (void) object_id; (void) request_id; (void) stream_id; (void) length; (void) args;

    HelloWorld topic;
    HelloWorld_deserialize_topic(ub, &topic);

    char key[20];
    snprintf(key, 20, "0x%X%X%X%X", session->info.key[0], session->info.key[1], session->info.key[2],
            session->info.key[3]);
    printf("Session %s: %s (%i)\n", key, topic.message, topic.index);
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

    // ------ SESSION 1 ------
    // Transport
    uxrUDPTransport transport_1;
    if (!uxr_init_udp_transport(&transport_1, UXR_IPv4, ip, port))
    {
        printf("Error at create transport.\n");
        return 1;
    }

    // Session
    uxrSession session_1;
    uxr_init_session(&session_1, &transport_1.comm, 0x11111111);
    uxr_set_topic_callback(&session_1, on_topic, NULL);
    if (!uxr_create_session(&session_1))
    {
        printf("Error at create session 1.\n");
        return 1;
    }

    // Streams
    uint8_t output_reliable_stream_buffer_1[BUFFER_SIZE];
    uxrStreamId reliable_out_1 = uxr_create_output_reliable_stream(&session_1, output_reliable_stream_buffer_1,
                    BUFFER_SIZE, STREAM_HISTORY);

    uint8_t input_reliable_stream_buffer_1[BUFFER_SIZE];
    uxrStreamId reliable_in_1 = uxr_create_input_reliable_stream(&session_1, input_reliable_stream_buffer_1,
                    BUFFER_SIZE, STREAM_HISTORY);

    // Create entities
    uxrObjectId participant_id_1 = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml_1 = "<dds>"
            "<participant>"
            "<rtps>"
            "<name>default_xrce_participant_1</name>"
            "</rtps>"
            "</participant>"
            "</dds>";
    uint16_t participant_req_1 = uxr_buffer_create_participant_xml(&session_1, reliable_out_1, participant_id_1, 0,
                    participant_xml_1, UXR_REPLACE);

    uxrObjectId topic_id_1_1 = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml_1_1 = "<dds>"
            "<topic>"
            "<name>HelloWorldTopic_1_to_2</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</dds>";
    uint16_t topic_req_1_1 = uxr_buffer_create_topic_xml(&session_1, reliable_out_1, topic_id_1_1, participant_id_1,
                    topic_xml_1_1, UXR_REPLACE);

    uxrObjectId topic_id_1_2 = uxr_object_id(0x02, UXR_TOPIC_ID);
    const char* topic_xml_1_2 = "<dds>"
            "<topic>"
            "<name>HelloWorldTopic_2_to_1</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</dds>";
    uint16_t topic_req_1_2 = uxr_buffer_create_topic_xml(&session_1, reliable_out_1, topic_id_1_2, participant_id_1,
                    topic_xml_1_2, UXR_REPLACE);

    uxrObjectId publisher_id_1 = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml_1 = "";
    uint16_t publisher_req_1 = uxr_buffer_create_publisher_xml(&session_1, reliable_out_1, publisher_id_1,
                    participant_id_1, publisher_xml_1, UXR_REPLACE);

    uxrObjectId datawriter_id_1 = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_xml_1 = "<dds>"
            "<data_writer>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>HelloWorldTopic_1_to_2</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</data_writer>"
            "</dds>";
    uint16_t datawriter_req_1 = uxr_buffer_create_datawriter_xml(&session_1, reliable_out_1, datawriter_id_1,
                    publisher_id_1, datawriter_xml_1, UXR_REPLACE);

    uxrObjectId subscriber_id_1 = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    const char* subscriber_xml_1 = "";
    uint16_t subscriber_req_1 = uxr_buffer_create_subscriber_xml(&session_1, reliable_out_1, subscriber_id_1,
                    participant_id_1, subscriber_xml_1, UXR_REPLACE);

    uxrObjectId datareader_id_1 = uxr_object_id(0x01, UXR_DATAREADER_ID);
    const char* datareader_xml_1 = "<dds>"
            "<data_reader>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>HelloWorldTopic_2_to_1</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</data_reader>"
            "</dds>";
    uint16_t datareader_req_1 = uxr_buffer_create_datareader_xml(&session_1, reliable_out_1, datareader_id_1,
                    subscriber_id_1, datareader_xml_1, UXR_REPLACE);

    // Send create entities message and wait its status
    uint16_t requests_1[] = {
        participant_req_1, topic_req_1_1, topic_req_1_2, publisher_req_1, datawriter_req_1, subscriber_req_1,
        datareader_req_1
    };
    uint8_t status_1[sizeof(requests_1) / 2];

    if (!uxr_run_session_until_all_status(&session_1, 1000, requests_1, status_1, sizeof(status_1)))
    {
        printf("Error at create entities session 1\n");
        return 1;
    }

    // ------ SESSION 2 ------
    // Transport
    uxrUDPTransport transport_2;
    if (!uxr_init_udp_transport(&transport_2, UXR_IPv4, ip, port))
    {
        printf("Error at create transport.\n");
        return 1;
    }

    // Session
    uxrSession session_2;
    uxr_init_session(&session_2, &transport_2.comm, 0x22222222);
    uxr_set_topic_callback(&session_2, on_topic, NULL);
    if (!uxr_create_session(&session_2))
    {
        printf("Error at create session 2.\n");
        return 1;
    }

    // Streams
    uint8_t output_reliable_stream_buffer_2[BUFFER_SIZE];
    uxrStreamId reliable_out_2 = uxr_create_output_reliable_stream(&session_2, output_reliable_stream_buffer_2,
                    BUFFER_SIZE, STREAM_HISTORY);

    uint8_t input_reliable_stream_buffer_2[BUFFER_SIZE];
    uxrStreamId reliable_in_2 = uxr_create_input_reliable_stream(&session_2, input_reliable_stream_buffer_2,
                    BUFFER_SIZE, STREAM_HISTORY);

    // Create entities
    uxrObjectId participant_id_2 = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml_2 = "<dds>"
            "<participant>"
            "<rtps>"
            "<name>default_xrce_participant_2</name>"
            "</rtps>"
            "</participant>"
            "</dds>";
    uint16_t participant_req_2 = uxr_buffer_create_participant_xml(&session_2, reliable_out_2, participant_id_2, 0,
                    participant_xml_2, UXR_REPLACE);

    uxrObjectId topic_id_2_1 = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml_2_1 = "<dds>"
            "<topic>"
            "<name>HelloWorldTopic_2_to_1</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</dds>";
    uint16_t topic_req_2_1 = uxr_buffer_create_topic_xml(&session_2, reliable_out_2, topic_id_2_1, participant_id_2,
                    topic_xml_2_1, UXR_REPLACE);

    uxrObjectId topic_id_2_2 = uxr_object_id(0x02, UXR_TOPIC_ID);
    const char* topic_xml_2_2 = "<dds>"
            "<topic>"
            "<name>HelloWorldTopic_1_to_2</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</dds>";
    uint16_t topic_req_2_2 = uxr_buffer_create_topic_xml(&session_2, reliable_out_2, topic_id_2_2, participant_id_2,
                    topic_xml_2_2, UXR_REPLACE);

    uxrObjectId publisher_id_2 = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml_2 = "";
    uint16_t publisher_req_2 = uxr_buffer_create_publisher_xml(&session_2, reliable_out_2, publisher_id_2,
                    participant_id_2, publisher_xml_2, UXR_REPLACE);

    uxrObjectId datawriter_id_2 = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_xml_2 = "<dds>"
            "<data_writer>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>HelloWorldTopic_2_to_1</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</data_writer>"
            "</dds>";
    uint16_t datawriter_req_2 = uxr_buffer_create_datawriter_xml(&session_2, reliable_out_2, datawriter_id_2,
                    publisher_id_2, datawriter_xml_2, UXR_REPLACE);

    uxrObjectId subscriber_id_2 = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    const char* subscriber_xml_2 = "";
    uint16_t subscriber_req_2 = uxr_buffer_create_subscriber_xml(&session_2, reliable_out_2, subscriber_id_2,
                    participant_id_2, subscriber_xml_2, UXR_REPLACE);

    uxrObjectId datareader_id_2 = uxr_object_id(0x01, UXR_DATAREADER_ID);
    const char* datareader_xml_2 = "<dds>"
            "<data_reader>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>HelloWorldTopic_1_to_2</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</data_reader>"
            "</dds>";
    uint16_t datareader_req_2 = uxr_buffer_create_datareader_xml(&session_2, reliable_out_2, datareader_id_2,
                    subscriber_id_2, datareader_xml_2, UXR_REPLACE);

    // Send create entities message and wait its status
    uint16_t requests_2[] = {
        participant_req_2, topic_req_2_1, topic_req_2_2, publisher_req_2, datawriter_req_2, subscriber_req_2,
        datareader_req_2
    };
    uint8_t status_2[sizeof(requests_2) / 2];

    if (!uxr_run_session_until_all_status(&session_2, 1000, requests_2, status_2, sizeof(status_2)))
    {
        printf("Error at create entities session 2\n");
        return 1;
    }


    // Request topics of both sessions
    uxrDeliveryControl delivery_control = {
        0
    };
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    uxr_buffer_request_data(&session_1, reliable_out_1, datareader_id_1, reliable_in_1,
            &delivery_control);
    uxr_buffer_request_data(&session_2, reliable_out_2, datareader_id_2, reliable_in_2,
            &delivery_control);

    // Write topics
    uint32_t count = 0;
    bool connected = true;
    while (connected)
    {
        // Session 1 publication
        HelloWorld topic_1 = {
            count, "Publisher 1 says hello"
        };
        ucdrBuffer ub_1;
        uint32_t topic_size_1 = HelloWorld_size_of_topic(&topic_1, 0);
        uxr_prepare_output_stream(&session_1, reliable_out_1, datawriter_id_1, &ub_1, topic_size_1);
        HelloWorld_serialize_topic(&ub_1, &topic_1);

        // Session 2 publication
        HelloWorld topic_2 = {
            count, "Publisher 2 says hello"
        };
        ucdrBuffer ub_2;
        uint32_t topic_size_2 = HelloWorld_size_of_topic(&topic_2, 0);
        uxr_prepare_output_stream(&session_2, reliable_out_2, datawriter_id_2, &ub_2, topic_size_2);
        HelloWorld_serialize_topic(&ub_2, &topic_2);

        connected = uxr_run_session_time(&session_1, 1000);
        connected &= uxr_run_session_time(&session_2, 1000);

        count++;
    }

    // Delete resources
    uxr_delete_session(&session_1);
    uxr_delete_session(&session_2);
    uxr_close_udp_transport(&transport_1);
    uxr_close_udp_transport(&transport_2);

    return 0;
}
