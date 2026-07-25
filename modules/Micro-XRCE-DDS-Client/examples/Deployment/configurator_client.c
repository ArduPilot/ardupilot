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
#include <string.h> //strcmp
#include <stdlib.h> //atoi

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_UDP_TRANSPORT_MTU* STREAM_HISTORY

#define ACTION_CREATE 1
#define ACTION_DELETE 2

#define TARGET_PUBLISHER 1 << 8
#define TARGET_SUBSCRIBER 2 << 8

static void on_status(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uint8_t status,
        void* args);

static void create_publisher(
        uxrSession* session,
        uint16_t ids);
static void create_subscriber(
        uxrSession* session,
        uint16_t ids);
static void delete_publisher(
        uxrSession* session,
        uint16_t ids);
static void delete_subscriber(
        uxrSession* session,
        uint16_t ids);

static void wait_status(
        uxrSession* session,
        uint16_t* requests);

int main(
        int args,
        char** argv)
{
    // Check args
    uint32_t key = 0;
    int create_delete = 0;
    int pub_sub = 0;
    uint16_t ids = 0;

    if (args == 7)
    {
        key = atoi(argv[2]);
        if (0 == strcmp(argv[3], "create"))
        {
            create_delete = ACTION_CREATE;
        }
        else if (0 == strcmp(argv[3], "delete"))
        {
            create_delete = ACTION_DELETE;
        }

        if (0 == strcmp(argv[4], "pub"))
        {
            pub_sub = TARGET_PUBLISHER;
        }
        else if (0 == strcmp(argv[4], "sub"))
        {
            pub_sub = TARGET_SUBSCRIBER;
        }
        ids = atoi(argv[6]);
    }

    if (args < 5 || 0 == strcmp("-h", argv[1]) || 0 == strcmp("--help", argv[1])
            || 0 != strcmp("--key", argv[1]) || 0 != strcmp("--id", argv[5])
            || 0 == atoi(argv[2]) || 0 == atoi(argv[6])
            || !create_delete || !pub_sub)
    {
        printf("usage: program [-h | --help] | --key <number> <'create'/'delete'> <'pub'/'sub'> --id <id>]\n");
        return 0;
    }

    // Transport
    uxrUDPTransport transport;
    if (!uxr_init_udp_transport(&transport, UXR_IPv4, "127.0.0.1", "2018"))
    {
        printf("Error at create transport.\n");
        return 1;
    }

    // Session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, key);
    uxr_set_status_callback(&session, on_status, NULL);
    if (!uxr_create_session(&session))
    {
        printf("Error at create session.\n");
        return 1;
    }

    // Streams
    uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
    uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    uint8_t input_reliable_stream_buffer[BUFFER_SIZE];
    uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    switch (create_delete | pub_sub)
    {
        case ACTION_CREATE | TARGET_PUBLISHER:
            create_publisher(&session, ids);
            break;

        case ACTION_CREATE | TARGET_SUBSCRIBER:
            create_subscriber(&session, ids);
            break;

        case ACTION_DELETE | TARGET_PUBLISHER:
            delete_publisher(&session, ids);
            break;

        case ACTION_DELETE | TARGET_SUBSCRIBER:
            delete_subscriber(&session, ids);
            break;
    }

    uxr_close_udp_transport(&transport);

    return 0;
}

void create_publisher(
        uxrSession* session,
        uint16_t id)
{
    uxrStreamId output = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);

    uxrObjectId participant_id = uxr_object_id(id, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
            "<participant>"
            "<rtps>"
            "<name>default_xrce_participant</name>"
            "</rtps>"
            "</participant>"
            "</dds>";
    uint16_t participant_req =
            uxr_buffer_create_participant_xml(session, output, participant_id, 0, participant_xml, 0);

    uxrObjectId topic_id = uxr_object_id(id, UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
            "<topic>"
            "<name>HelloWorldTopic</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(session, output, topic_id, participant_id, topic_xml, 0);

    uxrObjectId publisher_id = uxr_object_id(id, UXR_PUBLISHER_ID);
    const char* publisher_xml = "";
    uint16_t publisher_req = uxr_buffer_create_publisher_xml(session, output, publisher_id, participant_id,
                    publisher_xml, 0);

    uxrObjectId datawriter_id = uxr_object_id(id, UXR_DATAWRITER_ID);
    const char* datawriter_xml = "<dds>"
            "<data_writer>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>HelloWorldTopic</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</data_writer>"
            "</dds>";
    uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(session, output, datawriter_id, publisher_id,
                    datawriter_xml, 0);

    uint16_t requests[4] = {
        participant_req, topic_req, publisher_req, datawriter_req
    };
    wait_status(session, requests);
}

void create_subscriber(
        uxrSession* session,
        uint16_t id)
{
    uxrStreamId output = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);

    uxrObjectId participant_id = uxr_object_id(id, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
            "<participant>"
            "<rtps>"
            "<name>default_xrce_participant</name>"
            "</rtps>"
            "</participant>"
            "</dds>";
    uint16_t participant_req =
            uxr_buffer_create_participant_xml(session, output, participant_id, 0, participant_xml, 0);

    uxrObjectId topic_id = uxr_object_id(id, UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
            "<topic>"
            "<name>HelloWorldTopic</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(session, output, topic_id, participant_id, topic_xml, 0);

    uxrObjectId subscriber_id = uxr_object_id(id, UXR_SUBSCRIBER_ID);
    const char* subscriber_xml = "";
    uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(session, output, subscriber_id, participant_id,
                    subscriber_xml, 0);

    uxrObjectId datareader_id = uxr_object_id(id, UXR_DATAREADER_ID);
    const char* datareader_xml = "<dds>"
            "<data_reader>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>HelloWorldTopic</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</data_reader>"
            "</dds>";
    uint16_t datareader_req = uxr_buffer_create_datareader_xml(session, output, datareader_id, subscriber_id,
                    datareader_xml, 0);

    uint16_t requests[4] = {
        participant_req, topic_req, subscriber_req, datareader_req
    };
    wait_status(session, requests);
}

void delete_publisher(
        uxrSession* session,
        uint16_t id)
{
    uxrStreamId output = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);

    uint16_t datawriter_req = uxr_buffer_delete_entity(session, output, uxr_object_id(id, UXR_DATAWRITER_ID));
    uint16_t publisher_req = uxr_buffer_delete_entity(session, output, uxr_object_id(id, UXR_PUBLISHER_ID));
    uint16_t topic_req = uxr_buffer_delete_entity(session, output, uxr_object_id(id, UXR_TOPIC_ID));
    uint16_t participant_req = uxr_buffer_delete_entity(session, output, uxr_object_id(id, UXR_PARTICIPANT_ID));

    uint16_t requests[4] = {
        participant_req, topic_req, publisher_req, datawriter_req
    };
    wait_status(session, requests);
}

void delete_subscriber(
        uxrSession* session,
        uint16_t id)
{
    uxrStreamId output = uxr_stream_id(0, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);

    uint16_t datareader_req = uxr_buffer_delete_entity(session, output, uxr_object_id(id, UXR_DATAREADER_ID));
    uint16_t subscriber_req = uxr_buffer_delete_entity(session, output, uxr_object_id(id, UXR_SUBSCRIBER_ID));
    uint16_t topic_req = uxr_buffer_delete_entity(session, output, uxr_object_id(id, UXR_TOPIC_ID));
    uint16_t participant_req = uxr_buffer_delete_entity(session, output, uxr_object_id(id, UXR_PARTICIPANT_ID));

    uint16_t requests[4] = {
        participant_req, topic_req, subscriber_req, datareader_req
    };
    wait_status(session, requests);
}

void wait_status(
        uxrSession* session,
        uint16_t* requests)
{
    uint8_t status[4];
    if (uxr_run_session_until_all_status(session, 3000, requests, status, 4))
    {
        printf("Ok\n");
    }
    else
    {
        printf("Error at entities\n");
    }
}

void on_status(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uint8_t status,
        void* args)
{
    (void) session; (void) request_id; (void) args;

    if (status != UXR_STATUS_OK && status != UXR_STATUS_OK_MATCHED)
    {
        printf("Status error: 0x%02X at entity id '%u' of object type '%u'\n", status, object_id.id, object_id.type);
    }
}
