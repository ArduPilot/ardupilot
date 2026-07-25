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

#define STREAM_HISTORY  4
#define BUFFER_SIZE     100 * STREAM_HISTORY

bool flush_session(
        uxrSession* session,
        void* args)
{
    (void) args;
    return uxr_run_session_until_confirm_delivery(session, 1000);
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
    uxr_init_session(&session, &transport.comm, 0xAAAABBBB);
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
    uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    // Create entities
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
            "<participant>"
            "<rtps>"
            "<name>default_xrce_participant</name>"
            "</rtps>"
            "</participant>"
            "</dds>";
    uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0,
            participant_xml, UXR_REPLACE);
    uxr_run_session_until_confirm_delivery(&session, 100);
    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
            "<topic>"
            "<name>HelloWorldTopic</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</dds>";
    uxr_buffer_create_topic_xml(&session, reliable_out, topic_id, participant_id, topic_xml,
            UXR_REPLACE);
    uxr_run_session_until_confirm_delivery(&session, 100);
    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml = "";
    uxr_buffer_create_publisher_xml(&session, reliable_out, publisher_id, participant_id,
            publisher_xml, UXR_REPLACE);

    uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_xml = "<dds>"
            "<data_writer>"
            "<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>HelloWorldTopic</name>"
            "<dataType>HelloWorld</dataType>"
            "</topic>"
            "</data_writer>"
            "</dds>";
    uxr_buffer_create_datawriter_xml(&session, reliable_out, datawriter_id, publisher_id,
            datawriter_xml, UXR_REPLACE);

    uxr_run_session_until_confirm_delivery(&session, 100);

    // Write topic
    char buf[20000];
    memset(buf, 'A', sizeof(buf));

    ucdrBuffer ub;
    uxr_prepare_output_stream_fragmented(&session, reliable_out, datawriter_id, &ub, sizeof(buf), flush_session, NULL);
    ucdr_serialize_array_char(&ub, buf, sizeof(buf));

    uxr_run_session_until_confirm_delivery(&session, 1000);

    // Delete resources
    uxr_delete_session(&session);
    uxr_close_udp_transport(&transport);

    return 0;
}
