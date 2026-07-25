// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
//#include <uxr/client/core/serialization/xrce_protocol.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#define STREAM_HISTORY  2
#define BUFFER_SIZE     UXR_CONFIG_SERIAL_TRANSPORT_MTU* STREAM_HISTORY

void on_topic(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) session;
    (void) object_id;
    (void) request_id;
    (void) stream_id;
    (void) ub;
    (void) length;
    (void) args;
}

int main()
{
    // Open file descriptor and alloc memory.
    if (0 < mkfifo("/tmp/serial_fifo", S_IRWXU | S_IRWXG | S_IRWXO))
    {
        unlink("/tmp/serial_fifo");
        return 1;
    }
    int fd = open("/tmp/serial_fifo", O_RDWR | O_NONBLOCK);
    int* memory_flag = malloc(sizeof(int));

    // Transport.
    uxrSerialTransport transport = {
        0
    };
    (void) uxr_init_serial_transport(&transport, fd, 0, 1);

    // Session.
    uxrSession session;
    uxr_init_session(&session, &transport.comm, 0xAAAAAAAA);
    uxr_set_topic_callback(&session, on_topic, NULL);

    // Streams
    uint8_t output_reliable_stream_buffer[BUFFER_SIZE] = {
        0
    };
    uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE,
                    STREAM_HISTORY);

    uint8_t input_reliable_stream_buffer[BUFFER_SIZE] = {
        0
    };
    uxrStreamId reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE,
                    STREAM_HISTORY);

    // Create entities.
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_ref = "default_xrce_participant_profile";
    uint16_t participant_req = uxr_buffer_create_participant_ref(&session, reliable_out, participant_id, 0,
                    participant_ref, UXR_REPLACE);

    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml = "<dds><topic><name>HelloWorldTopic</name><dataType>HelloWorld</dataType></topic></dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(&session, reliable_out, topic_id, participant_id, topic_xml,
                    UXR_REPLACE);

    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml = "<publisher name=\"MyPublisher\">";
    uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, reliable_out, publisher_id, participant_id,
                    publisher_xml, UXR_REPLACE);

    uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_xml =
            "<profiles><publisher profile_name=\"default_xrce_publisher_profile\"><topic><kind>NO_KEY</kind><name>HelloWorldTopic</name><dataType>HelloWorld</dataType><historyQos><kind>KEEP_LAST</kind><depth>5</depth></historyQos><durability><kind>TRANSIENT_LOCAL</kind></durability></topic></publisher></profiles>";
    uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session, reliable_out, datawriter_id, publisher_id,
                    datawriter_xml, UXR_REPLACE);

    uxrObjectId subscriber_id = uxr_object_id(0x01, UXR_SUBSCRIBER_ID);
    const char* subscriber_xml = "<subscriber name=\"MySubscriber\">";
    uint16_t subscriber_req = uxr_buffer_create_subscriber_xml(&session, reliable_out, subscriber_id, participant_id,
                    subscriber_xml, UXR_REPLACE);

    uxrObjectId datareader_id = uxr_object_id(0x01, UXR_DATAREADER_ID);
    const char* datareader_xml =
            "<profiles><subscriber profile_name=\"default_xrce_subscriber_profile\"><topic><kind>NO_KEY</kind><name>HelloWorldTopic</name><dataType>HelloWorld</dataType><historyQos><kind>KEEP_LAST</kind><depth>5</depth></historyQos><durability><kind>TRANSIENT_LOCAL</kind></durability></topic></subscriber></profiles>";
    uint16_t datareader_req = uxr_buffer_create_datareader_xml(&session, reliable_out, datareader_id, subscriber_id,
                    datareader_xml, UXR_REPLACE);

    uint8_t status[6];
    uint16_t request[6] = {
        participant_req, topic_req, publisher_req, datawriter_req, subscriber_req, datareader_req
    };
    (void) uxr_run_session_until_all_status(&session, 0, request, status, 6);

    // Request topics
    uxrDeliveryControl delivery_control = {
        0
    };
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    (void) uxr_buffer_request_data(&session, reliable_out, datareader_id, reliable_in, &delivery_control);

    // Delete resources.
    uxr_delete_session(&session);
    uxr_close_serial_transport(&transport);

    // Free resources.
    int b = *memory_flag;
    free(memory_flag);
    unlink("/tmp/serial_fifo");

    return b;
}
