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

#include <uxr/client/core/session/session_info.h>
#include <uxr/client/core/type/xrce_types.h>
#include <uxr/client/util/time.h>
#include <uxr/client/config.h>

#include "../serialization/xrce_header_internal.h"
#include "../session/submessage_internal.h"
#include "log_internal.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#define YELLOW_DARK    "\x1B[0;33m"
#define PURPLE_DARK    "\x1B[0;35m"
#define GREY_LIGHT     "\x1B[0;37m"

#define GREY_DARK      "\x1B[1;30m"
#define RED            "\x1B[1;31m"
#define GREEN          "\x1B[1;32m"
#define YELLOW         "\x1B[1;33m"
#define BLUE           "\x1B[1;34m"
#define PURPLE         "\x1B[1;35m"
#define CYAN           "\x1B[1;36m"
#define WHITE          "\x1B[1;37m"

#define RESTORE_COLOR  "\x1B[0m"

#define SEND_ARROW       "==> "
#define ERROR_SEND_ARROW "!=> "
#define RECV_ARROW       "<== "
#define ERROR_RECV_ARROW "<=! "

/* Compute output buffer size. */
#if !defined(UXR_CONFIG_SERIAL_TRANSPORT_MTU)
#define UXR_CONFIG_SERIAL_TRANSPORT_MTU 0
#endif /* if !defined(UXR_CONFIG_SERIAL_TRANSPORT_MTU) */
#if !defined(UXR_CONFIG_UDP_TRANSPORT_MTU)
#define UXR_CONFIG_UDP_TRANSPORT_MTU 0
#endif /* if !defined(UXR_CONFIG_UDP_TRANSPORT_MTU) */
#if !defined(UXR_CONFIG_TCP_TRANSPORT_MTU)
#define UXR_CONFIG_TCP_TRANSPORT_MTU 0
#endif /* if !defined(UXR_CONFIG_TCP_TRANSPORT_MTU) */

#if UXR_CONFIG_UDP_TRANSPORT_MTU > UXR_CONFIG_TCP_TRANSPORT_MTU
#define MAX_UDP_TCP_MTU UXR_CONFIG_UDP_TRANSPORT_MTU
#else
#define MAX_UDP_TCP_MTU UXR_CONFIG_TCP_TRANSPORT_MTU
#endif /* if UXR_CONFIG_UDP_TRANSPORT_MTU > UXR_CONFIG_TCP_TRANSPORT_MTU */
#if UXR_CONFIG_SERIAL_TRANSPORT_MTU > MAX_UDP_TCP_MTU
#define MAX_MTU_CONFIG UXR_CONFIG_SERIAL_TRANSPORT_MTU
#else
#define MAX_MTU_CONFIG MAX_UDP_TCP_MTU
#endif /* if UXR_CONFIG_SERIAL_TRANSPORT_MTU > MAX_UDP_TCP_MTU */

#define DATA_TO_STRING_BUFFER     MAX_MTU_CONFIG* 3 + 100

static char* print_array_2(
        const uint8_t* array_2);
static const char* data_to_string(
        const uint8_t* data,
        uint32_t size);
static const char* request_to_string(
        const BaseObjectRequest* request);
static const char* reply_to_string(
        const BaseObjectReply* reply);
static void print_create_client_submessage(
        const char* pre,
        const CREATE_CLIENT_Payload* payload);
static void print_create_submessage(
        const char* pre,
        const CREATE_Payload* payload,
        uint8_t flags);
static void print_get_info_submessage(
        const char* pre,
        const GET_INFO_Payload* payload);
static void print_delete_submessage(
        const char* pre,
        const DELETE_Payload* payload);
static void print_status_agent_submessage(
        const char* pre,
        const STATUS_AGENT_Payload* payload);
static void print_status_submessage(
        const char* pre,
        const STATUS_Payload* payload);
static void print_info_submessage(
        const char* pre,
        const INFO_Payload* payload);
static void print_read_data_submessage(
        const char* pre,
        const READ_DATA_Payload* payload);
static void print_write_data_data_submessage(
        const char* pre,
        const WRITE_DATA_Payload_Data* payload);
static void print_data_data_submessage(
        const char* pre,
        const DATA_Payload_Data* payload);
static void print_acknack_submessage(
        const char* pre,
        const ACKNACK_Payload* payload);
static void print_heartbeat_submessage(
        const char* pre,
        const HEARTBEAT_Payload* payload);
static void print_fragment_submessage(
        const char* pre,
        uint16_t size,
        uint8_t flags);
static void print_header(
        size_t size,
        int direction,
        uint8_t stream_id,
        uint16_t seq_num,
        const uint8_t* client_key);
static void print_tail(
        int64_t initial_log_time);


//==================================================================
//                             PUBLIC
//==================================================================
void uxr_print_message(
        int direction,
        uint8_t* buffer,
        size_t size,
        const uint8_t* client_key)
{
    static int64_t initial_log_time = 0;

    const char* color;
    switch (direction)
    {
        case UXR_SEND:
            color = YELLOW;
            break;
        case UXR_ERROR_SEND:
            color = RED;
            break;
        case UXR_RECV:
            color = PURPLE;
            break;
        case UXR_ERROR_RECV:
            color = RED;
            break;
        default:
            color = WHITE;
            break;
    }

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer, (uint32_t)size);

    uint8_t session_id; uint8_t stream_id_raw; uint16_t seq_num; uint8_t key[CLIENT_KEY_SIZE];
    (void) uxr_deserialize_message_header(&ub, &session_id, &stream_id_raw, &seq_num, key);

    print_header(size, direction, stream_id_raw, seq_num, client_key);

    size_t submessage_counter = 0;
    uint8_t submessage_id; uint16_t length; uint8_t flags;
    while (uxr_read_submessage_header(&ub, &submessage_id, &length, &flags))
    {
        if (submessage_counter != 0)
        {
            printf("\n                                        ");
        }

        switch (submessage_id)
        {
            case SUBMESSAGE_ID_CREATE_CLIENT:
            {
                initial_log_time = uxr_millis();
                CREATE_CLIENT_Payload payload;
                uxr_deserialize_CREATE_CLIENT_Payload(&ub, &payload);
                print_create_client_submessage(color, &payload);
                break;
            }

            case SUBMESSAGE_ID_CREATE:
            {
                char string_buffer[DATA_TO_STRING_BUFFER];
                CREATE_Payload payload;
                payload.object_representation._.participant.base.representation._.xml_string_represenatation =
                        string_buffer;
                payload.object_representation._.publisher.base.representation._.string_represenatation = string_buffer;

                uxr_deserialize_CREATE_Payload(&ub, &payload);
                print_create_submessage(color, &payload, flags);
                break;
            }

            case SUBMESSAGE_ID_GET_INFO:
            {
                GET_INFO_Payload payload;
                uxr_deserialize_GET_INFO_Payload(&ub, &payload);
                print_get_info_submessage(color, &payload);
                break;
            }

            case SUBMESSAGE_ID_DELETE:
            {
                DELETE_Payload payload;
                uxr_deserialize_DELETE_Payload(&ub, &payload);
                print_delete_submessage(color, &payload);
                break;
            }

            case SUBMESSAGE_ID_STATUS_AGENT:
            {
                STATUS_AGENT_Payload payload;
                uxr_deserialize_STATUS_AGENT_Payload(&ub, &payload);
                print_status_agent_submessage(color, &payload);
                break;
            }

            case SUBMESSAGE_ID_STATUS:
            {
                STATUS_Payload payload;
                uxr_deserialize_STATUS_Payload(&ub, &payload);
                print_status_submessage(color, &payload);
                break;
            }

            case SUBMESSAGE_ID_INFO:
            {
                INFO_Payload payload;
                uxr_deserialize_INFO_Payload(&ub, &payload);
                print_info_submessage(color, &payload);
                break;
            }

            case SUBMESSAGE_ID_WRITE_DATA:
            {
                WRITE_DATA_Payload_Data payload;

                uint8_t* it = ub.iterator;
                uxr_deserialize_WRITE_DATA_Payload_Data(&ub, &payload);
                print_write_data_data_submessage(color, &payload);
                ub.iterator = it + length;
                break;
            }

            case SUBMESSAGE_ID_DATA:
            {
                DATA_Payload_Data payload;

                uint8_t* it = ub.iterator;
                uxr_deserialize_DATA_Payload_Data(&ub, &payload);
                print_data_data_submessage(color, &payload);
                ub.iterator = it + length;
                break;
            }

            case SUBMESSAGE_ID_READ_DATA:
            {
                char string_buffer[DATA_TO_STRING_BUFFER];
                READ_DATA_Payload payload;
                payload.read_specification.content_filter_expression = string_buffer;

                uxr_deserialize_READ_DATA_Payload(&ub, &payload);
                print_read_data_submessage(color, &payload);
                break;
            }

            case SUBMESSAGE_ID_HEARTBEAT:
            {
                HEARTBEAT_Payload payload;
                uxr_deserialize_HEARTBEAT_Payload(&ub, &payload);
                print_heartbeat_submessage(color, &payload);
                break;
            }

            case SUBMESSAGE_ID_ACKNACK:
            {
                ACKNACK_Payload payload;
                uxr_deserialize_ACKNACK_Payload(&ub, &payload);
                print_acknack_submessage(color, &payload);
                break;
            }

            case SUBMESSAGE_ID_FRAGMENT:
            {
                print_fragment_submessage(color, length, flags);
                ub.iterator += length;
                break;
            }

            default:
            {
                printf("%s[UNKNOWN SUBMESSAGE]%s", RED, RESTORE_COLOR);
                goto tail;
            }
        }

        //Check if must be force to advance to the length
        submessage_counter++;
    }
tail:
    print_tail(initial_log_time);
    printf(" \n");
}

void uxr_print_serialization(
        int direction,
        const uint8_t* buffer,
        size_t size)
{
    const char* dir;
    switch (direction)
    {
        case UXR_SEND:
            dir = SEND_ARROW;
            break;
        case UXR_ERROR_SEND:
            dir = ERROR_SEND_ARROW;
            break;
        case UXR_RECV:
            dir = RECV_ARROW;
            break;
        case UXR_ERROR_RECV:
            dir = ERROR_RECV_ARROW;
            break;
        default:
            dir = "";
            break;
    }

    printf("%s%s<< [%zu]: %s>>%s\n",
            dir,
            GREY_DARK,
            size,
            data_to_string(buffer, (uint32_t)size),
            RESTORE_COLOR);
}

//==================================================================
//                             PRIVATE
//==================================================================
char* print_array_2(
        const uint8_t* array_2)
{
    static char buffer[249];
    sprintf(buffer, "%02X%02X", array_2[0], array_2[1]);
    return buffer;
}

const char* data_to_string(
        const uint8_t* data,
        uint32_t size)
{
    static char buffer[DATA_TO_STRING_BUFFER];
    for (uint32_t i = 0; i < size; i++)
    {
        sprintf(buffer + 3 * i, "%02X ", data[i]);
    }
    buffer[3 * size] = '\0';
    return buffer;
}

const char* request_to_string(
        const BaseObjectRequest* request)
{
    static char buffer[256];
    int pos = sprintf(buffer, "req: 0x%s", print_array_2(request->request_id.data));
    sprintf(buffer + pos, " | obj: 0x%s", print_array_2(request->object_id.data));

    return buffer;
}

const char* reply_to_string(
        const BaseObjectReply* reply)
{
    static char buffer[256];

    char status[64];
    switch (reply->result.status)
    {
        case UXR_STATUS_OK:
            sprintf(status, "OK");
            break;
        case UXR_STATUS_OK_MATCHED:
            sprintf(status, "OK_MATCHED");
            break;
        case UXR_STATUS_ERR_DDS_ERROR:
            sprintf(status, "ERR_DDS_ERROR");
            break;
        case UXR_STATUS_ERR_MISMATCH:
            sprintf(status, "ERR_MISMATCH");
            break;
        case UXR_STATUS_ERR_ALREADY_EXISTS:
            sprintf(status, "ERR_ALREADY_EXISTS");
            break;
        case UXR_STATUS_ERR_DENIED:
            sprintf(status, "ERR_DENIED");
            break;
        case UXR_STATUS_ERR_UNKNOWN_REFERENCE:
            sprintf(status, "ERR_UNKNOWN_REFERENCE");
            break;
        case UXR_STATUS_ERR_INVALID_DATA:
            sprintf(status, "ERR_INVALID_DATA");
            break;
        case UXR_STATUS_ERR_INCOMPATIBLE:
            sprintf(status, "ERR_INCOMPATIBLE");
            break;
        case UXR_STATUS_ERR_RESOURCES:
            sprintf(status, "ERR_RESOURCES");
            break;
        default:
            sprintf(status, "UNKNOWN");
    }

    sprintf(buffer, "%s | %s",
            request_to_string(&reply->related_request),
            status);

    return buffer;
}

void print_create_client_submessage(
        const char* pre,
        const CREATE_CLIENT_Payload* payload)
{
    printf("%s[CREATE CLIENT | session: 0x%02X | key: %s]%s",
            pre,
            payload->client_representation.session_id,
            data_to_string(payload->client_representation.client_key.data, 4),
            RESTORE_COLOR);
}

void print_create_submessage(
        const char* pre,
        const CREATE_Payload* payload,
        uint8_t flags)
{
    char type[4];
    switch (payload->object_representation._.participant.base.representation.format)
    {
        case DDS_XRCE_REPRESENTATION_AS_XML_STRING:
            strcpy(type, "xml");
            break;

        case DDS_XRCE_REPRESENTATION_BY_REFERENCE:
            strcpy(type, "ref");
            break;

        case DDS_XRCE_REPRESENTATION_IN_BINARY:
            strcpy(type, "bin");
            break;

    }

    char content[DATA_TO_STRING_BUFFER];
    switch (payload->object_representation.kind)
    {
        case DDS_XRCE_OBJK_PARTICIPANT:
            sprintf(content, "PARTICIPANT | %s: %zu",
                    type,
                    strlen(
                        payload->object_representation._.participant.base.representation._.xml_string_represenatation) +
                    1);
            break;
        case DDS_XRCE_OBJK_TOPIC:
            sprintf(content, "TOPIC | obj: 0x%s | %s: %zu",
                    print_array_2(payload->object_representation._.data_reader.subscriber_id.data),
                    type,
                    strlen(payload->object_representation._.topic.base.representation._.xml_string_represenatation) +
                    1);
            break;
        case DDS_XRCE_OBJK_PUBLISHER:
            sprintf(content, "PUBLISHER | obj: 0x%s | %s: %zu",
                    print_array_2(payload->object_representation._.publisher.participant_id.data),
                    type,
                    strlen(payload->object_representation._.publisher.base.representation._.string_represenatation) +
                    1);
            break;
        case DDS_XRCE_OBJK_SUBSCRIBER:
            sprintf(content, "SUBSCRIBER | obj: 0x%s | %s: %zu",
                    print_array_2(payload->object_representation._.subscriber.participant_id.data),
                    type,
                    strlen(payload->object_representation._.subscriber.base.representation._.string_represenatation) +
                    1);
            break;
        case DDS_XRCE_OBJK_DATAWRITER:
            sprintf(content, "DATAWRITER | obj: 0x%s | %s: %zu",
                    print_array_2(payload->object_representation._.data_writer.publisher_id.data),
                    type,
                    strlen(
                        payload->object_representation._.data_writer.base.representation._.xml_string_represenatation) +
                    1);
            break;
        case DDS_XRCE_OBJK_DATAREADER:
            sprintf(content, "DATAREADER | obj: 0x%s | %s: %zu",
                    print_array_2(payload->object_representation._.data_reader.subscriber_id.data),
                    type,
                    strlen(
                        payload->object_representation._.data_reader.base.representation._.xml_string_represenatation) +
                    1);
            break;
        default:
            sprintf(content, "UNKNOWN");
    }

    const char* reuse_flag = (flags & UXR_REUSE) ? "REUSE" : "";
    const char* replace_flag = (flags & UXR_REPLACE) ? "REPLACE" : "";
    const char* separator =
            ((flags & UXR_REUSE && flags & UXR_REPLACE) || (!(flags & UXR_REUSE) && !(flags & UXR_REPLACE))) ? " " : "";

    printf("%s[CREATE | %s%s%s | %s | %s]%s",
            pre,
            reuse_flag,
            separator,
            replace_flag,
            request_to_string(&payload->base),
            content,
            RESTORE_COLOR);
}

void print_get_info_submessage(
        const char* pre,
        const GET_INFO_Payload* payload)
{
    const char* config = (payload->info_mask & 1) ? "CONFIG" : "";
    const char* activity = (payload->info_mask & 2) ? "ACTIVITY" : "";

    printf("%s[GET INFO | %s | %s | %s]%s",
            pre,
            request_to_string(&payload->base),
            config,
            activity,
            RESTORE_COLOR);
}

void print_delete_submessage(
        const char* pre,
        const DELETE_Payload* payload)
{
    printf("%s[DELETE | %s]%s",
            pre,
            request_to_string(&payload->base),
            RESTORE_COLOR);
}

void print_status_agent_submessage(
        const char* pre,
        const STATUS_AGENT_Payload* payload)
{
    (void) payload;
    printf("%s[STATUS AGENT]%s",
            pre,
            RESTORE_COLOR);
}

void print_status_submessage(
        const char* pre,
        const STATUS_Payload* payload)
{
    printf("%s[STATUS | %s]%s",
            pre,
            reply_to_string(&payload->base),
            RESTORE_COLOR);
}

void print_info_submessage(
        const char* pre,
        const INFO_Payload* payload)
{
    const uint8_t* ip = payload->object_info.activity._.agent.address_seq.data[0]._.medium_locator.address;
    uint16_t port = payload->object_info.activity._.agent.address_seq.data[0]._.medium_locator.locator_port;

    printf("%s[INFO | %s | ip: %u.%u.%u.%u | port: %u]%s",
            pre,
            reply_to_string(&payload->base),
            ip[0], ip[1], ip[2], ip[3],
            port,
            RESTORE_COLOR);
}

void print_read_data_submessage(
        const char* pre,
        const READ_DATA_Payload* payload)
{
    char format[128];
    switch (payload->read_specification.data_format)
    {
        case FORMAT_DATA:
            sprintf(format, "DATA");
            break;
        case FORMAT_DATA_SEQ:
            sprintf(format, "DATA_SEQ");
            break;
        case FORMAT_SAMPLE:
            sprintf(format, "SAMPLE");
            break;
        case FORMAT_SAMPLE_SEQ:
            sprintf(format, "SAMPLE_SEQ");
            break;
        case FORMAT_PACKED_SAMPLES:
            sprintf(format, "PACKED_SAMPLES");
            break;
        default:
            sprintf(format, "UNKNOWN");
            break;
    }

    printf("%s[READ DATA | format: %s | istream: 0x%02X | %s | dc: %s]%s",
            pre,
            format,
            payload->read_specification.preferred_stream_id,
            request_to_string(&payload->base),
            (payload->read_specification.optional_delivery_control) ? "yes" : "no",
            RESTORE_COLOR);
}

void print_write_data_data_submessage(
        const char* pre,
        const WRITE_DATA_Payload_Data* payload)
{
    printf("%s[WRITE DATA | format: DATA | %s]%s",
            pre,
            request_to_string(&payload->base),
            RESTORE_COLOR);
}

void print_data_data_submessage(
        const char* pre,
        const DATA_Payload_Data* payload)
{
    printf("%s[DATA | format: DATA | %s]%s",
            pre,
            request_to_string(&payload->base),
            RESTORE_COLOR);
}

void print_acknack_submessage(
        const char* pre,
        const ACKNACK_Payload* payload)
{
    char bitmask[17] = {
        0
    };
    for (int i = 0; i < 8; i++)
    {
        bitmask[15 - i] = (payload->nack_bitmap[1] & (1 << i)) ? '1' : '0';
        bitmask[7 - i] = (payload->nack_bitmap[0] & (1 << i)) ? '1' : '0';
    }

    printf("%s[ACKNACK | stream: 0x%02X | seq_num: %hu | bitmap: %s]%s",
            pre,
            payload->stream_id,
            payload->first_unacked_seq_num,
            bitmask,
            RESTORE_COLOR);
}

void print_heartbeat_submessage(
        const char* pre,
        const HEARTBEAT_Payload* payload)
{
    printf("%s[HEARTBEAT | stream: 0x%02X | first: %hu | last: %hu]%s",
            pre,
            payload->stream_id,
            payload->first_unacked_seq_nr,
            payload->last_unacked_seq_nr,
            RESTORE_COLOR);
}

void print_fragment_submessage(
        const char* pre,
        uint16_t size,
        uint8_t flags)
{
    printf("%s[FRAGMENT | size: %hu | %s]%s",
            pre,
            size,
            (FLAG_LAST_FRAGMENT & flags) ? "last" : "intermediate",
            RESTORE_COLOR);
}

void print_header(
        size_t size,
        int direction,
        uint8_t stream_id,
        uint16_t seq_num,
        const uint8_t* client_key)
{
    const char* arrow;
    const char* color;
    switch (direction)
    {
        case UXR_SEND:
            arrow = SEND_ARROW;
            color = YELLOW;
            break;
        case UXR_ERROR_SEND:
            arrow = ERROR_SEND_ARROW;
            color = RED;
            break;
        case UXR_RECV:
            arrow = RECV_ARROW;
            color = PURPLE;
            break;
        case UXR_ERROR_RECV:
            arrow = ERROR_RECV_ARROW;
            color = RED;
            break;
        default:
            arrow = "";
            color = WHITE;
            break;
    }

    char stream_representation;
    if (0 == stream_id)
    {
        stream_representation = 'n';
    }
    else if (0x80 <= stream_id)
    {
        stream_representation = 'r';
    }
    else // if(1 <= stream_id && 80 > stream_id)
    {
        stream_representation = 'b';
    }

    const char* client_key_str = client_key ? data_to_string(client_key, 4) : "-";
    printf("%s%s%3zu: %s(key: %s| %c:%2X |%3hu) %s",
            GREY_LIGHT,
            arrow,
            size,
            color,
            client_key_str,
            stream_representation,
            stream_id,
            seq_num,
            RESTORE_COLOR);
}

void print_tail(
        int64_t initial_log_time)
{
    int64_t ms = uxr_millis() - initial_log_time;
#ifdef WIN32
    printf(" %st: %I64ims%s", BLUE, ms, RESTORE_COLOR);
#else
    printf(" %st: %" PRId64 "ms%s", BLUE, ms, RESTORE_COLOR);
#endif /* ifdef WIN32 */
}
