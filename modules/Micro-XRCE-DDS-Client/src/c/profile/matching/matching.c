#include "matching_internal.h"

#include <uxr/client/core/type/xrce_types.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

//==================================================================
//                            PRIVATE
//==================================================================

// djb2 by Dan Bernstein: http://www.cse.yorku.ca/~oz/hash.html
void hash_string(
        unsigned char* str,
        char* hash,
        bool initial)
{
    hash_int_t int_hash = 5381;

    if (initial)
    {
        int_hash = *((hash_int_t*) hash);
    }

    int c;

    do {
        c = *str++;
        int_hash = ((int_hash << 5) + int_hash) + (hash_int_t)c; /* hash * 33 + c */
    } while (c);

    for (size_t i = 0; i < UXR_MATCHING_HASH_SIZE; i++)
    {
        hash[i] = ((char*)&int_hash)[i];
    }
}

// Find first occurrence of tag in XML
bool find_tag_xml(
        const char* xml,
        size_t len,
        char* tag,
        const char** content,
        size_t* content_len)
{
    size_t tag_len = strlen(tag);
    bool found_begin = false;
    bool found_end = false;
    for (size_t i = 1; i < len; i++)
    {
        if (!found_begin && 0 == memcmp(&xml[i], tag, tag_len) && xml[i - 1] == '<')
        {
            size_t tag_opener_len = 0;
            while (xml[i + tag_opener_len] != '>')
            {
                tag_opener_len++;
            }
            *content = &xml[i + tag_opener_len + 1];
            found_begin = true;
        }
        else if (found_begin && 0 == memcmp(&xml[i], tag, tag_len) && xml[i - 1] == '/')
        {
            *content_len = (size_t)(&xml[i - 2] - *content);
            found_end = true;
            break;
        }
    }

    return found_begin && found_end;
}

// Find property in first occurrence of tag in XML
bool find_tag_property(
        const char* xml,
        size_t len,
        const char* tag,
        char* property,
        const char** content,
        size_t* content_len)
{
    size_t tag_len = strlen(tag);
    size_t property_len = strlen(property);

    bool found_tag = false;
    bool found_property = false;
    for (size_t i = 1; i < len; i++)
    {
        if (!found_tag && 0 == memcmp(&xml[i], tag, tag_len) && xml[i - 1] == '<')
        {
            found_tag = true;
        }
        else if (found_tag && 0 == memcmp(&xml[i], property, property_len))
        {
            *content = &xml[i + property_len + 2];
            i += property_len + 2;
            *content_len = 0;
            while (xml[i + (*content_len)] != '"')
            {
                *content_len += 1;
            }
            found_property = true;
            break;
        }
    }

    return found_tag && found_property;
}

bool uxr_generate_hash_from_xml(
        const char* xml,
        uxrObjectId id,
        char* hash)
{
    bool found = true;
    char name_type_buffer[100];

    switch (id.type)
    {
        case UXR_DATAREADER_ID:
        case UXR_DATAWRITER_ID:
        {
            const char* data_reader_or_writer = (id.type == UXR_DATAREADER_ID) ? "data_reader\0" : "data_writer\0";
            char xml_strings[3][12] =
            {
                "dds",
                "",
                "topic"
            };
            memmove(xml_strings[1], data_reader_or_writer, 12);

            const char* content_in = xml;
            char* content_out;
            size_t content_len_in = strlen(content_in);
            size_t content_len_out;

            for (size_t i = 0; i < 3; i++)
            {
                if (find_tag_xml(content_in, content_len_in, xml_strings[i], (const char**)&content_out,
                        &content_len_out))
                {
                    content_in = content_out;
                    content_len_in = content_len_out;
                }
                else
                {
                    return false;
                }
            }

            size_t topic_name_len;
            size_t type_name_len;

            found &= find_tag_xml(content_in, content_len_in, "name", (const char**)&content_out, &topic_name_len);
            memcpy(name_type_buffer, content_out, topic_name_len);

            found &= find_tag_xml(content_in, content_len_in, "dataType", (const char**)&content_out, &type_name_len);
            memcpy(&name_type_buffer[topic_name_len], content_out, type_name_len);

            name_type_buffer[topic_name_len + type_name_len] = '\0';

            hash_string((unsigned char*) name_type_buffer, hash, false);
            break;
        }
        case UXR_REQUESTER_ID:
        case UXR_REPLIER_ID:
        {
            char* content_out;
            size_t service_name_len;
            size_t request_type_name_len;
            size_t reply_type_name_len;

            found &= find_tag_property(xml,
                            strlen(xml),
                            (id.type == UXR_REQUESTER_ID) ? "requester" : "replier",
                            "service_name",
                            (const char**)&content_out,
                            &service_name_len);
            if (found)
            {
                memcpy(name_type_buffer, content_out, service_name_len);
            }

            found &= find_tag_property(xml,
                            strlen(xml),
                            (id.type == UXR_REQUESTER_ID) ? "requester" : "replier",
                            "request_type",
                            (const char**)&content_out,
                            &request_type_name_len);
            if (found)
            {
                memcpy(&name_type_buffer[service_name_len], content_out, request_type_name_len);
            }

            found &= find_tag_property(xml,
                            strlen(xml),
                            (id.type == UXR_REQUESTER_ID) ? "requester" : "replier",
                            "reply_type",
                            (const char**)&content_out,
                            &reply_type_name_len);
            if (found)
            {
                memcpy(&name_type_buffer[service_name_len + request_type_name_len], content_out, reply_type_name_len);
                name_type_buffer[service_name_len + request_type_name_len + reply_type_name_len] = '\0';
            }

            if (found)
            {
                hash_string((unsigned char*) name_type_buffer, hash, false);
            }
            break;
        }
    }

    return found;
}

bool uxr_generate_hash_from_ref(
        const char* ref,
        char* hash)
{
    hash_string((unsigned char*) ref, hash, false);
    return true;
}

bool uxr_generate_hash_from_strings(
        char* hash,
        size_t number_strings,
        ...)
{
    va_list args;
    va_start(args, number_strings);

    char* s = va_arg(args, char*);
    hash_string((unsigned char*) s, hash, false);

    for (size_t i = 1; i < number_strings; i++)
    {
        s = va_arg(args, char*);
        hash_string((unsigned char*) s, hash, true);
    }

    va_end(args);
    return true;
}

bool uxr_match_endpoint_qosbinary(
        const OBJK_Endpoint_QosBinary* entity_1,
        const OBJK_Endpoint_QosBinary* entity_2)
{
    bool matched = true;
    matched &= entity_1->qos_flags == entity_2->qos_flags;
    return matched;
}
