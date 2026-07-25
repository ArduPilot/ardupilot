#include <uxr/client/core/session/create_entities_xml.h>
#include <uxr/client/core/type/xrce_types.h>

#include "common_create_entities_internal.h"
#include "../../profile/shared_memory/shared_memory_internal.h"

#include <string.h>

static uint16_t create_entity_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        const char* xml,
        uint8_t mode,
        CREATE_Payload* payload);

//==================================================================
//                              PUBLIC
//==================================================================
uint16_t uxr_buffer_create_participant_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t domain_id,
        const char* xml,
        uint8_t mode)
{
    //assert with the object_id type

    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_PARTICIPANT;
    payload.object_representation._.participant.domain_id = (int16_t)domain_id;

    return create_entity_xml(session, stream_id, object_id, xml, mode, &payload);
}

uint16_t uxr_buffer_create_topic_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* xml,
        uint8_t mode)
{
    //assert with the object_id type

    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_TOPIC;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.topic.participant_id.data);

    return create_entity_xml(session, stream_id, object_id, xml, mode, &payload);
}

uint16_t uxr_buffer_create_publisher_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* xml,
        uint8_t mode)
{
    //assert with the object_id type

    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_PUBLISHER;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.publisher.participant_id.data);

    return create_entity_xml(session, stream_id, object_id, xml, mode, &payload);
}

uint16_t uxr_buffer_create_subscriber_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* xml,
        uint8_t mode)
{
    //assert with the object_id type

    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_SUBSCRIBER;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.subscriber.participant_id.data);

    return create_entity_xml(session, stream_id, object_id, xml, mode, &payload);
}

uint16_t uxr_buffer_create_datawriter_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId publisher_id,
        const char* xml,
        uint8_t mode)
{
    //assert with the object_id type

    UXR_ADD_SHARED_MEMORY_ENTITY_XML(session, object_id, xml);

    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_DATAWRITER;
    uxr_object_id_to_raw(publisher_id, payload.object_representation._.data_writer.publisher_id.data);

    return create_entity_xml(session, stream_id, object_id, xml, mode, &payload);
}

uint16_t uxr_buffer_create_datareader_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId subscriber_id,
        const char* xml,
        uint8_t mode)
{
    //assert with the object_id type

    UXR_ADD_SHARED_MEMORY_ENTITY_XML(session, object_id, xml);

    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_DATAREADER;
    uxr_object_id_to_raw(subscriber_id, payload.object_representation._.data_reader.subscriber_id.data);

    return create_entity_xml(session, stream_id, object_id, xml, mode, &payload);
}

uint16_t uxr_buffer_create_requester_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* xml,
        uint8_t mode)
{
    UXR_ADD_SHARED_MEMORY_ENTITY_XML(session, object_id, xml);

    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_REQUESTER;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.requester.participant_id.data);

    return create_entity_xml(session, stream_id, object_id, xml, mode, &payload);
}

uint16_t uxr_buffer_create_replier_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* xml,
        uint8_t mode)
{
    UXR_ADD_SHARED_MEMORY_ENTITY_XML(session, object_id, xml);

    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_REPLIER;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.replier.participant_id.data);

    return create_entity_xml(session, stream_id, object_id, xml, mode, &payload);
}

//==================================================================
//                             PRIVATE
//==================================================================

inline uint16_t create_entity_xml(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        const char* xml,
        uint8_t mode,
        CREATE_Payload* payload)
{
    // Use participant access to access to the xml base of any object variant. //Future elegant change?
    payload->object_representation._.participant.base.representation.format = DDS_XRCE_REPRESENTATION_AS_XML_STRING;
    payload->object_representation._.participant.base.representation._.xml_string_represenatation = (char*)xml;

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t)(strlen(xml) + 1), mode, payload);
}
