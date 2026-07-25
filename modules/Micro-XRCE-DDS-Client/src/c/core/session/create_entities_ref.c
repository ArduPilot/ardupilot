#include <uxr/client/core/session/create_entities_ref.h>
#include <uxr/client/core/type/xrce_types.h>

#include "common_create_entities_internal.h"

#include <string.h>

static uint16_t create_entity_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        const char* ref,
        uint8_t mode,
        CREATE_Payload* payload);

//==================================================================
//                              PUBLIC
//==================================================================
uint16_t uxr_buffer_create_participant_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t domain_id,
        const char* ref,
        uint8_t mode)
{
    //assert with the object_id type

    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_PARTICIPANT;
    payload.object_representation._.participant.domain_id = (int16_t)domain_id;

    return create_entity_ref(session, stream_id, object_id, ref, mode, &payload);
}

uint16_t uxr_buffer_create_topic_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* ref,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_TOPIC;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.topic.participant_id.data);

    return create_entity_ref(session, stream_id, object_id, ref, mode, &payload);
}

uint16_t uxr_buffer_create_datawriter_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId publisher_id,
        const char* ref,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_DATAWRITER;
    uxr_object_id_to_raw(publisher_id, payload.object_representation._.data_writer.publisher_id.data);

    return create_entity_ref(session, stream_id, object_id, ref, mode, &payload);
}

uint16_t uxr_buffer_create_datareader_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId subscriber_id,
        const char* ref,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_DATAREADER;
    uxr_object_id_to_raw(subscriber_id, payload.object_representation._.data_reader.subscriber_id.data);

    return create_entity_ref(session, stream_id, object_id, ref, mode, &payload);
}

uint16_t uxr_buffer_create_requester_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* ref,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_REQUESTER;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.requester.participant_id.data);

    return create_entity_ref(session, stream_id, object_id, ref, mode, &payload);
}

uint16_t uxr_buffer_create_replier_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* ref,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_REPLIER;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.replier.participant_id.data);

    return create_entity_ref(session, stream_id, object_id, ref, mode, &payload);
}

//==================================================================
//                             PRIVATE
//==================================================================

inline uint16_t create_entity_ref(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        const char* ref,
        uint8_t mode,
        CREATE_Payload* payload)
{
    // Use participant access to access to the ref base of any object variant. //Future elegant change?
    payload->object_representation._.participant.base.representation.format = DDS_XRCE_REPRESENTATION_BY_REFERENCE;
    payload->object_representation._.participant.base.representation._.object_reference = (char*) ref;

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t)(strlen(ref) + 1), mode, payload);
}
