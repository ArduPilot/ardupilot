#include <uxr/client/core/session/create_entities_bin.h>

#include <uxr/client/core/type/xrce_types.h>

#include "common_create_entities_internal.h"
#include "../../profile/shared_memory/shared_memory_internal.h"

#include <string.h>

//==================================================================
//                              PUBLIC
//==================================================================
uint16_t uxr_buffer_create_participant_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t domain_id,
        const char* participant_name,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_PARTICIPANT;
    payload.object_representation._.participant.domain_id = (int16_t)domain_id;
    payload.object_representation._.participant.base.representation.format = DDS_XRCE_REPRESENTATION_IN_BINARY;

    OBJK_DomainParticipant_Binary participant;
    participant.optional_domain_reference = false;
    participant.optional_qos_profile_reference = false;

    if (participant_name != NULL)
    {
        participant.optional_qos_profile_reference = true;
        participant.qos_profile_reference = (char*) participant_name;
    }

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, payload.object_representation._.participant.base.representation._.binary_representation.data,
            UXR_BINARY_SEQUENCE_MAX);
    uxr_serialize_OBJK_DomainParticipant_Binary(&ub, &participant);
    payload.object_representation._.participant.base.representation._.binary_representation.size = (uint32_t) ub.offset;

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t) ub.offset, mode, &payload);
}

uint16_t uxr_buffer_create_topic_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* topic_name,
        const char* type_name,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_TOPIC;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.topic.participant_id.data);
    payload.object_representation._.topic.base.representation.format = DDS_XRCE_REPRESENTATION_IN_BINARY;

    OBJK_Topic_Binary topic;
    topic.topic_name = (char*) topic_name;
    topic.optional_type_name = true;
    topic.type_name = (char*) type_name;
    topic.optional_type_reference = false;

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, payload.object_representation._.topic.base.representation._.binary_representation.data,
            UXR_BINARY_SEQUENCE_MAX);
    uxr_serialize_OBJK_Topic_Binary(&ub, &topic);
    payload.object_representation._.topic.base.representation._.binary_representation.size = (uint32_t) ub.offset;

    UXR_ADD_SHARED_MEMORY_ENTITY_BIN(session, object_id, &topic);

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t) ub.offset, mode, &payload);
}

uint16_t uxr_buffer_create_publisher_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = DDS_XRCE_OBJK_PUBLISHER;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.publisher.participant_id.data);
    payload.object_representation._.publisher.base.representation.format = DDS_XRCE_REPRESENTATION_IN_BINARY;

    OBJK_Publisher_Binary publisher;
    publisher.optional_publisher_name = false;
    publisher.optional_qos = false;

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, payload.object_representation._.publisher.base.representation._.binary_representation.data,
            UXR_BINARY_SEQUENCE_MAX);
    uxr_serialize_OBJK_Publisher_Binary(&ub, &publisher);
    payload.object_representation._.publisher.base.representation._.binary_representation.size = (uint32_t) ub.offset;

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t) ub.offset, mode, &payload);
}

uint16_t uxr_buffer_create_subscriber_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = UXR_SUBSCRIBER_ID;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.subscriber.participant_id.data);
    payload.object_representation._.subscriber.base.representation.format = DDS_XRCE_REPRESENTATION_IN_BINARY;

    OBJK_Subscriber_Binary subscriber;
    subscriber.optional_subscriber_name = false;
    subscriber.optional_qos = false;

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, payload.object_representation._.subscriber.base.representation._.binary_representation.data,
            UXR_BINARY_SEQUENCE_MAX);
    uxr_serialize_OBJK_Subscriber_Binary(&ub, &subscriber);
    payload.object_representation._.subscriber.base.representation._.binary_representation.size = (uint32_t) ub.offset;

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t) ub.offset, mode, &payload);
}

uint16_t uxr_buffer_create_datawriter_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId publisher_id,
        uxrObjectId topic_id,
        uxrQoS_t qos,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = UXR_DATAWRITER_ID;
    uxr_object_id_to_raw(publisher_id, payload.object_representation._.data_writer.publisher_id.data);
    payload.object_representation._.data_writer.base.representation.format = DDS_XRCE_REPRESENTATION_IN_BINARY;

    OBJK_DataWriter_Binary datawriter;
    uxr_object_id_to_raw(topic_id, datawriter.topic_id.data);
    datawriter.optional_qos = true;
    datawriter.qos.optional_ownership_strength = false;
    datawriter.qos.base.optional_deadline_msec = false;
    datawriter.qos.base.optional_lifespan_msec = false;
    datawriter.qos.base.optional_user_data = false;

    datawriter.qos.base.optional_history_depth = qos.depth == 0 ? false : true;
    datawriter.qos.base.history_depth = qos.depth;

    datawriter.qos.base.qos_flags = 0;
    if (qos.reliability == UXR_RELIABILITY_RELIABLE)
    {
        datawriter.qos.base.qos_flags |= is_reliable;
    }
    if (qos.history == UXR_HISTORY_KEEP_LAST)
    {
        datawriter.qos.base.qos_flags |= is_history_keep_last;
    }

    switch (qos.durability)
    {
        case UXR_DURABILITY_VOLATILE:
            // datawriter.qos.base.qos_flags all flags zeroed
            break;
        case UXR_DURABILITY_TRANSIENT_LOCAL:
            datawriter.qos.base.qos_flags |= is_durability_transient_local;
            break;
        case UXR_DURABILITY_TRANSIENT:
            datawriter.qos.base.qos_flags |= is_durability_transient;
            break;
        case UXR_DURABILITY_PERSISTENT:
            datawriter.qos.base.qos_flags |= is_durability_persistent;
            break;
        default:
            break;
    }

    UXR_ADD_SHARED_MEMORY_ENTITY_BIN(session, object_id, &datawriter);

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, payload.object_representation._.data_writer.base.representation._.binary_representation.data,
            UXR_BINARY_SEQUENCE_MAX);
    uxr_serialize_OBJK_DataWriter_Binary(&ub, &datawriter);
    payload.object_representation._.data_writer.base.representation._.binary_representation.size = (uint32_t) ub.offset;

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t) ub.offset, mode, &payload);
}

uint16_t uxr_buffer_create_datareader_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId subscriber_id,
        uxrObjectId topic_id,
        uxrQoS_t qos,
        uint8_t mode)
{
    CREATE_Payload payload;
    payload.object_representation.kind = UXR_DATAREADER_ID;
    uxr_object_id_to_raw(subscriber_id, payload.object_representation._.data_reader.subscriber_id.data);
    payload.object_representation._.data_reader.base.representation.format = DDS_XRCE_REPRESENTATION_IN_BINARY;

    OBJK_DataReader_Binary datareader;
    uxr_object_id_to_raw(topic_id, datareader.topic_id.data);
    datareader.optional_qos = true;
    datareader.qos.optional_contentbased_filter = false;
    datareader.qos.optional_timebasedfilter_msec = false;
    datareader.qos.base.optional_deadline_msec = false;
    datareader.qos.base.optional_lifespan_msec = false;
    datareader.qos.base.optional_user_data = false;

    datareader.qos.base.optional_history_depth = qos.depth == 0 ? false : true;
    datareader.qos.base.history_depth = qos.depth;

    datareader.qos.base.qos_flags = 0;
    if (qos.reliability == UXR_RELIABILITY_RELIABLE)
    {
        datareader.qos.base.qos_flags |= is_reliable;
    }
    if (qos.history == UXR_HISTORY_KEEP_LAST)
    {
        datareader.qos.base.qos_flags |= is_history_keep_last;
    }

    switch (qos.durability)
    {
        case UXR_DURABILITY_VOLATILE:
            // datawriter.qos.base.qos_flags all flags zeroed
            break;
        case UXR_DURABILITY_TRANSIENT_LOCAL:
            datareader.qos.base.qos_flags |= is_durability_transient_local;
            break;
        case UXR_DURABILITY_TRANSIENT:
            datareader.qos.base.qos_flags |= is_durability_transient;
            break;
        case UXR_DURABILITY_PERSISTENT:
            datareader.qos.base.qos_flags |= is_durability_persistent;
            break;
        default:
            break;
    }

    UXR_ADD_SHARED_MEMORY_ENTITY_BIN(session, object_id, &datareader);

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, payload.object_representation._.data_reader.base.representation._.binary_representation.data,
            UXR_BINARY_SEQUENCE_MAX);
    uxr_serialize_OBJK_DataReader_Binary(&ub, &datareader);
    payload.object_representation._.data_reader.base.representation._.binary_representation.size = (uint32_t) ub.offset;

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t) ub.offset, mode, &payload);
}

uint16_t uxr_buffer_create_requester_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* service_name,
        const char* request_type,
        const char* reply_type,
        const char* request_topic_name,
        const char* reply_topic_name,
        uxrQoS_t qos,
        uint8_t mode)
{
    (void) qos;

    CREATE_Payload payload;
    payload.object_representation.kind = UXR_REQUESTER_ID;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.requester.participant_id.data);
    payload.object_representation._.requester.base.representation.format = DDS_XRCE_REPRESENTATION_IN_BINARY;

    OBJK_Requester_Binary requester;
    requester.service_name = (char*) service_name;
    requester.request_type = (char*) request_type;
    requester.reply_type = (char*) reply_type;
    requester.optional_reply_topic_name = true;
    requester.reply_topic_name = (char*) reply_topic_name;
    requester.optional_request_topic_name = true;
    requester.request_topic_name = (char*) request_topic_name;

    UXR_ADD_SHARED_MEMORY_ENTITY_BIN(session, object_id, &requester);

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, payload.object_representation._.requester.base.representation._.binary_representation.data,
            UXR_BINARY_SEQUENCE_MAX);
    uxr_serialize_OBJK_Requester_Binary(&ub, &requester);
    payload.object_representation._.requester.base.representation._.binary_representation.size = (uint32_t) ub.offset;

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t) ub.offset, mode, &payload);
}

uint16_t uxr_buffer_create_replier_bin(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uxrObjectId participant_id,
        const char* service_name,
        const char* request_type,
        const char* reply_type,
        const char* request_topic_name,
        const char* reply_topic_name,
        uxrQoS_t qos,
        uint8_t mode)
{
    (void) qos;

    CREATE_Payload payload;
    payload.object_representation.kind = UXR_REPLIER_ID;
    uxr_object_id_to_raw(participant_id, payload.object_representation._.replier.participant_id.data);
    payload.object_representation._.replier.base.representation.format = DDS_XRCE_REPRESENTATION_IN_BINARY;

    OBJK_Replier_Binary replier;
    replier.service_name = (char*) service_name;
    replier.request_type = (char*) request_type;
    replier.reply_type = (char*) reply_type;
    replier.optional_reply_topic_name = true;
    replier.reply_topic_name = (char*) reply_topic_name;
    replier.optional_request_topic_name = true;
    replier.request_topic_name = (char*) request_topic_name;

    UXR_ADD_SHARED_MEMORY_ENTITY_BIN(session, object_id, &replier);

    ucdrBuffer ub;
    ucdr_init_buffer(&ub, payload.object_representation._.replier.base.representation._.binary_representation.data,
            UXR_BINARY_SEQUENCE_MAX);
    uxr_serialize_OBJK_Replier_Binary(&ub, &replier);
    payload.object_representation._.replier.base.representation._.binary_representation.size = (uint32_t) ub.offset;

    return uxr_common_create_entity(session, stream_id, object_id, (uint16_t) ub.offset, mode, &payload);
}
