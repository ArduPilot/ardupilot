#include <uxr/client/config.h>

#include <uxr/client/core/session/session.h>
#include <uxr/client/core/type/xrce_types.h>
#include <uxr/client/profile/multithread/multithread.h>
#include <uxr/client/core/type/xrce_types.h>

#include "../../core/session/stream/stream_storage_internal.h"
#include "../matching/matching_internal.h"

#include <string.h>

#if defined(WIN32)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#else
#include <sys/types.h>
#endif /* if defined(WIN32) */

//==================================================================
//                             PRIVATE
//==================================================================
typedef struct uxr_shared_memory_buffer_item_t
{
    struct uxr_shared_memory_buffer_item_t* next;
    ucdrBuffer data;
    uint16_t data_size;
    uint16_t request_id;
} uxr_shared_memory_buffer_item_t;

typedef enum entity_descriptor_type_t
{
    UXR_SHARED_MEMORY_HASH_ENTITY,
    UXR_SHARED_MEMORY_BIN_ENTITY
} entity_descriptor_type_t;
typedef struct uxr_shared_memory_entity_t
{
    uxrObjectId object_id;
    uxrSession* session;
    entity_descriptor_type_t type;
    union
    {
        char hash[UXR_MATCHING_HASH_SIZE];
        OBJK_DataWriter_Binary datawriter;
        OBJK_DataReader_Binary datareader;
    } data;
    struct uxr_shared_memory_entity_t* related_topic;
} uxr_shared_memory_entity_t;

typedef struct uxr_shared_memory_matrix_block_t
{
    bool matched;
    struct uxr_shared_memory_buffer_item_t* list;
} uxr_shared_memory_matrix_block_t;

typedef struct uxr_shared_memory_map_t
{
    uxr_shared_memory_entity_t entities[UXR_CONFIG_SHARED_MEMORY_MAX_ENTITIES];
    size_t entities_len;
    uxr_shared_memory_matrix_block_t matrix[UXR_CONFIG_SHARED_MEMORY_MAX_ENTITIES][UXR_CONFIG_SHARED_MEMORY_MAX_ENTITIES
    ];
    uxr_shared_memory_buffer_item_t mem_pool[UXR_CONFIG_SHARED_MEMORY_STATIC_MEM_SIZE];
    uxr_shared_memory_buffer_item_t* mempool_freeitems;
    bool init;

#ifdef UCLIENT_PROFILE_MULTITHREAD
    uxrMutex lock;
#endif /* ifdef UCLIENT_PROFILE_MULTITHREAD */

} uxr_shared_memory_map_t;

static uxr_shared_memory_map_t uxr_sm_map = {
    0
};

// Double linked pool for ucdrBuffers

void uxr_prepend_to_list(
        uxr_shared_memory_buffer_item_t* member,
        uxr_shared_memory_buffer_item_t** list)
{
    member->next = *list;
    *list = member;
}

uxr_shared_memory_buffer_item_t* uxr_pop_head_from_list(
        uxr_shared_memory_buffer_item_t** list)
{
    uxr_shared_memory_buffer_item_t* member = *list;
    *list = (*list == NULL ) ? NULL : (*list)->next;
    return member;
}

#define UXR_SHARED_MEMORY_INIT() if (!uxr_sm_map.init){uxr_init_shared_memory();}

void uxr_init_shared_memory()
{
    uxr_sm_map.init = true;
    UXR_INIT_LOCK(&uxr_sm_map.lock);

    for (size_t i = 0; i < UXR_CONFIG_SHARED_MEMORY_STATIC_MEM_SIZE; i++)
    {
        uxr_prepend_to_list(&uxr_sm_map.mem_pool[i], &uxr_sm_map.mempool_freeitems);
    }
}

void uxr_clean_shared_memory()
{
    memset(&uxr_sm_map, 0, sizeof(uxr_sm_map));
}

// API
bool uxr_shared_memory_entity_compare(
        uxr_shared_memory_entity_t* e1,
        uxr_shared_memory_entity_t* e2)
{
    bool ret = true;
    for (uint8_t i = 0; i < sizeof(e1->session->info.key); i++)
    {
        ret &= e1->session->info.key[i] == e2->session->info.key[i];
    }

    ret &= e1->object_id.id == e2->object_id.id;
    ret &= e1->object_id.type == e2->object_id.type;

    return ret;
}

ssize_t uxr_shared_memory_get_entity_index(
        uxrSession* session,
        uxrObjectId* entity_id)
{
    uxr_shared_memory_entity_t aux;
    aux.session = session;
    aux.object_id = *entity_id;

    for (size_t i = 0; i < uxr_sm_map.entities_len; i++)
    {
        if (uxr_shared_memory_entity_compare(&aux, &uxr_sm_map.entities[i]))
        {
            return (ssize_t)i;
        }
    }
    return -1;
}

uxr_shared_memory_entity_t* uxr_shared_memory_get_entity(
        uxrSession* session,
        uxrObjectId* entity_id)
{
    uxr_shared_memory_entity_t aux;
    aux.session = session;
    aux.object_id = *entity_id;

    for (size_t i = 0; i < uxr_sm_map.entities_len; i++)
    {
        if (uxr_shared_memory_entity_compare(&aux, &uxr_sm_map.entities[i]))
        {
            return &uxr_sm_map.entities[i];
        }
    }
    return NULL;
}

void uxr_prepare_shared_memory(
        uxrSession* session,
        uxrObjectId entity_id,
        ucdrBuffer* ub,
        uint16_t data_size,
        uint16_t request_id)
{
    UXR_SHARED_MEMORY_INIT();
    UXR_LOCK(&uxr_sm_map.lock);

    ssize_t entity_index = uxr_shared_memory_get_entity_index(session, &entity_id);
    if (-1 == entity_index)
    {
        UXR_UNLOCK(&uxr_sm_map.lock);
        return;
    }

    for (size_t i = 0; i < uxr_sm_map.entities_len; i++)
    {
        if (uxr_sm_map.matrix[entity_index][i].matched)
        {
            uxr_shared_memory_buffer_item_t* item = uxr_pop_head_from_list(&uxr_sm_map.mempool_freeitems);
            if (item != NULL)
            {
                item->data = *ub;
                item->data_size = data_size;
                item->request_id = request_id;
                uxr_prepend_to_list(item, &uxr_sm_map.matrix[entity_index][i].list);
            }
        }
    }

    UXR_UNLOCK(&uxr_sm_map.lock);
}

void uxr_handle_shared_memory()
{
    UXR_SHARED_MEMORY_INIT();
    UXR_LOCK(&uxr_sm_map.lock);

    for (size_t i = 0; i < uxr_sm_map.entities_len; i++)
    {
        if (uxr_sm_map.entities[i].object_id.type != UXR_DATAWRITER_ID
                && uxr_sm_map.entities[i].object_id.type != UXR_REPLIER_ID
                && uxr_sm_map.entities[i].object_id.type != UXR_REQUESTER_ID
                )
        {
            continue;
        }

        for (size_t j = 0; j < uxr_sm_map.entities_len; j++)
        {
            while (uxr_sm_map.matrix[i][j].list != NULL)
            {
                uxrStreamId stream_id = {
                    .type = UXR_SHARED_MEMORY_STREAM
                };
                uxr_shared_memory_buffer_item_t* item =  uxr_pop_head_from_list(&uxr_sm_map.matrix[i][j].list);

                switch (uxr_sm_map.entities[j].object_id.type)
                {
                    case UXR_DATAREADER_ID:
                    {
                        if (NULL != uxr_sm_map.entities[j].session->on_topic)
                        {
                            uxr_sm_map.entities[j].session->on_topic(
                                uxr_sm_map.entities[j].session,
                                uxr_sm_map.entities[j].object_id,
                                0, // Req ID = 0 means shared memory?
                                stream_id,
                                &item->data,
                                item->data_size,
                                uxr_sm_map.entities[j].session->on_topic_args
                                );
                        }
                        break;
                    }

                    case UXR_REPLIER_ID:
                    {
                        if (NULL != uxr_sm_map.entities[j].session->on_request)
                        {
                            SampleIdentity sample_identity;
                            sample_identity.sequence_number.low = item->request_id;

                            // Save session key
                            memcpy(sample_identity.writer_guid.guidPrefix.data,
                                    uxr_sm_map.entities[i].session->info.key,
                                    sizeof(uxr_sm_map.entities[i].session->info.key));

                            // Save source uxrObjectId info
                            sample_identity.writer_guid.entityId.entityKey[1] =
                                    (uint8_t) (uxr_sm_map.entities[i].object_id.id >> 8) & 0xFF;
                            sample_identity.writer_guid.entityId.entityKey[0] =
                                    (uint8_t) uxr_sm_map.entities[i].object_id.id &
                                    0xFF;
                            sample_identity.writer_guid.entityId.entityKind = uxr_sm_map.entities[i].object_id.type;

                            uxr_sm_map.entities[j].session->on_request(
                                uxr_sm_map.entities[j].session,
                                uxr_sm_map.entities[j].object_id,
                                0, // Req ID = 0 means shared memory?
                                &sample_identity,
                                &item->data,
                                item->data_size,
                                uxr_sm_map.entities[j].session->on_topic_args);
                        }
                        break;
                    }

                    case UXR_REQUESTER_ID:
                    {
                        if (NULL != uxr_sm_map.entities[j].session->on_reply)
                        {
                            SampleIdentity sample_identity;
                            size_t offset = item->data.offset;
                            uxr_deserialize_SampleIdentity(&item->data, &sample_identity);

                            // Check destination session key and uxrObjectId info
                            if (!memcmp(uxr_sm_map.entities[j].session->info.key,
                                    sample_identity.writer_guid.guidPrefix.data,
                                    sizeof(uxr_sm_map.entities[j].session->info.key))
                                    && uxr_sm_map.entities[j].object_id.id ==
                                    (sample_identity.writer_guid.entityId.entityKey[1] >> 8) +
                                    sample_identity.writer_guid.entityId.entityKey[0]
                                    && sample_identity.writer_guid.entityId.entityKind ==
                                    uxr_sm_map.entities[j].object_id.type)
                            {
                                // TODO: Check this size calculation taking alignment
                                item->data_size = (uint16_t)(item->data_size - (item->data.offset - offset));
                                uxr_sm_map.entities[j].session->on_reply(
                                    uxr_sm_map.entities[j].session,
                                    uxr_sm_map.entities[j].object_id,
                                    0, // Req ID = 0 means shared memory?
                                    (uint16_t)sample_identity.sequence_number.low,
                                    &item->data,
                                    (size_t)item->data_size,
                                    uxr_sm_map.entities[j].session->on_topic_args);
                            }
                        }
                        break;
                    }
                    default:
                        break;
                }

                uxr_prepend_to_list(item, &uxr_sm_map.mempool_freeitems);
            }
        }
    }

    UXR_UNLOCK(&uxr_sm_map.lock);
}

bool uxr_match_binary_entities(
        uxr_shared_memory_entity_t* entity_1,
        uxr_shared_memory_entity_t* entity_2)
{
    if (!(entity_1->object_id.type == UXR_DATAREADER_ID  && entity_2->object_id.type == UXR_DATAWRITER_ID) &&
            !(entity_1->object_id.type == UXR_DATAWRITER_ID  && entity_2->object_id.type == UXR_DATAREADER_ID) &&
            !(entity_1->object_id.type == UXR_REQUESTER_ID   && entity_2->object_id.type == UXR_REPLIER_ID) &&
            !(entity_1->object_id.type == UXR_REPLIER_ID     && entity_2->object_id.type == UXR_REQUESTER_ID))
    {
        return false;
    }

    bool matched = true;
    switch (entity_1->object_id.type)
    {
        case UXR_DATAREADER_ID:
        {
            OBJK_DataReader_Binary* dr = (OBJK_DataReader_Binary*) &entity_1->data.datareader;
            OBJK_DataWriter_Binary* dw = (OBJK_DataWriter_Binary*) &entity_2->data.datawriter;
            matched &= uxr_match_endpoint_qosbinary(&dr->qos.base, &dw->qos.base);
            matched &= 0 == strcmp(entity_1->related_topic->data.hash, entity_2->related_topic->data.hash);
            break;
        }

        case UXR_DATAWRITER_ID:
        {
            OBJK_DataWriter_Binary* dw = (OBJK_DataWriter_Binary*) &entity_1->data.datawriter;
            OBJK_DataReader_Binary* dr = (OBJK_DataReader_Binary*) &entity_2->data.datareader;
            matched &= uxr_match_endpoint_qosbinary(&dr->qos.base, &dw->qos.base);
            matched &= 0 == strcmp(entity_1->related_topic->data.hash, entity_2->related_topic->data.hash);
            break;
        }
        case UXR_REQUESTER_ID:
        case UXR_REPLIER_ID:
        default:
            matched = false;
            break;
    }

    return matched;
}

void uxr_update_shared_memory_matching()
{
    UXR_LOCK(&uxr_sm_map.lock);

    for (size_t i = 0; i < uxr_sm_map.entities_len; i++)
    {
        for (size_t j = 0; j < uxr_sm_map.entities_len; j++)
        {
            if (i != j &&
                    uxr_sm_map.entities[i].type == uxr_sm_map.entities[j].type  &&
                    uxr_sm_map.entities[i].object_id.type != uxr_sm_map.entities[j].object_id.type &&
                    uxr_sm_map.entities[i].object_id.type != UXR_TOPIC_ID &&
                    uxr_sm_map.entities[j].object_id.type != UXR_TOPIC_ID)
            {
                switch (uxr_sm_map.entities[i].type)
                {
                    case UXR_SHARED_MEMORY_HASH_ENTITY:
                        uxr_sm_map.matrix[i][j].matched =
                                0 == memcmp(uxr_sm_map.entities[i].data.hash, uxr_sm_map.entities[j].data.hash,
                                        UXR_MATCHING_HASH_SIZE);
                        break;
                    case UXR_SHARED_MEMORY_BIN_ENTITY:
                        uxr_sm_map.matrix[i][j].matched = uxr_match_binary_entities(&uxr_sm_map.entities[i],
                                        &uxr_sm_map.entities[j]);
                        break;
                    default:
                        uxr_sm_map.matrix[i][j].matched = 0;
                }
            }
        }
    }

    UXR_UNLOCK(&uxr_sm_map.lock);
}

void uxr_add_shared_memory_entity_xml(
        uxrSession* session,
        uxrObjectId entity_id,
        const char* xml)
{
    UXR_SHARED_MEMORY_INIT();
    UXR_LOCK(&uxr_sm_map.lock);

    if (uxr_sm_map.entities_len <= UXR_CONFIG_SHARED_MEMORY_MAX_ENTITIES - 1)
    {
        uxr_sm_map.entities[uxr_sm_map.entities_len].object_id = entity_id;
        uxr_sm_map.entities[uxr_sm_map.entities_len].session = session;
        uxr_sm_map.entities[uxr_sm_map.entities_len].type = UXR_SHARED_MEMORY_HASH_ENTITY;
        uxr_generate_hash_from_xml(xml, entity_id, uxr_sm_map.entities[uxr_sm_map.entities_len].data.hash);
        uxr_sm_map.entities_len++;
        uxr_update_shared_memory_matching();
    }

    UXR_UNLOCK(&uxr_sm_map.lock);
}

void uxr_add_shared_memory_entity_bin(
        uxrSession* session,
        uxrObjectId entity_id,
        const void* entity)
{
    UXR_SHARED_MEMORY_INIT();
    UXR_LOCK(&uxr_sm_map.lock);

    if (uxr_sm_map.entities_len <= UXR_CONFIG_SHARED_MEMORY_MAX_ENTITIES - 1)
    {
        uxr_sm_map.entities[uxr_sm_map.entities_len].object_id = entity_id;
        uxr_sm_map.entities[uxr_sm_map.entities_len].session = session;
        uxr_sm_map.entities[uxr_sm_map.entities_len].type = UXR_SHARED_MEMORY_BIN_ENTITY;

        uxrObjectId related_object_id;
        switch (entity_id.type)
        {
            case UXR_DATAWRITER_ID:
                related_object_id = uxr_object_id_from_raw(((OBJK_DataWriter_Binary*) entity)->topic_id.data);
                uxr_sm_map.entities[uxr_sm_map.entities_len].data.datawriter = *((OBJK_DataWriter_Binary*) entity);
                uxr_sm_map.entities[uxr_sm_map.entities_len].related_topic = uxr_shared_memory_get_entity(session,
                                &related_object_id);
                break;
            case UXR_DATAREADER_ID:
                related_object_id = uxr_object_id_from_raw(((OBJK_DataReader_Binary*) entity)->topic_id.data);
                uxr_sm_map.entities[uxr_sm_map.entities_len].data.datareader = *((OBJK_DataReader_Binary*) entity);
                uxr_sm_map.entities[uxr_sm_map.entities_len].related_topic = uxr_shared_memory_get_entity(session,
                                &related_object_id);
                break;
            case UXR_REQUESTER_ID:
                uxr_sm_map.entities[uxr_sm_map.entities_len].type = UXR_SHARED_MEMORY_HASH_ENTITY;
                uxr_generate_hash_from_strings(
                    uxr_sm_map.entities[uxr_sm_map.entities_len].data.hash, 5,
                    ((OBJK_Requester_Binary*) entity)->service_name,
                    ((OBJK_Requester_Binary*) entity)->request_type,
                    ((OBJK_Requester_Binary*) entity)->reply_type,
                    ((OBJK_Requester_Binary*) entity)->request_topic_name,
                    ((OBJK_Requester_Binary*) entity)->reply_topic_name);
                uxr_sm_map.entities[uxr_sm_map.entities_len].related_topic = NULL;
                break;
            case UXR_REPLIER_ID:
                uxr_sm_map.entities[uxr_sm_map.entities_len].type = UXR_SHARED_MEMORY_HASH_ENTITY;
                uxr_generate_hash_from_strings(
                    uxr_sm_map.entities[uxr_sm_map.entities_len].data.hash, 5,
                    ((OBJK_Replier_Binary*) entity)->service_name,
                    ((OBJK_Replier_Binary*) entity)->request_type,
                    ((OBJK_Replier_Binary*) entity)->reply_type,
                    ((OBJK_Replier_Binary*) entity)->request_topic_name,
                    ((OBJK_Replier_Binary*) entity)->reply_topic_name);
                uxr_sm_map.entities[uxr_sm_map.entities_len].related_topic = NULL;
                break;
            case UXR_TOPIC_ID:
                uxr_sm_map.entities[uxr_sm_map.entities_len].type = UXR_SHARED_MEMORY_HASH_ENTITY;
                uxr_generate_hash_from_strings(
                    uxr_sm_map.entities[uxr_sm_map.entities_len].data.hash, 2,
                    ((OBJK_Topic_Binary*) entity)->topic_name,
                    ((OBJK_Topic_Binary*) entity)->type_name);
                uxr_sm_map.entities[uxr_sm_map.entities_len].related_topic = NULL;
            default:
                break;
        }
        uxr_sm_map.entities_len++;
        uxr_update_shared_memory_matching();
    }

    UXR_UNLOCK(&uxr_sm_map.lock);
}
