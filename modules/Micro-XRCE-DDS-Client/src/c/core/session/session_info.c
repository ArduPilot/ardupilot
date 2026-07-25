#include <uxr/client/defines.h>
#include <uxr/client/core/session/object_id.h>
#include <uxr/client/core/type/xrce_types.h>
#include <uxr/client/config.h>

#include "session_info_internal.h"
#include "submessage_internal.h"
#include "../serialization/xrce_header_internal.h"

#include <string.h>

#define VENDOR_ID_EPROSIMA COMPOUND_LITERAL(XrceVendorId){{0x01, 0x0F}}

#define RESERVED_REQUESTS_ID 9

static uint16_t generate_request_id(
        uxrSessionInfo* info);

static void process_delete_session_status(
        uxrSessionInfo* info,
        uint8_t status,
        uint16_t request_id);

//==================================================================
//                             PUBLIC
//==================================================================

void uxr_init_session_info(
        uxrSessionInfo* info,
        uint8_t id,
        uint32_t key)
{
    info->id = id;
    info->key[0] = (uint8_t)(key >> 24);
    info->key[1] = (uint8_t)((key << 8) >> 24);
    info->key[2] = (uint8_t)((key << 16) >> 24);
    info->key[3] = (uint8_t)((key << 24) >> 24);
    info->last_request_id = RESERVED_REQUESTS_ID;
    info->last_requested_status = UXR_STATUS_NONE;
}

void uxr_buffer_create_session(
        uxrSessionInfo* info,
        ucdrBuffer* ub,
        uint16_t mtu)
{
    CREATE_CLIENT_Payload payload = {
        0
    };
    payload.client_representation.xrce_cookie = DDS_XRCE_XRCE_COOKIE;
    payload.client_representation.xrce_version = DDS_XRCE_XRCE_VERSION;
    payload.client_representation.xrce_vendor_id = VENDOR_ID_EPROSIMA;
    payload.client_representation.client_key.data[0] = info->key[0];
    payload.client_representation.client_key.data[1] = info->key[1];
    payload.client_representation.client_key.data[2] = info->key[2];
    payload.client_representation.client_key.data[3] = info->key[3];
    payload.client_representation.session_id = info->id;
    payload.client_representation.optional_properties = false;

#ifdef UCLIENT_PROFILE_SHARED_MEMORY
    payload.client_representation.optional_properties = true;
    payload.client_representation.properties.data[payload.client_representation.properties.size].name = "uxr_sm";
    payload.client_representation.properties.data[payload.client_representation.properties.size].value = "1";
    payload.client_representation.properties.size++;
#endif /* ifdef UCLIENT_PROFILE_SHARED_MEMORY */

#ifdef UCLIENT_HARD_LIVELINESS_CHECK
    payload.client_representation.optional_properties = true;
    payload.client_representation.properties.data[payload.client_representation.properties.size].name = "uxr_hl";

    const char* str = UXR_CONFIG_HARD_LIVELINESS_CHECK_TIMEOUT_STR;

    if (strlen(str) > 6)
    {
        str = "999999";
    }

    char buffer[7];
    const size_t leading_zeros = 6 - strlen(str);
    memset(buffer, '0', leading_zeros);
    memcpy(buffer + leading_zeros, str, strlen(str));
    buffer[6] = '\0';

    payload.client_representation.properties.data[payload.client_representation.properties.size].value = buffer;
    payload.client_representation.properties.size++;
#endif /* ifdef UCLIENT_HARD_LIVELINESS_CHECK */

    payload.client_representation.mtu = mtu;

    info->last_request_id = UXR_REQUEST_LOGIN;

    (void) uxr_buffer_submessage_header(ub, SUBMESSAGE_ID_CREATE_CLIENT, CREATE_CLIENT_PAYLOAD_SIZE, 0);
    (void) uxr_serialize_CREATE_CLIENT_Payload(ub, &payload);
}

void uxr_buffer_delete_session(
        uxrSessionInfo* info,
        ucdrBuffer* ub)
{
    DELETE_Payload payload;
    payload.base.request_id = COMPOUND_LITERAL(RequestId){
        {
            0x00, UXR_REQUEST_LOGOUT
        }
    };
    payload.base.object_id = DDS_XRCE_OBJECTID_CLIENT;

    info->last_request_id = UXR_REQUEST_LOGOUT;

    (void) uxr_buffer_submessage_header(ub, SUBMESSAGE_ID_DELETE, DELETE_CLIENT_PAYLOAD_SIZE, 0);
    (void) uxr_serialize_DELETE_Payload(ub, &payload);
}

void uxr_read_create_session_status(
        uxrSessionInfo* info,
        ucdrBuffer* ub)
{
    STATUS_AGENT_Payload payload;
    (void) uxr_deserialize_STATUS_AGENT_Payload(ub, &payload);
    info->last_requested_status = payload.result.status;
}

void uxr_read_delete_session_status(
        uxrSessionInfo* info,
        ucdrBuffer* ub)
{
    STATUS_Payload payload;
    (void) uxr_deserialize_STATUS_Payload(ub, &payload);

    if (UXR_REQUEST_LOGOUT == info->last_request_id)
    {
        uxrObjectId object_id; uint16_t request_id;
        uxr_parse_base_object_request(&payload.base.related_request, &object_id, &request_id);
        process_delete_session_status(info, payload.base.result.status, request_id);
    }
}

void uxr_stamp_create_session_header(
        const uxrSessionInfo* info,
        uint8_t* buffer)
{
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer, MAX_HEADER_SIZE);

    uxr_serialize_message_header(&ub, info->id & SESSION_ID_WITHOUT_CLIENT_KEY, 0, 0, info->key);
}

void uxr_stamp_session_header(
        const uxrSessionInfo* info,
        uint8_t stream_id_raw,
        uxrSeqNum seq_num,
        uint8_t* buffer)
{
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, buffer, MAX_HEADER_SIZE);

    uxr_serialize_message_header(&ub, info->id, stream_id_raw, seq_num, info->key);
}

bool uxr_read_session_header(
        const uxrSessionInfo* info,
        ucdrBuffer* ub,
        uint8_t* stream_id_raw,
        uxrSeqNum* seq_num)
{
    bool must_be_read = ucdr_buffer_remaining(ub) > MAX_HEADER_SIZE;
    if (must_be_read)
    {
        uint8_t session_id; uint8_t key[CLIENT_KEY_SIZE];
        uxr_deserialize_message_header(ub, &session_id, stream_id_raw, seq_num, key);

        must_be_read = session_id == info->id;
        if (must_be_read)
        {
            if (SESSION_ID_WITHOUT_CLIENT_KEY > info->id)
            {
                must_be_read = (0 == memcmp(key, info->key, CLIENT_KEY_SIZE));
            }
        }
    }

    return must_be_read;
}

uint8_t uxr_session_header_offset(
        const uxrSessionInfo* info)
{
    return (SESSION_ID_WITHOUT_CLIENT_KEY > info->id) ? MAX_HEADER_SIZE : MIN_HEADER_SIZE;
}

uint16_t uxr_init_base_object_request(
        uxrSessionInfo* info,
        uxrObjectId object_id,
        BaseObjectRequest* base)
{
    uint16_t request_id = generate_request_id(info);

    base->request_id.data[0] = (uint8_t) (request_id >> 8);
    base->request_id.data[1] = (uint8_t) request_id;
    uxr_object_id_to_raw(object_id, base->object_id.data);

    return request_id;
}

void uxr_parse_base_object_request(
        const BaseObjectRequest* base,
        uxrObjectId* object_id,
        uint16_t* request_id)
{
    *object_id = uxr_object_id_from_raw(base->object_id.data);
    *request_id = (uint16_t)((((uint16_t) base->request_id.data[0]) << 8)
            + base->request_id.data[1]);
}

//==================================================================
//                            PRIVATE
//==================================================================
inline uint16_t generate_request_id(
        uxrSessionInfo* session)
{
    bool out_of_bounds = (UINT16_MAX == session->last_request_id || RESERVED_REQUESTS_ID >= session->last_request_id);
    session->last_request_id = (uint16_t)((out_of_bounds ? RESERVED_REQUESTS_ID : session->last_request_id) + 1);

    return session->last_request_id;
}

inline void process_delete_session_status(
        uxrSessionInfo* info,
        uint8_t status,
        uint16_t request_id)
{
    if (UXR_REQUEST_LOGOUT == request_id)
    {
        info->last_requested_status = status;
    }
}
