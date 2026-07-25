#include <uxr/client/profile/discovery/discovery.h>
#include <uxr/client/profile/transport/ip/ip.h>
#include <uxr/client/core/session/object_id.h>
#include <uxr/client/core/session/stream/seq_num.h>
#include <uxr/client/core/type/xrce_types.h>
#include <uxr/client/util/time.h>

#include "../../core/serialization/xrce_header_internal.h"
#include "../../core/session/submessage_internal.h"
#include "../../core/log/log_internal.h"
#include "transport/udp_transport_datagram_internal.h"

#include <string.h>

#define GET_INFO_MSG_SIZE   8
#define GET_INFO_REQUEST_ID 9

#define MULTICAST_DEFAULT_IP   "239.255.0.2"
#define MULTICAST_DEFAULT_PORT 7400

typedef struct CallbackData
{
    uxrOnAgentFound on_agent;
    void* args;

} CallbackData;

static void write_get_info_message(
        ucdrBuffer* ub);
static bool listen_info_message(
        uxrUDPTransportDatagram* transport,
        int period,
        CallbackData* callback);
static bool read_info_headers(
        ucdrBuffer* ub);
static bool read_info_message(
        ucdrBuffer* ub,
        CallbackData* callback);

//==================================================================
//                             PUBLIC
//==================================================================

void uxr_discovery_agents_default(
        uint32_t attempts,
        int period,
        uxrOnAgentFound on_agent_func,
        void* args)
{
    TransportLocator multicast;
    uxr_ip_to_locator(MULTICAST_DEFAULT_IP, (uint16_t)MULTICAST_DEFAULT_PORT, UXR_IPv4, &multicast);
    uxr_discovery_agents(attempts, period, on_agent_func, args, &multicast, 1);
}

void uxr_discovery_agents(
        uint32_t attempts,
        int period,
        uxrOnAgentFound on_agent_func,
        void* args,
        const TransportLocator* agent_list,
        size_t agent_list_size)
{
    CallbackData callback;
    callback.on_agent = on_agent_func;
    callback.args = args;

    uint8_t output_buffer[UXR_UDP_TRANSPORT_MTU_DATAGRAM];
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, output_buffer, UXR_UDP_TRANSPORT_MTU_DATAGRAM);
    write_get_info_message(&ub);
    size_t message_length = ucdr_buffer_length(&ub);

    uxrUDPTransportDatagram transport;
    if (uxr_init_udp_transport_datagram(&transport))
    {
        bool is_agent_found = false;
        for (uint32_t a = 0; a < attempts && !is_agent_found; ++a)
        {
            for (size_t i = 0; i < agent_list_size; ++i)
            {
                (void) uxr_udp_send_datagram_to(&transport, output_buffer, message_length, &agent_list[i]);
                UXR_DEBUG_PRINT_MESSAGE(UXR_SEND, output_buffer, message_length, 0);
            }

            int64_t timestamp = uxr_millis();
            int poll = period;
            while (0 < poll && !is_agent_found)
            {
                is_agent_found = listen_info_message(&transport, poll, &callback);
                poll -= (int)(uxr_millis() - timestamp);
            }
        }
        uxr_close_udp_transport_datagram(&transport);
    }
}

//==================================================================
//                             INTERNAL
//==================================================================

void write_get_info_message(
        ucdrBuffer* ub)
{
    GET_INFO_Payload payload;
    payload.base.request_id = (RequestId){{
                                              0x00, GET_INFO_REQUEST_ID
                                          }
    };
    payload.base.object_id = DDS_XRCE_OBJECTID_AGENT;
    payload.info_mask = INFO_CONFIGURATION | INFO_ACTIVITY;

    uxr_serialize_message_header(ub, SESSION_ID_WITHOUT_CLIENT_KEY, 0, 0, 0);
    (void) uxr_buffer_submessage_header(ub, SUBMESSAGE_ID_GET_INFO, GET_INFO_MSG_SIZE, 0);
    (void) uxr_serialize_GET_INFO_Payload(ub, &payload);
}

bool listen_info_message(
        uxrUDPTransportDatagram* transport,
        int poll,
        CallbackData* callback)
{
    uint8_t* input_buffer; size_t length;

    bool is_succeed = false;
    bool received = uxr_udp_recv_datagram(transport, &input_buffer, &length, poll);
    if (received)
    {
        UXR_DEBUG_PRINT_MESSAGE(UXR_RECV, input_buffer, length, 0);

        ucdrBuffer ub;
        ucdr_init_buffer(&ub, input_buffer, (uint32_t)length);
        if (read_info_headers(&ub))
        {
            is_succeed = read_info_message(&ub, callback);
        }
    }

    return is_succeed;
}

bool read_info_headers(
        ucdrBuffer* ub)
{
    uint8_t session_id; uint8_t stream_id_raw; uxrSeqNum seq_num; uint8_t key[CLIENT_KEY_SIZE];
    uxr_deserialize_message_header(ub, &session_id, &stream_id_raw, &seq_num, key);

    uint8_t id; uint16_t length; uint8_t flags;
    return uxr_read_submessage_header(ub, &id, &length, &flags);
}

bool uxr_deserialize_discovery_INFO_Payload(
        ucdrBuffer* buffer,
        INFO_Payload* output)
{
    bool ret = true;
    ret &= uxr_deserialize_BaseObjectReply(buffer, &output->base);
    ret &= ucdr_deserialize_bool(buffer, &output->object_info.optional_config);
    if (output->object_info.optional_config == true)
    {
        ret &= uxr_deserialize_ObjectVariant(buffer, &output->object_info.config);
    }

    ret &= ucdr_deserialize_bool(buffer, &output->object_info.optional_activity);
    if (output->object_info.optional_activity == true)
    {
        ret &= ucdr_deserialize_uint8_t(buffer, &output->object_info.activity.kind);
        ret &= output->object_info.activity.kind == DDS_XRCE_OBJK_AGENT;
        if (ret)
        {
            ret &= ucdr_deserialize_int16_t(buffer, &output->object_info.activity._.agent.availability);
            ret &= ucdr_deserialize_uint32_t(buffer, &output->object_info.activity._.agent.address_seq.size);

            // This function takes care of deserializing at least the possible address_seq items
            // if the sent sequence is too long for the allocated UXR_TRANSPORT_LOCATOR_SEQUENCE_MAX
            output->object_info.activity._.agent.address_seq.size =
                    (output->object_info.activity._.agent.address_seq.size > UXR_TRANSPORT_LOCATOR_SEQUENCE_MAX)   ?
                    UXR_TRANSPORT_LOCATOR_SEQUENCE_MAX                                                         :
                    output->object_info.activity._.agent.address_seq.size;

            for (uint32_t i = 0; i < output->object_info.activity._.agent.address_seq.size && ret; i++)
            {
                ret &= uxr_deserialize_TransportLocator(buffer,
                                &output->object_info.activity._.agent.address_seq.data[i]);
            }
        }
    }
    return ret;
}

bool read_info_message(
        ucdrBuffer* ub,
        CallbackData* callback)
{
    bool is_succeed = false;
    INFO_Payload payload;

    if (uxr_deserialize_discovery_INFO_Payload(ub, &payload))
    {
        XrceVersion* version = &payload.object_info.config._.agent.xrce_version;
        TransportLocatorSeq* locators = &payload.object_info.activity._.agent.address_seq;
        for (size_t i = 0; i < (size_t)locators->size; ++i)
        {
            TransportLocator* transport = &locators->data[i];
            if (0 == memcmp(version->data, DDS_XRCE_XRCE_VERSION.data, sizeof(DDS_XRCE_XRCE_VERSION.data)))
            {
                is_succeed = callback->on_agent(transport, callback->args);
            }
        }
    }

    return is_succeed;
}
