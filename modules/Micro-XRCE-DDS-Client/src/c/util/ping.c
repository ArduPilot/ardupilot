#include <uxr/client/util/ping.h>
#include <uxr/client/util/time.h>

#include <uxr/client/core/type/xrce_types.h>
#include <uxr/client/core/session/session.h>
#include <uxr/client/core/session/stream/seq_num.h>
#include <uxr/client/profile/multithread/multithread.h>

#include "../core/serialization/xrce_header_internal.h"
#include "../core/serialization/xrce_subheader_internal.h"
#include "../core/session/submessage_internal.h"

bool serialize_get_info_message(
        ucdrBuffer* ub,
        const uint8_t session_id);

pong_status_t uxr_acknack_pong(
        ucdrBuffer* buffer);

bool listen_info_message(
        uxrCommunication* comm,
        const int timeout);

bool uxr_run_session_until_pong(
        uxrSession* session,
        int timeout_ms);

bool uxr_read_session_header(
        const uxrSessionInfo* info,
        struct ucdrBuffer* ub,
        uint8_t* stream_id_raw,
        uxrSeqNum* seq_num);

void read_submessage_info(
        uxrSession* session,
        ucdrBuffer* submessage);

//==================================================================
//                             PUBLIC
//==================================================================
bool uxr_ping_agent_session(
        uxrSession* session,
        const int timeout_ms,
        const uint8_t attempts)
{
    uint8_t output_buffer[UXR_PING_BUF];
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, output_buffer, sizeof(output_buffer));

    bool ret = false;
    if (serialize_get_info_message(&ub, session->info.id))
    {
        size_t message_length = ucdr_buffer_length(&ub);

        UXR_LOCK_SESSION(session);
        for (size_t i = 0; !ret && i < attempts; i++)
        {
            ret = session->comm->send_msg(session->comm->instance, output_buffer, message_length);
            ret &= uxr_run_session_until_pong(session, timeout_ms);
        }
        UXR_UNLOCK_SESSION(session);
    }

    return ret;
}

bool uxr_ping_agent_attempts(
        uxrCommunication* comm,
        const int timeout_ms,
        const uint8_t attempts)
{
    bool agent_pong = false;
    uint8_t output_buffer[UXR_PING_BUF];
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, output_buffer, sizeof(output_buffer));

    if (serialize_get_info_message(&ub, SESSION_ID_WITHOUT_CLIENT_KEY))
    {
        size_t message_length = ucdr_buffer_length(&ub);

        for (size_t i = 0; !agent_pong && i < attempts; ++i)
        {
            UXR_LOCK_TRANSPORT(comm);

            comm->send_msg(
                comm->instance,
                output_buffer,
                message_length);

            int64_t timestamp = uxr_millis();
            int poll = timeout_ms;

            do
            {
                agent_pong = listen_info_message(comm, timeout_ms);
                poll -= (int)(uxr_millis() - timestamp);
                timestamp = uxr_millis();
            } while (0 < poll && !agent_pong);

            UXR_UNLOCK_TRANSPORT(comm);
        }
    }

    return agent_pong;
}

inline bool uxr_ping_agent(
        uxrCommunication* comm,
        const int timeout_ms)
{
    return uxr_ping_agent_attempts(comm, timeout_ms, 1);
}

//==================================================================
//                             PRIVATE
//==================================================================
bool serialize_get_info_message(
        ucdrBuffer* ub,
        const uint8_t session_id)
{
    bool res = true;
    GET_INFO_Payload gi_payload;

    gi_payload.base.request_id = (RequestId){{
                                                 0x00, GET_INFO_REQUEST_PING_ID
                                             }
    };
    gi_payload.base.object_id = DDS_XRCE_OBJECTID_AGENT;
    gi_payload.info_mask = INFO_ACTIVITY;

    uxr_serialize_message_header(ub, session_id, 0, 0, 0);
    res &= uxr_buffer_submessage_header(ub, SUBMESSAGE_ID_GET_INFO, GET_INFO_MSG_SIZE, 0);
    res &= uxr_serialize_GET_INFO_Payload(ub, &gi_payload);

    return res;
}

bool listen_info_message(
        uxrCommunication* comm,
        const int timeout)
{
    uint8_t* input_buffer = NULL;
    size_t len;

    bool success = comm->recv_msg(
        comm->instance,
        &input_buffer,
        &len,
        timeout);

    if (success)
    {
        ucdrBuffer ub;
        ucdr_init_buffer(&ub, input_buffer, len);

        uxrSessionInfo session_info_fake = {
            0
        };
        uint8_t stream_id_raw;
        uxrSeqNum seq_num;
        uxr_read_session_header(&session_info_fake, &ub, &stream_id_raw, &seq_num);
        success &= NO_PONG_STATUS != uxr_acknack_pong(&ub);
    }

    return success;
}

pong_status_t uxr_acknack_pong(
        ucdrBuffer* buffer)
{
    bool success = false;
    uxrSession fake_session = {
        0
    };

    if (ucdr_buffer_remaining(buffer) > SUBHEADER_SIZE)
    {
        uint8_t id = 0;
        uint8_t flags = 0;
        uint16_t length = 0;
        uxr_deserialize_submessage_header(buffer, &id, &flags, &length);
        success = ucdr_buffer_remaining(buffer) >= length;

        if (success && id == SUBMESSAGE_ID_INFO)
        {
            fake_session.on_pong_flag = NO_PONG_STATUS;
            read_submessage_info(&fake_session, buffer);
        }
    }

    return fake_session.on_pong_flag;
}
