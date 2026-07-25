#include <uxr/client/core/session/session.h>
#include <uxr/client/util/time.h>
#include <uxr/client/core/communication/communication.h>
#include <uxr/client/core/type/xrce_types.h>
#include <uxr/client/config.h>

#include "submessage_internal.h"
#include "session_internal.h"
#include "session_info_internal.h"
#include "stream/stream_storage_internal.h"
#include "stream/common_reliable_stream_internal.h"
#include "stream/input_best_effort_stream_internal.h"
#include "stream/input_reliable_stream_internal.h"
#include "stream/output_best_effort_stream_internal.h"
#include "stream/output_reliable_stream_internal.h"
#include "stream/seq_num_internal.h"
#include "../serialization/xrce_subheader_internal.h"
#include "../log/log_internal.h"
#include "../../util/time_internal.h"
#include <uxr/client/profile/multithread/multithread.h>
#include "../../profile/shared_memory/shared_memory_internal.h"

#ifdef UCLIENT_PROFILE_SHARED_MEMORY
#define PROFILE_SHARED_MEMORY_ADD_SIZE 21
#else
#define PROFILE_SHARED_MEMORY_ADD_SIZE 0
#endif /* ifdef UCLIENT_PROFILE_SHARED_MEMORY */

#ifdef UCLIENT_HARD_LIVELINESS_CHECK
#define HARD_LIVELINESS_CHECK_ADD_SIZE 26
#else
#define HARD_LIVELINESS_CHECK_ADD_SIZE 0
#endif /* ifdef UCLIENT_HARD_LIVELINESS_CHECK */

#define CREATE_SESSION_PROPERTIES_MAX_SIZE PROFILE_SHARED_MEMORY_ADD_SIZE + HARD_LIVELINESS_CHECK_ADD_SIZE


#define CREATE_SESSION_MAX_MSG_SIZE (MAX_HEADER_SIZE + SUBHEADER_SIZE + CREATE_CLIENT_PAYLOAD_SIZE + \
    CREATE_SESSION_PROPERTIES_MAX_SIZE)

#define DELETE_SESSION_MAX_MSG_SIZE (MAX_HEADER_SIZE + SUBHEADER_SIZE + DELETE_CLIENT_PAYLOAD_SIZE)
#define HEARTBEAT_MAX_MSG_SIZE      (MAX_HEADER_SIZE + SUBHEADER_SIZE + HEARTBEAT_PAYLOAD_SIZE)
#define ACKNACK_MAX_MSG_SIZE        (MAX_HEADER_SIZE + SUBHEADER_SIZE + ACKNACK_PAYLOAD_SIZE)
#define TIMESTAMP_PAYLOAD_SIZE      8
#define TIMESTAMP_MAX_MSG_SIZE      (MAX_HEADER_SIZE + SUBHEADER_SIZE + TIMESTAMP_PAYLOAD_SIZE)

static bool listen_message(
        uxrSession* session,
        int poll_ms);
static bool listen_message_reliably(
        uxrSession* session,
        int poll_ms);

static bool wait_session_status(
        uxrSession* session,
        uint8_t* buffer,
        size_t length,
        size_t attempts);

static bool send_message(
        const uxrSession* session,
        uint8_t* buffer,
        size_t length);
static bool recv_message(
        const uxrSession* session,
        uint8_t** buffer,
        size_t* length,
        int poll_ms);

static void write_submessage_heartbeat(
        const uxrSession* session,
        uxrStreamId stream);
static void write_submessage_acknack(
        const uxrSession* session,
        uxrStreamId stream);

static void read_message(
        uxrSession* session,
        ucdrBuffer* message);
static void read_stream(
        uxrSession* session,
        ucdrBuffer* message,
        uxrStreamId id,
        uxrSeqNum seq_num);
static void read_submessage_list(
        uxrSession* session,
        ucdrBuffer* submessages,
        uxrStreamId stream_id);
static void read_submessage(
        uxrSession* session,
        ucdrBuffer* submessage,
        uint8_t submessage_id,
        uxrStreamId stream_id,
        uint16_t length,
        uint8_t flags);

static void read_submessage_status(
        uxrSession* session,
        ucdrBuffer* submessage);
static void read_submessage_data(
        uxrSession* session,
        ucdrBuffer* submessage,
        uint16_t length,
        uxrStreamId stream_id,
        uint8_t format);
static void read_submessage_heartbeat(
        uxrSession* session,
        ucdrBuffer* submessage);
static void read_submessage_acknack(
        uxrSession* session,
        ucdrBuffer* submessage);
static void read_submessage_timestamp_reply(
        uxrSession* session,
        ucdrBuffer* submessage);
static void read_submessage_get_info(
        uxrSession* session,
        ucdrBuffer* submessage);
void read_submessage_info(
        uxrSession* session,
        ucdrBuffer* submessage);
#ifdef PERFORMANCE_TESTING
static void read_submessage_performance(
        uxrSession* session,
        ucdrBuffer* submessage,
        uint16_t length);
#endif /* ifdef PERFORMANCE_TESTING */

static void process_status(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uint8_t status);
static void process_timestamp_reply(
        uxrSession* session,
        TIMESTAMP_REPLY_Payload* timestamp);

static FragmentationInfo on_get_fragmentation_info(
        uint8_t* submessage_header);

static bool run_session_until_sync(
        uxrSession* session,
        int timeout);

//==================================================================
//                             PUBLIC
//==================================================================

void uxr_init_session(
        uxrSession* session,
        uxrCommunication* comm,
        uint32_t key)
{
    UXR_INIT_LOCK(&session->mutex);

    session->comm = comm;

    session->request_list = NULL;
    session->status_list = NULL;
    session->request_status_list_size = 0;

    session->on_status = NULL;
    session->on_status_args = NULL;
    session->on_topic = NULL;
    session->on_topic_args = NULL;

    session->on_time = NULL;
    session->on_time_args = NULL;
    session->time_offset = 0;
    session->synchronized = false;

    uxr_init_session_info(&session->info, 0x81, key);
    uxr_init_stream_storage(&session->streams);
}

void uxr_set_status_callback(
        uxrSession* session,
        uxrOnStatusFunc on_status_func,
        void* args)
{
    session->on_status = on_status_func;
    session->on_status_args = args;
}

void uxr_set_topic_callback(
        uxrSession* session,
        uxrOnTopicFunc on_topic_func,
        void* args)
{
    session->on_topic = on_topic_func;
    session->on_topic_args = args;
}

void uxr_set_time_callback(
        uxrSession* session,
        uxrOnTimeFunc on_time_func,
        void* args)
{
    session->on_time = on_time_func;
    session->on_time_args = args;
}

void uxr_set_request_callback(
        uxrSession* session,
        uxrOnRequestFunc on_request_func,
        void* args)
{
    session->on_request = on_request_func;
    session->on_request_args = args;
}

void uxr_set_reply_callback(
        uxrSession* session,
        uxrOnReplyFunc on_reply_func,
        void* args)
{
    session->on_reply = on_reply_func;
    session->on_reply_args = args;
}

#ifdef PERFORMANCE_TESTING
void uxr_set_performance_callback(
        uxrSession* session,
        uxrOnPerformanceFunc on_echo_func,
        void* args)
{
    session->on_performance = on_echo_func;
    session->on_performance_args = args;
}

#endif /* ifdef PERFORMANCE_TESTING */

bool uxr_create_session_retries(
        uxrSession* session,
        size_t retries)
{
    uxr_reset_stream_storage(&session->streams);

    uint8_t create_session_buffer[CREATE_SESSION_MAX_MSG_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer_origin_offset(&ub, create_session_buffer, CREATE_SESSION_MAX_MSG_SIZE, 0u, uxr_session_header_offset(
                &session->info));

    uxr_buffer_create_session(&session->info, &ub, (uint16_t)(session->comm->mtu - INTERNAL_RELIABLE_BUFFER_OFFSET));
    uxr_stamp_create_session_header(&session->info, ub.init);

    bool received = wait_session_status(session, create_session_buffer, ucdr_buffer_length(&ub), (size_t) retries);
    bool created = received && UXR_STATUS_OK == session->info.last_requested_status;

    if (created)
    {
        uxr_reset_stream_storage(&session->streams);
    }

    return created;
}

bool uxr_create_session(
        uxrSession* session)
{
    return uxr_create_session_retries(session, UXR_CONFIG_MAX_SESSION_CONNECTION_ATTEMPTS);
}

bool uxr_delete_session_retries(
        uxrSession* session,
        size_t retries)
{
    UXR_CLEAN_SHARED_MEMORY();

    uint8_t delete_session_buffer[DELETE_SESSION_MAX_MSG_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer_origin_offset(&ub, delete_session_buffer, DELETE_SESSION_MAX_MSG_SIZE, 0u, uxr_session_header_offset(
                &session->info));

    uxr_buffer_delete_session(&session->info, &ub);
    uxr_stamp_session_header(&session->info, 0, 0, ub.init);

    bool received = wait_session_status(session, delete_session_buffer, ucdr_buffer_length(&ub), retries);
    return received && UXR_STATUS_OK == session->info.last_requested_status;
}

bool uxr_delete_session(
        uxrSession* session)
{
    return uxr_delete_session_retries(session, UXR_CONFIG_MAX_SESSION_CONNECTION_ATTEMPTS);
}

uxrStreamId uxr_create_output_best_effort_stream(
        uxrSession* session,
        uint8_t* buffer,
        size_t size)
{
    uint8_t header_offset = uxr_session_header_offset(&session->info);
    return uxr_add_output_best_effort_buffer(&session->streams, buffer, size, header_offset);
}

uxrStreamId uxr_create_output_reliable_stream(
        uxrSession* session,
        uint8_t* buffer,
        size_t size,
        uint16_t history)
{
    uint8_t header_offset = uxr_session_header_offset(&session->info);
    return uxr_add_output_reliable_buffer(&session->streams, buffer, size, history, header_offset);
}

uxrStreamId uxr_create_input_best_effort_stream(
        uxrSession* session)
{
    return uxr_add_input_best_effort_buffer(&session->streams);
}

uxrStreamId uxr_create_input_reliable_stream(
        uxrSession* session,
        uint8_t* buffer,
        size_t size,
        uint16_t history)
{
    return uxr_add_input_reliable_buffer(&session->streams, buffer, size, history, on_get_fragmentation_info);
}

bool uxr_run_session_time(
        uxrSession* session,
        int timeout_ms)
{
    UXR_LOCK_SESSION(session);

    uxr_flash_output_streams(session);

    bool timeout = false;
    while (!timeout)
    {
        timeout = !listen_message_reliably(session, timeout_ms);
    }

    bool ret = uxr_output_streams_confirmed(&session->streams);
    UXR_UNLOCK_SESSION(session);

    return ret;
}

bool uxr_run_session_timeout(
        uxrSession* session,
        int timeout_ms)
{
    UXR_LOCK_SESSION(session);

    int64_t start_timestamp = uxr_millis();
    int remaining_time = timeout_ms;

    uxr_flash_output_streams(session);

    do
    {
        listen_message_reliably(session, remaining_time);
        remaining_time = timeout_ms - (int)(uxr_millis() - start_timestamp);
    }
    while (remaining_time > 0);

    bool ret = uxr_output_streams_confirmed(&session->streams);
    UXR_UNLOCK_SESSION(session);

    return ret;
}

bool uxr_run_session_until_data(
        uxrSession* session,
        int timeout_ms)
{
    UXR_LOCK_SESSION(session);

    int64_t start_timestamp = uxr_millis();
    int remaining_time = timeout_ms;

    uxr_flash_output_streams(session);

    session->on_data_flag = false;
    do
    {
        listen_message_reliably(session, remaining_time);
        if (session->on_data_flag)
        {
            break;
        }
        remaining_time = timeout_ms - (int)(uxr_millis() - start_timestamp);
    }
    while (remaining_time > 0);

    bool ret = session->on_data_flag;

    UXR_UNLOCK_SESSION(session);
    return ret;
}

bool uxr_run_session_until_timeout(
        uxrSession* session,
        int timeout_ms)
{
    UXR_LOCK_SESSION(session);

    uxr_flash_output_streams(session);

    bool ret = listen_message_reliably(session, timeout_ms);
    UXR_UNLOCK_SESSION(session);

    return ret;
}

bool uxr_run_session_until_confirm_delivery(
        uxrSession* session,
        int timeout_ms)
{
    UXR_LOCK_SESSION(session);

    int64_t start_timestamp = uxr_millis();
    int remaining_time = timeout_ms;

    uxr_flash_output_streams(session);

    while (remaining_time >= 0 && !uxr_output_streams_confirmed(&session->streams))
    {
        listen_message_reliably(session, remaining_time);

        remaining_time = timeout_ms - (int)(uxr_millis() - start_timestamp);
    }

    bool ret = uxr_output_streams_confirmed(&session->streams);

    UXR_UNLOCK_SESSION(session);
    return ret;
}

bool uxr_run_session_until_all_status(
        uxrSession* session,
        int timeout_ms,
        const uint16_t* request_list,
        uint8_t* status_list,
        size_t list_size)
{
    UXR_LOCK_SESSION(session);

    uxr_flash_output_streams(session);

    for (unsigned i = 0; i < list_size; ++i)
    {
        status_list[i] = UXR_STATUS_NONE;
    }


    session->request_list = request_list;
    session->status_list = status_list;
    session->request_status_list_size = list_size;

    bool status_confirmed = false;

    int64_t start_timestamp = uxr_millis();
    int remaining_time = timeout_ms;

    do
    {
        listen_message_reliably(session, remaining_time);
        status_confirmed = true;
        remaining_time = timeout_ms - (int)(uxr_millis() - start_timestamp);
        for (size_t i = 0; i < list_size && status_confirmed; ++i)
        {
            status_confirmed = status_list[i] != UXR_STATUS_NONE
                    || request_list[i] == UXR_INVALID_REQUEST_ID;         //CHECK: better give an error? an assert?
        }
    }
    while (remaining_time > 0 && !status_confirmed);

    session->request_status_list_size = 0;

    bool status_ok = true;
    for (size_t i = 0; i < list_size && status_ok; ++i)
    {
        status_ok = status_list[i] == UXR_STATUS_OK || status_list[i] == UXR_STATUS_OK_MATCHED;
    }

    UXR_UNLOCK_SESSION(session);

    return status_ok;
}

bool uxr_run_session_until_one_status(
        uxrSession* session,
        int timeout_ms,
        const uint16_t* request_list,
        uint8_t* status_list,
        size_t list_size)
{
    UXR_LOCK_SESSION(session);

    uxr_flash_output_streams(session);

    for (unsigned i = 0; i < list_size; ++i)
    {
        status_list[i] = UXR_STATUS_NONE;
    }

    session->request_list = request_list;
    session->status_list = status_list;
    session->request_status_list_size = list_size;

    bool status_confirmed = false;

    int64_t start_timestamp = uxr_millis();
    int remaining_time = timeout_ms;

    do
    {
        listen_message_reliably(session, timeout_ms);
        remaining_time = timeout_ms - (int)(uxr_millis() - start_timestamp);
        for (unsigned i = 0; i < list_size && !status_confirmed; ++i)
        {
            status_confirmed = status_list[i] != UXR_STATUS_NONE
                    || request_list[i] == UXR_INVALID_REQUEST_ID;         //CHECK: better give an error? an assert?
        }
    }
    while (remaining_time > 0 && !status_confirmed);

    session->request_status_list_size = 0;
    UXR_UNLOCK_SESSION(session);

    return status_confirmed;
}

bool uxr_sync_session(
        uxrSession* session,
        int timeout)
{
    UXR_LOCK_SESSION(session);

    uint8_t timestamp_buffer[TIMESTAMP_MAX_MSG_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer_origin_offset(&ub, timestamp_buffer, sizeof(timestamp_buffer), 0u,
            uxr_session_header_offset(&session->info));
    uxr_buffer_submessage_header(&ub, SUBMESSAGE_ID_TIMESTAMP, TIMESTAMP_PAYLOAD_SIZE, 0);

    TIMESTAMP_Payload timestamp;
    int64_t nanos = uxr_nanos();
    timestamp.transmit_timestamp.seconds = (int32_t)(nanos / 1000000000);
    timestamp.transmit_timestamp.nanoseconds = (uint32_t)(nanos % 1000000000);
    (void) uxr_serialize_TIMESTAMP_Payload(&ub, &timestamp);

    uxr_stamp_session_header(&session->info, 0, 0, ub.init);
    send_message(session, timestamp_buffer, ucdr_buffer_length(&ub));
    bool ret = run_session_until_sync(session, timeout);
    UXR_UNLOCK_SESSION(session);

    return ret;
}

int64_t uxr_epoch_millis(
        uxrSession* session)
{
    return uxr_epoch_nanos(session) / 1000000;
}

int64_t uxr_epoch_nanos(
        uxrSession* session)
{
    return uxr_nanos() - session->time_offset;
}

#ifdef PERFORMANCE_TESTING
bool uxr_buffer_performance(
        uxrSession* session,
        uxrStreamId stream_id,
        uint64_t epoch_time,
        uint8_t* buf,
        uint16_t len,
        bool echo)
{
    bool rv = false;
    PERFORMANCE_Payload payload;
    payload.epoch_time_lsb = (uint32_t)(epoch_time & UINT32_MAX);
    payload.epoch_time_msb = (uint32_t)(epoch_time >> 32);
    payload.buf = buf;
    payload.len = len;
    ucdrBuffer mb;
    const uint16_t payload_length = (uint16_t)(sizeof(payload.epoch_time_lsb) +
            sizeof(payload.epoch_time_msb) +
            len);

    uint8_t flags = (echo) ? UXR_ECHO : 0;
    if (uxr_prepare_stream_to_write_submessage(session, stream_id, payload_length, &mb, SUBMESSAGE_ID_PERFORMANCE,
            flags))
    {
        (void) uxr_serialize_PERFORMANCE_Payload(&mb, &payload);
        rv = true;
    }
    return rv;
}

#endif /* ifdef PERFORMANCE_TESTING */

void uxr_flash_output_streams(
        uxrSession* session)
{
    UXR_HANDLE_SHARED_MEMORY();

    for (uint8_t i = 0; i < session->streams.output_best_effort_size; ++i)
    {
        uxrOutputBestEffortStream* stream = &session->streams.output_best_effort[i];
        uxrStreamId id = uxr_stream_id(i, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);

        uint8_t* buffer; size_t length; uxrSeqNum seq_num;

        UXR_LOCK_STREAM_ID(session, id);

        if (uxr_prepare_best_effort_buffer_to_send(stream, &buffer, &length, &seq_num))
        {
            uxr_stamp_session_header(&session->info, id.raw, seq_num, buffer);
            send_message(session, buffer, length);
        }

        UXR_UNLOCK_STREAM_ID(session, id);
    }

    for (uint8_t i = 0; i < session->streams.output_reliable_size; ++i)
    {
        uxrOutputReliableStream* stream = &session->streams.output_reliable[i];
        uxrStreamId id = uxr_stream_id(i, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);

        uint8_t* buffer; size_t length; uxrSeqNum seq_num;

        UXR_LOCK_STREAM_ID(session, id);

        while (uxr_prepare_next_reliable_buffer_to_send(stream, &buffer, &length, &seq_num))
        {
            uxr_stamp_session_header(&session->info, id.raw, seq_num, buffer);
            send_message(session, buffer, length);
        }

        UXR_UNLOCK_STREAM_ID(session, id);
    }
}

//==================================================================
//                             PRIVATE
//==================================================================
bool uxr_run_session_until_pong(
        uxrSession* session,
        int timeout_ms)
{
    int64_t start_timestamp = uxr_millis();
    int remaining_time = timeout_ms;

    uxr_flash_output_streams(session);

    session->on_pong_flag = NO_PONG_STATUS;
    do
    {
        listen_message_reliably(session, remaining_time);
        if (NO_PONG_STATUS != session->on_pong_flag)
        {
            break;
        }
        remaining_time = timeout_ms - (int)(uxr_millis() - start_timestamp);
    }
    while (remaining_time > 0);

    bool ret = PONG_IN_SESSION_STATUS == session->on_pong_flag;

    return ret;
}

bool listen_message(
        uxrSession* session,
        int poll_ms)
{
    uint8_t* data; size_t length;

    UXR_LOCK_ALL_INPUT_STREAMS(session);
    bool must_be_read = recv_message(session, &data, &length, poll_ms);
    if (must_be_read)
    {
        ucdrBuffer ub;
        ucdr_init_buffer(&ub, data, (uint32_t)length);
        read_message(session, &ub);
    }
    UXR_UNLOCK_ALL_INPUT_STREAMS(session);

    return must_be_read;
}

bool listen_message_reliably(
        uxrSession* session,
        int poll_ms)
{
    bool received = false;
    int32_t poll = (poll_ms >= 0) ? poll_ms : INT32_MAX;
    do
    {
        int64_t next_heartbeat_timestamp = INT64_MAX;
        int64_t timestamp = uxr_millis();
        for (uint8_t i = 0; i < session->streams.output_reliable_size; ++i)
        {
            uxrOutputReliableStream* stream = &session->streams.output_reliable[i];
            uxrStreamId id = uxr_stream_id(i, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);

            UXR_LOCK_STREAM_ID(session, id);
            if (uxr_update_output_stream_heartbeat_timestamp(stream, timestamp))
            {
                write_submessage_heartbeat(session, id);
            }

            if (stream->next_heartbeat_timestamp < next_heartbeat_timestamp)
            {
                next_heartbeat_timestamp = stream->next_heartbeat_timestamp;
            }
            UXR_UNLOCK_STREAM_ID(session, id);
        }

        int32_t poll_to_next_heartbeat =
                (next_heartbeat_timestamp != INT64_MAX) ? (int32_t)(next_heartbeat_timestamp - timestamp) : poll;
        if (0 == poll_to_next_heartbeat)
        {
            poll_to_next_heartbeat = 1;
        }

        int poll_chosen = (poll_to_next_heartbeat < poll) ? poll_to_next_heartbeat : poll;
        received = listen_message(session, poll_chosen);
        if (!received)
        {
            poll -= poll_chosen;
        }
    }
    while (!received && poll > 0);

    return received;
}

bool wait_session_status(
        uxrSession* session,
        uint8_t* buffer,
        size_t length,
        size_t attempts)
{
    session->info.last_requested_status = UXR_STATUS_NONE;

    if (0 == attempts)
    {
        send_message(session, buffer, length);
        return true;
    }

    for (size_t i = 0; i < attempts && session->info.last_requested_status == UXR_STATUS_NONE; ++i)
    {
        send_message(session, buffer, length);

        int64_t start_timestamp = uxr_millis();
        int remaining_time = UXR_CONFIG_MIN_SESSION_CONNECTION_INTERVAL;

        do
        {
            listen_message(session, remaining_time);
            remaining_time = UXR_CONFIG_MIN_SESSION_CONNECTION_INTERVAL - (int)(uxr_millis() - start_timestamp);
        } while (remaining_time > 0 && session->info.last_requested_status == UXR_STATUS_NONE);
    }

    return session->info.last_requested_status != UXR_STATUS_NONE;
}

inline bool send_message(
        const uxrSession* session,
        uint8_t* buffer,
        size_t length)
{
    bool sent = session->comm->send_msg(session->comm->instance, buffer, length);

    UXR_DEBUG_PRINT_MESSAGE((sent) ? UXR_SEND : UXR_ERROR_SEND, buffer, length, session->info.key);
    return sent;
}

inline bool recv_message(
        const uxrSession* session,
        uint8_t** buffer,
        size_t* length,
        int poll_ms)
{
    bool received = session->comm->recv_msg(session->comm->instance, buffer, length, poll_ms);

    if (received)
    {
        UXR_DEBUG_PRINT_MESSAGE(UXR_RECV, *buffer, *length, session->info.key);
    }
    return received;
}

void write_submessage_heartbeat(
        const uxrSession* session,
        uxrStreamId id)
{
    uint8_t heartbeat_buffer[HEARTBEAT_MAX_MSG_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer_origin_offset(&ub, heartbeat_buffer, HEARTBEAT_MAX_MSG_SIZE, 0u,
            uxr_session_header_offset(&session->info));

    const uxrOutputReliableStream* stream = &session->streams.output_reliable[id.index];

    /* Buffer submessage header. */
    uxr_buffer_submessage_header(&ub, SUBMESSAGE_ID_HEARTBEAT, HEARTBEAT_PAYLOAD_SIZE, 0);

    /* Buffer HEARTBEAT. */
    HEARTBEAT_Payload payload;
    payload.first_unacked_seq_nr = uxr_seq_num_add(stream->last_acknown, 1);
    payload.last_unacked_seq_nr = stream->last_sent;
    payload.stream_id = id.raw;
    (void) uxr_serialize_HEARTBEAT_Payload(&ub, &payload);

    /* Stamp message header. */
    uxr_stamp_session_header(&session->info, 0, 0, ub.init);
    send_message(session, heartbeat_buffer, ucdr_buffer_length(&ub));
}

void write_submessage_acknack(
        const uxrSession* session,
        uxrStreamId id)
{
    uint8_t acknack_buffer[ACKNACK_MAX_MSG_SIZE];
    ucdrBuffer ub;
    ucdr_init_buffer_origin_offset(&ub, acknack_buffer, ACKNACK_MAX_MSG_SIZE, 0u,
            uxr_session_header_offset(&session->info));

    const uxrInputReliableStream* stream = &session->streams.input_reliable[id.index];

    /* Buffer submessage header. */
    uxr_buffer_submessage_header(&ub, SUBMESSAGE_ID_ACKNACK, ACKNACK_PAYLOAD_SIZE, 0);

    /* Buffer ACKNACK. */
    ACKNACK_Payload payload;
    uint16_t nack_bitmap = uxr_compute_acknack(stream, &payload.first_unacked_seq_num);
    payload.nack_bitmap[0] = (uint8_t)(nack_bitmap >> 8);
    payload.nack_bitmap[1] = (uint8_t)((nack_bitmap << 8) >> 8);
    payload.stream_id = id.raw;
    (void) uxr_serialize_ACKNACK_Payload(&ub, &payload);

    /* Stamp message header. */
    uxr_stamp_session_header(&session->info, 0, 0, ub.init);
    send_message(session, acknack_buffer, ucdr_buffer_length(&ub));
}

void read_message(
        uxrSession* session,
        ucdrBuffer* ub)
{
    uint8_t stream_id_raw = 0;
    uxrSeqNum seq_num;
    if (uxr_read_session_header(&session->info, ub, &stream_id_raw, &seq_num))
    {
        uxrStreamId id = uxr_stream_id_from_raw(stream_id_raw, UXR_INPUT_STREAM);
        read_stream(session, ub, id, seq_num);
    }
}

void read_stream(
        uxrSession* session,
        ucdrBuffer* ub,
        uxrStreamId stream_id,
        uxrSeqNum seq_num)
{
    switch (stream_id.type)
    {
        case UXR_NONE_STREAM:
        {
            stream_id = uxr_stream_id_from_raw(0x00, UXR_INPUT_STREAM);
            read_submessage_list(session, ub, stream_id);
            break;
        }
        case UXR_BEST_EFFORT_STREAM:
        {
            uxrInputBestEffortStream* stream = uxr_get_input_best_effort_stream(&session->streams, stream_id.index);
            if (stream && uxr_receive_best_effort_message(stream, seq_num))
            {
                read_submessage_list(session, ub, stream_id);
            }
            break;
        }
        case UXR_RELIABLE_STREAM:
        {
            uxrInputReliableStream* stream = uxr_get_input_reliable_stream(&session->streams, stream_id.index);
            bool input_buffer_used;
            if (stream &&
                    uxr_receive_reliable_message(stream, seq_num, ub->iterator, ucdr_buffer_remaining(
                        ub), &input_buffer_used))
            {
                if (!input_buffer_used)
                {
                    read_submessage_list(session, ub, stream_id);
                }

                ucdrBuffer next_mb;
                while (uxr_next_input_reliable_buffer_available(stream, &next_mb, SUBHEADER_SIZE))
                {
                    read_submessage_list(session, &next_mb, stream_id);
                }
            }
            write_submessage_acknack(session, stream_id);
            break;
        }
        default:
            break;
    }
}

void read_submessage_list(
        uxrSession* session,
        ucdrBuffer* submessages,
        uxrStreamId stream_id)
{
    uint8_t id; uint16_t length; uint8_t flags;
    while (uxr_read_submessage_header(submessages, &id, &length, &flags))
    {
        read_submessage(session, submessages, id, stream_id, length, flags);
    }
}

void read_submessage(
        uxrSession* session,
        ucdrBuffer* submessage,
        uint8_t submessage_id,
        uxrStreamId stream_id,
        uint16_t length,
        uint8_t flags)
{
    switch (submessage_id)
    {
        case SUBMESSAGE_ID_STATUS_AGENT:
            if (stream_id.type == UXR_NONE_STREAM)
            {
                uxr_read_create_session_status(&session->info, submessage);
            }
            break;

        case SUBMESSAGE_ID_STATUS:
            if (stream_id.type == UXR_NONE_STREAM)
            {
                uxr_read_delete_session_status(&session->info, submessage);
            }
            else
            {
                read_submessage_status(session, submessage);
            }
            break;

        case SUBMESSAGE_ID_DATA:
            read_submessage_data(session, submessage, length, stream_id, flags & FORMAT_MASK);
            break;

        case SUBMESSAGE_ID_HEARTBEAT:
            read_submessage_heartbeat(session, submessage);
            break;

        case SUBMESSAGE_ID_ACKNACK:
            read_submessage_acknack(session, submessage);
            break;

        case SUBMESSAGE_ID_TIMESTAMP_REPLY:
            read_submessage_timestamp_reply(session, submessage);
            break;

        case SUBMESSAGE_ID_GET_INFO:
            read_submessage_get_info(session, submessage);
            break;

        case SUBMESSAGE_ID_INFO:
            read_submessage_info(session, submessage);
            break;

#ifdef PERFORMANCE_TESTING
        case SUBMESSAGE_ID_PERFORMANCE:
            read_submessage_performance(session, submessage, length);
            break;
#endif /* ifdef PERFORMANCE_TESTING */

        default:
            break;
    }
}

void read_submessage_status(
        uxrSession* session,
        ucdrBuffer* submessage)
{
    STATUS_Payload payload;
    uxr_deserialize_STATUS_Payload(submessage, &payload);

    uxrObjectId object_id; uint16_t request_id;
    uxr_parse_base_object_request(&payload.base.related_request, &object_id, &request_id);

    uint8_t status = payload.base.result.status;
    process_status(session, object_id, request_id, status);
}

extern void read_submessage_format(
        uxrSession* session,
        ucdrBuffer* data,
        uint16_t length,
        uint8_t format,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id);

void read_submessage_data(
        uxrSession* session,
        ucdrBuffer* submessage,
        uint16_t length,
        uxrStreamId stream_id,
        uint8_t format)
{
    BaseObjectRequest base;
    uxr_deserialize_BaseObjectRequest(submessage, &base);
    length = (uint16_t)(length - 4); //CHANGE: by a future size_of_BaseObjectRequest

    uxrObjectId object_id;
    uint16_t request_id;
    uxr_parse_base_object_request(&base, &object_id, &request_id);

    process_status(session, object_id, request_id, UXR_STATUS_OK);
    read_submessage_format(session, submessage, length, format, stream_id, object_id, request_id);
}

void read_submessage_heartbeat(
        uxrSession* session,
        ucdrBuffer* submessage)
{
    HEARTBEAT_Payload heartbeat;
    uxr_deserialize_HEARTBEAT_Payload(submessage, &heartbeat);
    uxrStreamId id = uxr_stream_id_from_raw(heartbeat.stream_id, UXR_INPUT_STREAM);

    uxrInputReliableStream* stream = uxr_get_input_reliable_stream(&session->streams, id.index);
    if (stream)
    {
        uxr_process_heartbeat(stream, heartbeat.first_unacked_seq_nr, heartbeat.last_unacked_seq_nr);
        write_submessage_acknack(session, id);
    }
}

void read_submessage_acknack(
        uxrSession* session,
        ucdrBuffer* submessage)
{
    ACKNACK_Payload acknack;
    uxr_deserialize_ACKNACK_Payload(submessage, &acknack);
    uxrStreamId id = uxr_stream_id_from_raw(acknack.stream_id, UXR_INPUT_STREAM);

    uxrOutputReliableStream* stream = uxr_get_output_reliable_stream(&session->streams, id.index);
    if (stream)
    {
        UXR_LOCK_STREAM_ID(session, id);

        uint16_t nack_bitmap = (uint16_t)(((uint16_t)acknack.nack_bitmap[0] << 8) + acknack.nack_bitmap[1]);
        uxr_process_acknack(stream, nack_bitmap, acknack.first_unacked_seq_num);

        uint8_t* buffer; size_t length;
        uxrSeqNum seq_num_it = uxr_begin_output_nack_buffer_it(stream);
        while (uxr_next_reliable_nack_buffer_to_send(stream, &buffer, &length, &seq_num_it))
        {
            send_message(session, buffer, length);
        }

        UXR_UNLOCK_STREAM_ID(session, id);
    }
}

void read_submessage_timestamp_reply(
        uxrSession* session,
        ucdrBuffer* submessage)
{
    TIMESTAMP_REPLY_Payload timestamp_reply;
    uxr_deserialize_TIMESTAMP_REPLY_Payload(submessage, &timestamp_reply);

    process_timestamp_reply(session, &timestamp_reply);
}

void read_submessage_get_info(
        uxrSession* session,
        ucdrBuffer* submessage)
{
    GET_INFO_Payload get_info_payload = {
        0
    };
    INFO_Payload info_payload = {
        0
    };

    uxr_deserialize_GET_INFO_Payload(submessage, &get_info_payload);

    info_payload.base.related_request.request_id = get_info_payload.base.request_id;

    uint8_t buffer[12];
    ucdrBuffer ub;
    ucdr_init_buffer_origin_offset(&ub, buffer, sizeof(buffer), 0u, uxr_session_header_offset(&session->info));

    uxr_serialize_INFO_Payload(&ub, &info_payload);
    uxr_stamp_session_header(&session->info, 0, 0, ub.init);

    send_message(session, buffer, ucdr_buffer_length(&ub));
}

void read_submessage_info(
        uxrSession* session,
        ucdrBuffer* submessage)
{
    INFO_Payload info_payload;

    bool success = true;

    success &= uxr_deserialize_BaseObjectReply(submessage, &info_payload.base);
    bool active_session = info_payload.base.result.implementation_status;

    success &= ucdr_deserialize_bool(submessage, &info_payload.object_info.optional_config);

    if (info_payload.object_info.optional_config)
    {
        success &= uxr_deserialize_ObjectVariant(submessage, &info_payload.object_info.config);
    }

    success &= ucdr_deserialize_bool(submessage, &info_payload.object_info.optional_activity);
    if (info_payload.object_info.optional_activity)
    {
        success &= ucdr_deserialize_uint8_t(submessage, &info_payload.object_info.activity.kind);
        if (success && DDS_XRCE_OBJK_AGENT == info_payload.object_info.activity.kind)
        {
            success &= ucdr_deserialize_int16_t(submessage,
                            &info_payload.object_info.activity._.agent.availability);
            session->on_pong_flag = (success && (info_payload.object_info.activity._.agent.availability > 0)) ?
                    (active_session ?
                    PONG_IN_SESSION_STATUS :
                    PONG_NO_SESSION_STATUS) :
                    NO_PONG_STATUS;
        }
    }
}

#ifdef PERFORMANCE_TESTING
void read_submessage_performance(
        uxrSession* session,
        ucdrBuffer* submessage,
        uint16_t length)
{
    ucdrBuffer mb_performance;
    ucdr_init_buffer(&mb_performance, submessage->iterator, length);
    session->on_performance(session, &mb_performance, session->on_performance_args);
}

#endif /* ifdef PERFORMANCE_TESTING */

void process_status(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uint8_t status)
{
    if (session->on_status != NULL)
    {
        session->on_status(session, object_id, request_id, status, session->on_status_args);
    }

    for (unsigned i = 0; i < session->request_status_list_size; ++i)
    {
        if (request_id == session->request_list[i])
        {
            session->status_list[i] = status;
            break;
        }
    }
}

void process_timestamp_reply(
        uxrSession* session,
        TIMESTAMP_REPLY_Payload* timestamp)
{
    if (session->on_time != NULL)
    {
        session->on_time(session,
                uxr_nanos(),
                uxr_convert_to_nanos(timestamp->receive_timestamp.seconds, timestamp->receive_timestamp.nanoseconds),
                uxr_convert_to_nanos(timestamp->transmit_timestamp.seconds, timestamp->transmit_timestamp.nanoseconds),
                uxr_convert_to_nanos(timestamp->originate_timestamp.seconds,
                timestamp->originate_timestamp.nanoseconds),
                session->on_time_args);
    }
    else
    {
        int64_t t3 = uxr_nanos();
        int64_t t0 = uxr_convert_to_nanos(timestamp->originate_timestamp.seconds,
                        timestamp->originate_timestamp.nanoseconds);
        int64_t t1 = uxr_convert_to_nanos(timestamp->receive_timestamp.seconds,
                        timestamp->receive_timestamp.nanoseconds);
        int64_t t2 = uxr_convert_to_nanos(timestamp->transmit_timestamp.seconds,
                        timestamp->transmit_timestamp.nanoseconds);
        session->time_offset = ((t0 + t3) - (t1 + t2)) / 2;
    }
    session->synchronized = true;
}

bool uxr_prepare_stream_to_write_submessage(
        uxrSession* session,
        uxrStreamId stream_id,
        size_t payload_size,
        ucdrBuffer* ub,
        uint8_t submessage_id,
        uint8_t mode)
{
    bool available = false;
    size_t submessage_size = SUBHEADER_SIZE + payload_size + uxr_submessage_padding(payload_size);

    switch (stream_id.type)
    {
        case UXR_BEST_EFFORT_STREAM:
        {
            uxrOutputBestEffortStream* stream = uxr_get_output_best_effort_stream(&session->streams, stream_id.index);
            available = stream && uxr_prepare_best_effort_buffer_to_write(stream, submessage_size, ub);
            break;
        }
        case UXR_RELIABLE_STREAM:
        {
            uxrOutputReliableStream* stream = uxr_get_output_reliable_stream(&session->streams, stream_id.index);
            available = stream && uxr_prepare_reliable_buffer_to_write(stream, submessage_size, ub);
            break;
        }
        default:
            break;
    }

    if (available)
    {
        (void) uxr_buffer_submessage_header(ub, submessage_id, (uint16_t)payload_size, mode);
    }

    return available;
}

FragmentationInfo on_get_fragmentation_info(
        uint8_t* submessage_header)
{
    ucdrBuffer ub;
    ucdr_init_buffer(&ub, submessage_header, SUBHEADER_SIZE);

    uint8_t id; uint16_t length; uint8_t flags;
    uxr_read_submessage_header(&ub, &id, &length, &flags);

    FragmentationInfo fragmentation_info;
    if (SUBMESSAGE_ID_FRAGMENT == id)
    {
        fragmentation_info = FLAG_LAST_FRAGMENT & flags ? LAST_FRAGMENT : INTERMEDIATE_FRAGMENT;
    }
    else
    {
        fragmentation_info = NO_FRAGMENTED;
    }
    return fragmentation_info;
}

bool run_session_until_sync(
        uxrSession* session,
        int timeout)
{
    int64_t start_timestamp = uxr_millis();
    int remaining_time = timeout;
    session->synchronized = false;

    do
    {
        listen_message_reliably(session, remaining_time);
        remaining_time = timeout - (int)(uxr_millis() - start_timestamp);
    } while (remaining_time > 0 && !session->synchronized);

    return session->synchronized;
}
