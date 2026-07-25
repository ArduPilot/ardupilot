#include <uxr/client/core/session/read_access.h>
#include <uxr/client/core/type/xrce_types.h>

#include "session_internal.h"
#include "session_info_internal.h"
#include "submessage_internal.h"
#include <uxr/client/profile/multithread/multithread.h>

extern void read_submessage_format(
        uxrSession* session,
        ucdrBuffer* data,
        uint16_t length,
        uint8_t format,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id);

static void read_format_data(
        uxrSession* session,
        ucdrBuffer* payload,
        const uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id);

static void read_format_sample(
        uxrSession* session,
        ucdrBuffer* payload,
        uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id);

static void read_format_data_seq(
        uxrSession* session,
        ucdrBuffer* payload,
        uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id);

static void read_format_sample_seq(
        uxrSession* session,
        ucdrBuffer* payload,
        uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id);

static void read_format_packed_samples(
        uxrSession* session,
        ucdrBuffer* payload,
        uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id);

//==================================================================
//                             PUBLIC
//==================================================================
uint16_t uxr_buffer_request_data(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId datareader_id,
        uxrStreamId data_stream_id,
        const uxrDeliveryControl* const control)
{
    uint16_t request_id = UXR_INVALID_REQUEST_ID;

    READ_DATA_Payload payload;
    payload.read_specification.preferred_stream_id = data_stream_id.raw;
    payload.read_specification.data_format = FORMAT_DATA;
    payload.read_specification.optional_content_filter_expression = false; //not supported yet
    payload.read_specification.optional_delivery_control = (control != NULL);

    if (control != NULL)
    {
        payload.read_specification.delivery_control.max_bytes_per_seconds = control->max_bytes_per_second;
        payload.read_specification.delivery_control.max_elapsed_time = control->max_elapsed_time;
        payload.read_specification.delivery_control.max_samples = control->max_samples;
        payload.read_specification.delivery_control.min_pace_period = control->min_pace_period;
    }

    // Change this when microcdr supports size_of function.
    size_t payload_length = 0; //READ_DATA_Payload_size(&payload);
    payload_length += 4; // (request id + object_id), no padding.
    payload_length += 4; // stream, format, and two optionals.
    payload_length += (control != NULL) ? 8 : 0; // delivery control

    ucdrBuffer ub;

    UXR_LOCK_STREAM_ID(session, stream_id);

    if (uxr_prepare_stream_to_write_submessage(session, stream_id, payload_length, &ub, SUBMESSAGE_ID_READ_DATA, 0))
    {
        request_id = uxr_init_base_object_request(&session->info, datareader_id, &payload.base);
        (void) uxr_serialize_READ_DATA_Payload(&ub, &payload);
    }

    UXR_UNLOCK_STREAM_ID(session, stream_id);

    return request_id;
}

uint16_t uxr_buffer_cancel_data(
        uxrSession* session,
        uxrStreamId stream_id,
        uxrObjectId datareader_id)
{
    uxrDeliveryControl delivery_control = {
        0
    };
    uxrStreamId in_stream = {
        0
    };
    return uxr_buffer_request_data(session, stream_id, datareader_id, in_stream, &delivery_control);
}

//==================================================================
//                            PRIVATE
//==================================================================
void read_submessage_format(
        uxrSession* session,
        ucdrBuffer* data,
        uint16_t length,
        uint8_t format,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id)
{
    switch (format)
    {
        case FORMAT_DATA:
            read_format_data(session, data, length, stream_id, object_id, request_id);
            break;

        case FORMAT_SAMPLE:
            read_format_sample(session, data, length, stream_id, object_id, request_id);
            break;

        case FORMAT_DATA_SEQ:
            read_format_data_seq(session, data, length, stream_id, object_id, request_id);
            break;

        case FORMAT_SAMPLE_SEQ:
            read_format_sample_seq(session, data, length, stream_id, object_id, request_id);
            break;

        case FORMAT_PACKED_SAMPLES:
            read_format_packed_samples(session, data, length, stream_id, object_id, request_id);
            break;

        default:
            break;
    }
}

inline void read_format_data(
        uxrSession* session,
        ucdrBuffer* ub,
        const uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id)
{
    ucdrBuffer temp_buffer;
    ucdr_init_buffer(&temp_buffer, ub->iterator, (size_t)(ub->final - ub->iterator));
    ucdr_set_on_full_buffer_callback(&temp_buffer, ub->on_full_buffer, ub->args);
    if (ub->args)
    {
        uxrInputReliableStream* stream = (uxrInputReliableStream*) ub->args;
        stream->cleanup_flag = false;
    }

    switch (object_id.type)
    {
        case UXR_DATAREADER_ID:
        {
            if (NULL != session->on_topic)
            {
                session->on_topic(session, object_id, request_id, stream_id, &temp_buffer, length,
                        session->on_topic_args);
                session->on_data_flag = true;
            }
            break;
        }
        case UXR_REPLIER_ID:
        {
            if (NULL != session->on_request)
            {
                SampleIdentity sample_id;
                size_t offset = temp_buffer.offset;

                if (uxr_deserialize_SampleIdentity(&temp_buffer, &sample_id))
                {
                    uint16_t request_length = (uint16_t)(length - (temp_buffer.offset - offset));
                    ucdr_init_buffer(&temp_buffer, temp_buffer.iterator,
                            (size_t)(temp_buffer.final - temp_buffer.iterator));
                    ucdr_set_on_full_buffer_callback(&temp_buffer, ub->on_full_buffer, ub->args);

                    session->on_request(
                        session,
                        object_id,
                        request_id,
                        &sample_id,
                        &temp_buffer,
                        request_length,
                        session->on_request_args);

                    session->on_data_flag = true;
                }
            }
            break;
        }
        case UXR_REQUESTER_ID:
        {
            if (NULL != session->on_reply)
            {
                BaseObjectRequest request;
                size_t offset = temp_buffer.offset;

                if (uxr_deserialize_BaseObjectRequest(&temp_buffer, &request))
                {
                    uint16_t reply_length = (uint16_t)(length - (temp_buffer.offset - offset));
                    ucdr_init_buffer(&temp_buffer, temp_buffer.iterator,
                            (size_t)(temp_buffer.final - temp_buffer.iterator));
                    ucdr_set_on_full_buffer_callback(&temp_buffer, ub->on_full_buffer, ub->args);

                    session->on_reply(
                        session,
                        object_id,
                        request_id,
                        (uint16_t)((request.request_id.data[0] << 8) + request.request_id.data[1]),
                        &temp_buffer,
                        reply_length,
                        session->on_reply_args);

                    session->on_data_flag = true;
                }
            }
            ub->iterator += length;
            break;
        }
        default:
            break;
    }

    if (ub->args)
    {
        uxrInputReliableStream* stream = (uxrInputReliableStream*) ub->args;
        stream->cleanup_flag = true;
    }
    ucdr_advance_buffer(ub, length);
}

void read_format_sample(
        uxrSession* session,
        ucdrBuffer* payload,
        uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id)
{
    (void) session; (void) payload; (void) length; (void) stream_id; (void) object_id; (void) request_id;
    //TODO
}

void read_format_data_seq(
        uxrSession* session,
        ucdrBuffer* payload,
        uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id)
{
    (void) session; (void) payload; (void) length; (void) stream_id; (void) object_id; (void) request_id;
    //TODO
}

void read_format_sample_seq(
        uxrSession* session,
        ucdrBuffer* payload,
        uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id)
{
    (void) session; (void) payload; (void) length; (void) stream_id; (void) object_id; (void) request_id;
    //TODO
}

void read_format_packed_samples(
        uxrSession* session,
        ucdrBuffer* payload,
        uint16_t length,
        uxrStreamId stream_id,
        uxrObjectId object_id,
        uint16_t request_id)
{
    (void) session; (void) payload; (void) length; (void) stream_id; (void) object_id; (void) request_id;
    //TODO
}
