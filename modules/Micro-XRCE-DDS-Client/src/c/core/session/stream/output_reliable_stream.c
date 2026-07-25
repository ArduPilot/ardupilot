#include "output_reliable_stream_internal.h"

#include <uxr/client/config.h>
#include <ucdr/microcdr.h>
#include <string.h>

#include "./seq_num_internal.h"
#include "./output_reliable_stream_internal.h"
#include "./common_reliable_stream_internal.h"
#include "../submessage_internal.h"
#include <uxr/client/profile/multithread/multithread.h>

#define MIN_HEARTBEAT_TIME_INTERVAL ((int64_t) UXR_CONFIG_MIN_HEARTBEAT_TIME_INTERVAL) // ms

//==================================================================
//                             PUBLIC
//==================================================================
void uxr_init_output_reliable_stream(
        uxrOutputReliableStream* stream,
        uint8_t* buffer,
        size_t size,
        uint16_t history,
        uint8_t header_offset)
{
    // assert for history (must be 2^)

    stream->base.buffer = buffer;
    stream->base.size = size;
    stream->base.history = history;
    stream->offset = header_offset;

    UXR_INIT_LOCK(&stream->mutex);

    uxr_reset_output_reliable_stream(stream);
}

void uxr_reset_output_reliable_stream(
        uxrOutputReliableStream* stream)
{
    for (uint16_t i = 0; i < stream->base.history; ++i)
    {
        uxr_set_reliable_buffer_size(&stream->base, i, stream->offset);
    }

    stream->last_written = 0;
    stream->last_sent = SEQ_NUM_MAX;
    stream->last_acknown = SEQ_NUM_MAX;

    stream->next_heartbeat_timestamp = INT64_MAX;
    stream->next_heartbeat_tries = 0;
    stream->send_lost = false;
}

bool uxr_prepare_reliable_buffer_to_write(
        uxrOutputReliableStream* stream,
        size_t length,
        ucdrBuffer* ub)
{
    bool available_to_write = false;
    uxrSeqNum seq_num = stream->last_written;
    size_t buffer_capacity = uxr_get_reliable_buffer_capacity(&stream->base);
    uint8_t* buffer = uxr_get_reliable_buffer(&stream->base, seq_num);
    size_t buffer_size = uxr_get_reliable_buffer_size(&stream->base, seq_num);

    uint16_t available_block_size = (uint16_t)(buffer_capacity - (uint16_t)(stream->offset + SUBHEADER_SIZE));
    size_t remaining_blocks = get_available_free_slots(stream);

    // Aligment required for inserting an XRCE subheader
    buffer_size += ucdr_alignment(buffer_size, 4);

    /* Check if the message fit in the current buffer */
    if (buffer_size + length <= buffer_capacity)
    {
        /* Check if there is space in the stream history to write */
        uxrSeqNum last_available = uxr_seq_num_add(stream->last_acknown, stream->base.history);
        available_to_write = (0 >= uxr_seq_num_cmp(seq_num, last_available));
        if (available_to_write)
        {
            size_t final_buffer_size = buffer_size + length;
            uxr_set_reliable_buffer_size(&stream->base, seq_num, final_buffer_size);
            ucdr_init_buffer_origin_offset(ub, buffer, (uint32_t)final_buffer_size, 0u, (uint32_t)buffer_size);
        }
    }
    /* Check if the message fit in a new empty buffer */
    else if (stream->offset + length <= buffer_capacity)
    {
        /* Check if there is space in the stream history to write */
        seq_num = uxr_seq_num_add(stream->last_written, 1);
        uxrSeqNum last_available = uxr_seq_num_add(stream->last_acknown, stream->base.history);
        available_to_write = (0 >= uxr_seq_num_cmp(seq_num, last_available));
        if (available_to_write)
        {
            buffer = uxr_get_reliable_buffer(&stream->base, seq_num);
            size_t final_buffer_size = stream->offset + length;
            uxr_set_reliable_buffer_size(&stream->base, seq_num, final_buffer_size);
            ucdr_init_buffer_origin_offset(ub, buffer, (uint32_t)final_buffer_size, 0u, stream->offset);
            stream->last_written = seq_num;
        }
    }
    /* Check if the message fit in a fragmented message */
    else if (length <= available_block_size * remaining_blocks)
    {
        /* Check if the current buffer free space is too small */
        if (buffer_size + (size_t)SUBHEADER_SIZE >= buffer_capacity)
        {
            seq_num = uxr_seq_num_add(seq_num, 1);
            buffer = uxr_get_reliable_buffer(&stream->base, seq_num);
            buffer_size = uxr_get_reliable_buffer_size(&stream->base, seq_num);
        }

        uint16_t first_fragment_size = (uint16_t)(buffer_capacity - (uint16_t)(buffer_size + SUBHEADER_SIZE));
        size_t remaining_size = length - first_fragment_size;
        size_t last_fragment_size = 0;
        uint16_t necessary_complete_blocks = 0;
        if (0 == (remaining_size % available_block_size))
        {
            last_fragment_size = available_block_size;
            necessary_complete_blocks = (uint16_t)((remaining_size / available_block_size));
        }
        else
        {
            last_fragment_size = remaining_size % available_block_size;
            necessary_complete_blocks = (uint16_t)((remaining_size / available_block_size) + 1);
        }

        available_to_write = necessary_complete_blocks <= remaining_blocks;
        if (available_to_write)
        {
            ucdrBuffer temp_ub;
            uint16_t fragment_size = first_fragment_size;
            for (uint16_t i = 0; i < necessary_complete_blocks; i++)
            {
                ucdr_init_buffer_origin_offset(
                    &temp_ub,
                    uxr_get_reliable_buffer(&stream->base, seq_num),
                    buffer_capacity,
                    0u,
                    uxr_get_reliable_buffer_size(&stream->base, seq_num));
                uxr_buffer_submessage_header(&temp_ub, SUBMESSAGE_ID_FRAGMENT, fragment_size, 0);
                uxr_set_reliable_buffer_size(&stream->base, seq_num, buffer_capacity);
                seq_num = uxr_seq_num_add(seq_num, 1);
                fragment_size = available_block_size;
            }

            //  Prepare last fragment
            ucdr_init_buffer_origin_offset(
                &temp_ub,
                uxr_get_reliable_buffer(&stream->base, seq_num),
                buffer_capacity,
                0u,
                uxr_get_reliable_buffer_size(&stream->base, seq_num));
            uxr_buffer_submessage_header(&temp_ub, SUBMESSAGE_ID_FRAGMENT, (uint16_t)last_fragment_size,
                    FLAG_LAST_FRAGMENT);
            uxr_set_reliable_buffer_size(&stream->base, seq_num,
                    stream->offset + (size_t)(SUBHEADER_SIZE) + last_fragment_size);

            // Prepare user buffer
            ucdr_init_buffer(
                ub,
                buffer + buffer_size + SUBHEADER_SIZE,
                (uint32_t)(buffer_capacity - buffer_size - SUBHEADER_SIZE));
            ucdr_set_on_full_buffer_callback(ub, on_full_output_buffer, stream);
            stream->last_written = seq_num;
        }
    }

    return available_to_write;
}

bool uxr_prepare_next_reliable_buffer_to_send(
        uxrOutputReliableStream* stream,
        uint8_t** buffer,
        size_t* length,
        uxrSeqNum* seq_num)
{
    *seq_num = uxr_seq_num_add(stream->last_sent, 1);
    *buffer = uxr_get_reliable_buffer(&stream->base, *seq_num);
    *length = uxr_get_reliable_buffer_size(&stream->base, *seq_num);

    bool data_to_send = 0 >= uxr_seq_num_cmp(*seq_num, stream->last_written)
            && *length > stream->offset
            && uxr_seq_num_sub(stream->last_sent, stream->last_acknown) != stream->base.history;
    if (data_to_send)
    {
        stream->last_sent = *seq_num;
        if (stream->last_sent == stream->last_written)
        {
            stream->last_written = uxr_seq_num_add(stream->last_written, 1);
        }
    }

    return data_to_send;
}

bool uxr_update_output_stream_heartbeat_timestamp(
        uxrOutputReliableStream* stream,
        int64_t current_timestamp)
{
    bool must_confirm = false;
    if (0 > uxr_seq_num_cmp(stream->last_acknown, stream->last_sent))
    {
        if (0 == stream->next_heartbeat_tries)
        {
            stream->next_heartbeat_tries = 1;
        }
        else if (current_timestamp >= stream->next_heartbeat_timestamp)
        {
            stream->next_heartbeat_tries++;
            must_confirm = true;
        }

        stream->next_heartbeat_timestamp = current_timestamp + MIN_HEARTBEAT_TIME_INTERVAL;
    }
    else
    {
        stream->next_heartbeat_timestamp = INT64_MAX;
    }

    return must_confirm;
}

uxrSeqNum uxr_begin_output_nack_buffer_it(
        const uxrOutputReliableStream* stream)
{
    return stream->last_acknown;
}

bool uxr_next_reliable_nack_buffer_to_send(
        uxrOutputReliableStream* stream,
        uint8_t** buffer,
        size_t* length,
        uxrSeqNum* seq_num_it)
{
    bool it_updated = false;
    if (stream->send_lost)
    {
        bool check_next_buffer = true;
        while (check_next_buffer && !it_updated)
        {
            *seq_num_it = uxr_seq_num_add(*seq_num_it, 1);
            check_next_buffer = 0 >= uxr_seq_num_cmp(*seq_num_it, stream->last_sent);
            if (check_next_buffer)
            {
                *buffer = uxr_get_reliable_buffer(&stream->base, *seq_num_it);
                *length = uxr_get_reliable_buffer_size(&stream->base, *seq_num_it);
                it_updated = *length != stream->offset;
            }
        }

        if (!it_updated)
        {
            stream->send_lost = false;
        }
    }

    return it_updated;
}

void uxr_process_acknack(
        uxrOutputReliableStream* stream,
        uint16_t bitmap,
        uxrSeqNum first_unacked_seq_num)
{
    uxrSeqNum last_acked_seq_num = uxr_seq_num_sub(first_unacked_seq_num, 1);
    size_t buffers_to_clean = uxr_seq_num_sub(last_acked_seq_num, stream->last_acknown);
    for (size_t i = 0; i < buffers_to_clean; i++)
    {
        stream->last_acknown = uxr_seq_num_add(stream->last_acknown, 1);
        uxr_set_reliable_buffer_size(&stream->base, stream->last_acknown, stream->offset);
    }

    stream->send_lost = (0 < bitmap);

    /* reset heartbeat interval */
    stream->next_heartbeat_tries = 0;
}

bool uxr_is_output_up_to_date(
        const uxrOutputReliableStream* stream)
{
    return 0 == uxr_seq_num_cmp(stream->last_acknown, stream->last_sent);
}

//==================================================================
//                             PRIVATE
//==================================================================
bool on_full_output_buffer(
        ucdrBuffer* ub,
        void* args)
{
    uxrOutputReliableStream* stream = (uxrOutputReliableStream*) args;

    uint16_t history_position = (uint16_t)(1 + uxr_get_reliable_buffer_history_position(&stream->base, ub->init));
    uint8_t* buffer = uxr_get_reliable_buffer(&stream->base, history_position);
    size_t buffer_size = uxr_get_reliable_buffer_size(&stream->base, history_position);

    ucdr_init_buffer_origin(
        ub,
        buffer + stream->offset + SUBHEADER_SIZE,
        (uint32_t)(buffer_size - stream->offset - SUBHEADER_SIZE),
        ub->offset);
    ucdr_set_on_full_buffer_callback(ub, on_full_output_buffer, stream);

    return false;
}

uint16_t get_available_free_slots(
        uxrOutputReliableStream* stream)
{
    uint16_t free_slots = 0;
    for (uint16_t i = 0; i < stream->base.history; i++)
    {
        if (uxr_get_reliable_buffer_size(&stream->base, i) == stream->offset)
        {
            free_slots++;
        }
    }
    return free_slots;
}
