#include <ucdr/microcdr.h>

#include "./seq_num_internal.h"
#include "./input_reliable_stream_internal.h"
#include "./common_reliable_stream_internal.h"
#include "../submessage_internal.h"
#include <uxr/client/profile/multithread/multithread.h>

#include <string.h>

static bool check_last_fragment(
        uxrInputReliableStream* stream,
        uxrSeqNum* last);
static uxrSeqNum uxr_get_first_unacked(
        const uxrInputReliableStream* stream);
static bool on_full_input_buffer(
        ucdrBuffer* ub,
        void* args);

//==================================================================
//                             PUBLIC
//==================================================================
void uxr_init_input_reliable_stream(
        uxrInputReliableStream* stream,
        uint8_t* buffer,
        size_t size,
        uint16_t history,
        OnGetFragmentationInfo on_get_fragmentation_info)
{
    // assert for history (must be 2^)
    stream->base.buffer = buffer;
    stream->base.size = size;
    stream->base.history = history;
    stream->on_get_fragmentation_info = on_get_fragmentation_info;
    stream->cleanup_flag = false;

    UXR_INIT_LOCK(&stream->mutex);

    uxr_reset_input_reliable_stream(stream);
}

void uxr_reset_input_reliable_stream(
        uxrInputReliableStream* stream)
{
    for (uint16_t i = 0; i < stream->base.history; ++i)
    {
        uxr_set_reliable_buffer_size(&stream->base, i, 0);
    }

    stream->last_handled = SEQ_NUM_MAX;
    stream->last_announced = SEQ_NUM_MAX;
}

bool uxr_receive_reliable_message(
        uxrInputReliableStream* stream,
        uint16_t seq_num,
        uint8_t* buffer,
        size_t length,
        bool* message_stored)
{
    bool ready_to_read = false;

    /* Check if the seq_num is valid for the stream state */
    uxrSeqNum last_history = uxr_seq_num_add(stream->last_handled, stream->base.history);
    if (0 > uxr_seq_num_cmp(stream->last_handled, seq_num) && 0 <= uxr_seq_num_cmp(last_history, seq_num))
    {
        /* Process the message */
        FragmentationInfo fragmentation_info = stream->on_get_fragmentation_info(buffer);
        uxrSeqNum next = uxr_seq_num_add(stream->last_handled, 1);

        if ((NO_FRAGMENTED == fragmentation_info) && (seq_num == next))
        {
            stream->last_handled = next;
            ready_to_read = true;
            *message_stored = false;
        }
        else
        {
            /* Check if the message received is not already received */
            uint8_t* internal_buffer = uxr_get_reliable_buffer(&stream->base, seq_num);
            if (0 == uxr_get_reliable_buffer_size(&stream->base, seq_num))
            {
                memcpy(internal_buffer, buffer, length);
                uxr_set_reliable_buffer_size(&stream->base, seq_num, length);
                *message_stored = true;

                if (NO_FRAGMENTED != fragmentation_info)
                {
                    uxrSeqNum last;
                    if (check_last_fragment(stream, &last))
                    {
                        ready_to_read = true;
                    }
                }
            }
        }
    }

    if (0 > uxr_seq_num_cmp(stream->last_announced, seq_num))
    {
        stream->last_announced = seq_num;
    }

    return ready_to_read;
}

bool uxr_next_input_reliable_buffer_available(
        uxrInputReliableStream* stream,
        ucdrBuffer* ub,
        size_t fragment_offset)
{
    uxrSeqNum next = uxr_seq_num_add(stream->last_handled, 1);
    uint8_t* internal_buffer = uxr_get_reliable_buffer(&stream->base, next);
    size_t length = uxr_get_reliable_buffer_size(&stream->base, next);
    bool available_to_read = (0 != length);
    if (available_to_read)
    {
        FragmentationInfo fragmentation_info = stream->on_get_fragmentation_info(internal_buffer);
        if (NO_FRAGMENTED == fragmentation_info)
        {
            ucdr_init_buffer(ub, internal_buffer, (uint32_t)length);
            uxr_set_reliable_buffer_size(&stream->base, next, 0);
            stream->last_handled = next;
        }
        else
        {
            uxrSeqNum last;
            available_to_read = check_last_fragment(stream, &last);
            if (available_to_read)
            {
                uxr_set_reliable_buffer_size(&stream->base, next, 0);
                ucdr_init_buffer(ub, internal_buffer + fragment_offset, (uint32_t)(length - fragment_offset));
                ucdr_set_on_full_buffer_callback(ub, on_full_input_buffer, stream);
                stream->last_handled = last;
            }
        }
    }

    return available_to_read;
}

void uxr_process_heartbeat(
        uxrInputReliableStream* stream,
        uxrSeqNum first_seq_num,
        uxrSeqNum last_seq_num)
{
    (void)first_seq_num;
    //TODO: Checks the first_seq_num to avoid hacks.

    if (0 > uxr_seq_num_cmp(stream->last_announced, last_seq_num))
    {
        stream->last_announced = last_seq_num;
    }
}

bool uxr_is_input_up_to_date(
        const uxrInputReliableStream* stream)
{
    return stream->last_announced == stream->last_handled;
}

uint16_t uxr_compute_acknack(
        const uxrInputReliableStream* stream,
        uxrSeqNum* from)
{
    *from = uxr_get_first_unacked(stream);
    uint16_t buffers_to_ack = uxr_seq_num_sub(stream->last_announced, uxr_seq_num_sub(*from, 1));
    uint16_t nack_bitmap = 0;

    for (size_t i = 0; i < buffers_to_ack; ++i)
    {
        uxrSeqNum seq_num = uxr_seq_num_add(*from, (uxrSeqNum)i);
        if (0 == uxr_get_reliable_buffer_size(&stream->base, seq_num))
        {
            nack_bitmap = (uint16_t)(nack_bitmap | (1 << i));
        }
    }

    return nack_bitmap;
}

//==================================================================
//                             PRIVATE
//==================================================================
bool check_last_fragment(
        uxrInputReliableStream* stream,
        uxrSeqNum* last_fragment)
{
    uxrSeqNum next = stream->last_handled;
    bool more_messages;
    bool found = false;
    do
    {
        next = uxr_seq_num_add(next, 1);
        uint8_t* next_buffer = uxr_get_reliable_buffer(&stream->base, next);
        more_messages = (0 != uxr_get_reliable_buffer_size(&stream->base, next));
        if (more_messages)
        {
            FragmentationInfo next_fragmentation_info = stream->on_get_fragmentation_info(next_buffer);
            more_messages = INTERMEDIATE_FRAGMENT == next_fragmentation_info;
            if (LAST_FRAGMENT == next_fragmentation_info)
            {
                found = true;
                break;
            }
        }
    }
    while (more_messages);

    *last_fragment = next;
    return found;
}

uxrSeqNum uxr_get_first_unacked(
        const uxrInputReliableStream* stream)
{
    uxrSeqNum first_unknown = stream->last_handled;
    for (size_t i = 0; i < stream->base.history; ++i)
    {
        uxrSeqNum seq_num = uxr_seq_num_add(stream->last_handled, (uint16_t)(i + 1));
        if (0 == uxr_get_reliable_buffer_size(&stream->base, seq_num))
        {
            first_unknown = seq_num;
            break;
        }
    }

    return first_unknown;
}

bool on_full_input_buffer(
        ucdrBuffer* ub,
        void* args)
{
    uxrInputReliableStream* stream = (uxrInputReliableStream*) args;

    uint16_t history_position = (uint16_t)(1 + uxr_get_reliable_buffer_history_position(&stream->base, ub->init));
    uint8_t* buffer = uxr_get_reliable_buffer(&stream->base, history_position);
    size_t buffer_size = uxr_get_reliable_buffer_size(&stream->base, history_position);

    if (stream->cleanup_flag)
    {
        uxr_set_reliable_buffer_size(&stream->base, history_position, 0);
    }

    // IMPORTANT: This situation only happens when stream->base.history is not power of two.
    if (buffer_size < SUBHEADER_SIZE)
    {
        return true;
    }

    ucdr_init_buffer_origin(
        ub,
        buffer + SUBHEADER_SIZE,
        (uint32_t)(buffer_size - SUBHEADER_SIZE),
        ub->offset);
    ucdr_set_on_full_buffer_callback(ub, on_full_input_buffer, stream);

    return false;
}
