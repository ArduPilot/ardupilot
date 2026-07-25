#include "stream_storage_internal.h"
#include "input_best_effort_stream_internal.h"
#include "input_reliable_stream_internal.h"
#include "output_best_effort_stream_internal.h"
#include "output_reliable_stream_internal.h"
#include <uxr/client/profile/multithread/multithread.h>

//==================================================================
//                             PUBLIC
//==================================================================
void uxr_init_stream_storage(
        uxrStreamStorage* storage)
{
    storage->output_best_effort_size = 0;
    storage->output_reliable_size = 0;
    storage->input_best_effort_size = 0;
    storage->input_reliable_size = 0;
}

void uxr_reset_stream_storage(
        uxrStreamStorage* storage)
{
    for (unsigned i = 0; i < storage->output_best_effort_size; ++i)
    {
        uxr_reset_output_best_effort_stream(&storage->output_best_effort[i]);
    }

    for (unsigned i = 0; i < storage->input_best_effort_size; ++i)
    {
        uxr_reset_input_best_effort_stream(&storage->input_best_effort[i]);
    }

    for (unsigned i = 0; i < storage->output_reliable_size; ++i)
    {
        uxr_reset_output_reliable_stream(&storage->output_reliable[i]);
    }

    for (unsigned i = 0; i < storage->input_reliable_size; ++i)
    {
        uxr_reset_input_reliable_stream(&storage->input_reliable[i]);
    }
}

uxrStreamId uxr_add_output_best_effort_buffer(
        uxrStreamStorage* storage,
        uint8_t* buffer,
        size_t size,
        uint8_t header_offset)
{
    uint8_t index = storage->output_best_effort_size++;
    //TODO: assert for index
    uxrOutputBestEffortStream* stream = &storage->output_best_effort[index];
    uxr_init_output_best_effort_stream(stream, buffer, size, header_offset);
    return uxr_stream_id(index, UXR_BEST_EFFORT_STREAM, UXR_OUTPUT_STREAM);
}

uxrStreamId uxr_add_output_reliable_buffer(
        uxrStreamStorage* storage,
        uint8_t* buffer,
        size_t size,
        uint16_t history,
        uint8_t header_offset)
{
    uint8_t index = storage->output_reliable_size++;
    //TODO: assert for index
    uxrOutputReliableStream* stream = &storage->output_reliable[index];
    uxr_init_output_reliable_stream(stream, buffer, size, history, header_offset);
    return uxr_stream_id(index, UXR_RELIABLE_STREAM, UXR_OUTPUT_STREAM);
}

uxrStreamId uxr_add_input_best_effort_buffer(
        uxrStreamStorage* storage)
{
    uint8_t index = storage->input_best_effort_size++;
    //TODO: assert for index
    uxrInputBestEffortStream* stream = &storage->input_best_effort[index];
    uxr_init_input_best_effort_stream(stream);
    return uxr_stream_id(index, UXR_BEST_EFFORT_STREAM, UXR_INPUT_STREAM);
}

uxrStreamId uxr_add_input_reliable_buffer(
        uxrStreamStorage* storage,
        uint8_t* buffer,
        size_t size,
        uint16_t history,
        OnGetFragmentationInfo on_get_fragmentation_info)
{
    uint8_t index = storage->input_reliable_size++;
    //TODO: assert for index
    uxrInputReliableStream* stream = &storage->input_reliable[index];
    uxr_init_input_reliable_stream(stream, buffer, size, history, on_get_fragmentation_info);
    return uxr_stream_id(index, UXR_RELIABLE_STREAM, UXR_INPUT_STREAM);
}

uxrOutputBestEffortStream* uxr_get_output_best_effort_stream(
        uxrStreamStorage* storage,
        uint8_t index)
{
    if (index < storage->output_best_effort_size)
    {
        return &storage->output_best_effort[index];
    }
    return NULL;
}

uxrOutputReliableStream* uxr_get_output_reliable_stream(
        uxrStreamStorage* storage,
        uint8_t index)
{
    if (index < storage->output_reliable_size)
    {
        return &storage->output_reliable[index];
    }
    return NULL;
}

uxrInputBestEffortStream* uxr_get_input_best_effort_stream(
        uxrStreamStorage* storage,
        uint8_t index)
{
    if (index < storage->input_best_effort_size)
    {
        return &storage->input_best_effort[index];
    }
    return NULL;
}

uxrInputReliableStream* uxr_get_input_reliable_stream(
        uxrStreamStorage* storage,
        uint8_t index)
{
    if (index < storage->input_reliable_size)
    {
        return &storage->input_reliable[index];
    }
    return NULL;
}

bool uxr_output_streams_confirmed(
        const uxrStreamStorage* storage)
{
    bool up_to_date = true;
    for (unsigned i = 0; i < storage->output_reliable_size && up_to_date; ++i)
    {
        UXR_LOCK((uxrMutex*) &storage->output_reliable[i].mutex);
        up_to_date = uxr_is_output_up_to_date(&storage->output_reliable[i]);
        UXR_UNLOCK((uxrMutex*) &storage->output_reliable[i].mutex);
    }
    return up_to_date;
}
