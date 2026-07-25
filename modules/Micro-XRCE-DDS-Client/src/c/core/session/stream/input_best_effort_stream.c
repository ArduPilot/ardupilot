#include "input_best_effort_stream_internal.h"
#include "seq_num_internal.h"
#include <uxr/client/profile/multithread/multithread.h>

//==================================================================
//                             PUBLIC
//==================================================================
void uxr_init_input_best_effort_stream(
        uxrInputBestEffortStream* stream)
{
    stream->last_handled = SEQ_NUM_MAX;

    UXR_INIT_LOCK(&stream->mutex);
}

void uxr_reset_input_best_effort_stream(
        uxrInputBestEffortStream* stream)
{
    stream->last_handled = SEQ_NUM_MAX;
}

bool uxr_receive_best_effort_message(
        uxrInputBestEffortStream* stream,
        uxrSeqNum seq_num)
{
    bool available_to_read = (0 > uxr_seq_num_cmp(stream->last_handled, seq_num));
    if (available_to_read)
    {
        stream->last_handled = seq_num;
    }

    return available_to_read;
}
