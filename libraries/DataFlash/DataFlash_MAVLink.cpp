/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash Remote(via MAVLink) logging
*/

#include "DataFlash_MAVLink.h"

#if DATAFLASH_MAVLINK_SUPPORT

#include "LogStructure.h"

#define REMOTE_LOG_DEBUGGING 0

#if REMOTE_LOG_DEBUGGING
#include <stdio.h>
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;


// initialisation
void DataFlash_MAVLink::Init()
{
    DataFlash_Backend::Init();

    _blocks = NULL;
    while (_blockcount >= 8) { // 8 is a *magic* number
        _blocks = (struct dm_block *) malloc(_blockcount * sizeof(_blocks[0]));
        if (_blocks != NULL) {
            break;
        }
        _blockcount /= 2;
    }

    if (_blocks == NULL) {
        return;
    }

    free_all_blocks();
    stats_init();

    _initialised = true;
    _logging_started = true; // in actual fact, we throw away
                             // everything until a client connects.
                             // This stops calls to start_new_log from
                             // the vehicles
}

uint16_t DataFlash_MAVLink::bufferspace_available() {
    return (_blockcount_free * 200 + remaining_space_in_current_block());
}

uint8_t DataFlash_MAVLink::remaining_space_in_current_block() {
    // note that _current_block *could* be NULL ATM.
    return (MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN - _latest_block_len);
}

void DataFlash_MAVLink::enqueue_block(dm_block_queue_t &queue, struct dm_block *block)
{
    if (queue.youngest != NULL) {
        queue.youngest->next = block;
    } else {
        queue.oldest = block;
    }
    queue.youngest = block;
}

struct DataFlash_MAVLink::dm_block *DataFlash_MAVLink::dequeue_seqno(DataFlash_MAVLink::dm_block_queue_t &queue, uint32_t seqno)
{
    struct dm_block *prev = NULL;
    for (struct dm_block *block=queue.oldest; block != NULL; block=block->next) {
        if (block->seqno == seqno) {
            if (prev == NULL) {
                if (queue.youngest == queue.oldest) {
                    queue.oldest = NULL;
                    queue.youngest = NULL;
                } else {
                    queue.oldest = block->next;
                }
            } else {
                if (queue.youngest == block) {
                    queue.youngest = prev;
                }
                prev->next = block->next;
            }
            block->next = NULL;
            return block;
        }
        prev = block;
    }
    return NULL;
}

bool DataFlash_MAVLink::free_seqno_from_queue(uint32_t seqno, dm_block_queue_t &queue)
{
    struct dm_block *block = dequeue_seqno(queue, seqno);
    if (block != NULL) {
        block->next = _blocks_free;
        _blocks_free = block;
        _blockcount_free++; // comment me out to expose a bug!
        return true;
    }
    return false;
}
    
/* Write a block of data at current offset */

// DM_write: 70734 events, 0 overruns, 167806us elapsed, 2us avg, min 1us max 34us 0.620us rms
bool DataFlash_MAVLink::WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    if (!_initialised || !_sending_to_client || !_writes_enabled) {
        return false;
    }

    if (! WriteBlockCheckStartupMessages()) {
        return false;
    }

    if (bufferspace_available() < size) {
        if (_startup_messagewriter->finished()) {
            // do not count the startup packets as being dropped...
            dropped++;
        }
        return false;
    }

    uint16_t copied = 0;

    while (copied < size) {
        if (_current_block == NULL) {
            _current_block = next_block();
            if (_current_block == NULL) {
                // should not happen - there's a sanity check above
                internal_error();
                return false;
            }
        }
        uint16_t remaining_to_copy = size - copied;
        uint16_t _curr_remaining = remaining_space_in_current_block();
        uint16_t to_copy = (remaining_to_copy > _curr_remaining) ? _curr_remaining : remaining_to_copy;
        memcpy(&(_current_block->buf[_latest_block_len]), &((const uint8_t *)pBuffer)[copied], to_copy);
        copied += to_copy;
        _latest_block_len += to_copy;
        if (_latest_block_len == MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN) {
            //block full, mark it to be sent:
            enqueue_block(_blocks_pending, _current_block);
            _current_block = next_block();
        }
    }

    if (!_writing_startup_messages) {
        // push_log_blocks();
    }

    return true;
}

//Get a free block
struct DataFlash_MAVLink::dm_block *DataFlash_MAVLink::next_block()
{
    DataFlash_MAVLink::dm_block *ret = _blocks_free;
    if (ret != NULL) {
        _blocks_free = ret->next;
        _blockcount_free--;
        ret->seqno = _next_seq_num++;
        ret->last_sent = 0;
        ret->next = NULL;
        _latest_block_len = 0;
    }
    return ret;
}

void DataFlash_MAVLink::free_all_blocks()
{
    _blocks_free = NULL;
    _current_block = NULL;

    _blocks_pending.sent_count = 0;
    _blocks_pending.oldest = _blocks_pending.youngest = NULL;
    _blocks_retry.sent_count = 0;
    _blocks_retry.oldest = _blocks_retry.youngest = NULL;
    _blocks_sent.sent_count = 0;
    _blocks_sent.oldest = _blocks_sent.youngest = NULL;

    // add blocks to the free stack:
    for(uint8_t i=0; i < _blockcount; i++) {
        _blocks[i].next = _blocks_free;
        _blocks_free = &_blocks[i];
        // this value doesn't really matter, but it stops valgrind
        // complaining when acking blocks (we check seqno before
        // state).  Also, when we receive ACKs we check seqno, and we
        // want to ack the *real* block zero!
        _blocks[i].seqno = 9876543;
    }
    _blockcount_free = _blockcount;

    _latest_block_len = 0;
}

void DataFlash_MAVLink::handle_ack(mavlink_channel_t chan,
                                   mavlink_message_t* msg,
                                   uint32_t seqno)
{
    if (!_initialised) {
        return;
    }
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_STOP) {
        Debug("Received stop-logging packet");
        if (_sending_to_client) {
            _sending_to_client = false;
            _last_response_time = AP_HAL::millis();
        }
        return;
    }
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_START) {
        if (!_sending_to_client) {
            Debug("Starting New Log");
            free_all_blocks();
            // _current_block = next_block();
            // if (_current_block == NULL) {
            //     Debug("No free blocks?!!!\n");
            //     return;
            // }
            stats_init();
            _sending_to_client = true;
            _target_system_id = msg->sysid;
            _target_component_id = msg->compid;
            _chan = chan;
            _next_seq_num = 0;
            _startup_messagewriter->reset();
            _last_response_time = AP_HAL::millis();
            Debug("Target: (%u/%u)", _target_system_id, _target_component_id);
        }
        return;
    }

    // check SENT blocks (VERY likely to be first on the list):
    if (free_seqno_from_queue(seqno, _blocks_sent)) {
        // celebrate
        _last_response_time = AP_HAL::millis();
    } else if(free_seqno_from_queue(seqno, _blocks_retry)) {
        // party
        _last_response_time = AP_HAL::millis();
    } else {
        // probably acked already and put on the free list.
    }
}

void DataFlash_MAVLink::remote_log_block_status_msg(mavlink_channel_t chan,
                                                    mavlink_message_t* msg)
{
    mavlink_remote_log_block_status_t packet;
    mavlink_msg_remote_log_block_status_decode(msg, &packet);
    if(packet.status == 0){
        handle_retry(packet.seqno);
    } else{
        handle_ack(chan, msg, packet.seqno);
    }
}

void DataFlash_MAVLink::handle_retry(uint32_t seqno)
{
    if (!_initialised || !_sending_to_client) {
        return;
    }

    struct dm_block *victim = dequeue_seqno(_blocks_sent, seqno);
    if (victim != NULL) {
        _last_response_time = AP_HAL::millis();
        enqueue_block(_blocks_retry, victim);
    }
}

void DataFlash_MAVLink::set_channel(mavlink_channel_t chan)
{
    _chan = chan;
}

void DataFlash_MAVLink::internal_error() {
    internal_errors++;
    DataFlash_Backend::internal_error();
}
void DataFlash_MAVLink::stats_init() {
    dropped = 0;
    internal_errors = 0;
    stats.resends = 0;
    stats_reset();
}
void DataFlash_MAVLink::stats_reset() {
    stats.state_free = 0;
    stats.state_free_min = -1; // unsigned wrap
    stats.state_free_max = 0;
    stats.state_pending = 0;
    stats.state_pending_min = -1; // unsigned wrap
    stats.state_pending_max = 0;
    stats.state_retry = 0;
    stats.state_retry_min = -1; // unsigned wrap
    stats.state_retry_max = 0;
    stats.state_sent = 0;
    stats.state_sent_min = -1; // unsigned wrap
    stats.state_sent_max = 0;
    stats.collection_count = 0;
}

void DataFlash_MAVLink::Log_Write_DF_MAV(DataFlash_MAVLink &df)
{
    if (df.stats.collection_count == 0) {
        return;
    }
    struct log_DF_MAV_Stats pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DF_MAV_STATS),
        timestamp         : AP_HAL::millis(),
        seqno             : df._next_seq_num-1,
        dropped           : df.dropped,
        retries           : df._blocks_retry.sent_count,
        resends           : df.stats.resends,
        internal_errors   : df.internal_errors,
        state_free_avg    : (uint8_t)(df.stats.state_free/df.stats.collection_count),
        state_free_min    : df.stats.state_free_min,
        state_free_max    : df.stats.state_free_max,
        state_pending_avg : (uint8_t)(df.stats.state_pending/df.stats.collection_count),
        state_pending_min : df.stats.state_pending_min,
        state_pending_max : df.stats.state_pending_max,
        state_sent_avg    : (uint8_t)(df.stats.state_sent/df.stats.collection_count),
        state_sent_min    : df.stats.state_sent_min,
        state_sent_max    : df.stats.state_sent_max,
        // state_retry_avg   : (uint8_t)(df.stats.state_retry/df.stats.collection_count),
        // state_retry_min    : df.stats.state_retry_min,
        // state_retry_max    : df.stats.state_retry_max
    };
    WriteBlock(&pkt,sizeof(pkt));
}

void DataFlash_MAVLink::stats_log()
{
    if (!_initialised || !_logging_started) {
        return;
    }
    if (stats.collection_count == 0) {
        return;
    }
    Log_Write_DF_MAV(*this);
#if REMOTE_LOG_DEBUGGING
    printf("D:%d Retry:%d Resent:%d E:%d SF:%d/%d/%d SP:%d/%d/%d SS:%d/%d/%d SR:%d/%d/%d\n",
           dropped,
           _blocks_retry.sent_count,
           stats.resends,
           internal_errors,
           stats.state_free_min,
           stats.state_free_max,
           stats.state_free/stats.collection_count,
           stats.state_pending_min,
           stats.state_pending_max,
           stats.state_pending/stats.collection_count,
           stats.state_sent_min,
           stats.state_sent_max,
           stats.state_sent/stats.collection_count,
           stats.state_retry_min,
           stats.state_retry_max,
           stats.state_retry/stats.collection_count
        );
#endif
    stats_reset();
}

uint8_t DataFlash_MAVLink::stack_size(struct dm_block *stack)
{
    uint8_t ret = 0;
    for (struct dm_block *block=stack; block != NULL; block=block->next) {
        ret++;
    }
    return ret;
}
uint8_t DataFlash_MAVLink::queue_size(dm_block_queue_t queue)
{
    return stack_size(queue.oldest);
}

void DataFlash_MAVLink::stats_collect()
{
    if (!_initialised || !_logging_started) {
        return;
    }
    uint8_t pending = queue_size(_blocks_pending);
    uint8_t sent = queue_size(_blocks_sent);
    uint8_t retry = queue_size(_blocks_retry);
    uint8_t sfree = stack_size(_blocks_free);

    if (sfree != _blockcount_free) {
        internal_error();
    }
    stats.state_pending += pending;
    stats.state_sent += sent;
    stats.state_free += sfree;
    stats.state_retry += retry;

    if (pending < stats.state_pending_min) {
        stats.state_pending_min = pending;
    }
    if (pending > stats.state_pending_max) {
        stats.state_pending_max = pending;
    }
    if (retry < stats.state_retry_min) {
        stats.state_retry_min = retry;
    }
    if (retry > stats.state_retry_max) {
        stats.state_retry_max = retry;
    }
    if (sent < stats.state_sent_min) {
        stats.state_sent_min = sent;
    }
    if (sent > stats.state_sent_max) {
        stats.state_sent_max = sent;
    }
    if (sfree < stats.state_free_min) {
        stats.state_free_min = sfree;
    }
    if (sfree > stats.state_free_max) {
        stats.state_free_max = sfree;
    }
    
    stats.collection_count++;
}

/* while we "successfully" send log blocks from a queue, move them to
 * the sent list. DO NOT call this for blocks already sent!
*/
bool DataFlash_MAVLink::send_log_blocks_from_queue(dm_block_queue_t &queue)
{
    uint8_t sent_count = 0;
    while (queue.oldest != NULL) {
        if (sent_count++ > _max_blocks_per_send_blocks) {
            return false;
        }
        if (! send_log_block(*queue.oldest)) {
            return false;
        }
        queue.sent_count++;
        struct DataFlash_MAVLink::dm_block *tmp = dequeue_seqno(queue,queue.oldest->seqno);
        if (tmp != NULL) { // should never be NULL
            enqueue_block(_blocks_sent, tmp);
        } else {
            internal_error();
        }
    }
    return true;
}

void DataFlash_MAVLink::push_log_blocks()
{
    if (!_initialised || !_logging_started ||!_sending_to_client) {
        return;
    }

    DataFlash_Backend::WriteMoreStartupMessages();

    if (! send_log_blocks_from_queue(_blocks_retry)) {
        return;
    }

    if (! send_log_blocks_from_queue(_blocks_pending)) {
        return;
    }
}

void DataFlash_MAVLink::do_resends(uint32_t now)
{
    if (!_initialised || !_logging_started ||!_sending_to_client) {
        return;
    }

    uint8_t count_to_send = 5;
    if (_blockcount < count_to_send) {
        count_to_send = _blockcount;
    }
    uint32_t oldest = now - 100; // 100 milliseconds before resend.  Hmm.
    while (count_to_send-- > 0) {
        for (struct dm_block *block=_blocks_sent.oldest; block != NULL; block=block->next) {
            // only want to send blocks every now-and-then:
            if (block->last_sent < oldest) {
                if (! send_log_block(*block)) {
                    // failed to send the block; try again later....
                    return;
                }
                stats.resends++;
            }
        }
    }
}

void DataFlash_MAVLink::periodic_10Hz(const uint32_t now)
{
    do_resends(now);
    stats_collect();
}
void DataFlash_MAVLink::periodic_1Hz(const uint32_t now)
{
    if (_sending_to_client &&
        _last_response_time + 10000 < _last_send_time) {
        // other end appears to have timed out!
        Debug("Client timed out");
        _sending_to_client = false;
        return;
    }
    stats_log();
}

void DataFlash_MAVLink::periodic_fullrate(uint32_t now)
{
    push_log_blocks();
}

//TODO: handle full txspace properly
bool DataFlash_MAVLink::send_log_block(struct dm_block &block)
{
    mavlink_channel_t chan = mavlink_channel_t(_chan - MAVLINK_COMM_0);
    if (!_initialised) {
       return false;
    }
    if (comm_get_txspace(chan) < MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK_LEN) {
        return false;
    }
    if (comm_get_txspace(chan) < 500) {
        return false;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (rand() < 0.1) {
        return false;
    }
#endif
    
#if DF_MAVLINK_DISABLE_INTERRUPTS
    irqstate_t istate = irqsave();
#endif

// DM_packing: 267039 events, 0 overruns, 8440834us elapsed, 31us avg, min 31us max 32us 0.488us rms
    hal.util->perf_begin(_perf_packing);

    mavlink_message_t msg;
    mavlink_status_t *chan_status = mavlink_get_channel_status(chan);
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink_seq++;
    // Debug("Sending block (%d)", block.seqno);
    mavlink_msg_remote_log_data_block_pack(mavlink_system.sysid,
                                           MAV_COMP_ID_LOG,
                                           &msg,
                                           _target_system_id,
                                           _target_component_id,
                                           block.seqno,
                                           block.buf);

    hal.util->perf_end(_perf_packing);

#if DF_MAVLINK_DISABLE_INTERRUPTS
    irqrestore(istate);
#endif

    block.last_sent = AP_HAL::millis();
    chan_status->current_tx_seq = saved_seq;

    // _last_send_time is set even if we fail to send the packet; if
    // the txspace is repeatedly chockas we should not add to the
    // problem and stop attempting to log
    _last_send_time = AP_HAL::millis();

    _mavlink_resend_uart(chan, &msg);

    return true;
}
#endif
