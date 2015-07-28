/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash Remote(via MAVLink) logging
*/

#include <AP_HAL.h>

#ifdef HAL_BOARD_REMOTE_LOG_PORT
#include "DataFlash_MAVLink.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <assert.h>
#include <AP_Math.h>
#include <stdio.h>
#include <time.h>
#include "../AP_HAL/utility/RingBuffer.h"

#define REMOTE_LOG_DEBUGGING 0

#if REMOTE_LOG_DEBUGGING
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;
#ifndef MAV_SYS_ID_LOG
#define MAV_SYS_ID_LOG  1
#endif
#ifndef MAV_COMP_ID_LOG
#define MAV_COMP_ID_LOG 50
#endif
/*
  constructor
 */
DataFlash_MAVLink::DataFlash_MAVLink(DataFlash_Class &front, mavlink_channel_t chan) :
    DataFlash_Backend(front),
    _chan(chan),
    _initialised(false),
    _next_seq_num(0),
    _current_block(NULL),
    _latest_block_len(0),
    _next_block_number_to_resend(0),
    _last_response_time(0),
    _blockcount(32), // this may get reduced in Init if allocation fails
    _blockcount_free(0),
    _logging_started(false),
    _sending_to_client(false),
    _pushing_blocks(false)
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    ,_perf_errors(perf_alloc(PC_COUNT, "DF_errors")),
    _perf_overruns(perf_alloc(PC_COUNT, "DF_overruns"))
#endif
{
    memset(&mavlink, 0, sizeof(mavlink));
}


// initialisation
void DataFlash_MAVLink::Init(const struct LogStructure *structure, uint8_t num_types)
{
    DataFlash_Backend::Init(structure, num_types);

    mavlink.system_id = MAV_SYS_ID_LOG;
    mavlink.component_id = MAV_COMP_ID_LOG;

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
/* Write a block of data at current offset */
bool DataFlash_MAVLink::WriteBlock(const void *pBuffer, uint16_t size)
{   
    if (!_initialised || !_sending_to_client || !_writes_enabled) {
        return false;
    }

    if (_front._startup_messagewriter != NULL && !_front._startup_messagewriter->finished()) {
        // the block writer is running ATM.
        if (!_pushing_blocks) {
            // we didn't come from push_log_blocks, so this is some random
            // caller hoping to write blocks out
            push_log_blocks();
            if (!_front._startup_messagewriter->finished()) {
                // sorry!  currently busy writing out startup messages...
                stats.dropped++;
                return false;
            }
        }
    }
    if (bufferspace_available() < size) {
        if (_front._startup_messagewriter->finished()) {
            // do not count the startup packets as being dropped...
            stats.dropped++;
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
            _current_block->state = BLOCK_STATE_SEND_PENDING;
            _current_block = next_block();
        }
    }

    if (!_pushing_blocks) {
        push_log_blocks();
    }
    return true;
}

//Get a free block
struct DataFlash_MAVLink::dm_block *DataFlash_MAVLink::next_block()
{
    for(uint8_t i = 0; i < _blockcount; i++){
        if (_blocks[i].state == BLOCK_STATE_FREE) {
            _blocks[i].state = BLOCK_STATE_FILLING;
            _blockcount_free--;
            _blocks[i].seqno = _next_seq_num++;
            _blocks[i].last_sent = 0;
            _latest_block_len = 0;
            return &_blocks[i];
        }
    }
    return NULL;
}

void DataFlash_MAVLink::free_all_blocks()
{
    for(uint8_t i=0; i < _blockcount; i++) {
        _blocks[i].state = BLOCK_STATE_FREE;
        // this value doesn't really matter, but it stops valgrind
        // complaining when acking blocks (we check seqno before
        // state).  Also, when we receive ACKs we check seqno, and we
        // want to ack the *real* block zero!
        _blocks[i].seqno = 9876543;
    }
    _blockcount_free = _blockcount;
    _current_block = NULL;
    _latest_block_len = 0;
}

void DataFlash_MAVLink::handle_ack(mavlink_message_t* msg, const uint32_t seqno)
{
    if (!_initialised) {
        return;
    }
    _last_response_time = hal.scheduler->millis();
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_STOP) {
        Debug("Received stop-logging packet\n");
        _sending_to_client = false;
        return;
    }
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_START && !_sending_to_client) {
        Debug("\nStarting New Log!!\n");
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
        _next_seq_num = 0;
        _front.StartNewLog();
        return;
    }
    for(uint8_t block = 0; block < _blockcount; block++){
        if(_blocks[block].seqno == seqno) {
            if (_blocks[block].state != BLOCK_STATE_FREE) {
                _blocks[block].state = BLOCK_STATE_FREE;
                _blockcount_free++;
            }
            return;
        }
    }
}

void DataFlash_MAVLink::remote_log_block_status_msg(mavlink_message_t* msg)
{
    mavlink_remote_log_block_status_t packet;
    mavlink_msg_remote_log_block_status_decode(msg, &packet);
    if(packet.status == 0){
        handle_retry(packet.seqno);
    } else{
        handle_ack(msg, packet.seqno);
    }
}

void DataFlash_MAVLink::handle_retry(uint32_t seqno)
{
    if (!_initialised || !_sending_to_client) {
        return;
    }
    _last_response_time = hal.scheduler->millis();
    for(uint8_t block = 0; block < _blockcount; block++){
        if(_blocks[block].seqno == seqno) {
            // we do not reset the block's sequence number when
            // freeing it; what would we set it to?  If we get a retry
            // for a block already freed, we do not want to free it
            // again...
            if (_blocks[block].state != BLOCK_STATE_FREE) {
                _blocks[block].state = BLOCK_STATE_SEND_RETRY;
            }
            return;
        }
    }
}

void DataFlash_MAVLink::set_channel(mavlink_channel_t chan)
{
    _chan = chan;
}

void DataFlash_MAVLink::internal_error() {
    stats.internal_errors++;
    DataFlash_Backend::internal_error();
}
void DataFlash_MAVLink::stats_init() {
    stats.dropped = 0;
    stats.internal_errors = 0;
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

void DataFlash_MAVLink::stats_log()
{
    if (!_initialised || !_logging_started) {
        return;
    }
    if (stats.collection_count == 0) {
        return;
    }
    _front.Log_Write_DF_MAV(*this);
#if REMOTE_LOG_DEBUGGING
    printf("D:%d E:%d SF:%d/%d/%d SP:%d/%d/%d SS:%d/%d/%d SR:%d/%d/%d\n",
           stats.dropped,
           stats.internal_errors,
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

void DataFlash_MAVLink::stats_collect()
{
    if (!_initialised || !_logging_started) {
        return;
    }
    uint8_t pending = 0;
    uint8_t sent = 0;
    uint8_t retry = 0;
    uint8_t sfree = 0;
    for(uint8_t block = 0; block < _blockcount; block++){
        switch(_blocks[block].state) {
        case BLOCK_STATE_SEND_RETRY:
            retry++;
            break;
        case BLOCK_STATE_SEND_PENDING:
            pending++;
            break;
        case BLOCK_STATE_SENT:
            sent++;
            break;
        case BLOCK_STATE_FREE:
            sfree++;
            break;
        case BLOCK_STATE_FILLING:
            break;
        };
    }
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

void DataFlash_MAVLink::push_log_blocks()
{
    if (!_initialised || !_logging_started ||!_sending_to_client) {
        return;
    }

    _pushing_blocks = true;

    DataFlash_Backend::write_more_preface_messages();

    // here's an argument for keeping linked lists for each state!
    // firstly, send out any blocks that have been requested:
    for(uint8_t block = 0; block < _blockcount; block++){
        if (_blocks[block].state == BLOCK_STATE_SEND_RETRY) {
            if (! send_log_block(_blocks[block])) {
                _pushing_blocks = false;
                return;
            }
        }
    }

    // next, send out any blocks already sent if they haven't been
    // sent for a while (trying to catch the situation where both the
    // original block and any nacks for it have been lost)

    // count ensures we go through the list of blocks at most once
    // _next_block_number_to_resend remembers where we were up to for next time
    uint8_t count = 0;
    uint32_t now = hal.scheduler->millis();
    uint32_t oldest = now - 10; // 100 milliseconds before resend.  Hmm.
    while (count++ < _blockcount) {
        if (_blocks[_next_block_number_to_resend].state != BLOCK_STATE_SENT) {
            continue;
        }
        if (_blocks[_next_block_number_to_resend].last_sent > oldest) {
            continue;
        }
        if (! send_log_block(_blocks[_next_block_number_to_resend])) {
            _pushing_blocks = false;
            return;
        }
        _next_block_number_to_resend++;
        if (_next_block_number_to_resend >= _blockcount) {
            _next_block_number_to_resend = 0;
        }
    }

    // here's an argument for keeping linked lists for each state!
    // firstly, send out any pending blocks, in any order:
    for(uint8_t block = 0; block < _blockcount; block++){
        if (_blocks[block].state == BLOCK_STATE_SEND_PENDING) {
            if (! send_log_block(_blocks[block])) {
                _pushing_blocks = false;
                return;
            }
        }
    }

    _pushing_blocks = false;
}

void DataFlash_MAVLink::periodic_10Hz(const uint32_t now)
{
    stats_collect();
}
void DataFlash_MAVLink::periodic_1Hz(const uint32_t now)
{
    if (_sending_to_client &&
        _last_response_time + 10000 < now) {
        // other end appears to have timed out!
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
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    if (rand() < 0.9) {
        return false;
    }
#endif
    
    mavlink_message_t msg;
    mavlink_status_t *chan_status = mavlink_get_channel_status(chan);
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink.seq++;
    Debug("Sending block (%d)", block.seqno);
    mavlink_msg_remote_log_data_block_pack(mavlink_system.sysid,
                                           MAV_COMP_ID_LOG,
                                           &msg,
                                           _target_system_id,
                                           _target_component_id,
                                           block.seqno,
                                           block.buf);
    block.state = BLOCK_STATE_SENT;
    block.last_sent = hal.scheduler->millis();
    chan_status->current_tx_seq = saved_seq;

    _mavlink_resend_uart(chan, &msg);

    return true;
}
#endif
