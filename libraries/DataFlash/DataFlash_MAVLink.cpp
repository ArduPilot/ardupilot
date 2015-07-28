/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash Remote(via serial) logging - file oriented variant
*/

#include <AP_HAL.h>

#ifdef HAL_BOARD_REMOTE_LOG_PORT
#include "DataFlash.h"
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
    _block_max_size(BUFFER_BLOCK_SIZE),
    _next_seq_num(0),
    _latest_block_len(0),
    _next_block_number_to_resend(0),
    _stats_last_collected_time(0),
    _stats_last_logged_time(0)
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

    _blockcount = _blockcount_max;
    _blocks = NULL;
    while (_blocks == NULL && _blockcount > 16) { // 16 is a *magic* number
        _blocks = (struct dm_block *) malloc(_blockcount * sizeof(_blocks[0]));
    }

    if (_blocks == NULL) {
        return;
    }

    _initialised = true;
}

/* Write a block of data at current offset */
uint16_t DataFlash_MAVLink::bufferspace_available() {
    return (_blockcount_free * 200);
}

void DataFlash_MAVLink::WriteBlock(const void *pBuffer, uint16_t size)
{
    if (!_initialised || !_logging_started) {
        return;
    }

    if (bufferspace_available() < size) {
        stats.dropped++;
        return;
    }
    if (_current_block == NULL) {
        _current_block = next_block();
        if (_current_block == NULL) {
            stats.internal_errors++;
            return;
        }
    }

    uint16_t copied = 0;

    while (copied < size) {
        uint16_t remaining_to_copy = size - copied;
        uint16_t _curr_remaining = _block_max_size - _latest_block_len;
        uint16_t to_copy = (remaining_to_copy > _curr_remaining) ? _curr_remaining : remaining_to_copy;
        memcpy(&(_current_block->buf[_latest_block_len]), &((const uint8_t *)pBuffer)[copied], to_copy);
        copied += to_copy;
        _latest_block_len += to_copy;
        if (_latest_block_len == _block_max_size) {
            //block full, mark it to be sent:
            _current_block->state = BLOCK_STATE_SEND_PENDING;
            _current_block = next_block();
            if (_current_block == NULL) {
                // should not happen - there's a sanity check above
                stats.internal_errors++;
                break;
            }
        }
    }

    push_log_blocks();
}

//Get address of empty block to ovewrite
struct DataFlash_MAVLink::dm_block *DataFlash_MAVLink::next_block()
{
    for(uint8_t i = 0; i < _blockcount; i++){
        if (_blocks[i].state == BLOCK_STATE_FREE) {
            _blocks[i].state = BLOCK_STATE_FILLING;
            _blocks[i].seqno = _next_seq_num++;
            _blocks[i].last_sent = 0;
            _latest_block_len = 0;
            _blockcount_free--;
            return &_blocks[i];
        }
    }
    return NULL;
}

void DataFlash_MAVLink::handle_ack(mavlink_message_t* msg,
                                   uint32_t seqno)
{
    if (!_initialised) {
        return;
    }
    _last_response_time = hal.scheduler->millis();
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_STOP) {
        Debug("Received stop-logging packet\n");
        _logging_started = false;
        return;
    }
    if(seqno == MAV_REMOTE_LOG_DATA_BLOCK_START){
        Debug("\nStarting New Log!!\n");
        for(uint8_t i=0; i < _blockcount; i++) {
            _blocks[i].state = BLOCK_STATE_FREE;
        }
        _blockcount_free = _blockcount;
        _current_block = next_block();
        if (_current_block == NULL) {
            Debug("No free blocks?!!!\n");
            return;
        }
        stats_init();
        _logging_started = true;
        _target_system_id = msg->sysid;
        _target_component_id = msg->compid;
        _front.StartNewLog();
        return;
    }
    for(uint8_t block = 0; block < _blockcount; block++){
        if(_blocks[block].seqno == seqno) {
            _blocks[block].state = BLOCK_STATE_FREE;
            _blockcount_free++;
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
    if (!_initialised) {
        return;
    }
    _last_response_time = hal.scheduler->millis();
    for(uint8_t block = 0; block < _blockcount; block++){
        if(_blocks[block].seqno == seqno) {
            _blocks[block].state = BLOCK_STATE_SEND_PENDING;
            return;
        }
    }
}

void DataFlash_MAVLink::set_channel(mavlink_channel_t chan)
{
    _chan = chan;
}

void DataFlash_MAVLink::stats_init() {
    stats.dropped = 0;
    stats.internal_errors = 0;
    stats_reset();
}
void DataFlash_MAVLink::stats_reset() {
    stats.state_free = 0;
    stats.state_free_min = -1;
    stats.state_free_max = 0;
    stats.state_pending = 0;
    stats.state_pending_min = -1;
    stats.state_pending_max = 0;
    stats.state_sent = 0;
    stats.state_sent_min = -1;
    stats.state_sent_max = 0;
    stats.collection_count = 0;
}

void DataFlash_MAVLink::stats_log()
{
    _front.Log_Write_DF_MAV(*this);
#if REMOTE_LOG_DEBUGGING
    printf("D:%d E:%d SF:%d/%d/%d SP:%d/%d/%d SS:%d/%d/%d\n",
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
           stats.state_sent/stats.collection_count
        );
#endif
    stats_reset();
}

void DataFlash_MAVLink::stats_collect()
{
    uint8_t pending = 0;
    uint8_t sent = 0;
    uint8_t sfree = 0;
    for(uint8_t block = 0; block < _blockcount; block++){
        switch(_blocks[block].state) {
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
    stats.state_pending += pending;
    stats.state_sent += sent;
    stats.state_free += sfree;

    if (pending < stats.state_pending_min) {
        stats.state_pending_min = pending;
    }
    if (pending > stats.state_pending_max) {
        stats.state_pending_max = pending;
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

void DataFlash_MAVLink::stats_handle()
{
    uint32_t now = hal.scheduler->millis();
    if (now - _stats_last_collected_time > 100) {
        // 10 Hz collection time on stats
        stats_collect();
        _stats_last_collected_time = now;
    }
    if (now - _stats_last_logged_time > 1000) {
        // each second.... FIXME put the number elsewhere
        stats_log();
        _stats_last_logged_time = now;
    }
}

void DataFlash_MAVLink::push_log_blocks()
{
    if (!_initialised || !_logging_started) {
        return;
    }

    // here's an argument for keeping linked lists for each state!
    // firstly, send out any pending blocks, in any order:
    for(uint8_t block = 0; block < _blockcount; block++){
        if (_blocks[block].state == BLOCK_STATE_SEND_PENDING) {
            if (! send_log_block(_blocks[block])) {
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
    uint32_t oldest = now - 100; // 100 milliseconds before resend.  Hmm.
    while (count++ < _blockcount) {
        if (_blocks[_next_block_number_to_resend].state != BLOCK_STATE_SENT) {
            continue;
        }
        if (_blocks[_next_block_number_to_resend].last_sent > oldest) {
            continue;
        }
        if (! send_log_block(_blocks[_next_block_number_to_resend])) {
            return;
        }
        _next_block_number_to_resend++;
        if (_next_block_number_to_resend >= _blockcount) {
            _next_block_number_to_resend = 0;
        }
    }
}

void DataFlash_MAVLink::periodic_tasks()
{
    stats_handle();
    push_log_blocks();
}


//TODO: handle full txspace properly
bool DataFlash_MAVLink::send_log_block(struct dm_block &block)
{
    mavlink_channel_t chan = mavlink_channel_t(_chan - MAVLINK_COMM_0);
    if (!_initialised || comm_get_txspace(chan) < 255){
       return false;
    }
    mavlink_message_t msg;
    mavlink_status_t *chan_status = mavlink_get_channel_status(chan);
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink.seq++;
    Debug("Sending block (%d)", block.seqno);
    uint16_t len = mavlink_msg_remote_log_data_block_pack(mavlink_system.sysid,
                                                          MAV_COMP_ID_LOG,
                                                          &msg,
                                                          _target_system_id,
                                                          _target_component_id,
                                                          block.seqno,
                                                          block.buf);
    if(comm_get_txspace(chan) < len){
        return false;
    }
    block.state = BLOCK_STATE_SENT;
    block.last_sent = hal.scheduler->millis();
    chan_status->current_tx_seq = saved_seq;
    _mavlink_resend_uart(chan, &msg);

    return true;
}
#endif
