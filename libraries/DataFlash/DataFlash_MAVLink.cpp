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
#define MAV_COMP_ID_LOG 1
#endif
/*
  constructor
 */
DataFlash_MAVLink::DataFlash_MAVLink(mavlink_channel_t chan) :
    _chan(chan),
    _initialised(false),
    _total_blocks(NUM_BUFFER_BLOCKS),
    _block_max_size(BUFFER_BLOCK_SIZE),
    _latest_block_num(0),
    _cur_block_address(0),
    _latest_block_len(0)
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
    memset(_block_num, 0, sizeof(_block_num));  
    DataFlash_Class::Init(structure, num_types);

    mavlink.system_id = MAV_SYS_ID_LOG;
    mavlink.component_id = MAV_COMP_ID_LOG;
    _initialised = true;
}


/* Write a block of data at current offset */
void DataFlash_MAVLink::WriteBlock(const void *pBuffer, uint16_t size)
{   
    if (!_initialised || !_logging_started) {
        return;
    }

    uint16_t copied = 0;
    while (copied < size) {
        uint16_t remaining_to_copy = size - copied;
        uint16_t _curr_remaining = _block_max_size - _latest_block_len;
        uint16_t to_copy = (remaining_to_copy > _curr_remaining) ? _curr_remaining : remaining_to_copy;
        memcpy(&_buf[_cur_block_address][_latest_block_len], &((const uint8_t *)pBuffer)[copied], to_copy);
        copied += to_copy;
        _latest_block_len += to_copy;
        if (_latest_block_len == _block_max_size) {
            _block_num[_cur_block_address] = _latest_block_num++;
            send_log_block(_cur_block_address);  //block full send it
            _cur_block_address = next_block_address();
            _latest_block_len = 0;
        }
    }
}

//Get address of empty block to ovewrite
int8_t DataFlash_MAVLink::next_block_address()
{

    uint8_t oldest_block_address = 0;
    for(uint8_t block = 0; block < _total_blocks; block++){
        if(_block_num[oldest_block_address] > _block_num[block]){
            oldest_block_address = block;
            if (_block_num[oldest_block_address] == 0) {
                break;
            }
        }
    }
    
    if (_block_num[oldest_block_address] != 0) {
        perf_count(_perf_overruns);
    }
    Debug("%d \n", oldest_block_address);
    return oldest_block_address;
}

void DataFlash_MAVLink::handle_ack(uint32_t block_num)
{
    if (!_initialised) {
        return;
    }
    if(block_num == 4294967294){ // 2^32-2
        Debug("Received stop-logging packet\n");
        _logging_started = false;
        return;
    }
    if(block_num == 4294967295){
        Debug("\nStarting New Log!!\n");
        memset(_block_num, 0, sizeof(_block_num));
        _latest_block_num = 0;
        _cur_block_address = 0;
        _latest_block_len = 0;
        _logging_started = true;
        StartNewLog();
        return;
    }
    for(uint8_t block = 0; block < _total_blocks; block++){
        if(_block_num[block] == block_num) {
            _block_num[block] = 0;                  //forget the block if ack is received
            return;
        }
    }
}

void DataFlash_MAVLink::handle_retry(uint32_t block_num)
{
    if (!_initialised) {
        return;
    }
    for(uint8_t block = 0; block < _total_blocks; block++){
        if(_block_num[block] == block_num) {
            send_log_block(block);
            return;
        }
    }
}

void DataFlash_MAVLink::set_channel(mavlink_channel_t chan)
{
    _chan = chan;
}

//TODO: handle full txspace properly
void DataFlash_MAVLink::send_log_block(uint32_t block_address)
{
    mavlink_channel_t chan = mavlink_channel_t(_chan - MAVLINK_COMM_0);
    if (!_initialised){
       return; 
    }
    mavlink_message_t msg;
    mavlink_status_t *chan_status = mavlink_get_channel_status(chan);
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink.seq++;
    Debug("Data Sent!!\n");
    uint16_t len = mavlink_msg_remote_log_data_block_pack(mavlink.system_id, 
                                                          mavlink.component_id, 
                                                          &msg,
                                                          _block_max_size,
                                                          _block_num[block_address],
                                                          _buf[block_address]);
    if(comm_get_txspace(chan) < len){
        return;
    }
    chan_status->current_tx_seq = saved_seq;
    _mavlink_resend_uart(chan, &msg);
}
#endif
