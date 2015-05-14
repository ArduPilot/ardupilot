/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash Remote(via serial) logging - file oriented variant
*/

#include <AP_HAL.h>

#if HAL_OS_POSIX_IO
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


extern const AP_HAL::HAL& hal;

/*
  constructor
 */
DataFlash_MAVLink::DataFlash_MAVLink(mavlink_channel_t chan) :
    _chan(chan),
    _initialised(false),
    _total_blocks(80),
    _block_max_size(200),
    _latest_block_num(0),
    _cur_block_address(0),
    _latest_block_len(0)
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    ,_perf_errors(perf_alloc(PC_COUNT, "DF_errors")),
    _perf_overruns(perf_alloc(PC_COUNT, "DF_overruns"))
#endif
{ }


// initialisation
void DataFlash_MAVLink::Init(const struct LogStructure *structure, uint8_t num_types)
{
    memset(_block_num, 0, sizeof(_block_num));  
    DataFlash_Class::Init(structure, num_types);

    _initialised = true;
}


/* Write a block of data at current offset */
void DataFlash_MAVLink::WriteBlock(const void *pBuffer, uint16_t size)
{   
    if (!_initialised) {
        return;
    }


    uint8_t block_address = _cur_block_address;

    uint8_t remaining_size = _block_max_size - _latest_block_len;
    uint8_t start_address = _latest_block_len;
    if(remaining_size >= size){
        //single memcpy
        
        memcpy(&_buf[block_address][start_address], pBuffer, size);
        _latest_block_len+=size;

        if(_latest_block_len == _block_max_size){
            _block_num[block_address] = ++_latest_block_num;
            send_log_block(block_address);  //block full send it
            _cur_block_address = next_block_address();
        }
    } else {
        //do memcpy twice
        memcpy(&_buf[block_address][start_address], pBuffer, remaining_size);

        _block_num[block_address] = ++_latest_block_num;
        //send data here
        send_log_block(block_address);

        _cur_block_address = next_block_address();
        block_address = _cur_block_address;

        //printf("%d\n",block_address);
        memcpy(_buf[block_address], &((char *)pBuffer)[remaining_size], size - remaining_size);
        _latest_block_len += (size-remaining_size);
    }
}

//Get address of empty block to ovewrite
int8_t DataFlash_MAVLink::next_block_address()
{

    uint8_t oldest_block_address = 0;
    for(uint8_t block = 0; block < _total_blocks; block++){
        if(_block_num[oldest_block_address] > _block_num[block]){
            oldest_block_address = block;
        }
    }
    
    _latest_block_len=0;    //reset block size
    //printf("%d \n", oldest_block_address);
    return oldest_block_address;
}

void DataFlash_MAVLink::handle_ack(uint32_t block_num)
{
    if (!_initialised) {
        return;
    }
    if(block_num == 0){
        printf("\nStarting New Log!!\n");
        memset(_block_num, 0, sizeof(_block_num));
        _latest_block_num = 0;
        _cur_block_address = 0;
        _latest_block_len = 0;
        StartNewLog();
        return;
    }
    for(uint8_t block = 0; block < _total_blocks; block++){
        if(_block_num[block] == block_num) {
            _block_num[block] = 0;                  //forget the block if ack is received
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
        }
    }
}
//TODO: handle full txspace properly
void DataFlash_MAVLink::send_log_block(uint32_t block_address)
{

    if (!_initialised || comm_get_txspace(_chan) < 255){
       return; 
    }
    //printf("Data Sent!!\n");
    mavlink_msg_remote_log_data_block_send(_chan,_block_max_size,_block_num[block_address],_buf[block_address]);
    
}
#endif // HAL_OS_POSIX_IO

