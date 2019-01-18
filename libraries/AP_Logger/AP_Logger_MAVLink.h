/* 
   AP_Logger logging - MAVLink variant

   - transfers blocks of the open log file to a client using MAVLink
 */
#pragma once

#define DATAFLASH_MAVLINK_SUPPORT 1

#if DATAFLASH_MAVLINK_SUPPORT

#include <AP_HAL/AP_HAL.h>

#include "AP_Logger_Backend.h"

extern const AP_HAL::HAL& hal;

#define DF_MAVLINK_DISABLE_INTERRUPTS 0

class AP_Logger_MAVLink : public AP_Logger_Backend
{
public:
    // constructor
    AP_Logger_MAVLink(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
        AP_Logger_Backend(front, writer),
        _max_blocks_per_send_blocks(8)
        ,_perf_packing(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "DM_packing"))
        {
            _blockcount = 1024*((uint8_t)_front._params.mav_bufsize) / sizeof(struct dm_block);
            // ::fprintf(stderr, "DM: Using %u blocks\n", _blockcount);
        }

    // initialisation
    void Init() override;

    // in actual fact, we throw away everything until a client connects.
    // This stops calls to start_new_log from the vehicles
    bool logging_started() const override { return _initialised; }

    void stop_logging() override;

    /* Write a block of data at current offset */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size,
                               bool is_critical) override;

    // initialisation
    bool CardInserted(void) const override { return true; }

    // erase handling
    void EraseAll() override {}

    bool NeedPrep() override { return false; }
    void Prep() override { }

    // high level interface
    uint16_t find_last_log(void) override { return 0; }
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) override {}
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override {}
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override { return 0; }
    uint16_t get_num_logs(void) override { return 0; }

    void push_log_blocks() override;

    void remote_log_block_status_msg(mavlink_channel_t chan, mavlink_message_t* msg) override;

protected:

    bool WritesOK() const override;

private:

    struct dm_block {
        uint32_t seqno;
        uint8_t buf[MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN];
        uint32_t last_sent;
        struct dm_block *next;
    };
    bool send_log_block(struct dm_block &block);
    void handle_ack(mavlink_channel_t chan, mavlink_message_t* msg, uint32_t seqno);
    void handle_retry(uint32_t block_num);
    void do_resends(uint32_t now);
    void free_all_blocks();

    // a stack for free blocks, queues for pending, sent, retries and sent
    struct dm_block_queue {
        uint32_t sent_count;
        struct dm_block *oldest;
        struct dm_block *youngest;
    };
    typedef struct dm_block_queue dm_block_queue_t ;
    void enqueue_block(dm_block_queue_t &queue, struct dm_block *block);
    bool queue_has_block(dm_block_queue_t &queue, struct dm_block *block);
    struct dm_block *dequeue_seqno(dm_block_queue_t &queue, uint32_t seqno);
    bool free_seqno_from_queue(uint32_t seqno, dm_block_queue_t &queue);
    bool send_log_blocks_from_queue(dm_block_queue_t &queue);
    uint8_t stack_size(struct dm_block *stack);
    uint8_t queue_size(dm_block_queue_t queue);
    
    struct dm_block *_blocks_free;
    dm_block_queue_t _blocks_sent;
    dm_block_queue_t _blocks_pending;
    dm_block_queue_t _blocks_retry;

    struct _stats {
        // the following are reset any time we log stats (see "reset_stats")
        uint32_t resends;
        uint8_t collection_count;
        uint16_t state_free; // cumulative across collection period
        uint8_t state_free_min;
        uint8_t state_free_max;
        uint16_t state_pending; // cumulative across collection period
        uint8_t state_pending_min;
        uint8_t state_pending_max;
        uint16_t state_retry; // cumulative across collection period
        uint8_t state_retry_min;
        uint8_t state_retry_max;
        uint16_t state_sent; // cumulative across collection period
        uint8_t state_sent_min;
        uint8_t state_sent_max;
    } stats;

    // this method is used when reporting system status over mavlink
    bool logging_enabled() const override { return true; }
    bool logging_failed() const override;

    mavlink_channel_t _chan;
    uint8_t _target_system_id;
    uint8_t _target_component_id;

    // this controls the maximum number of blocks we will push from
    // the pending and send queues in any call to push_log_blocks.
    // push_log_blocks is called by periodic_tasks.  Each block is 200
    // bytes.  In Plane, at 50Hz, a _max_blocks_per_send_blocks of 2
    // means we will push at most 2*50*200 == 20KB of logs per second
    // _max_blocks_per_send_blocks has to be high enough to push all
    // of the logs, but low enough that we don't spend way too much
    // time packing messages in any one loop
    const uint8_t _max_blocks_per_send_blocks;
    
    uint32_t _next_seq_num;
    uint16_t _latest_block_len;
    uint32_t _last_response_time;
    uint32_t _last_send_time;
    uint8_t _next_block_number_to_resend;
    bool _sending_to_client;

    void Log_Write_DF_MAV(AP_Logger_MAVLink &df);

    uint32_t bufferspace_available() override; // in bytes
    uint8_t remaining_space_in_current_block();
    // write buffer
    uint8_t _blockcount_free;
    uint8_t _blockcount;
    struct dm_block *_blocks;
    struct dm_block *_current_block;
    struct dm_block *next_block();

    void periodic_10Hz(uint32_t now) override;
    void periodic_1Hz() override;
    void periodic_fullrate() override;
    
    void stats_init();
    void stats_reset();
    void stats_collect();
    void stats_log();
    uint32_t _stats_last_collected_time;
    uint32_t _stats_last_logged_time;
    uint8_t mavlink_seq;

    /* we currently ignore requests to start a new log.  Notionally we
     * could close the currently logging session and hope the client
     * re-opens one */
    uint16_t start_new_log(void) override {
        return 0;
    }
    // performance counters
    AP_HAL::Util::perf_counter_t  _perf_errors;
    AP_HAL::Util::perf_counter_t  _perf_packing;
    AP_HAL::Util::perf_counter_t  _perf_overruns;

    HAL_Semaphore semaphore;
};

#endif // DATAFLASH_MAVLINK_SUPPORT
