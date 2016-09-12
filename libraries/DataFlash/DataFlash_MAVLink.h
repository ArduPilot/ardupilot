/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - MAVLink variant

   - transfers blocks of the open log file to a client using MAVLink
 */
#pragma once

#define DATAFLASH_MAVLINK_SUPPORT 1

#if DATAFLASH_MAVLINK_SUPPORT

#include <AP_HAL/AP_HAL.h>

#include "DataFlash_Backend.h"

extern const AP_HAL::HAL& hal;

#define DF_MAVLINK_DISABLE_INTERRUPTS 0

class DataFlash_MAVLink : public DataFlash_Backend
{
    friend class DataFlash_Class; // for access to stats on Log_Df_Mav_Stats
public:
    // constructor
    DataFlash_MAVLink(DataFlash_Class &front, DFMessageWriter_DFLogStart *writer) :
        DataFlash_Backend(front, writer),
        _max_blocks_per_send_blocks(8),
        _blockcount(32) // this may get reduced in Init if allocation fails
        ,_perf_packing(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "DM_packing"))
        { }

    // initialisation
    void Init() override;

    bool logging_started() { return _logging_started; }

    void stop_logging();

    /* Write a block of data at current offset */
    bool WritePrioritisedBlock(const void *pBuffer, uint16_t size,
                               bool is_critical) override;

    // initialisation
    bool CardInserted(void) override { return true; }

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
    
    void LogReadProcess(uint16_t log_num,
                        uint16_t start_page, uint16_t end_page, 
                        print_mode_fn printMode,
                        AP_HAL::BetterStream *port) override {}
    void DumpPageInfo(AP_HAL::BetterStream *port) override {}
    void ShowDeviceInfo(AP_HAL::BetterStream *port) override {}
    void ListAvailableLogs(AP_HAL::BetterStream *port) override {}

    // enum dm_block_state {
    //     BLOCK_STATE_FREE = 17,
    //     BLOCK_STATE_FILLING,
    //     BLOCK_STATE_SEND_PENDING,
    //     BLOCK_STATE_SEND_RETRY,
    //     BLOCK_STATE_SENT
    // };
    struct dm_block {
        uint32_t seqno;
        uint8_t buf[MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN];
        uint32_t last_sent;
        struct dm_block *next;
    };
    void push_log_blocks();
    virtual bool send_log_block(struct dm_block &block);
    virtual void handle_ack(mavlink_channel_t chan, mavlink_message_t* msg, uint32_t seqno);
    virtual void handle_retry(uint32_t block_num);
    void do_resends(uint32_t now);
    virtual void set_channel(mavlink_channel_t chan);
    virtual void remote_log_block_status_msg(mavlink_channel_t chan, mavlink_message_t* msg) override;
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

protected:
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
    bool logging_enabled() const { return true; }
    bool logging_failed() const;

private:
    mavlink_channel_t _chan;
    uint8_t _target_system_id;
    uint8_t _target_component_id;

    bool _initialised;

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
    bool _logging_started;
    uint32_t _last_response_time;
    uint32_t _last_send_time;
    uint8_t _next_block_number_to_resend;
    bool _sending_to_client;

    void Log_Write_DF_MAV(DataFlash_MAVLink &df);
    
    void internal_error();
    uint16_t bufferspace_available() override; // in bytes
    uint8_t remaining_space_in_current_block();
    // write buffer
    uint8_t _blockcount_free;
    uint8_t _blockcount;
    struct dm_block *_blocks;
    struct dm_block *_current_block;
    struct dm_block *next_block();

    void periodic_10Hz(uint32_t now);
    void periodic_1Hz(uint32_t now);
    void periodic_fullrate(uint32_t now);
    
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
    bool ReadBlock(void *pkt, uint16_t size) override {
        return false;
    }
    // performance counters
    AP_HAL::Util::perf_counter_t  _perf_errors;
    AP_HAL::Util::perf_counter_t  _perf_packing;
    AP_HAL::Util::perf_counter_t  _perf_overruns;
};

#endif // DATAFLASH_MAVLINK_SUPPORT
