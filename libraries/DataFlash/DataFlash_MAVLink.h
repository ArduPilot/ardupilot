/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */

#ifndef DataFlash_MAVLink_h
#define DataFlash_MAVLink_h

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <systemlib/perf_counter.h>
#else
#define perf_begin(x)
#define perf_end(x)
#define perf_count(x)
#endif

#include "DataFlash_Backend.h"

class DataFlash_MAVLink : public DataFlash_Backend
{
    friend class DataFlash_Class; // for access to stats on Log_Df_Mav_Stats
public:
    // constructor
    DataFlash_MAVLink(DataFlash_Class &front, mavlink_channel_t chan) :
        DataFlash_Backend(front),
        _chan(chan),
        _initialised(false),
        _next_seq_num(0),
        _current_block(NULL),
        _latest_block_len(0),
        _next_block_number_to_resend(0),
        _last_response_time(0),
        _last_send_time(0),
        _blockcount(32), // this may get reduced in Init if allocation fails
        _blockcount_free(0),
        _logging_started(false),
        _sending_to_client(false),
        _pushing_blocks(false),
        mavlink_seq(0)
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        ,_perf_errors(perf_alloc(PC_COUNT, "DF_errors")),
        _perf_overruns(perf_alloc(PC_COUNT, "DF_overruns"))
#endif
        { }

    // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types);

    bool logging_started() { return _logging_started; }

    /* Write a block of data at current offset */
    bool WriteBlock(const void *pBuffer, uint16_t size);

    // initialisation
    bool CardInserted(void) { return true; }

    // erase handling
    bool NeedErase(void){ return false; }
    void EraseAll(){}

    // high level interface
    uint16_t find_last_log(void) { return 0; }
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page) {}
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) {}
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) { return 0; }
    uint16_t get_num_logs(void){return 0;}
    
    void LogReadProcess(uint16_t log_num,
                        uint16_t start_page, uint16_t end_page, 
                        void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                        AP_HAL::BetterStream *port) {}
    void DumpPageInfo(AP_HAL::BetterStream *port) {}
    void ShowDeviceInfo(AP_HAL::BetterStream *port) {}
    void ListAvailableLogs(AP_HAL::BetterStream *port) {}

    enum dm_block_state {
        BLOCK_STATE_FREE = 17,
        BLOCK_STATE_FILLING,
        BLOCK_STATE_SEND_PENDING,
        BLOCK_STATE_SEND_RETRY,
        BLOCK_STATE_SENT
    };
    struct dm_block {
        uint32_t seqno;
        uint8_t buf[MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN];
        dm_block_state state;
        uint32_t last_sent;
    };
    void push_log_blocks();
    virtual bool send_log_block(struct dm_block &block);
    virtual void handle_ack(mavlink_message_t* msg, uint32_t seqno);
    virtual void handle_retry(uint32_t block_num);
    void do_resends(uint32_t now);
    virtual void set_channel(mavlink_channel_t chan);
    virtual void remote_log_block_status_msg(mavlink_message_t* msg);
    void free_all_blocks();

protected:
    struct _stats {
        uint32_t dropped;
        uint8_t internal_errors; // uint8_t - wishful thinking?
        // the following are reset any time we log stats (see "reset_stats")
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

private:
    bool _pushing_blocks;
    mavlink_channel_t _chan;
    uint8_t _target_system_id;
    uint8_t _target_component_id;

    bool _initialised;

    uint32_t _next_seq_num;
    uint16_t _latest_block_len;
    bool _logging_started;
    uint32_t _last_response_time;
    uint32_t _last_send_time;
    uint8_t _next_block_number_to_resend;
    bool _sending_to_client;

    void internal_error();
    uint16_t bufferspace_available(); // in bytes
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

    uint16_t start_new_log(void) { return 0; }
    void ReadBlock(void *pkt, uint16_t size) {}
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // performance counters
    perf_counter_t  _perf_write;
    perf_counter_t  _perf_errors;
    perf_counter_t  _perf_overruns;
#endif
};


#endif // DataFlash_MAVLink_h

