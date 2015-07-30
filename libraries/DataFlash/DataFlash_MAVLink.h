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

class DataFlash_MAVLink : public DataFlash_Backend
{
public:
    // constructor
    DataFlash_MAVLink(DataFlash_Class &front) :
        DataFlash_Backend(front),
        _initialised(false),
        _cur_block_address(0),
        _latest_block_len(0),
        _latest_block_num(0),
        _logging_started(false),
        _sending_to_client(false),
        _only_critical_blocks(false),
        _is_critical_block{0},
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
    void WriteBlock(const void *pBuffer, uint16_t size);

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
                        print_mode_fn print_mode,
                        AP_HAL::BetterStream *port) {}
    void DumpPageInfo(AP_HAL::BetterStream *port) {}
    void ShowDeviceInfo(AP_HAL::BetterStream *port) {}
    void ListAvailableLogs(AP_HAL::BetterStream *port) {}
    virtual void send_log_block(uint32_t block_address);
    virtual void handle_ack(mavlink_channel_t chan, mavlink_message_t* msg,
                            uint32_t block_num);
    virtual void handle_retry(uint32_t block_num);
    virtual void remote_log_block_status_msg(mavlink_channel_t chan,
                                             mavlink_message_t* msg);
    void WriteCriticalBlock(const void *pBuffer, uint16_t size);
    void _WriteBlock(const void *pBuffer, uint16_t size, bool iscritical);
private:

    mavlink_channel_t _chan;
    uint8_t _target_system_id;
    uint8_t _target_component_id;

    bool _initialised;
    uint32_t _last_response_time;

    uint8_t _cur_block_address;
    uint16_t _latest_block_len;
    uint32_t _latest_block_num;
    bool _logging_started;
    bool _sending_to_client;
    bool _only_critical_blocks;

    // write buffer
    // FIXME: allocate this like we do in DataFlash_File
    static const uint16_t _total_blocks = 120;
    static const uint16_t _block_max_size = 200;
    uint8_t _buf[_total_blocks][_block_max_size];
    uint32_t _block_num[_total_blocks];
    bool _is_critical_block[_total_blocks];
    uint8_t mavlink_seq;

    uint16_t start_new_log(void) { return 0; }
    bool ReadBlock(void *pkt, uint16_t size) { return false; }
    int8_t next_block_address();
    bool _buffer_empty();
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // performance counters
    perf_counter_t  _perf_write;
    perf_counter_t  _perf_errors;
    perf_counter_t  _perf_overruns;
#endif
};


#endif // DataFlash_MAVLink_h

