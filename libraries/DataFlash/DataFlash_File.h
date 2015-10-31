/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */

#ifndef DataFlash_File_h
#define DataFlash_File_h

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <systemlib/perf_counter.h>
#else
#define perf_begin(x)
#define perf_end(x)
#define perf_count(x)
#endif

#include "DataFlash_Backend.h"

class DataFlash_File : public DataFlash_Backend
{
public:
    // constructor
    DataFlash_File(const struct LogStructure *structure, uint8_t num_types,
                   DFMessageWriter *, const char *log_directory);

    // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types);
    bool CardInserted(void);

    // erase handling
    bool NeedErase(void);
    void EraseAll();

    /* Write a block of data at current offset */
    bool WriteBlock(const void *pBuffer, uint16_t size);
    uint16_t bufferspace_available();
    
    // high level interface
    uint16_t find_last_log(void);
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs();
    uint16_t start_new_log(void);
    void LogReadProcess(const uint16_t log_num,
                        uint16_t start_page, uint16_t end_page, 
                        void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                        AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);

    // possibly time-consuming preparations handling
    void Prep_MinSpace();

    void periodic_tasks();

protected:
    void push_log_blocks();
private:
    int _write_fd;
    int _read_fd;
    uint16_t _read_fd_log_num;
    uint32_t _read_offset;
    uint32_t _write_offset;
    volatile bool _initialised;
    volatile bool _open_error;
    const char *_log_directory;

    uint16_t _cached_oldest_log;

    /*
      read a block
    */
    void ReadBlock(void *pkt, uint16_t size);

    uint16_t _log_num_from_list_entry(const uint16_t list_entry);

    uint16_t find_oldest_log();
    int64_t disk_space_avail();
    int64_t disk_space();
    float avail_space_percent();

    bool file_exists(const char *filename) const;
    bool log_exists(const uint16_t lognum) const;

    const float min_avail_space_percent;

    // write buffer
    uint8_t *_writebuf;
    uint16_t _writebuf_size;
    const uint16_t _writebuf_chunk;
    volatile uint16_t _writebuf_head;
    volatile uint16_t _writebuf_tail;
    uint32_t _last_write_time;

    /* construct a file name given a log number. Caller must free. */
    char *_log_file_name(const uint16_t log_num) const;
    char *_lastlog_file_name() const;
    uint32_t _get_log_size(const uint16_t log_num) const;
    uint32_t _get_log_time(const uint16_t log_num) const;

    void stop_logging(void);

    void _io_timer(void);

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // performance counters
    perf_counter_t  _perf_write;
    perf_counter_t  _perf_fsync;
    perf_counter_t  _perf_errors;
    perf_counter_t  _perf_overruns;
#endif
};


#endif // DataFlash_File_h

