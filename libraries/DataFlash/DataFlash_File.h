/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */

#ifndef DataFlash_File_h
#define DataFlash_File_h

#if HAL_OS_POSIX_IO

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
    DataFlash_File(DataFlash_Class &front, const char *log_directory);

    // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types);
    bool CardInserted(void);

    // erase handling
    void EraseAll();

    // possibly time-consuming preparation handling:
    bool NeedPrep();
    void Prep();

    /* Write a block of data at current offset */
    bool WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical);
    uint16_t bufferspace_available();

    // high level interface
    uint16_t find_last_log(void);
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs(void);
    bool _log_exists(uint16_t log_num);
    uint16_t start_new_log(void);
    void LogReadProcess(uint16_t log_num,
                        uint16_t start_page, uint16_t end_page, 
                        print_mode_fn print_mode,
                        AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    void flush(void);
#endif
    void periodic_fullrate(const uint32_t now);
    
private:
    int _write_fd;
    int _read_fd;
    uint16_t _read_fd_log_num;
    uint32_t _read_offset;
    uint32_t _write_offset;
    volatile bool _initialised;
    volatile bool _open_error;
    const char *_log_directory;

    /*
      read a block
    */
    bool ReadBlock(void *pkt, uint16_t size);

    // possibly time-consuming preparations handling
    void Prep_MinSpace();
    uint16_t find_first_log(void);
    uint64_t disk_space_avail();
    uint64_t disk_space();
    float avail_space_percent();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // I always seem to have less than 10% free space on my laptop:
    const float min_avail_space_percent = 0.1f;
#else
    const float min_avail_space_percent = 10.0f;
#endif
    // write buffer
    uint8_t *_writebuf;
    uint16_t _writebuf_size;
    const uint16_t _writebuf_chunk;
    volatile uint16_t _writebuf_head;
    volatile uint16_t _writebuf_tail;
    uint32_t _last_write_time;

    /* construct a file name given a log number. Caller must free. */
    char *_log_file_name(uint16_t log_num);
    char *_lastlog_file_name(void);
    uint32_t _get_log_size(uint16_t log_num);
    uint32_t _get_log_time(uint16_t log_num);

    void stop_logging(void);

    void _io_timer(void);

    uint16_t critical_message_reserved_space() const {
        // possibly make this a proportional to buffer size?
        return 1024;
    };
    uint16_t non_messagewriter_message_reserved_space() const {
        // possibly make this a proportional to buffer size?
        return 1024;
    };

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // performance counters
    perf_counter_t  _perf_write;
    perf_counter_t  _perf_fsync;
    perf_counter_t  _perf_errors;
    perf_counter_t  _perf_overruns;
#endif
};

#endif // HAL_OS_POSIX_IO

#endif // DataFlash_File_h
