/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */
#pragma once

#if HAL_OS_POSIX_IO

#include "DataFlash_Backend.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
/*
  the QURT port has a limited range of system calls available. It
  cannot provide all the facilities that DataFlash_File wants. It can
  provide enough to be useful though, which is what
  DATAFLASH_FILE_MINIMAL is for
 */
#define DATAFLASH_FILE_MINIMAL 1
#else
#define DATAFLASH_FILE_MINIMAL 0
#endif

class DataFlash_File : public DataFlash_Backend
{
public:
    // constructor
    DataFlash_File(DataFlash_Class &front,
                   DFMessageWriter_DFLogStart *,
                   const char *log_directory);

    // initialisation
    void Init() override;
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
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs() override;
    uint16_t start_new_log(void) override;
    void LogReadProcess(const uint16_t log_num,
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

    // this method is used when reporting system status over mavlink
    bool logging_enabled() const;
    bool logging_failed() const;

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
    bool ReadBlock(void *pkt, uint16_t size) override;

    uint16_t _log_num_from_list_entry(const uint16_t list_entry);

    // possibly time-consuming preparations handling
    void Prep_MinSpace();
    uint16_t find_oldest_log();
    int64_t disk_space_avail();
    int64_t disk_space();
    float avail_space_percent();

    bool file_exists(const char *filename) const;
    bool log_exists(const uint16_t lognum) const;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // I always seem to have less than 10% free space on my laptop:
    const float min_avail_space_percent = 0.1f;
#else
    const float min_avail_space_percent = 10.0f;
#endif
    // write buffer
    uint8_t *_writebuf;
    uint32_t _writebuf_size; // in bytes; may be == 65536, thus 32-bits
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

    uint16_t critical_message_reserved_space() const {
        // possibly make this a proportional to buffer size?
        uint16_t ret = 1024;
        if (ret > _writebuf_size) {
            // in this case you will only get critical messages
            ret = _writebuf_size;
        }
        return ret;
    };
    uint16_t non_messagewriter_message_reserved_space() const {
        // possibly make this a proportional to buffer size?
        uint16_t ret = 1024;
        if (ret >= _writebuf_size) {
            // need to allow messages out from the messagewriters.  In
            // this case while you have a messagewriter you won't get
            // any other messages.  This should be a corner case!
            ret = 0;
        }
        return ret;
    };

    // free-space checks; filling up SD cards under NuttX leads to
    // corrupt filesystems which cause loss of data, failure to gather
    // data and failures-to-boot.
    uint64_t _free_space_last_check_time; // microseconds
    const uint32_t _free_space_check_interval = 1000000UL; // microseconds
    const uint32_t _free_space_min_avail = 8388608; // bytes

    AP_HAL::Semaphore *semaphore;
    
    // performance counters
    AP_HAL::Util::perf_counter_t  _perf_write;
    AP_HAL::Util::perf_counter_t  _perf_fsync;
    AP_HAL::Util::perf_counter_t  _perf_errors;
    AP_HAL::Util::perf_counter_t  _perf_overruns;
};

#endif // HAL_OS_POSIX_IO
