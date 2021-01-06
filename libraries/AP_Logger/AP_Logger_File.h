/* 
   AP_Logger logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */
#pragma once

#include <AP_Filesystem/AP_Filesystem.h>

#include <AP_HAL/utility/RingBuffer.h>
#include "AP_Logger_Backend.h"

#if HAL_LOGGING_FILESYSTEM_ENABLED

#ifndef HAL_LOGGER_WRITE_CHUNK_SIZE
#define HAL_LOGGER_WRITE_CHUNK_SIZE 4096
#endif

class AP_Logger_File : public AP_Logger_Backend
{
public:
    // constructor
    AP_Logger_File(AP_Logger &front,
                   LoggerMessageWriter_DFLogStart *,
                   const char *log_directory);

    // initialisation
    void Init() override;
    bool CardInserted(void) const override;

    // erase handling
    void EraseAll() override;

    /* Write a block of data at current offset */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    uint32_t bufferspace_available() override;

    // high level interface
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) override;
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override;
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override;
    uint16_t get_num_logs() override;
    void start_new_log(void) override;
    uint16_t find_oldest_log() override;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    void flush(void) override;
#endif
    void periodic_1Hz() override;
    void periodic_fullrate() override;

    // this method is used when reporting system status over mavlink
    bool logging_failed() const override;

    bool logging_started(void) const override {
        return _iothread.logging_started();
    }
    void io_timer(void) override {
        _iothread.timer();
    }

protected:

    bool WritesOK() const override;
    bool StartNewLogOK() const override;

private:
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    bool _need_rtc_update;
#endif

    bool io_thread_alive() const;
    uint8_t io_thread_warning_decimation_counter;

    // write buffer
    ByteBuffer _writebuf{0};

    void stop_logging(void) override;

    uint32_t last_messagewrite_message_sent;

    // semaphore mediates access to the ringbuffer
    HAL_Semaphore semaphore;

    const char *last_io_operation = "";

    // not actually a thread, but called by the thread created in AP_Logger
    class IOThread {
    public:

        void timer(void);
        // do we have a recent open error?
        bool recent_open_error(void) const;
        bool logging_started() const { return _write_fd != -1; }

        void EraseAll();
        void stop_logging(void);

        /* construct a file name given a log number. Caller must free. */
        char *_log_file_name(const uint16_t log_num) const;
        char *_log_file_name_long(const uint16_t log_num) const;
        char *_log_file_name_short(const uint16_t log_num) const;
        char *_lastlog_file_name() const;

        uint16_t find_last_log();
        uint16_t find_oldest_log();
        int64_t disk_space_avail();
        int64_t disk_space();
        void start_new_log(void);
        bool WritesOK() const;
        int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
        uint16_t get_num_logs();

        const char *_log_directory;
        int64_t min_bytes_free;
        AP_Logger_File *_backend;

        uint32_t _heartbeat;
        bool _last_write_failed;

    private:

        // log-writing state:
        int _write_fd = -1;
        char *_write_filename;
        uint32_t _write_offset;
        uint32_t _last_write_ms;
        uint32_t _last_write_time;

        // log-reading state:
        int _read_fd = -1;
        uint16_t _read_fd_log_num;
        uint32_t _read_offset;
        void get_log_boundaries(const uint16_t list_entry,
                                uint32_t & start_page,
                                uint32_t & end_page);

        uint16_t log_num_from_list_entry(const uint16_t list_entry);
        void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);

        volatile uint32_t _open_error_ms;

        bool file_exists(const char *filename) const;
        bool log_exists(const uint16_t lognum) const;

        void ensure_log_directory_exists();

        uint32_t _get_log_size(const uint16_t log_num);
        uint32_t _get_log_time(const uint16_t log_num);

        // possibly time-consuming preparations handling
        void Prep_MinSpace();
        bool space_cleared;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        void flush(void);
#endif

        void check_message_queue();
        void retry_logging_open();
        void handle_write_buffer();

        // free-space checks; filling up SD cards under NuttX leads to
        // corrupt filesystems which cause loss of data, failure to gather
        // data and failures-to-boot.
        uint32_t _free_space_last_check_time; // milliseconds
        const uint32_t _free_space_check_interval = 1000UL; // milliseconds
        const uint32_t _free_space_min_avail = 8388608; // bytes

        // performance counters
        AP_HAL::Util::perf_counter_t  _perf_write;
        AP_HAL::Util::perf_counter_t  _perf_fsync;
        AP_HAL::Util::perf_counter_t  _perf_errors;
        AP_HAL::Util::perf_counter_t  _perf_overruns;
    } _iothread;

    enum class IOThreadRequestType {
        EraseAll,
        StartNewLog,
        Flush,
        KillWriteFD,
        FindLastLog,
        FindOldestLog,
        GetLogBoundaries,
        GetLogData,
        GetLogInfo,
        GetNumLogs,
        StopLogging,
    };
    class IOThreadRequest {
    public:
        IOThreadRequestType type;
        void *data;
        bool complete;
    };
    ObjectBuffer_TS<IOThreadRequest*> thread_requests;
    bool complete_iothread_request(IOThreadRequestType request, void *data=nullptr);
};

#endif // HAL_LOGGING_FILESYSTEM_ENABLED
