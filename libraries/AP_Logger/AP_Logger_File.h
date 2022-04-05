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

// not actually a thread, but called by the thread created in AP_Logger
class LoggerBackendThread_File : public LoggerBackendThread
{
public:

    void timer(void) override;
    // do we have a recent open error?
    bool recent_open_error(void) const;
    bool logging_started() const override { return _write_fd != -1; }

    void EraseAll() override;
    void stop_logging(void) override;

    /* construct a file name given a log number. Caller must free. */
    char *_log_file_name(const uint16_t log_num) const;
    char *_log_file_name_long(const uint16_t log_num) const;
    char *_log_file_name_short(const uint16_t log_num) const;
    char *_lastlog_file_name() const;

    uint16_t find_last_log() override;
    uint16_t find_oldest_log() override;
    int64_t disk_space_avail() override;
    int64_t disk_space() override;
    void start_new_log(void) override;
    bool WritesOK() const;
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override;
    uint16_t get_num_logs() override;
    void handle_request(LoggerThreadRequest &request) override;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    void flush(void) override;
#endif

    const char *_log_directory;
    int64_t min_bytes_free;

    uint32_t _heartbeat;
    bool _last_write_failed;

    // write buffer
    ByteBuffer writebuf{0};
    // semaphore mediates access to the ringbuffer
    HAL_Semaphore writebuf_semaphore;

    const char *last_io_operation = "";

private:

    // log-writing state:
    int _write_fd = -1;
    char *_write_filename;
    uint32_t _write_offset;
    uint32_t _last_write_ms;
    uint32_t _last_write_time;

    // methods and state for checking we're likely to be able to open files:
    bool check_writability();
    bool update_check_writability();
    bool _writable;
    uint32_t last_writable_check_time_ms;

    // log-reading state:
    int _read_fd = -1;
    uint16_t _read_fd_log_num;
    uint32_t _read_offset;
    void get_log_boundaries(const uint16_t list_entry,
                            uint32_t & start_page,
                            uint32_t & end_page) override;

    uint16_t log_num_from_list_entry(const uint16_t list_entry);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override;

    volatile uint32_t _open_error_ms;

    bool file_exists(const char *filename) const;
    bool log_exists(const uint16_t lognum) const;

    void ensure_log_directory_exists();

    uint32_t _get_log_size(const uint16_t log_num);
    uint32_t _get_log_time(const uint16_t log_num);

    // possibly time-consuming preparations handling
    void Prep_MinSpace();
    bool space_cleared;

    void retry_logging_open();
    void handle_write_buffer();

    // free-space checks; filling up SD cards under NuttX leads to
    // corrupt filesystems which cause loss of data, failure to gather
    // data and failures-to-boot.
    uint32_t _free_space_last_check_time; // milliseconds
    const uint32_t _free_space_check_interval = 1000UL; // milliseconds
    const uint32_t _free_space_min_avail = 8388608; // bytes

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // flag to indicate we need to adjust timestamp on currently open
    // file should we receive a real-time-clock value
    bool _need_rtc_update;
#endif
};

class AP_Logger_File : public AP_Logger_Backend
{
public:
    // constructor
    AP_Logger_File(AP_Logger &front, LoggerMessageWriter_DFLogStart *);

    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return new AP_Logger_File(front, ls);
    }

    // initialisation
    void Init() override;
    bool CardInserted(void) const override;

    /* Write a block of data at current offset */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    uint32_t bufferspace_available() override;

    void periodic_1Hz() override;
    void periodic_fullrate() override;

    // this method is used when reporting system status over mavlink
    bool logging_failed() const override;

protected:

    bool WritesOK() const override;
    bool StartNewLogOK() const override;

private:

    bool io_thread_alive() const;
    uint8_t io_thread_warning_decimation_counter;

    void stop_logging(void) override;

    uint32_t last_messagewrite_message_sent;

    // not actually a thread, but rather an object with methods called
    // on a thread:
    LoggerBackendThread_File _iothread_file;
};

#endif // HA_LOGGING_FILESYSTEM_ENABLED
