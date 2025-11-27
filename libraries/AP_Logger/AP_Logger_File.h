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
#if AP_FILESYSTEM_LITTLEFS_ENABLED
#define HAL_LOGGER_WRITE_CHUNK_SIZE 2048
#elif AP_FILESYSTEM_FATFS_ENABLED
#define HAL_LOGGER_WRITE_CHUNK_SIZE (AP_Filesystem_FATFS::get_io_size())
#else
#define HAL_LOGGER_WRITE_CHUNK_SIZE AP_FATFS_MIN_IO_SIZE
#endif
#endif

class AP_Logger_File : public AP_Logger_Backend
{
public:
    // constructor
    AP_Logger_File(AP_Logger &front,
                   LoggerMessageWriter_DFLogStart *);

    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_File(front, ls);
    }

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
    void end_log_transfer() override;
    uint16_t get_num_logs() override;
    void start_new_log(void) override;
    uint16_t find_oldest_log() override;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    void flush(void) override;
#endif
    void periodic_1Hz() override;
    void periodic_fullrate() override;

    // this method is used for mavlink system status and arming checks
    bool logging_failed() const override;

    bool logging_started(void) const override { return _write_fd != -1; }
    void io_timer(void) override;

protected:

    bool WritesOK() const override;
    bool StartNewLogOK() const override;
    void PrepForArming_start_logging() override;

private:
    int _write_fd = -1;
    char *_write_filename;
    bool last_log_is_marked_discard;
    uint32_t _last_write_ms;
#if AP_RTC_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    bool _need_rtc_update;
#endif
    
    int _read_fd = -1;
    uint16_t _read_fd_log_num;
    uint32_t _read_offset;
    uint32_t _write_offset;
    volatile uint32_t _open_error_ms;
    const char *_log_directory;
    bool _last_write_failed;

    uint32_t _io_timer_heartbeat;
    bool io_thread_alive() const;
    uint8_t io_thread_warning_decimation_counter;

    // do we have a recent open error?
    bool recent_open_error(void) const;

    // possibly time-consuming preparations handling
    void Prep_MinSpace();
    int64_t disk_space_avail();
    int64_t disk_space();

    void ensure_log_directory_exists();

    bool file_exists(const char *filename) const;
    bool log_exists(const uint16_t lognum) const;

    bool dirent_to_log_num(const dirent *de, uint16_t &log_num) const;
    bool write_lastlog_file(uint16_t log_num);

    // write buffer
    ByteBuffer _writebuf{0};
    const uint16_t _writebuf_chunk = HAL_LOGGER_WRITE_CHUNK_SIZE;
    uint32_t _last_write_time;

    /* construct a file name given a log number. Caller must free. */
    char *_log_file_name(const uint16_t log_num) const;
    char *_lastlog_file_name() const;
    uint32_t _get_log_size(const uint16_t log_num);
    uint32_t _get_log_time(const uint16_t log_num);

    void stop_logging(void) override;

    uint32_t last_messagewrite_message_sent;

    // free-space checks; filling up SD cards under NuttX leads to
    // corrupt filesystems which cause loss of data, failure to gather
    // data and failures-to-boot.
    uint32_t _free_space_last_check_time; // milliseconds
    const uint32_t _free_space_check_interval = 1000UL; // milliseconds
#if AP_FILESYSTEM_LITTLEFS_ENABLED
#if AP_FILESYSTEM_LITTLEFS_FLASH_TYPE == AP_FILESYSTEM_FLASH_W25NXX
    const uint32_t _free_space_min_avail = 1024 * 1024; // bytes
#else
    const uint32_t _free_space_min_avail = 1024 * 256; // bytes
#endif
#else
    const uint32_t _free_space_min_avail = 8388608; // bytes
#endif

    // semaphore mediates access to the ringbuffer
    HAL_Semaphore semaphore;
    // write_fd_semaphore mediates access to write_fd so the frontend
    // can open/close files without causing the backend to write to a
    // bad fd
    HAL_Semaphore write_fd_semaphore;

    // async erase state
    struct {
        bool was_logging;
        uint16_t log_num;
    } erase;
    void erase_next(void);

    const char *last_io_operation = "";

    bool start_new_log_pending;
};

#endif // HAL_LOGGING_FILESYSTEM_ENABLED
