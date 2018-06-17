/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT && (defined(BOARD_SDCARD_NAME) || defined(BOARD_DATAFLASH_FATFS))

#include <AP_HAL/utility/RingBuffer.h>
#include "DataFlash_Backend.h"

#include <sd/SD.h>

class DataFlash_File : public DataFlash_Backend
{
public:
    // constructor
    DataFlash_File(DataFlash_Class &front,
                   DFMessageWriter_DFLogStart *,
                   const char *log_directory);

    // initialisation
    void Init() override;
    bool CardInserted(void) const;

    // erase handling
    void EraseAll();

    // possibly time-consuming preparation handling:
    bool NeedPrep();
    void Prep();

    /* Write a block of data at current offset */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical);
    uint32_t bufferspace_available();

    // high level interface
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs() override;
    uint16_t start_new_log(void) override;

    void periodic_1Hz(const uint32_t now) override;
    void periodic_fullrate(const uint32_t now);

    // this method is used when reporting system status over mavlink
    bool logging_enabled() const;
    bool logging_failed() const;

    void vehicle_was_disarmed() override;

    bool logging_started(void) const override { return !(!(_write_fd)); }

    void PrepForArming();

protected:
    bool WritesOK() const override;
    bool StartNewLogOK() const override;

private:
    File _write_fd;
    File _read_fd;
    uint16_t _read_fd_log_num;
    uint32_t _read_offset;
    uint32_t _write_offset;
    volatile bool _open_error;
    const char *_log_directory;

    uint32_t _io_timer_heartbeat;
    bool io_thread_alive() const;

    uint16_t _cached_oldest_log;
    uint16_t _last_oldest_log;

    uint16_t _log_num_from_list_entry(const uint16_t list_entry);

    // possibly time-consuming preparations handling
    void Prep_MinSpace();
    uint16_t find_oldest_log();

    bool file_exists(const char *filename) const;
    bool log_exists(const uint16_t lognum) const;

    const float min_avail_space_percent = 10.0f;

    // write buffer
    ByteBuffer _writebuf;
    const uint16_t _writebuf_chunk;
    uint32_t _last_write_time;

    /* construct a file name given a log number. Caller must free. */
    char *_log_file_name(const uint16_t log_num) const;
    char *_lastlog_file_name() const;
    uint32_t _get_log_size(const uint16_t log_num) const;
    uint32_t _get_log_time(const uint16_t log_num) const;

    void stop_logging(void);

    void _io_timer(void);

    uint32_t critical_message_reserved_space() const {
        // possibly make this a proportional to buffer size?
        uint32_t ret = 1024;
        if (ret > _writebuf.get_size()) {
            // in this case you will only get critical messages
            ret = _writebuf.get_size();
        }
        return ret;
    };
    uint32_t non_messagewriter_message_reserved_space() const {
        // possibly make this a proportional to buffer size?
        uint32_t ret = 1024;
        if (ret >= _writebuf.get_size()) {
            // need to allow messages out from the messagewriters.  In
            // this case while you have a messagewriter you won't get
            // any other messages.  This should be a corner case!
            ret = 0;
        }
        return ret;
    };

    float avail_space_percent(uint32_t *free = NULL);

    AP_HAL::Semaphore *semaphore;

    bool has_data;

#define Daysto32(year, mon)     (((year - 1) / 4) + MONTAB(year)[mon])
 
/////////////////////////////////////////////////////////////////////
 
    static uint32_t to_timestamp(const struct tm *t);

    bool _busy;    
};

#endif
