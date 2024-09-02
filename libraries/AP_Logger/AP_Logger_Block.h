/*
   AP_Logger logging - block oriented variant
 */
#pragma once

#include "AP_Logger_Backend.h"

#if HAL_LOGGING_BLOCK_ENABLED

#define BLOCK_LOG_VALIDATE 0

class AP_Logger_Block : public AP_Logger_Backend {
public:
    AP_Logger_Block(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer);

    virtual void Init(void) override;
    virtual bool CardInserted(void) const override = 0;

    // erase handling
    void EraseAll() override;

    // high level interface
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t list_entry, uint32_t & start_page, uint32_t & end_page) override;
    void get_log_info(uint16_t list_entry, uint32_t &size, uint32_t &time_utc) override;
    int16_t get_log_data(uint16_t list_entry, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override WARN_IF_UNUSED;
    void end_log_transfer() override { }
    uint16_t get_num_logs() override;
    void start_new_log(void) override;
    uint32_t bufferspace_available() override;
    void stop_logging(void) override;
    void stop_logging_async(void) override;
    bool logging_failed() const override;
    bool logging_started(void) const override { return log_write_started; }
    void io_timer(void) override;

protected:
    /* Write a block of data at current offset */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    void periodic_1Hz() override;
    void periodic_10Hz(const uint32_t now) override;
    bool WritesOK() const override;

    // get the current sector from the current page
    uint32_t get_sector(uint32_t current_page) const {
        return ((current_page - 1) / df_PagePerSector);
    }

    // get the current block from the current page
    uint32_t get_block(uint32_t current_page) const {
        return ((current_page - 1) / df_PagePerBlock);
    }

    // number of bytes in a page
    uint32_t df_PageSize;
    // number of pages in a (generally 64k) block
    uint16_t df_PagePerBlock;
    // number of pages in a (generally 4k) sector
    uint16_t df_PagePerSector;
    // number of pages on the chip
    uint32_t df_NumPages;
    volatile bool log_write_started;

    uint8_t *buffer;
    uint32_t last_messagewrite_message_sent;
    uint32_t df_Read_PageAdr;

private:
    /*
      functions implemented by the board specific backends
     */
    virtual void BufferToPage(uint32_t PageAdr) = 0;
    virtual void PageToBuffer(uint32_t PageAdr) = 0;
    virtual void SectorErase(uint32_t SectorAdr) = 0;
    virtual void Sector4kErase(uint32_t SectorAdr) = 0;
    virtual void StartErase() = 0;
    virtual bool InErase() = 0;
    void         flash_test(void);

    struct PACKED PageHeader {
        uint32_t FilePage;
        uint16_t FileNumber;
#if BLOCK_LOG_VALIDATE
        uint32_t crc;
#endif
    };

    struct PACKED FileHeader {
        uint32_t utc_secs;
    };

    // semaphore to mediate access to the chip
    HAL_Semaphore sem;
    // semaphore to mediate access to the ring buffer
    HAL_Semaphore write_sem;
    ByteBuffer writebuf;

    // state variables
    uint16_t df_Read_BufferIdx;
    uint32_t df_PageAdr;    // current page address for writes
    // file numbers
    uint16_t df_FileNumber;
    uint16_t df_Write_FileNumber;
    uint32_t df_FileTime;
    // relative page index of the current read/write file starting at 1
    uint32_t df_FilePage;
    uint32_t df_Write_FilePage;
    // page to wipe from in the case of corruption
    uint32_t df_EraseFrom;

    // offset from adding FMT messages to log data
    bool adding_fmt_headers;

    // are we waiting on an erase to finish?
    volatile bool erase_started;
    // were we logging before the erase started?
    volatile bool new_log_pending;
    // have we been asked to stop logging safely?
    volatile bool stop_log_pending;
    // latch to make sure we only write out the full message once
    volatile bool chip_full;
    // io thread health
    volatile uint32_t io_timer_heartbeat;
    uint8_t warning_decimation_counter;

    volatile enum class StatusMessage {
        NONE,
        ERASE_COMPLETE,
        RECOVERY_COMPLETE,
    } status_msg;

    // read size bytes of data to a page. The caller must ensure that
    // the data fits within the page, otherwise it will wrap to the
    // start of the page
    bool BlockRead(uint16_t IntPageAdr, void *pBuffer, uint16_t size);

    // erase handling
    bool NeedErase(void);
    void validate_log_structure();

    // internal high level functions
    int16_t get_log_data_raw(uint16_t log_num, uint32_t page, uint32_t offset, uint16_t len, uint8_t *data) WARN_IF_UNUSED;
    // read from the page address and return the file number at that location
    uint16_t StartRead(uint32_t PageAdr);
    // read the headers at the current read point returning the file number
    uint16_t ReadHeaders();
    uint32_t find_last_page(void);
    uint32_t find_last_page_of_log(uint16_t log_number);
    bool is_wrapped(void);
    void StartWrite(uint32_t PageAdr);
    void FinishWrite(void);

    // Read methods
    bool ReadBlock(void *pBuffer, uint16_t size);

    void StartLogFile(uint16_t FileNumber);
    // file numbers
    uint16_t GetFileNumber() const;

    void _print_log_formats(AP_HAL::BetterStream *port);

    // callback on IO thread
    bool io_thread_alive() const;
    void write_log_page();
};

#endif  // HAL_LOGGING_BLOCK_ENABLED
