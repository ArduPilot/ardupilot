/*
   AP_Logger logging - block oriented variant
 */
#pragma once

#include "AP_Logger_Backend.h"

class AP_Logger_Block : public AP_Logger_Backend {
public:
    AP_Logger_Block(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer);

    virtual void Init(void) override;
    virtual bool CardInserted(void) const override = 0;

    // erase handling
    void EraseAll() override;

    bool NeedPrep(void) override;
    void Prep() override;
    void PrepForArming() override;

    // high level interface
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) override;
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override;
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override;
    uint16_t get_num_logs() override;
    uint16_t start_new_log(void) override;
    uint32_t bufferspace_available() override;
    void stop_logging(void) override { log_write_started = false; }
    bool logging_enabled() const override { return true; }
    bool logging_failed() const override { return false; }
    bool logging_started(void) const override { return log_write_started; }

protected:
    /* Write a block of data at current offset */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;

private:
    /*
      functions implemented by the board specific backends
     */
    virtual void BufferToPage(uint32_t PageAdr) = 0;
    virtual void PageToBuffer(uint32_t PageAdr) = 0;
    virtual void SectorErase(uint32_t PageAdr) = 0;
    virtual void StartErase() = 0;
    virtual bool InErase() = 0;

    struct PageHeader {
        uint16_t FileNumber;
        uint16_t FilePage;
    };

    HAL_Semaphore_Recursive sem;
    ByteBuffer writebuf;

    // state variables
    uint16_t df_Read_BufferIdx;
    uint32_t df_PageAdr;
    uint32_t df_Read_PageAdr;
    uint16_t df_FileNumber;
    uint32_t df_FilePage;

    // offset from adding FMT messages to log data
    bool adding_fmt_headers;

    // are we waiting on an erase to finish?
    bool erase_started;

    // read size bytes of data to a page. The caller must ensure that
    // the data fits within the page, otherwise it will wrap to the
    // start of the page
    bool BlockRead(uint16_t IntPageAdr, void *pBuffer, uint16_t size);

    // erase handling
    bool NeedErase(void);

    // internal high level functions
    int16_t get_log_data_raw(uint16_t log_num, uint32_t page, uint32_t offset, uint16_t len, uint8_t *data);
    void StartRead(uint32_t PageAdr);
    uint32_t find_last_page(void);
    uint32_t find_last_page_of_log(uint16_t log_number);
    bool check_wrapped(void);
    void StartWrite(uint32_t PageAdr);
    void FinishWrite(void);

    // Read methods
    bool ReadBlock(void *pBuffer, uint16_t size);

    // file numbers
    void SetFileNumber(uint16_t FileNumber);
    uint16_t GetFileNumber();

    void _print_log_formats(AP_HAL::BetterStream *port);

    // callback on IO thread
    void io_timer(void);

protected:
    // page handling
    uint32_t df_PageSize;
    uint16_t df_PagePerSector;
    uint32_t df_NumPages;
    bool log_write_started;

    static const uint16_t page_size_max = 256;
    uint8_t *buffer;

    bool WritesOK() const override;
};
