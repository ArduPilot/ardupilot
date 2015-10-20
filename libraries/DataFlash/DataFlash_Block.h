/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - block oriented variant
 */

#ifndef DataFlash_block_h
#define DataFlash_block_h

#include "DataFlash_Backend.h"

#include <stdint.h>

class DataFlash_Block : public DataFlash_Backend
{
public:
    DataFlash_Block(DataFlash_Class &front) :
        DataFlash_Backend(front) { }

    // initialisation
    virtual void Init(const struct LogStructure *structure, uint8_t num_types) = 0;
    virtual bool CardInserted(void) = 0;

    // erase handling
    void EraseAll();

    bool NeedPrep(void);
    void Prep();

    /* Write a block of data at current offset */
    bool WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical);

    // high level interface
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data_raw(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs() override;
    uint16_t start_new_log(void);
#ifndef DATAFLASH_NO_CLI
    void LogReadProcess(const uint16_t list_entry,
                        uint16_t start_page, uint16_t end_page, 
                        print_mode_fn print_mode,
                        AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);
#endif

    uint16_t bufferspace_available();

private:
    struct PageHeader {
        uint16_t FileNumber;
        uint16_t FilePage;
    };

    // DataFlash Log variables...
    uint8_t df_BufferNum;
    uint8_t df_Read_BufferNum;
    uint16_t df_BufferIdx;
    uint16_t df_Read_BufferIdx;
    uint16_t df_PageAdr;
    uint16_t df_Read_PageAdr;
    uint16_t df_FileNumber;
    uint16_t df_FilePage;

    // offset from adding FMT messages to log data
    bool adding_fmt_headers;

    /*
      functions implemented by the board specific backends
     */
    virtual void WaitReady() = 0;
    virtual void BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait) = 0;
    virtual void PageToBuffer(uint8_t BufferNum, uint16_t PageAdr) = 0;
    virtual void PageErase(uint16_t PageAdr) = 0;
    virtual void BlockErase(uint16_t BlockAdr) = 0;
    virtual void ChipErase() = 0;

    // write size bytes of data to a page. The caller must ensure that
    // the data fits within the page, otherwise it will wrap to the
    // start of the page
    virtual void BlockWrite(uint8_t BufferNum, uint16_t IntPageAdr, 
                            const void *pHeader, uint8_t hdr_size,
                            const void *pBuffer, uint16_t size) = 0;
    
    // read size bytes of data to a page. The caller must ensure that
    // the data fits within the page, otherwise it will wrap to the
    // start of the page
    virtual bool BlockRead(uint8_t BufferNum, uint16_t IntPageAdr, void *pBuffer, uint16_t size) = 0;

    // erase handling
    bool NeedErase(void);

    // internal high level functions
    void StartRead(uint16_t PageAdr);
    uint16_t find_last_page(void);
    uint16_t find_last_page_of_log(uint16_t log_number);
    bool check_wrapped(void);
    uint16_t GetPage(void);
    uint16_t GetWritePage(void);
    void StartWrite(uint16_t PageAdr);
    void FinishWrite(void);

    // Read methods
    bool ReadBlock(void *pBuffer, uint16_t size);

    // file numbers
    void SetFileNumber(uint16_t FileNumber);
    uint16_t GetFilePage();
    uint16_t GetFileNumber();

    void _print_log_formats(AP_HAL::BetterStream *port);
    
protected:
    uint8_t df_manufacturer;
    uint16_t df_device;

    // page handling
    uint16_t df_PageSize;
    uint16_t df_NumPages;

    virtual void ReadManufacturerID() = 0;
};


#include "DataFlash_APM1.h"
#include "DataFlash_APM2.h"
#include "DataFlash_SITL.h"
#include "DataFlash_Empty.h"

#endif // DataFlash_block_h

