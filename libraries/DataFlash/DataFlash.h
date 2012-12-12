/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

#include <stdint.h>

#define DF_OVERWRITE_DATA 1 // 0: When reach the end page stop, 1: Start overwriting from page 1

// the last page holds the log format in first 4 bytes. Please change
// this if (and only if!) the low level format changes
#define DF_LOGGING_FORMAT    0x28122011

// we use an invalie logging format to test the chip erase
#define DF_LOGGING_FORMAT_INVALID   0x28122012

class DataFlash_Class
{
private:
    // DataFlash Log variables...
    uint8_t df_BufferNum;
    uint8_t df_Read_BufferNum;
    uint16_t df_BufferIdx;
    uint16_t df_Read_BufferIdx;
    uint16_t df_PageAdr;
    uint16_t df_Read_PageAdr;
    uint8_t df_Stop_Write;
    uint16_t df_FileNumber;
    uint16_t df_FilePage;

    virtual void WaitReady() = 0;
    virtual void BufferWrite (uint8_t BufferNum, uint16_t IntPageAdr, uint8_t Data) = 0;
    virtual void BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait) = 0;
    virtual void PageToBuffer(uint8_t BufferNum, uint16_t PageAdr) = 0;
    virtual uint8_t BufferRead (uint8_t BufferNum, uint16_t IntPageAdr) = 0;
    virtual void PageErase(uint16_t PageAdr) = 0;
    virtual void BlockErase(uint16_t BlockAdr) = 0;
    virtual void ChipErase() = 0;

    // internal high level functions
    int16_t find_last_page(void);
    int16_t find_last_page_of_log(uint16_t log_number);
    bool check_wrapped(void);

public:
    uint8_t df_manufacturer;
    uint16_t df_device;
    uint16_t df_PageSize;

    DataFlash_Class() {
    }                        // Constructor

    virtual void Init(void) = 0;
    virtual void ReadManufacturerID() = 0;
    virtual bool CardInserted(void) = 0;

    int16_t GetPage(void);
    int16_t GetWritePage(void);

    // erase handling
    void EraseAll();
    bool NeedErase(void);

    // Write methods
    void StartWrite(int16_t PageAdr);
    void FinishWrite(void);
    void WriteByte(uint8_t data);
    void WriteInt(int16_t data);
    void WriteLong(int32_t data);

    // Read methods
    void StartRead(int16_t PageAdr);
    uint8_t ReadByte();
    int16_t ReadInt();
    int32_t ReadLong();

    // file numbers
    void SetFileNumber(uint16_t FileNumber);
    uint16_t GetFileNumber();
    uint16_t GetFilePage();

    // page handling
    uint16_t df_NumPages;

    // high level interface
    int16_t find_last_log(void);
    void get_log_boundaries(uint8_t log_num, int16_t & start_page, int16_t & end_page);
    uint8_t get_num_logs(void);
    void start_new_log(void);

};

#include "DataFlash_APM1.h"
#include "DataFlash_APM2.h"
#include "DataFlash_SITL.h"

#endif
