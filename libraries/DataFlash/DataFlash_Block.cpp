/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       DataFlash.cpp - DataFlash log library generic code
 */

#include <AP_HAL.h>
#include "DataFlash.h"

extern AP_HAL::HAL& hal;

// the last page holds the log format in first 4 bytes. Please change
// this if (and only if!) the low level format changes
#define DF_LOGGING_FORMAT    0x28122013

// *** DATAFLASH PUBLIC FUNCTIONS ***
void DataFlash_Block::StartWrite(uint16_t PageAdr)
{
    df_BufferIdx  = 0;
    df_BufferNum  = 0;
    df_PageAdr    = PageAdr;
    WaitReady();
}

void DataFlash_Block::FinishWrite(void)
{
    // Write Buffer to flash, NO WAIT
    BufferToPage(df_BufferNum, df_PageAdr, 0);      
    df_PageAdr++;
    // If we reach the end of the memory, start from the begining    
    if (df_PageAdr > df_NumPages)
        df_PageAdr = 1;

    // switch buffer
    df_BufferNum ^= 1;
    df_BufferIdx = 0;
}

void DataFlash_Block::WriteBlock(const void *pBuffer, uint16_t size)
{
    if (!CardInserted()) {
        return;
    }
    while (size > 0) {
        uint16_t n = df_PageSize - df_BufferIdx;
        if (n > size) {
            n = size;
        }

        if (df_BufferIdx == 0) {
            // if we are at the start of a page we need to insert a
            // page header
            if (n > df_PageSize - sizeof(struct PageHeader)) {
                n -= sizeof(struct PageHeader);
            }
            struct PageHeader ph = { df_FileNumber, df_FilePage };
            BlockWrite(df_BufferNum, df_BufferIdx, &ph, sizeof(ph), pBuffer, n);
            df_BufferIdx += n + sizeof(ph);
        } else {
            BlockWrite(df_BufferNum, df_BufferIdx, NULL, 0, pBuffer, n);
            df_BufferIdx += n;
        }

        size -= n;
        pBuffer = (const void *)(n + (uintptr_t)pBuffer);

        if (df_BufferIdx == df_PageSize) {
            FinishWrite();
            df_FilePage++;
        }
    }
}


// Get the last page written to
uint16_t DataFlash_Block::GetWritePage()
{
    return df_PageAdr;
}

// Get the last page read
uint16_t DataFlash_Block::GetPage()
{
    return df_Read_PageAdr;
}

void DataFlash_Block::StartRead(uint16_t PageAdr)
{
    df_Read_BufferNum = 0;
    df_Read_PageAdr   = PageAdr;

    WaitReady();

    // copy flash page to buffer
    PageToBuffer(df_Read_BufferNum, df_Read_PageAdr);

    // We are starting a new page - read FileNumber and FilePage
    struct PageHeader ph;
    BlockRead(df_Read_BufferNum, 0, &ph, sizeof(ph));
    df_FileNumber = ph.FileNumber;
    df_FilePage   = ph.FilePage;
    df_Read_BufferIdx = sizeof(ph);
}

void DataFlash_Block::ReadBlock(void *pBuffer, uint16_t size)
{
    while (size > 0) {
        uint16_t n = df_PageSize - df_Read_BufferIdx;
        if (n > size) {
            n = size;
        }

        WaitReady();

        BlockRead(df_Read_BufferNum, df_Read_BufferIdx, pBuffer, n);
        size -= n;
        pBuffer = (void *)(n + (uintptr_t)pBuffer);
        
        df_Read_BufferIdx += n;

        if (df_Read_BufferIdx == df_PageSize) {
            df_Read_PageAdr++;
            if (df_Read_PageAdr > df_NumPages) {
                df_Read_PageAdr = 1;
            }
            PageToBuffer(df_Read_BufferNum, df_Read_PageAdr);

            // We are starting a new page - read FileNumber and FilePage
            struct PageHeader ph;
            BlockRead(df_Read_BufferNum, 0, &ph, sizeof(ph));
            df_FileNumber = ph.FileNumber;
            df_FilePage   = ph.FilePage;

            df_Read_BufferIdx = sizeof(ph);
        }
    }
}

void DataFlash_Block::SetFileNumber(uint16_t FileNumber)
{
    df_FileNumber = FileNumber;
    df_FilePage = 1;
}

uint16_t DataFlash_Block::GetFileNumber()
{
    return df_FileNumber;
}

uint16_t DataFlash_Block::GetFilePage()
{
    return df_FilePage;
}

void DataFlash_Block::EraseAll()
{
    for (uint16_t j = 1; j <= (df_NumPages+1)/8; j++) {
        BlockErase(j);
        if (j%6 == 0) {
            hal.scheduler->delay(6);
        }
    }
    // write the logging format in the last page
    hal.scheduler->delay(100);
    StartWrite(df_NumPages+1);
    uint32_t version = DF_LOGGING_FORMAT;
    WriteBlock(&version, sizeof(version));
    FinishWrite();
    hal.scheduler->delay(100);
}

/*
 *  we need to erase if the logging format has changed
 */
bool DataFlash_Block::NeedErase(void)
{
    uint32_t version;
    StartRead(df_NumPages+1);
    ReadBlock(&version, sizeof(version));
    return version != DF_LOGGING_FORMAT;
}
