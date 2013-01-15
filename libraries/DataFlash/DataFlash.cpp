/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       DataFlash.cpp - DataFlash log library generic code
 */

#include <AP_HAL.h>
#include "DataFlash.h"

extern AP_HAL::HAL& hal;

// *** DATAFLASH PUBLIC FUNCTIONS ***
void DataFlash_Class::StartWrite(int16_t PageAdr)
{
    df_BufferIdx  = 0;
    df_BufferNum  = 0;
    df_PageAdr    = PageAdr;
    WaitReady();
}

void DataFlash_Class::FinishWrite(void)
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

void DataFlash_Class::WriteBlock(const void *pBuffer, uint16_t size)
{
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


void DataFlash_Class::WriteByte(uint8_t data)
{
    WriteBlock(&data, sizeof(data));
}

void DataFlash_Class::WriteInt(int16_t data)
{
    WriteBlock(&data, sizeof(data));
}

void DataFlash_Class::WriteLong(int32_t data)
{
    WriteBlock(&data, sizeof(data));
}

// Get the last page written to
int16_t DataFlash_Class::GetWritePage()
{
    return df_PageAdr;
}

// Get the last page read
int16_t DataFlash_Class::GetPage()
{
    return df_Read_PageAdr;
}

void DataFlash_Class::StartRead(int16_t PageAdr)
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

void DataFlash_Class::ReadBlock(void *pBuffer, uint16_t size)
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

uint8_t DataFlash_Class::ReadByte()
{
    uint8_t result = 0;
    ReadBlock(&result, sizeof(result));
    return result;
}

int16_t DataFlash_Class::ReadInt()
{
    int16_t result;
    ReadBlock(&result, sizeof(result));
    return result;
}

int32_t DataFlash_Class::ReadLong()
{
    int32_t result;
    ReadBlock(&result, sizeof(result));
    return result;
}

void DataFlash_Class::SetFileNumber(uint16_t FileNumber)
{
    df_FileNumber = FileNumber;
    df_FilePage = 1;
}

uint16_t DataFlash_Class::GetFileNumber()
{
    return df_FileNumber;
}

uint16_t DataFlash_Class::GetFilePage()
{
    return df_FilePage;
}

void DataFlash_Class::EraseAll()
{
    for(uint16_t j = 1; j <= (df_NumPages+1)/8; j++) {
        BlockErase(j);
        if (j%6 == 0) {
            hal.scheduler->delay(6);
        }
    }
    // write the logging format in the last page
    StartWrite(df_NumPages+1);
    WriteLong(DF_LOGGING_FORMAT);
    FinishWrite();
}

/*
 *  we need to erase if the logging format has changed
 */
bool DataFlash_Class::NeedErase(void)
{
    StartRead(df_NumPages+1);
    return ReadLong() != DF_LOGGING_FORMAT;
}

