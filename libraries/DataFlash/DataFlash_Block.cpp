/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       DataFlash.cpp - DataFlash log library generic code
 */

#include "DataFlash_Block.h"

#include <AP_HAL/AP_HAL.h>

extern AP_HAL::HAL& hal;

// the last page holds the log format in first 4 bytes. Please change
// this if (and only if!) the low level format changes
#define DF_LOGGING_FORMAT    0x28122013

uint16_t DataFlash_Block::bufferspace_available()
{
    // because DataFlash_Block devices are ring buffers, we *always*
    // have room...
    return df_NumPages * df_PageSize; 
}

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

bool DataFlash_Block::WritePrioritisedBlock(const void *pBuffer, uint16_t size,
    bool is_critical)
{
    // is_critical is ignored - we're a ring buffer and never run out
    // of space.  possibly if we do more complicated bandwidth
    // limiting we can reservice bandwidth based on is_critical
    if (!CardInserted() || !log_write_started || !_writes_enabled) {
        return false;
    }

    if (! WriteBlockCheckStartupMessages()) {
        return false;
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
                n = df_PageSize - sizeof(struct PageHeader);
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

    return true;
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

    // disable writing while reading
    log_write_started = false;

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

bool DataFlash_Block::ReadBlock(void *pBuffer, uint16_t size)
{
    while (size > 0) {
        uint16_t n = df_PageSize - df_Read_BufferIdx;
        if (n > size) {
            n = size;
        }

        WaitReady();

        if (!BlockRead(df_Read_BufferNum, df_Read_BufferIdx, pBuffer, n)) {
            return false;
        }
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
            if (!BlockRead(df_Read_BufferNum, 0, &ph, sizeof(ph))) {
                return false;
            }
            df_FileNumber = ph.FileNumber;
            df_FilePage   = ph.FilePage;

            df_Read_BufferIdx = sizeof(ph);
        }
    }
    return true;
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
    log_write_started = false;
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
    log_write_started = true;
    _writes_enabled = true;
    WriteBlock(&version, sizeof(version));
    log_write_started = false;
    FinishWrite();
    hal.scheduler->delay(100);
}

bool DataFlash_Block::NeedPrep(void)
{
    return NeedErase();
}

void DataFlash_Block::Prep()
{
    if (hal.util->get_soft_armed()) {
        // do not want to do any filesystem operations while we are e.g. flying
        return;
    }
    if (NeedErase()) {
        EraseAll();
    }
}

/*
 *  we need to erase if the logging format has changed
 */
bool DataFlash_Block::NeedErase(void)
{
    uint32_t version = 0;
    StartRead(df_NumPages+1);
    if (!ReadBlock(&version, sizeof(version))) {
        return true;
    }
    StartRead(1);
    return version != DF_LOGGING_FORMAT;
}

/**
  get raw data from a log
 */
int16_t DataFlash_Block::get_log_data_raw(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    uint16_t data_page_size = df_PageSize - sizeof(struct PageHeader);

    if (offset >= data_page_size) {
        page += offset / data_page_size;
        offset = offset % data_page_size;
        if (page > df_NumPages) {
            // pages are one based, not zero
            page = 1 + page - df_NumPages;
        }
    }
    if (log_write_started || df_Read_PageAdr != page) {
        StartRead(page);
    }

    df_Read_BufferIdx = offset + sizeof(struct PageHeader);
    if (!ReadBlock(data, len)) {
        return -1;
    }

    return (int16_t)len;
}

/**
  get data from a log, accounting for adding FMT headers
 */
int16_t DataFlash_Block::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    if (offset == 0) {
        uint8_t header[3];
        get_log_data_raw(log_num, page, 0, 3, header);
        adding_fmt_headers = (header[0] != HEAD_BYTE1 || header[1] != HEAD_BYTE2 || header[2] != LOG_FORMAT_MSG);
    }
    uint16_t ret = 0;

    if (adding_fmt_headers) {
        // the log doesn't start with a FMT message, we need to add
        // them
        const uint16_t fmt_header_size = num_types() * sizeof(struct log_Format);
        while (offset < fmt_header_size && len > 0) {
            struct log_Format pkt;
            uint8_t t = offset / sizeof(pkt);
            uint8_t ofs = offset % sizeof(pkt);
            Log_Fill_Format(structure(t), pkt);
            uint8_t n = sizeof(pkt) - ofs;
            if (n > len) {
                n = len;
            }
            memcpy(data, ofs + (uint8_t *)&pkt, n);
            data += n;
            offset += n;
            len -= n;
            ret += n;
        }
        offset -= fmt_header_size;
    }

    if (len > 0) {
        ret += get_log_data_raw(log_num, page, offset, len, data);
    }

    return ret;
}
