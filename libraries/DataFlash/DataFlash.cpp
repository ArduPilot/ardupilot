/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       DataFlash.cpp - DataFlash log library generic code
 */

#include <stdint.h>
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
    return df_Read_PageAdr-1;
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



// This function determines the number of whole or partial log files in the DataFlash
// Wholly overwritten files are (of course) lost.
uint8_t DataFlash_Class::get_num_logs(void)
{
    uint16_t lastpage;
    uint16_t last;
    uint16_t first;

    if (find_last_page() == 1) {
        return 0;
    }

    StartRead(1);

    if (GetFileNumber() == 0XFFFF) {
        return 0;
    }

    lastpage = find_last_page();
    StartRead(lastpage);
    last = GetFileNumber();
    StartRead(lastpage + 2);
    first = GetFileNumber();
    if(first > last) {
        StartRead(1);
        first = GetFileNumber();
    }
    if(last == first)
    {
        return 1;
    } else {
        return (last - first + 1);
    }
}

// This function starts a new log file in the DataFlash
void DataFlash_Class::start_new_log(void)
{
    uint16_t last_page = find_last_page();

    StartRead(last_page);
    //Serial.print("last page: ");	Serial.println(last_page);
    //Serial.print("file #: ");	Serial.println(GetFileNumber());
    //Serial.print("file page: ");	Serial.println(GetFilePage());

    if(find_last_log() == 0 || GetFileNumber() == 0xFFFF) {
        SetFileNumber(1);
        StartWrite(1);
        //Serial.println("start log from 0");
        return;
    }

    // Check for log of length 1 page and suppress
    if(GetFilePage() <= 1) {
        SetFileNumber(GetFileNumber());                 // Last log too short, reuse its number
        StartWrite(last_page);                                          // and overwrite it
        //Serial.println("start log from short");
    } else {
        if(last_page == 0xFFFF) last_page=0;
        SetFileNumber(GetFileNumber()+1);
        StartWrite(last_page + 1);
        //Serial.println("start log normal");
    }
}

// This function finds the first and last pages of a log file
// The first page may be greater than the last page if the DataFlash has been filled and partially overwritten.
void DataFlash_Class::get_log_boundaries(uint8_t log_num, int16_t & start_page, int16_t & end_page)
{
    int16_t num = get_num_logs();
    int16_t look;

    if(num == 1)
    {
        StartRead(df_NumPages);
        if(GetFileNumber() == 0xFFFF)
        {
            start_page = 1;
            end_page = find_last_page_of_log((uint16_t)log_num);
        } else {
            end_page = find_last_page_of_log((uint16_t)log_num);
            start_page = end_page + 1;
        }

    } else {
        if(log_num==1) {
            StartRead(df_NumPages);
            if(GetFileNumber() == 0xFFFF) {
                start_page = 1;
            } else {
                start_page = find_last_page() + 1;
            }
        } else {
            if(log_num == find_last_log() - num + 1) {
                start_page = find_last_page() + 1;
            } else {
                look = log_num-1;
                do {
                    start_page = find_last_page_of_log(look) + 1;
                    look--;
                } while (start_page <= 0 && look >=1);
            }
        }
    }
    if(start_page == (int16_t)df_NumPages+1 || start_page == 0) start_page=1;
    end_page = find_last_page_of_log((uint16_t)log_num);
    if(end_page <= 0) end_page = start_page;
}

bool DataFlash_Class::check_wrapped(void)
{
    StartRead(df_NumPages);
    if(GetFileNumber() == 0xFFFF)
        return 0;
    else
        return 1;
}


// This funciton finds the last log number
int16_t DataFlash_Class::find_last_log(void)
{
    int16_t last_page = find_last_page();
    StartRead(last_page);
    return GetFileNumber();
}

// This function finds the last page of the last file
int16_t DataFlash_Class::find_last_page(void)
{
    uint16_t look;
    uint16_t bottom = 1;
    uint16_t top = df_NumPages;
    uint32_t look_hash;
    uint32_t bottom_hash;
    uint32_t top_hash;

    StartRead(bottom);
    bottom_hash = ((int32_t)GetFileNumber()<<16) | GetFilePage();

    while(top-bottom > 1) {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int32_t)GetFileNumber()<<16 | GetFilePage();
        if (look_hash >= 0xFFFF0000) look_hash = 0;

        if(look_hash < bottom_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
            bottom_hash = look_hash;
        }
    }

    StartRead(top);
    top_hash = ((int32_t)GetFileNumber()<<16) | GetFilePage();
    if (top_hash >= 0xFFFF0000) {
        top_hash = 0;
    }
    if (top_hash > bottom_hash) {
        return top;
    }

    return bottom;
}

// This function finds the last page of a particular log file
int16_t DataFlash_Class::find_last_page_of_log(uint16_t log_number)
{
    uint16_t look;
    uint16_t bottom;
    uint16_t top;
    uint32_t look_hash;
    uint32_t check_hash;

    if(check_wrapped())
    {
        StartRead(1);
        bottom = GetFileNumber();
        if (bottom > log_number)
        {
            bottom = find_last_page();
            top = df_NumPages;
        } else {
            bottom = 1;
            top = find_last_page();
        }
    } else {
        bottom = 1;
        top = find_last_page();
    }

    check_hash = (int32_t)log_number<<16 | 0xFFFF;

    while(top-bottom > 1)
    {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int32_t)GetFileNumber()<<16 | GetFilePage();
        if (look_hash >= 0xFFFF0000) look_hash = 0;

        if(look_hash > check_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
        }
    }

    StartRead(top);
    if (GetFileNumber() == log_number) return top;

    StartRead(bottom);
    if (GetFileNumber() == log_number) return bottom;

    return -1;
}
