/*
  block based logging, for boards with flash logging
 */

#include "AP_Logger_Block.h"

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern AP_HAL::HAL& hal;

// the last page holds the log format in first 4 bytes. Please change
// this if (and only if!) the low level format changes
#define DF_LOGGING_FORMAT    0x1901201A

AP_Logger_Block::AP_Logger_Block(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
    writebuf(0),
    AP_Logger_Backend(front, writer)
{
    buffer = (uint8_t *)hal.util->malloc_type(page_size_max, AP_HAL::Util::MEM_DMA_SAFE);
    if (buffer == nullptr) {
        AP_HAL::panic("Out of DMA memory for logging");
    }
}

// init is called after backend init
void AP_Logger_Block::Init(void)
{
    if (CardInserted()) {
        // reserve space for version in last sector
        df_NumPages -= df_PagePerBlock;

        // determine and limit file backend buffersize
        uint32_t bufsize = _front._params.file_bufsize;
        if (bufsize > 64) {
            bufsize = 64;
        }
        bufsize *= 1024;

        // If we can't allocate the full size, try to reduce it until we can allocate it
        while (!writebuf.set_size(bufsize) && bufsize >= df_PageSize * df_PagePerBlock) {
            hal.console->printf("AP_Logger_Block: Couldn't set buffer size to=%u\n", (unsigned)bufsize);
            bufsize >>= 1;
        }

        if (!writebuf.get_size()) {
            hal.console->printf("Out of memory for logging\n");
            return;
        }

        hal.console->printf("AP_Logger_Block: buffer size=%u\n", (unsigned)bufsize);
        _initialised = true;
    }

    WITH_SEMAPHORE(sem);

    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Logger_Block::io_timer, void));

    AP_Logger_Backend::Init();
}

uint32_t AP_Logger_Block::bufferspace_available()
{
    // because AP_Logger_Block devices are ring buffers, we *always*
    // have room...
    return df_NumPages * df_PageSize;
}

// *** LOGGER PUBLIC FUNCTIONS ***
void AP_Logger_Block::StartWrite(uint32_t PageAdr)
{
    df_PageAdr    = PageAdr;
    log_write_started = true;
}

void AP_Logger_Block::FinishWrite(void)
{
    // Write Buffer to flash
    BufferToPage(df_PageAdr);
    df_PageAdr++;

    // If we reach the end of the memory, start from the beginning
    if (df_PageAdr > df_NumPages) {
        df_PageAdr = 1;
    }

    // when starting a new sector, erase it
    if ((df_PageAdr-1) % df_PagePerBlock == 0) {
        SectorErase(df_PageAdr / df_PagePerBlock);
    }
}

bool AP_Logger_Block::WritesOK() const
{
    if (!CardInserted()) {
        return false;
    }
    return true;
}

bool AP_Logger_Block::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    // is_critical is ignored - we're a ring buffer and never run out
    // of space.  possibly if we do more complicated bandwidth
    // limiting we can reserve bandwidth based on is_critical
    if (!WritesOK()) {
        return false;
    }

    if (! WriteBlockCheckStartupMessages()) {
        return false;
    }

    if (writebuf.space() < size) {
        // no room in buffer
        return false;
    }

    writebuf.write((uint8_t*)pBuffer, size);
    return true;
}


void AP_Logger_Block::StartRead(uint32_t PageAdr)
{
    df_Read_PageAdr   = PageAdr;

    // copy flash page to buffer
    if (erase_started) {
        memset(buffer, 0xff, df_PageSize);
    } else {
        PageToBuffer(df_Read_PageAdr);
    }

    // We are starting a new page - read FileNumber and FilePage
    struct PageHeader ph;
    BlockRead(0, &ph, sizeof(ph));
    df_FileNumber = ph.FileNumber;
    df_FilePage   = ph.FilePage;
    df_Read_BufferIdx = sizeof(ph);
}

bool AP_Logger_Block::ReadBlock(void *pBuffer, uint16_t size)
{
    if (erase_started) {
        return false;
    }
    while (size > 0) {
        uint16_t n = df_PageSize - df_Read_BufferIdx;
        if (n > size) {
            n = size;
        }

        if (!BlockRead(df_Read_BufferIdx, pBuffer, n)) {
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
            if (erase_started) {
                memset(buffer, 0xff, df_PageSize);
            } else {
                PageToBuffer(df_Read_PageAdr);
            }

            // We are starting a new page - read FileNumber and FilePage
            struct PageHeader ph;
            if (!BlockRead(0, &ph, sizeof(ph))) {
                return false;
            }
            df_FileNumber = ph.FileNumber;
            df_FilePage   = ph.FilePage;

            df_Read_BufferIdx = sizeof(ph);
        }
    }
    return true;
}

void AP_Logger_Block::SetFileNumber(uint16_t FileNumber)
{
    df_FileNumber = FileNumber;
    df_FilePage = 1;
}

uint16_t AP_Logger_Block::GetFileNumber()
{
    return df_FileNumber;
}

void AP_Logger_Block::EraseAll()
{
    WITH_SEMAPHORE(sem);
    if (erase_started) {
        // already erasing
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Chip erase started");
    // reset the format version and wrapped status so that any incomplete erase will be caught
    Sector4kErase(get_sector(df_NumPages));

    log_write_started = false;

    StartErase();
    erase_started = true;
}

bool AP_Logger_Block::NeedPrep(void)
{
    return NeedErase();
}

void AP_Logger_Block::Prep()
{
    WITH_SEMAPHORE(sem);
    if (hal.util->get_soft_armed()) {
        // do not want to do any filesystem operations while we are e.g. flying
        return;
    }
    if (NeedErase()) {
        EraseAll();
    }
    validate_log_structure();
}

/*
 *  we need to erase if the logging format has changed
 */
bool AP_Logger_Block::NeedErase(void)
{
    uint32_t version = 0;
    StartRead(df_NumPages+1); // last page

    BlockRead(0, &version, sizeof(version));
    StartRead(1);
    if (version == DF_LOGGING_FORMAT) {
        return false;
    }
    return true;
}

/*
 * iterate through all of the logs files looking for ones that are corrupted and correct.
 */
void AP_Logger_Block::validate_log_structure()
{
    WITH_SEMAPHORE(sem);
    bool wrapped = check_wrapped();
    uint32_t page = 1;
    uint32_t page_start = 1;

    StartRead(page);
    uint16_t file = GetFileNumber();
    uint16_t first_file = file;
    uint16_t next_file = file;
    uint16_t last_file = 0;

    while (file != 0xFFFF && page <= df_NumPages && (file == next_file || (wrapped && file < next_file))) {
        uint32_t end_page = find_last_page_of_log(file);
        if (end_page == 0 || end_page < page) { // this can happen and may be responsible for corruption that we have seen
            break;
        }
        page = end_page + 1;
        StartRead(page);
        file = GetFileNumber();
        next_file++;
        // skip over the rest of an erased blcok
        if (wrapped && file == 0xFFFF) {
            StartRead((get_block(page) + 1) * df_PagePerBlock + 1);
            file = GetFileNumber();
        }
        if (wrapped && file < next_file) {
            page_start = page;
            next_file = file;
            first_file = file;
        } else if (last_file < next_file) {
            last_file = file;
        }
        if (file == next_file) {
            hal.console->printf("Found complete log %d at %X-%X\n", int(file), unsigned(page), unsigned(find_last_page_of_log(file)));
        }
    }

    if (file != 0xFFFF && file != next_file && page <= df_NumPages && page > 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Found corrupt log %d at 0x%04X, erasing", int(file), unsigned(page));
        df_EraseFrom = page;
    } else if (next_file != 0xFFFF && page > 0 && next_file > 1) { // chip is empty
        gcs().send_text(MAV_SEVERITY_INFO, "Found %d complete logs at 0x%04X-0x%04X", int(next_file - first_file), unsigned(page_start), unsigned(page - 1));
    }
}

/**
  get raw data from a log
 */
int16_t AP_Logger_Block::get_log_data_raw(uint16_t log_num, uint32_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    WITH_SEMAPHORE(sem);
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
int16_t AP_Logger_Block::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    WITH_SEMAPHORE(sem);
    if (offset == 0) {
        uint8_t header[3];
        if (get_log_data_raw(log_num, page, 0, 3, header) == -1) {
            return -1;
        }
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
            Fill_Format(structure(t), pkt);
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
        const int16_t bytes = get_log_data_raw(log_num, page, offset, len, data);
        if (bytes == -1) {
            return ret == 0 ? -1 : ret;
        }
        ret += bytes;
    }

    return ret;
}


// This function determines the number of whole or partial log files in the AP_Logger
// Wholly overwritten files are (of course) lost.
uint16_t AP_Logger_Block::get_num_logs(void)
{
    WITH_SEMAPHORE(sem);
    uint32_t lastpage;
    uint32_t last;

    if (!CardInserted() || find_last_page() == 1) {
        return 0;
    }

    StartRead(1);
    uint32_t first = GetFileNumber();
    
    if (first == 0xFFFF) {
        return 0;
    }

    lastpage = find_last_page();
    StartRead(lastpage);
    last = GetFileNumber();
    if (check_wrapped()) {
        // if we wrapped then the rest of the block will be filled with 0xFFFF because we always erase
        // a block before writing to it, in order to find the first page we therefore have to read after the
        // next block boundary
        StartRead((get_block(lastpage) + 1) * df_PagePerBlock + 1);
        first = GetFileNumber();
    }

    if (last == first) {
        return 1;
    }

    return (last - first + 1);
}


// This function starts a new log file in the AP_Logger
void AP_Logger_Block::start_new_log(void)
{
    WITH_SEMAPHORE(sem);
    uint32_t last_page = find_last_page();

    StartRead(last_page);

    if (find_last_log() == 0 || GetFileNumber() == 0xFFFF) {
        SetFileNumber(1);
        StartWrite(1);
        return;
    }

    uint16_t new_log_num;

    // Check for log of length 1 page and suppress
    if (df_FilePage <= 1) {
        new_log_num = GetFileNumber();
        // Last log too short, reuse its number
        // and overwrite it
        SetFileNumber(new_log_num);
        StartWrite(last_page);
    } else {
        new_log_num = GetFileNumber()+1;
        if (last_page == 0xFFFF) {
            last_page=0;
        }
        SetFileNumber(new_log_num);
        StartWrite(last_page + 1);
    }
    return;
}

// This function finds the first and last pages of a log file
// The first page may be greater than the last page if the AP_Logger has been filled and partially overwritten.
void AP_Logger_Block::get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page)
{
    WITH_SEMAPHORE(sem);
    uint16_t num = get_num_logs();
    uint32_t look;

    if (num == 1) {
        StartRead(df_NumPages);
        if (GetFileNumber() == 0xFFFF) {
            start_page = 1;
            end_page = find_last_page_of_log((uint16_t)log_num);
        } else {
            end_page = find_last_page_of_log((uint16_t)log_num);
            start_page = end_page + 1;
        }
    } else {
        if (log_num==1) {
            StartRead(df_NumPages);
            if (GetFileNumber() == 0xFFFF) {
                start_page = 1;
            } else {
                start_page = find_last_page() + 1;
            }
        } else {
            if (log_num == find_last_log() - num + 1) {
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
    if (start_page == df_NumPages+1 || start_page == 0) {
        start_page = 1;
    }
    end_page = find_last_page_of_log(log_num);
    if (end_page == 0) {
        end_page = start_page;
    }

}

bool AP_Logger_Block::check_wrapped(void)
{
    StartRead(df_NumPages);
    return GetFileNumber() != 0xFFFF;
}


// This function finds the last log number
uint16_t AP_Logger_Block::find_last_log(void)
{
    WITH_SEMAPHORE(sem);
    uint32_t last_page = find_last_page();
    StartRead(last_page);
    return GetFileNumber();
}

// This function finds the last page of the last file
uint32_t AP_Logger_Block::find_last_page(void)
{
    uint32_t look;
    uint32_t bottom = 1;
    uint32_t top = df_NumPages;
    uint64_t look_hash;
    uint64_t bottom_hash;
    uint64_t top_hash;

    WITH_SEMAPHORE(sem);

    StartRead(bottom);
    bottom_hash = ((int64_t)GetFileNumber()<<32) | df_FilePage;

    while (top-bottom > 1) {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int64_t)GetFileNumber()<<32 | df_FilePage;
        // erased sector so can discount everything above
        if (look_hash >= 0xFFFF00000000) {
            look_hash = 0;
        }

        if (look_hash < bottom_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
            bottom_hash = look_hash;
        }
    }

    StartRead(top);
    top_hash = ((int64_t)GetFileNumber()<<32) | df_FilePage;
    if (top_hash >= 0xFFFF00000000) {
        top_hash = 0;
    }
    if (top_hash > bottom_hash) {
        return top;
    }

    return bottom;
}

// This function finds the last page of a particular log file
uint32_t AP_Logger_Block::find_last_page_of_log(uint16_t log_number)
{
    uint32_t look;
    uint32_t bottom;
    uint32_t top;
    uint64_t look_hash;
    uint64_t check_hash;

    WITH_SEMAPHORE(sem);

    if (check_wrapped()) {
        StartRead(1);
        bottom = GetFileNumber();
        if (bottom > log_number) {
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

    check_hash = (int64_t)log_number<<32 | 0xFFFFFFFF;

    while (top-bottom > 1) {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int64_t)GetFileNumber()<<32 | df_FilePage;
        if (look_hash >= 0xFFFF00000000) {
            look_hash = 0;
        }

        if (look_hash > check_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
        }
    }

    StartRead(top);
    if (GetFileNumber() == log_number) {
        return top;
    }

    StartRead(bottom);
    if (GetFileNumber() == log_number) {
        return bottom;
    }

    gcs().send_text(MAV_SEVERITY_ERROR, "No last page of log %d at top=%X or bot=%X", int(log_number), unsigned(top), unsigned(bottom));
    return 0;
}


void AP_Logger_Block::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc)
{
    uint32_t start, end;

    WITH_SEMAPHORE(sem);

    get_log_boundaries(log_num, start, end);
    if (end >= start) {
        size = (end + 1 - start) * (uint32_t)df_PageSize;
    } else {
        size = (df_NumPages + end - start) * (uint32_t)df_PageSize;
    }
    time_utc = 0;
}


void AP_Logger_Block::PrepForArming()
{
    if (logging_started()) {
        return;
    }
    start_new_log();
}

// read size bytes of data from the buffer
bool AP_Logger_Block::BlockRead(uint16_t IntPageAdr, void *pBuffer, uint16_t size)
{
    memcpy(pBuffer, &buffer[IntPageAdr], size);
    return true;
}

/*
  IO timer running on IO thread
 */
void AP_Logger_Block::io_timer(void)
{
    if (!_initialised) {
        return;
    }

    WITH_SEMAPHORE(sem);

    if (erase_started) {
        if (InErase()) {
            return;
        }
        // write the logging format in the last page
        StartWrite(df_NumPages+1);
        uint32_t version = DF_LOGGING_FORMAT;
        memset(buffer, 0, df_PageSize);
        memcpy(buffer, &version, sizeof(version));
        FinishWrite();
        erase_started = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Chip erase complete");
        return;
    }

    if (df_EraseFrom > 0) {
        const uint32_t sectors = df_NumPages / df_PagePerSector;
        const uint32_t sectors_in_64k = 0x10000 / (df_PagePerSector * df_PageSize);
        uint32_t next_sector = get_sector(df_EraseFrom);
        const uint32_t aligned_sector = sectors - (((df_NumPages - df_EraseFrom + 1) / df_PagePerSector) / sectors_in_64k) * sectors_in_64k;
        while (next_sector < aligned_sector) {
            Sector4kErase(next_sector);
            next_sector++;
        }
        uint16_t blocks_erased = 0;
        while (next_sector < sectors) {
            blocks_erased++;
            SectorErase(next_sector / sectors_in_64k);
            next_sector += sectors_in_64k;
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Log recovery complete, erased %d blocks", unsigned(blocks_erased));
        df_EraseFrom = 0;
    }

    if (!CardInserted() || !log_write_started) {
        return;
    }

    while (writebuf.available() >= df_PageSize - sizeof(struct PageHeader)) {
        struct PageHeader ph;
        ph.FileNumber = df_FileNumber;
        ph.FilePage = df_FilePage;
        memcpy(buffer, &ph, sizeof(ph));
        writebuf.read(&buffer[sizeof(ph)], df_PageSize - sizeof(ph));
        FinishWrite();
        df_FilePage++;
    }
}
