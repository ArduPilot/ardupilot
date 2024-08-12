/*
  block based logging, for boards with flash logging
 */

#include "AP_Logger_config.h"

#if HAL_LOGGING_BLOCK_ENABLED

#include "AP_Logger_Block.h"
#include "AP_Logger.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_RTC/AP_RTC.h>
#include <GCS_MAVLink/GCS.h>

const extern AP_HAL::HAL& hal;

// the last page holds the log format in first 4 bytes. Please change
// this if (and only if!) the low level format changes
#define DF_LOGGING_FORMAT    0x1901201B

AP_Logger_Block::AP_Logger_Block(AP_Logger &front, LoggerMessageWriter_DFLogStart *writer) :
    AP_Logger_Backend(front, writer),
    writebuf(0)
{
    df_stats_clear();
}

// Init() is called after driver Init(), it is the responsibility of the driver to make sure the 
// device is ready to accept commands before Init() is called
void AP_Logger_Block::Init(void)
{
    // buffer is used for both reads and writes so access must always be within the semaphore
    buffer = (uint8_t *)hal.util->malloc_type(df_PageSize, AP_HAL::Util::MEM_DMA_SAFE);
    if (buffer == nullptr) {
        AP_HAL::panic("Out of DMA memory for logging");
    }

    //flash_test();

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
            DEV_PRINTF("AP_Logger_Block: Couldn't set buffer size to=%u\n", (unsigned)bufsize);
            bufsize >>= 1;
        }

        if (!writebuf.get_size()) {
            DEV_PRINTF("Out of memory for logging\n");
            return;
        }

        DEV_PRINTF("AP_Logger_Block: buffer size=%u\n", (unsigned)bufsize);
        _initialised = true;
    }

    WITH_SEMAPHORE(sem);

    if (NeedErase()) {
        EraseAll();
    } else {
        validate_log_structure();
    }
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
        // if we have wrapped over an existing log, force the oldest to be recalculated
        if (_cached_oldest_log > 0) {
            uint16_t log_num = StartRead(df_PageAdr);
            if (log_num != 0xFFFF && log_num >= _cached_oldest_log) {
                _cached_oldest_log = 0;
            }
        }
        // are we about to erase a sector with our own headers in it?
        if (df_Write_FilePage > df_NumPages - df_PagePerBlock) {
            chip_full = true;
            return;
        }
        SectorErase(get_block(df_PageAdr));
    }
}

bool AP_Logger_Block::WritesOK() const
{
    if (!CardInserted() || erase_started) {
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

    if (!WriteBlockCheckStartupMessages()) {
        _dropped++;
        return false;
    }

    WITH_SEMAPHORE(write_sem);

    const uint32_t space = writebuf.space();

    if (_writing_startup_messages &&
        _startup_messagewriter->fmt_done()) {
        // the state machine has called us, and it has finished
        // writing format messages out.  It can always get back to us
        // with more messages later, so let's leave room for other
        // things:
        const uint32_t now = AP_HAL::millis();
        const bool must_dribble = (now - last_messagewrite_message_sent) > 100;
        if (!must_dribble &&
            space < non_messagewriter_message_reserved_space(writebuf.get_size())) {
            // this message isn't dropped, it will be sent again...
            return false;
        }
        last_messagewrite_message_sent = now;
    } else {
        // we reserve some amount of space for critical messages:
        if (!is_critical && space < critical_message_reserved_space(writebuf.get_size())) {
            _dropped++;
            return false;
        }
    }

    // if no room for entire message - drop it:
    if (space < size) {
        _dropped++;
        return false;
    }

    writebuf.write((uint8_t*)pBuffer, size);
    df_stats_gather(size, writebuf.space());

    return true;
}

// read from the page address and return the file number at that location
uint16_t AP_Logger_Block::StartRead(uint32_t PageAdr)
{
    // copy flash page to buffer
    if (erase_started) {
        df_Read_PageAdr = PageAdr;
        memset(buffer, 0xff, df_PageSize);
    } else {
        PageToBuffer(PageAdr);
    }
    return ReadHeaders();
}

// read the headers at the current read point returning the file number
uint16_t AP_Logger_Block::ReadHeaders()
{
    // We are starting a new page - read FileNumber and FilePage
    struct PageHeader ph;
    BlockRead(0, &ph, sizeof(ph));
    df_FileNumber = ph.FileNumber;
    df_FilePage   = ph.FilePage;
#if BLOCK_LOG_VALIDATE
    if (ph.crc != DF_LOGGING_FORMAT + df_FilePage && df_FileNumber != 0xFFFF) {
        printf("ReadHeaders: invalid block read at %d\n", df_Read_PageAdr);
    }
#endif
    df_Read_BufferIdx = sizeof(ph);
    // we are at the start of a file, read the file header
    if (df_FilePage == 1) {
        struct FileHeader fh;
        BlockRead(sizeof(ph), &fh, sizeof(fh));
        df_FileTime = fh.utc_secs;
        df_Read_BufferIdx += sizeof(fh);
    }

    return df_FileNumber;
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
            uint32_t new_page_addr = df_Read_PageAdr + 1;
            if (new_page_addr > df_NumPages) {
                new_page_addr = 1;
            }
            if (erase_started) {
                memset(buffer, 0xff, df_PageSize);
                df_Read_PageAdr = new_page_addr;
            } else {
                PageToBuffer(new_page_addr);
            }

            // We are starting a new page - read FileNumber and FilePage
            ReadHeaders();
        }
    }
    return true;
}

// initialize the log data for the given file number
void AP_Logger_Block::StartLogFile(uint16_t FileNumber)
{
    df_FileNumber = FileNumber;
    df_Write_FileNumber = FileNumber;
    df_FilePage = 1;
    df_Write_FilePage = 1;
}

uint16_t AP_Logger_Block::GetFileNumber() const
{
    return df_FileNumber;
}

void AP_Logger_Block::EraseAll()
{
    if (hal.util->get_soft_armed()) {
        // do not want to do any filesystem operations while we are e.g. flying
        return;
    }

    // push out the message before stopping logging
    if (!erase_started) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Chip erase started");
    }

    WITH_SEMAPHORE(sem);

    if (erase_started) {
        // already erasing
        return;
    }
    erase_started = true;

    // remember what we were doing
    new_log_pending = log_write_started;

    // throw away everything
    log_write_started = false;
    writebuf.clear();

    // reset the format version and wrapped status so that any incomplete erase will be caught
    Sector4kErase(get_sector(df_NumPages));

    StartErase();
}

void AP_Logger_Block::periodic_1Hz()
{
    AP_Logger_Backend::periodic_1Hz();

    if (rate_limiter == nullptr &&
        (_front._params.blk_ratemax > 0 ||
         _front._params.disarm_ratemax > 0 ||
         _front._log_pause)) {
        // setup rate limiting if log rate max > 0Hz or log pause of streaming entries is requested
        rate_limiter = new AP_Logger_RateLimiter(_front, _front._params.blk_ratemax, _front._params.disarm_ratemax);
    }
    
    if (!io_thread_alive()) {
        if (warning_decimation_counter == 0 && _initialised) {
            // we don't print this error unless we did initialise. When _initialised is set to true
            // we register the IO timer callback
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Logger: IO thread died");
        }
        if (warning_decimation_counter++ > 57) {
            warning_decimation_counter = 0;
        }
        _initialised = false;
    } else if (chip_full) {
        if (warning_decimation_counter == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Chip full, logging stopped");
        }
        if (warning_decimation_counter++ > 57) {
            warning_decimation_counter = 0;
        }
    }
}

// EraseAll is asynchronous, but we must not start a new
// log in a child thread so this task picks up the hint from the io timer
// keeping locking to a minimum
void AP_Logger_Block::periodic_10Hz(const uint32_t now)
{
    if (erase_started || InErase()) {
        return;
    }

    // don't print status messages in io thread, do it here
    switch (status_msg) {
    case StatusMessage::ERASE_COMPLETE:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Chip erase complete");
        status_msg = StatusMessage::NONE;
        break;
    case StatusMessage::RECOVERY_COMPLETE:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Log recovery complete");
        status_msg = StatusMessage::NONE;
        break;
    case StatusMessage::NONE:
        break;
    }

    // EraseAll should only set this in the main thread
    if (new_log_pending) {
        start_new_log();
    }
}

/*
 *  we need to erase if the logging format has changed
 */
bool AP_Logger_Block::NeedErase(void)
{
    uint32_t version = 0;
    PageToBuffer(df_NumPages+1); // last page
    BlockRead(0, &version, sizeof(version));
    if (version == DF_LOGGING_FORMAT) {
        // only leave the read point in a sane place if we are not about to destroy everything
        StartRead(1);
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
    bool wrapped = is_wrapped();
    uint32_t page = 1;
    uint32_t page_start = 1;

    uint16_t file = StartRead(page);
    uint16_t first_file = file;
    uint16_t next_file = file;
    uint16_t last_file = 0;

    while (file != 0xFFFF && page <= df_NumPages && (file == next_file || (wrapped && file < next_file))) {
        uint32_t end_page = find_last_page_of_log(file);
        if (end_page == 0 || end_page < page) { // this can happen and may be responsible for corruption that we have seen
            break;
        }
        page = end_page + 1;
        file = StartRead(page);
        next_file++;
        // skip over the rest of an erased block
        if (wrapped && file == 0xFFFF) {
            file = StartRead((get_block(page) + 1) * df_PagePerBlock + 1);
        }
        if (wrapped && file < next_file) {
            page_start = page;
            next_file = file;
            first_file = file;
        } else if (last_file < next_file) {
            last_file = file;
        }
        if (file == next_file) {
            DEV_PRINTF("Found complete log %d at %X-%X\n", int(file), unsigned(page), unsigned(find_last_page_of_log(file)));
        }
    }

    if (file != 0xFFFF && file != next_file && page <= df_NumPages && page > 0) {
        DEV_PRINTF("Found corrupt log %d at 0x%04X, erasing", int(file), unsigned(page));
        df_EraseFrom = page;
    } else if (next_file != 0xFFFF && page > 0 && next_file > 1) { // chip is empty
        DEV_PRINTF("Found %d complete logs at 0x%04X-0x%04X", int(next_file - first_file), unsigned(page_start), unsigned(page - 1));
    }
}

/**
 * get raw data from a log - page is the start page of the log, offset is the offset within the log starting at that page
 */
int16_t AP_Logger_Block::get_log_data_raw(uint16_t log_num, uint32_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    WITH_SEMAPHORE(sem);
    const uint16_t data_page_size = df_PageSize - sizeof(struct PageHeader);
    const uint16_t first_page_size = data_page_size - sizeof(struct FileHeader);

    // offset is the true offset in the file, so we have to calculate the offset accounting for page headers
    if (offset >= first_page_size) {
        offset -= first_page_size;
        page = page + offset / data_page_size + 1;
        offset %= data_page_size;

        if (page > df_NumPages) {
            page = page % df_NumPages;
        }
    }

    // Sanity check we haven't been asked for an offset beyond the end of the log
    if (StartRead(page) != log_num) {
        return -1;
    }

    df_Read_BufferIdx += offset;

    if (!ReadBlock(data, len)) {
        return -1;
    }

    return (int16_t)len;
}

/**
  get data from a log, accounting for adding FMT headers
 */
int16_t AP_Logger_Block::get_log_data(uint16_t list_entry, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    const uint16_t log_num = log_num_from_list_entry(list_entry);

    if (log_num == 0) {
        // that failed - probably no logs
        return -1;
    }

    //printf("get_log_data(%d, %d, %d, %d)\n", log_num, page, offset, len);
    WITH_SEMAPHORE(sem);

    uint16_t ret = 0;
    if (len > 0) {
        const int16_t bytes = get_log_data_raw(log_num, page, offset, len, data);
        if (bytes == -1) {
            return -1;
        }
        ret += bytes;
    }

    return ret;
}


// This function determines the number of whole log files in the AP_Logger
// partial logs are rejected as without the headers they are relatively useless
uint16_t AP_Logger_Block::get_num_logs(void)
{
    WITH_SEMAPHORE(sem);
    uint32_t lastpage;
    uint32_t last;

    if (!CardInserted() || find_last_page() == 1) {
        return 0;
    }

    uint32_t first = StartRead(1);
    
    if (first == 0xFFFF) {
        return 0;
    }

    lastpage = find_last_page();
    last = StartRead(lastpage);

    if (is_wrapped()) {
        // if we wrapped then the rest of the block will be filled with 0xFFFF because we always erase
        // a block before writing to it, in order to find the first page we therefore have to read after the
        // next block boundary
        first = StartRead((get_block(lastpage) + 1) * df_PagePerBlock + 1);
        // unless we happen to land on the first page of the file that is being overwritten we skip to the next file
        if (df_FilePage > 1) {
            first++;
        }
    }

    if (last == first) {
        return 1;
    }

    return (last - first + 1);
}

// stop logging immediately
void AP_Logger_Block::stop_logging(void)
{
    WITH_SEMAPHORE(sem);

    log_write_started = false;

    // nuke writing any previous log
    writebuf.clear();
}

// stop logging and flush any remaining data
void AP_Logger_Block::stop_logging_async(void)
{
    stop_log_pending = true;
}

// This function starts a new log file in the AP_Logger
// no actual data should be written to the storage here
// that should all be handled by the IO thread
void AP_Logger_Block::start_new_log(void)
{
    if (erase_started) {
        // already erasing
        return;
    }

    WITH_SEMAPHORE(sem);

    if (logging_started()) {
        stop_logging();
    }

    // no need to schedule this anymore
    new_log_pending = false;

    uint32_t last_page = find_last_page();

    StartRead(last_page);

    log_write_started = true;
    uint16_t new_log_num = 1;

    if (find_last_log() == 0 || GetFileNumber() == 0xFFFF) {
        StartLogFile(new_log_num);
        StartWrite(1);
    // Check for log of length 1 page and suppress
    } else if (df_FilePage <= 1) {
        new_log_num = GetFileNumber();
        // Last log too short, reuse its number
        // and overwrite it
        StartLogFile(new_log_num);
        StartWrite(last_page);
    } else {
        new_log_num = GetFileNumber()+1;
        if (last_page == 0xFFFF) {
            last_page=0;
        }
        StartLogFile(new_log_num);
        StartWrite(last_page + 1);
    }

    // save UTC time in the first 4 bytes so that we can retrieve it later
    uint64_t utc_usec;
    FileHeader hdr {};
    if (AP::rtc().get_utc_usec(utc_usec)) {
        hdr.utc_secs = utc_usec / 1000000U;
    }
    writebuf.write((uint8_t*)&hdr, sizeof(FileHeader));

    start_new_log_reset_variables();

    return;
}

// This function finds the first and last pages of a log file
// The first page may be greater than the last page if the AP_Logger has been filled and partially overwritten.
void AP_Logger_Block::get_log_boundaries(uint16_t list_entry, uint32_t & start_page, uint32_t & end_page)
{
    const uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // that failed - probably no logs
        start_page = 0;
        end_page = 0;
        return;
    }

    WITH_SEMAPHORE(sem);
    uint16_t num = get_num_logs();
    uint32_t look;

    end_page = find_last_page_of_log(log_num);

    if (num == 1 || log_num == 1) {
        if (!is_wrapped()) {
            start_page = 1;
        } else {
            StartRead(end_page);
            start_page = (end_page + df_NumPages - df_FilePage) % df_NumPages + 1;
        }
    } else {
        // looking for the first log which might have a gap in front of it
        if (list_entry == 1) {
            StartRead(end_page);
            if (end_page > df_FilePage) { // log is not wrapped
                start_page = end_page - df_FilePage + 1;
            } else { // log is wrapped
                start_page = (end_page + df_NumPages - df_FilePage) % df_NumPages + 1;
            }
        } else {
            look = log_num-1;
            do {
                start_page = find_last_page_of_log(look) + 1;
                look--;
            } while (start_page <= 0 && look >=1);
        }
    }

    if (start_page == df_NumPages + 1 || start_page == 0) {
        start_page = 1;
    }

    if (end_page == 0) {
        end_page = start_page;
    }

}

// return true if logging has wrapped around to the beginning of the chip
bool AP_Logger_Block::is_wrapped(void)
{
    return StartRead(df_NumPages) != 0xFFFF;
}


// This function finds the last log number
uint16_t AP_Logger_Block::find_last_log(void)
{
    WITH_SEMAPHORE(sem);
    uint32_t last_page = find_last_page();
    return StartRead(last_page);
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

    if (is_wrapped()) {
        bottom = StartRead(1);
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

    if (StartRead(top) == log_number) {
        return top;
    }

    if (StartRead(bottom) == log_number) {
        return bottom;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "No last page of log %d at top=%X or bot=%X", int(log_number), unsigned(top), unsigned(bottom));
    return 0;
}

void AP_Logger_Block::get_log_info(uint16_t list_entry, uint32_t &size, uint32_t &time_utc)
{
    uint32_t start, end;

    WITH_SEMAPHORE(sem);

    get_log_boundaries(list_entry, start, end);
    if (end >= start) {
        size = (end + 1 - start) * (uint32_t)(df_PageSize - sizeof(PageHeader));
    } else {
        size = (df_NumPages + end + 1 - start) * (uint32_t)(df_PageSize - sizeof(PageHeader));
    }

    size -= sizeof(FileHeader);

    //printf("LOG %d(%d), %d-%d, size %d\n", log_num_from_list_entry(list_entry), list_entry, start, end, size);

    StartRead(start);

    // the log we are currently writing
    if (df_FileTime == 0 && df_FileNumber == df_Write_FileNumber) {
        uint64_t utc_usec;
        if (AP::rtc().get_utc_usec(utc_usec)) {
            df_FileTime = utc_usec / 1000000U;
        }
    }
    time_utc = df_FileTime;
}

// read size bytes of data from the buffer
bool AP_Logger_Block::BlockRead(uint16_t IntPageAdr, void *pBuffer, uint16_t size)
{
    memcpy(pBuffer, &buffer[IntPageAdr], size);
    return true;
}

bool AP_Logger_Block::logging_failed() const
{
    if (!_initialised) {
        return true;
    }
    if (!io_thread_alive()) {
        return true;
    }
    if (chip_full) {
        return true;
    }

    return false;
}

// detect whether the IO thread is running, since this is considered a catastrophic failure for the logging system
// better be really, really sure
bool AP_Logger_Block::io_thread_alive() const
{
    // if the io thread hasn't had a heartbeat in 3s it is dead
    return (AP_HAL::millis() - io_timer_heartbeat) < 3000U || !hal.scheduler->is_system_initialized();
}

/*
  IO timer running on IO thread
  The IO timer runs every 1ms or at 1Khz. The standard flash chip can write roughly 130Kb/s
  so there is little point in trying to write more than 130 bytes - or 1 page (256 bytes).
  The W25Q128FV datasheet gives tpp as typically 0.7ms yielding an absolute maximum rate of
  365Kb/s or just over a page per cycle.
 */
void AP_Logger_Block::io_timer(void)
{
    uint32_t tnow = AP_HAL::millis();
    io_timer_heartbeat = tnow;

    // don't write anything for the first 2s to give the dataflash chip a chance to be ready
    if (!_initialised || tnow < 2000) {
        return;
    }

    if (erase_started) {
        WITH_SEMAPHORE(sem);

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
        chip_full = false;
        status_msg = StatusMessage::ERASE_COMPLETE;
        return;
    }

    if (df_EraseFrom > 0) {
        WITH_SEMAPHORE(sem);

        const uint32_t sectors = df_NumPages / df_PagePerSector;
        const uint32_t block_size = df_PagePerBlock * df_PageSize;
        const uint32_t sectors_in_block = block_size / (df_PagePerSector * df_PageSize);
        uint32_t next_sector = get_sector(df_EraseFrom);
        const uint32_t aligned_sector = sectors - (((df_NumPages - df_EraseFrom + 1) / df_PagePerSector) / sectors_in_block) * sectors_in_block;
        while (next_sector < aligned_sector) {
            Sector4kErase(next_sector);
            io_timer_heartbeat = AP_HAL::millis();
            next_sector++;
        }
        while (next_sector < sectors) {
            SectorErase(next_sector / sectors_in_block);
            io_timer_heartbeat = AP_HAL::millis();
            next_sector += sectors_in_block;
        }
        status_msg = StatusMessage::RECOVERY_COMPLETE;
        df_EraseFrom = 0;
    }

    if (!CardInserted() || new_log_pending || chip_full) {
        return;
    }

    // we have been asked to stop logging, flush everything
    if (stop_log_pending) {
        WITH_SEMAPHORE(sem);

        log_write_started = false;

        // complete writing any previous log, a page at a time to avoid holding the lock for too long
        if (writebuf.available()) {
            write_log_page();
        } else {
            writebuf.clear();
            stop_log_pending = false;
        }

    // write at most one page
    } else if (writebuf.available() >= df_PageSize - sizeof(struct PageHeader)) {
        WITH_SEMAPHORE(sem);

        write_log_page();
    }
}

// write out a page of log data
void AP_Logger_Block::write_log_page()
{
    struct PageHeader ph;
    ph.FileNumber = df_Write_FileNumber;
    ph.FilePage = df_Write_FilePage;
#if BLOCK_LOG_VALIDATE
    ph.crc = DF_LOGGING_FORMAT + df_Write_FilePage;
#endif
    memcpy(buffer, &ph, sizeof(ph));
    const uint32_t pagesize = df_PageSize - sizeof(ph);
    uint32_t nbytes = writebuf.read(&buffer[sizeof(ph)], pagesize);
    if (nbytes <  pagesize) {
        memset(&buffer[sizeof(ph) + nbytes], 0, pagesize - nbytes);
    }
    FinishWrite();
    df_Write_FilePage++;
}

void AP_Logger_Block::flash_test()
{
    const uint32_t pages_to_check = 128;
    for (uint32_t i=1; i<=pages_to_check; i++) {
        if ((i-1) % df_PagePerBlock == 0) {
            printf("Block erase %u\n", get_block(i));
            SectorErase(get_block(i));
        }
        memset(buffer, uint8_t(i), df_PageSize);
        if (i<5) {
            printf("Flash fill 0x%x\n", uint8_t(i));
        } else if (i==5) {
            printf("Flash fill pages 5-%u\n", pages_to_check);
        }
        BufferToPage(i);
    }
    for (uint32_t i=1; i<=pages_to_check; i++) {
        if (i<5) {
            printf("Flash check 0x%x\n", uint8_t(i));
        } else if (i==5) {
            printf("Flash check pages 5-%u\n", pages_to_check);
        }
        PageToBuffer(i);
        uint32_t bad_bytes = 0;
        uint32_t first_bad_byte = 0;
        for (uint32_t j=0; j<df_PageSize; j++) {
            if (buffer[j] != uint8_t(i)) {
                bad_bytes++;
                if (bad_bytes == 1) {
                    first_bad_byte = j;
                }
            }
        }
        if (bad_bytes > 0) {
            printf("Test failed: page %u, %u of %u bad bytes, first=0x%x\n",
                i, bad_bytes, df_PageSize, buffer[first_bad_byte]);
        }
    }
}

#endif // HAL_LOGGING_BLOCK_ENABLED
