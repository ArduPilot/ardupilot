/* 
   AP_Logger logging - file oriented variant

   This uses SD library to create log files called logs/NN.bin in the
   given directory

   SD Card Rates 
    - deletion rate seems to be ~50 files/second.
    - stat seems to be ~150/second
    - readdir loop of 511 entry directory ~62,000 microseconds
 */


#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT && (defined(BOARD_SDCARD_NAME) || defined(BOARD_DATAFLASH_FATFS))

#include "AP_Logger_File_sd.h"

#include <AP_Common/AP_Common.h>
#include <assert.h>
#include <AP_Math/AP_Math.h>
#include <time.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Param_Helper/AP_Param_Helper.h>

extern const AP_HAL::HAL& hal;

#if defined(BOARD_DATAFLASH_FATFS)
 #define MAX_LOG_FILES 9U
#else
 #define MAX_LOG_FILES 50U
#endif

#define DATAFLASH_PAGE_SIZE 1024UL

#define MAX_FILE_SIZE 2048 * 1024L // not more 2MB

/*
  constructor
 */
AP_Logger_File::AP_Logger_File(AP_Logger &front,
                               LoggerMessageWriter_DFLogStart *writer,
                               const char *log_directory) :
    AP_Logger_Backend(front, writer),
    _write_fd(File()),
    _read_fd(File()),
    _log_directory(log_directory),
    _writebuf(0),
    _writebuf_chunk(4096)
{}


void AP_Logger_File::Init()
{
    AP_Logger_Backend::Init();

    if(HAL_F4Light::state.sd_busy) return; // SD mounted via USB

    // create the log directory if need be
    const char* custom_dir = hal.util->get_custom_log_directory();
    if (custom_dir != nullptr){
        _log_directory = custom_dir;
    }

    if (! SD.exists(_log_directory) ) {
        char buf[80];
        const char *cp, *rp;
        char *wp;
        for(cp=_log_directory + strlen(_log_directory);cp>_log_directory && *cp != '/';cp--){   }
        for(wp=buf, rp=_log_directory; rp<cp;){
            *wp++ = *rp++;
        }
        *wp++ = 0;
        SD.mkdir(buf);

        if (!SD.mkdir(_log_directory)) {
            
            printf("Failed to create log directory %s: %s\n", _log_directory, SD.strError(SD.lastError));
            gcs().send_text(MAV_SEVERITY_WARNING,"Failed to create log directory %s: %s", _log_directory, SD.strError(SD.lastError));
            _log_directory="0:";
        }
    }

    // determine and limit file backend buffersize
    uint32_t bufsize = _front._params.file_bufsize;
    if (bufsize > 64) {
        bufsize = 64; // DMA limitations.
    }
    bufsize *= 1024;

    // If we can't allocate the full size, try to reduce it until we can allocate it
    while (!_writebuf.set_size(bufsize) && bufsize >= _writebuf_chunk) {
        printf("AP_Logger_File: Couldn't set buffer size to=%u\n", (unsigned)bufsize);
        bufsize /= 2;
    }

    if (!_writebuf.get_size()) {
        printf("Out of memory for logging\n");
        return;
    }

    printf("AP_Logger_File: buffer size=%u\n", (unsigned)bufsize);

    _initialised = true;
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Logger_File::_io_timer, void));
}

bool AP_Logger_File::file_exists(const char *filename) const
{
    return SD.exists(filename);
}

bool AP_Logger_File::log_exists(const uint16_t lognum) const
{
    char *filename = _log_file_name(lognum);
    if (filename == nullptr) {
        // internal_error();
        return false; // ?!
    }
    bool ret = file_exists(filename);
    free(filename);
    return ret;
}

void AP_Logger_File::periodic_1Hz()
{
    if (!(_write_fd) || !_initialised || _open_error || _busy) return; // too early

    if (!io_thread_alive()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No IO Thread Heartbeat");
        // If you try to close the file here then it will almost
        // certainly block.  Since this is the main thread, this is
        // likely to cause a crash.
//        _write_fd.close();
        _write_fd.sync();
        printf("\nLoging aborted\n");
        _open_error = true;
        _initialised = false;
    }
}

void AP_Logger_File::periodic_fullrate()
{
    AP_Logger_Backend::push_log_blocks();
}

uint32_t AP_Logger_File::bufferspace_available()
{
    const uint32_t space = _writebuf.space();
    const uint32_t crit = critical_message_reserved_space();

    return (space > crit) ? space - crit : 0;
}

// return true for CardInserted() if we successfully initialized
bool AP_Logger_File::CardInserted(void) const
{
    return _initialised && !_open_error && !HAL_F4Light::state.sd_busy;
}

// returns the available space in _log_directory as a percentage
// returns -1.0f on error
float AP_Logger_File::avail_space_percent(uint32_t *free)
{
    
    uint32_t space;
    int32_t avail = SD.getfree(_log_directory, &space);
    if(free) *free = avail;
    if (avail == -1) {
        return -1.0f;
    }

    return (avail/(float)space) * 100;
}

#if 0 // why such hard?

// find_oldest_log - find oldest log in _log_directory
// returns 0 if no log was found
uint16_t AP_Logger_File::find_oldest_log()
{
    if (_cached_oldest_log != 0) {
        return _cached_oldest_log;
    }

    uint16_t last_log_num = find_last_log();
    if (last_log_num == 0) {
        return 0;
    }

    uint16_t current_oldest_log = 0; // 0 is invalid

    // We could count up to find_last_log(), but if people start
    // relying on the min_avail_space_percent feature we could end up
    // doing a *lot* of asprintf()s and stat()s
    File dir = SD.open(_log_directory);
    if (!dir) {
        // internal_error();
        printf("error opening logs dir: %s", SD.strError(SD.lastError));
        return 0;
    }

    // we only count files which look like xxx.BIN
    while(1){
        File de=dir.openNextFile();
        if(!de) {
            if(SD.lastError){
                printf("error scanning logs: %s", SD.strError(SD.lastError));
            }
            break;
        }
        
        char *nm = de.name();
        de.close();
        
        uint8_t length = strlen(nm);
        if (length < 5) {
            // not long enough for \d+[.]BIN
            continue;
        }
        if (strncmp(&nm[length-4], ".BIN", 4)) {
            // doesn't end in .BIN
            continue;
        }

        uint16_t thisnum = strtoul(nm, nullptr, 10);
        if (current_oldest_log == 0) {
            current_oldest_log = thisnum;
        } else {
            if (current_oldest_log <= last_log_num) {
                if (thisnum > last_log_num) {
                    current_oldest_log = thisnum;
                } else if (thisnum < current_oldest_log) {
                    current_oldest_log = thisnum;
                }
            } else { // current_oldest_log > last_log_num
                if (thisnum > last_log_num) {
                    if (thisnum < current_oldest_log) {
                        current_oldest_log = thisnum;
                    }
                }
            }
        }
    }
    dir.close();
    _cached_oldest_log = current_oldest_log;

    return current_oldest_log;
}

#else

// result cached later
uint16_t AP_Logger_File::find_oldest_log()
{

    if (_cached_oldest_log != 0) {
        return _cached_oldest_log;
    }

    uint16_t i;
    if(_last_oldest_log >=MAX_LOG_FILES-1) _last_oldest_log=1;
    for (i=_last_oldest_log; i<MAX_LOG_FILES; i++) {
        if ( log_exists(i)) {
            _cached_oldest_log=i;
            return i;
        }
    }
    
    return 0;
}
#endif


void AP_Logger_File::Prep_MinSpace()
{
    const uint16_t first_log_to_remove = find_oldest_log();
    if (first_log_to_remove != 0) {

        _last_oldest_log = _cached_oldest_log + 1;
        _cached_oldest_log = 0;

        uint16_t log_to_remove = first_log_to_remove;

        uint16_t count = 0;
        do {
            uint32_t free_sp;
            float avail = avail_space_percent(&free_sp);

            if (avail < 0) {             // internal_error()
#if defined(BOARD_DATAFLASH_FATFS)
                printf("error getting free space, formatting!\n");
                SD.format(_log_directory);
                return;
#elif defined(BOARD_SDCARD_CS_PIN)
                if(hal_param_helper->_sd_format){
                    printf("error getting free space, formatting!\n");
                    gcs().send_text(MAV_SEVERITY_WARNING,"error getting free space, formatting!");
                    SD.format(_log_directory);
                    return;
                }
#endif
                break;
            }
            if (avail >= min_avail_space_percent && free_sp*512 >= MAX_FILE_SIZE) { // not less 2MB - space for one file
                break;
            }
            if (count++ > MAX_LOG_FILES+10) {
                // *way* too many deletions going on here.  Possible internal error.
                break;
            }
            char *filename_to_remove = _log_file_name(log_to_remove);
            if (filename_to_remove == nullptr) {
                break;
            }
            if (SD.exists(filename_to_remove)) {
                printf("Removing (%s) for minimum-space requirements (%.2f%% < %.0f%%) %.1fMb\n",
                                    filename_to_remove, (double)avail, (double)min_avail_space_percent, free_sp/(1024.*2));
                if (!SD.remove(filename_to_remove)) {
                    printf("Failed to remove %s: %s\n", filename_to_remove, SD.strError(SD.lastError));
                }
                free(filename_to_remove);
            }
            log_to_remove++;
            if (log_to_remove > MAX_LOG_FILES) {
                log_to_remove = 1;
            }
        } while (log_to_remove != first_log_to_remove);

    }
// check the result
#if defined(BOARD_DATAFLASH_FATFS)
    float avail = avail_space_percent();
    if (avail <= 0) {   // erase don't helps
        printf("erase don't get free space, formatting!\n");
        SD.format(_log_directory);
    }
#endif

}


void AP_Logger_File::Prep() {
    if (hal.util->get_soft_armed()) {
        // do not want to do any filesystem operations while we are e.g. flying
        return;
    }
    Prep_MinSpace();
}

bool AP_Logger_File::NeedPrep()
{
    if (!CardInserted()) {
        // should not have been called?!
        return false;
    }

    if (avail_space_percent() < min_avail_space_percent) {
        return true;
    }

    return false;
}

/*
  construct a log file name given a log number. 
  Note: Caller must free.
 */
char *AP_Logger_File::_log_file_name(const uint16_t log_num) const
{
    char *buf = (char *)malloc(256);
    if(!buf) return nullptr;
    
    sprintf(buf, "%s/%u.BIN", _log_directory, (unsigned)log_num);

    return buf;
}

/*
  return path name of the lastlog.txt marker file
  Note: Caller must free.
 */
char *AP_Logger_File::_lastlog_file_name(void) const
{
    char *buf = (char *)malloc(256);
    if(!buf) return nullptr;

    sprintf(buf, "%s/LASTLOG.TXT", _log_directory);
    return buf;
}


// remove all log files
void AP_Logger_File::EraseAll()
{
    uint16_t log_num;
    const bool was_logging = (_write_fd != -1);
    stop_logging();

    for (log_num=1; log_num<=MAX_LOG_FILES; log_num++) {
        char *fname = _log_file_name(log_num);
        if (fname == nullptr) {
            break;
        }
        SD.remove(fname);
        free(fname);
    }
    char *fname = _lastlog_file_name();
    if (fname != nullptr) {
        SD.remove(fname);
        free(fname);
    }

    _cached_oldest_log = 0;

    if (was_logging) {
        start_new_log();
    }
}


bool AP_Logger_File::WritesOK() const
{
    if (!_write_fd) {
        return false;
    }
    if (_open_error) {
        return false;
    }
    return true;
}


bool AP_Logger_File::StartNewLogOK() const
{
    if (_open_error) {
        return false;
    }
    return AP_Logger_Backend::StartNewLogOK();
}


/* Write a block of data at current offset */
bool AP_Logger_File::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    if (! WriteBlockCheckStartupMessages()) {
        _dropped++;
        return false;
    }

    if (!semaphore.take(1)) {
        return false;
    }
        
    uint32_t space = _writebuf.space();

    if (_writing_startup_messages &&
        _startup_messagewriter->fmt_done()) {
        // the state machine has called us, and it has finished
        // writing format messages out.  It can always get back to us
        // with more messages later, so let's leave room for other
        // things:
        if (space < non_messagewriter_message_reserved_space()) {
            // this message isn't dropped, it will be sent again...
            semaphore.give();
            return false;
        }
    } else {
        // we reserve some amount of space for critical messages:
        if (!is_critical && space < critical_message_reserved_space()) {
//            printf("dropping NC block! size=%d\n", size);
            _dropped++;
            semaphore.give();
            return false;
        }
    }

    // if no room for entire message - drop it:
    if (space < size) {
//        hal.util->perf_count(_perf_overruns);
        printf("dropping block! size=%d\n", size);

        _dropped++;
        semaphore.give();
        return false;
    }

    _writebuf.write((uint8_t*)pBuffer, size);
    has_data=true;
    
    semaphore.give();
    return true;
}

/*
  find the highest log number
 */
uint16_t AP_Logger_File::find_last_log()
{
    unsigned ret = 0;
    char *fname = _lastlog_file_name();
    if (fname == nullptr) {
        return ret;
    }
    File fd = SD.open(fname, FILE_READ);
    free(fname);
    if (fd) {
        char buf[10];
        memset(buf, 0, sizeof(buf));
        if (fd.read(buf, sizeof(buf)-1) > 0) {
            ret = strtoul(buf, nullptr, 10); // зачем тащить толстую функцию зря
//            sscanf(buf, "%u", &ret);            
        }
        fd.close();    
    }
    return ret;
}

uint32_t AP_Logger_File::_get_log_size(const uint16_t log_num) const
{
    char *fname = _log_file_name(log_num);
    if (fname == nullptr) {
        return 0;
    }

    File fd = SD.open(fname, FILE_READ);
    free(fname);

    if(!fd) return 0;
    
    uint32_t sz= fd.size();
    fd.close();

    return sz;
}

uint32_t AP_Logger_File::_get_log_time(const uint16_t log_num) const
{
    char *fname = _log_file_name(log_num);
    if (fname == nullptr) {
        return 0;
    }

    FILINFO fno;

    int8_t ret = SD.stat(fname, &fno);
    free(fname);

    if(ret<0) return 0;

    uint16_t date=fno.fdate,
             time=fno.ftime;
        
    
    struct tm t;
    
    t.tm_sec  = FAT_SECOND(time); //     seconds after the minute        0-61*
    t.tm_min  = FAT_MINUTE(time); //    minutes after the hour  0-59
    t.tm_hour = FAT_HOUR(time); //     hours since midnight    0-23
    t.tm_mday = FAT_DAY(date); //     day of the month        1-31
    t.tm_mon  = FAT_MONTH(date); //     months since January    0-11
    t.tm_year = FAT_YEAR(date); //     years since 1900        
//    t.tm_yday int     days since January 1    0-365
    t.tm_isdst =false;

    
    return to_timestamp(&t);
}

/*
  convert a list entry number back into a log number (which can then
  be converted into a filename).  A "list entry number" is a sequence
  where the oldest log has a number of 1, the second-from-oldest 2,
  and so on.  Thus the highest list entry number is equal to the
  number of logs.
*/
uint16_t AP_Logger_File::_log_num_from_list_entry(const uint16_t list_entry)
{
    uint16_t oldest_log = find_oldest_log();
    if (oldest_log == 0) {
        // We don't have any logs...
        return 0;
    }

    uint32_t log_num = oldest_log + list_entry - 1;
    
    if (log_num > MAX_LOG_FILES) {
        log_num -= MAX_LOG_FILES;
    }
    while(!log_exists(log_num)){ // skip gaps
        log_num++;
        if(log_num>MAX_LOG_FILES) {
            log_num=MAX_LOG_FILES;
            break;
        }
    }

    return (uint16_t)log_num;
}

/*
  find the number of pages in a log
 */
void AP_Logger_File::get_log_boundaries(const uint16_t list_entry, uint16_t & start_page, uint16_t & end_page)
{
    const uint16_t log_num = _log_num_from_list_entry(list_entry);
    //if (log_num == 0) {
    if (! log_exists(log_num)) {
        // that failed - probably no logs
        start_page = 0;
        end_page = 0;
        return;
    }

    start_page = 0;
    end_page = _get_log_size(log_num) / DATAFLASH_PAGE_SIZE;
}

/*
  retrieve data from a log file
 */
int16_t AP_Logger_File::get_log_data(const uint16_t list_entry, const uint16_t page, const uint32_t offset, const uint16_t len, uint8_t *data)
{
    if (!_initialised || _open_error) {
        return -1;
    }

    const uint16_t log_num = _log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // that failed - probably no logs
        return -1;
    }

    if (_read_fd && log_num != _read_fd_log_num) {
        _read_fd.close();
    }
    if (!(_read_fd)) {
        char *fname = _log_file_name(log_num);
        if (fname == nullptr) {
            return -1;
        }
        stop_logging();
        _read_fd = SD.open(fname, O_RDONLY);
        if (!(_read_fd)) {
            _open_error = true;

            printf("Log read open fail for %s: %s\n", fname, SD.strError(SD.lastError));
            free(fname);
            return -1;            
        }
        free(fname);
        _read_offset = 0;
        _read_fd_log_num = log_num;
    }

    int16_t ret = (int16_t)_read_fd.read(data, len);
    if (ret > 0) {
        _read_offset += ret;
    }
    return ret;
}

/*
  find size and date of a log
 */
void AP_Logger_File::get_log_info(const uint16_t list_entry, uint32_t &size, uint32_t &time_utc)
{
    uint16_t log_num = _log_num_from_list_entry(list_entry);
//    if (log_num == 0) {
    if (! log_exists(log_num)) {

        // that failed - probably no logs
        size = 0;
        time_utc = 0;
        return;
    }

    size = _get_log_size(log_num);
    time_utc = _get_log_time(log_num);
}



/*
  get the number of logs [no more - note that the log numbers must be consecutive ]
 */
uint16_t AP_Logger_File::get_num_logs()
{
    uint16_t ret = 0;
    uint16_t high = find_last_log();
    uint16_t i;
    for (i=high; i>0; i--) {
        if (log_exists(i)) {
            ret++;
                if(_cached_oldest_log == 0 || i<_cached_oldest_log)
                    _cached_oldest_log=i;
        }
    }

    if (i == 0) {
        for (i=MAX_LOG_FILES; i>high; i--) {
            if ( log_exists(i)) {
                ret++;
                if(_cached_oldest_log == 0 || i<_cached_oldest_log)
                    _cached_oldest_log=i;
            }
        }
    }
    return ret;
}

/*
  stop logging
 */
void AP_Logger_File::stop_logging(void)
{
    if (_write_fd) {
        _write_fd.close();
    }
}


void AP_Logger_File::PrepForArming()
{
    if (logging_started()) {
        return;
    }
    start_new_log();
}

/*
  start writing to a new log file
 */
uint16_t AP_Logger_File::start_new_log(void)
{
    stop_logging();

    start_new_log_reset_variables();

    if (_open_error) {
        // we have previously failed to open a file - don't try again
        // to prevent us trying to open files while in flight
        return 0xFFFF;
    }

    if (_read_fd) {
        _read_fd.close();
    }

    uint16_t log_num = find_last_log();
    // re-use empty logs if possible
    if (_get_log_size(log_num) > 0 || log_num == 0) {
        log_num++;
    }
    if (log_num > MAX_LOG_FILES) {
        log_num = 1;
    }
    _cached_oldest_log = 0;

    bool was_ovf=false;

    char *fname;
    while(1) { // try to create log file

        fname = _log_file_name(log_num);
        if (fname == nullptr) {
            _open_error = true;
            return 0xFFFF;      // no memory
        }

        _write_fd = SD.open(fname, O_WRITE|O_CREAT|O_TRUNC);

        if (_write_fd) {     // file opened
            free(fname);
            break;
        }
        
        // opening failed
        printf("Log open fail for %s: %s\n",fname, SD.strError(SD.lastError));
        free(fname);
        if(SD.lastError == FR_DISK_ERR) {
                _initialised = false; // no space
                _open_error = true;   // don't try any more
                printf("\nLoging aborted\n");
                return 0xFFFF;
        }
        
        log_num++;                          // if not at end - try to open next log
            
        if (log_num >= MAX_LOG_FILES) {
            log_num = 1;
            if(was_ovf) {
                _initialised = false; // no space
                _open_error = true;   // don't try any more
                printf("\nLoging stopped\n");
                return 0xFFFF;
            }
            was_ovf=true;
        }
    }
    _write_offset = 0;
    _writebuf.clear();
    has_data = false;

    // now update lastlog.txt with the new log number
    fname = _lastlog_file_name();

    File fd = SD.open(fname, O_WRITE|O_CREAT);
    free(fname);
    if (!fd) {
        return 0xFFFF;
    }

    char buf[30];
    snprintf(buf, sizeof(buf), "%u\r\n", (unsigned)log_num);
    const ssize_t to_write = strlen(buf);
    const ssize_t written = fd.write((uint8_t *)buf, to_write);
    fd.close();

    if (written < to_write) {
        return 0xFFFF;
    }

    return log_num;
}

void AP_Logger_File::_io_timer(void)
{
    uint32_t tnow = AP_HAL::millis();
    _io_timer_heartbeat = tnow;

    if (!(_write_fd) || !_initialised || _open_error || !has_data) {
        return;
    }

    uint32_t nbytes = _writebuf.available();
    if (nbytes == 0) {
        return;
    }
    if (nbytes < _writebuf_chunk && 
        tnow - _last_write_time < 2000UL) {
        // write in _writebuf_chunk-sized chunks, but always write at
        // least once per 2 seconds if data is available
        has_data=false;
        return;
    }

    _last_write_time = tnow;
    if (nbytes > _writebuf_chunk) {
        // be kind to the FAT filesystem
        nbytes = _writebuf_chunk;
    }

    uint32_t size;
    const uint8_t *head = _writebuf.readptr(size);
    nbytes = MIN(nbytes, size);

    // try to align writes on a 512 byte boundary to avoid filesystem reads
    if ((nbytes + _write_offset) % 512 != 0) {
        uint32_t ofs = (nbytes + _write_offset) % 512;
        if (ofs < nbytes) {
            nbytes -= ofs;
        }
    }
    
    if(nbytes==0) return;

    ssize_t nwritten = _write_fd.write(head, nbytes);
    if (nwritten <= 0) {
        FRESULT err=SD.lastError;
        printf("\nLog write %ld bytes fails: %s\n",nbytes, SD.strError(err));
        gcs().send_text(MAV_SEVERITY_WARNING,"Log write %ld bytes fails: %s",nbytes, SD.strError(err));
//        stop_logging();
        _write_fd.close();
#if defined(BOARD_DATAFLASH_FATFS)
        if(FR_INT_ERR == err || FR_NO_FILESYSTEM == err) { // internal error - bad filesystem
            gcs().send_text(MAV_SEVERITY_INFO, "Formatting AP_Logger, please wait");
            uint32_t t=AP_HAL::millis();            
            _busy = true; // format requires a long time and 1s task will kill process
            SD.format(_log_directory);
            _busy = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Formatting complete in %ldms", AP_HAL::millis() - t);
            start_new_log();            // re-open logging
            if(_write_fd) {             // success?
                nwritten = _write_fd.write(head, nbytes); // ok, try to write again
                if(nwritten>0) {                        // if ok 
                    _write_offset += nwritten;          //   then mark data as written
                    _writebuf.advance(nwritten);
                    _write_fd.sync();                  //   and fix it on SD
                    return; 
                }
            }
        } else 
#else
        if(FR_INT_ERR == err || FR_NO_FILESYSTEM == err || FR_INVALID_OBJECT == err) { // internal error - bad filesystem
            gcs().send_text(MAV_SEVERITY_INFO, "logging cancelled");
            _initialised = false;
            _open_error = true;
        } else 
                
#endif
        {
            _busy = true; // Prep_MinSpace requires a long time and 1s task will kill process
            Prep_MinSpace();
            _busy = false;
            start_new_log();             // re-open logging
            if(_write_fd) {             // success?
                nwritten = _write_fd.write(head, nbytes); // ok, try to write again
                if(nwritten>0) {                        // if ok 
                    _write_offset += nwritten;          //   then mark data as written
                    _writebuf.advance(nwritten);
                    _write_fd.sync();                   //   and fix it on SD
                    return; 
                }
            }
        }    

        _initialised = false;
    } else {
        _write_offset += nwritten;
        _writebuf.advance(nwritten);
        /*
          the best strategy for minimizing corruption on microSD cards
          seems to be to write in 4k chunks and fsync the file on each
          chunk, ensuring the directory entry is updated after each
          write.
         */
        _write_fd.sync();
        
#if defined(BOARD_DATAFLASH_FATFS)    // limit file size in some MBytes and reopen new log file

        if(_write_fd.size() >= MAX_FILE_SIZE) { // size > 2M
            stop_logging(); 
            uint32_t t = AP_HAL::millis();
            _busy = true;
            Prep_MinSpace();
            _busy = false;
            printf("\nlog file reopened in %ldms\n", AP_HAL::millis() - t);
            start_new_log();             // re-start logging        
        }
#endif
    }
}

// this sensor is enabled if we should be logging at the moment
bool AP_Logger_File::logging_enabled() const
{
    if (hal.util->get_soft_armed() ||
        _front.log_while_disarmed()) {
        return true;
    }
    return false;
}

bool AP_Logger_File::io_thread_alive() const
{
    uint32_t tnow = AP_HAL::millis();
    // if the io thread hasn't had a heartbeat in a 5 second then it is dead
    if(_io_timer_heartbeat + 5000 > tnow) return true;
    
    return false;
}

bool AP_Logger_File::logging_failed() const
{
    bool op=false;
    
    if(_write_fd) op=true;

    if (!op &&
        (hal.util->get_soft_armed() ||
         _front.log_while_disarmed())) {
        return true;
    }
    if (_open_error) {
        return true;
    }
    if (!io_thread_alive()) {
        // No heartbeat in a second.  IO thread is dead?! Very Not Good.
        return true;
    }

    return false;
}


void AP_Logger_File::vehicle_was_disarmed()
{
    if (_front._params.file_disarm_rot) {
        // rotate our log.  Closing the current one and letting the
        // logging restart naturally based on log_disarmed should do
        // the trick:
        stop_logging();
    }
}


 
/////////////////////////////////////////////////////////////////////
//  функция конвертации между UNIX-временем и обычным представлением в виде даты и времени суток
#define _TBIAS_DAYS             ((70 * (uint32_t)365) + 17)
#define _TBIAS_SECS             (_TBIAS_DAYS * (xtime_t)86400)
#define _TBIAS_YEAR             1900
#define MONTAB(year)            ((((year) & 03) || ((year) == 0)) ? mos : lmos)
 
const uint16_t     lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const uint16_t     mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
 
#define Daysto32(year, mon)     (((year - 1) / 4) + MONTAB(year)[mon])
 
uint32_t AP_Logger_File::to_timestamp(const struct tm *t)
{       /* convert time structure to scalar time */
    int32_t           days;
    uint32_t          secs;
    int32_t           mon, year;
 
    /* Calculate number of days. */
    mon = t->tm_mon - 1;
    year = t->tm_year - _TBIAS_YEAR;
    days  = Daysto32(year, mon) - 1;
    days += 365 * year;
    days += t->tm_mday;
    days -= _TBIAS_DAYS;
 
    /* Calculate number of seconds. */
    secs  = 3600 * t->tm_hour;
    secs += 60 * t->tm_min;
    secs += t->tm_sec;
 
    secs += (days * (uint32_t)86400);
 
    return (secs);
}
#endif
