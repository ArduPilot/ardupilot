/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logs/NN.bin in the
   given directory
 */

#include <AP_HAL.h>

#if HAL_OS_POSIX_IO
#include "DataFlash.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <AP_Math.h>
#include <stdio.h>
#include <time.h>
#include <dirent.h>
#include "../AP_HAL/utility/RingBuffer.h"

extern const AP_HAL::HAL& hal;

#define MAX_LOG_FILES 500U
#define DATAFLASH_PAGE_SIZE 1024UL

/*
  constructor
 */
DataFlash_File::DataFlash_File(const char *log_directory) :
    _write_fd(-1),
    _read_fd(-1),
    _read_offset(0),
    _write_offset(0),
    _initialised(false),
    _open_error(false),
    _log_directory(log_directory),
    _writebuf(NULL),
    _writebuf_size(16*1024),
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
    // V1 gets IO errors with larger than 512 byte writes
    _writebuf_chunk(512),
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V40)
    _writebuf_chunk(512),
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
    _writebuf_chunk(512),
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V50)
    _writebuf_chunk(512),
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
    _writebuf_chunk(512),
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
    _writebuf_chunk(512),
#elif defined(CONFIG_ARCH_BOARD_VRHERO_V10)
    _writebuf_chunk(512),
#else
    _writebuf_chunk(4096),
#endif
    _writebuf_head(0),
    _writebuf_tail(0),
    _last_write_time(0)
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    ,_perf_write(perf_alloc(PC_ELAPSED, "DF_write")),
    _perf_fsync(perf_alloc(PC_ELAPSED, "DF_fsync")),
    _perf_errors(perf_alloc(PC_COUNT, "DF_errors"))
#endif
{}


// initialisation
void DataFlash_File::Init(const struct LogStructure *structure, uint8_t num_types)
{
    DataFlash_Class::Init(structure, num_types);
    // create the log directory if need be
    int ret;
    struct stat st;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // try to cope with an existing lowercase log directory
    // name. NuttX does not handle case insensitive VFAT well
    DIR *d = opendir("/fs/microsd/APM");
    if (d != NULL) {
        for (struct dirent *de=readdir(d); de; de=readdir(d)) {
            if (strcmp(de->d_name, "logs") == 0) {
                rename("/fs/microsd/APM/logs", "/fs/microsd/APM/OLDLOGS");
                break;
            }
        }
        closedir(d);
    }
#endif

    ret = stat(_log_directory, &st);
    if (ret == -1) {
        ret = mkdir(_log_directory, 0777);
    }
    if (ret == -1) {
        hal.console->printf("Failed to create log directory %s", _log_directory);
        return;
    }
    if (_writebuf != NULL) {
        free(_writebuf);
    }

    /*
      if we can't allocate the full writebuf then try reducing it
      until we can allocate it
     */
    while (_writebuf == NULL && _writebuf_size >= _writebuf_chunk) {
        _writebuf = (uint8_t *)malloc(_writebuf_size);
        if (_writebuf == NULL) {
            _writebuf_size /= 2;
        }
    }
    if (_writebuf == NULL) {
        hal.console->printf("Out of memory for logging\n");
        return;        
    }
    _writebuf_head = _writebuf_tail = 0;
    _initialised = true;
    hal.scheduler->register_io_process(AP_HAL_MEMBERPROC(&DataFlash_File::_io_timer));
}

// return true for CardInserted() if we successfully initialised
bool DataFlash_File::CardInserted(void)
{
    return _initialised && !_open_error;
}


// erase handling
bool DataFlash_File::NeedErase(void)
{
    // we could add a format marker at the start of a file?
    return false;
}

/*
  construct a log file name given a log number. 
  Note: Caller must free.
 */
char *DataFlash_File::_log_file_name(uint16_t log_num)
{
    char *buf = NULL;
    asprintf(&buf, "%s/%u.BIN", _log_directory, (unsigned)log_num);
    return buf;
}

/*
  return path name of the lastlog.txt marker file
  Note: Caller must free.
 */
char *DataFlash_File::_lastlog_file_name(void)
{
    char *buf = NULL;
    asprintf(&buf, "%s/LASTLOG.TXT", _log_directory);
    return buf;
}


// remove all log files
void DataFlash_File::EraseAll()
{
    uint16_t log_num;
    stop_logging();
    for (log_num=0; log_num<MAX_LOG_FILES; log_num++) {
        char *fname = _log_file_name(log_num);
        if (fname == NULL) {
            break;
        }
        unlink(fname);
        free(fname);
    }
    char *fname = _lastlog_file_name();
    if (fname != NULL) {
        unlink(fname);
        free(fname);
    }
}

/* Write a block of data at current offset */
void DataFlash_File::WriteBlock(const void *pBuffer, uint16_t size)
{
    if (_write_fd == -1 || !_initialised || _open_error || !_writes_enabled) {
        return;
    }
    uint16_t _head;
    uint16_t space = BUF_SPACE(_writebuf);
    if (space < size) {
        // discard the whole write, to keep the log consistent
        return;
    }

    if (_writebuf_tail < _head) {
        // perform as single memcpy
        assert(_writebuf_tail+size <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], pBuffer, size);
        BUF_ADVANCETAIL(_writebuf, size);
    } else {
        // perform as two memcpy calls
        uint16_t n = _writebuf_size - _writebuf_tail;
        if (n > size) n = size;
        assert(_writebuf_tail+n <= _writebuf_size);
        memcpy(&_writebuf[_writebuf_tail], pBuffer, n);
        BUF_ADVANCETAIL(_writebuf, n);
        pBuffer = (const void *)(((const uint8_t *)pBuffer) + n);
        n = size - n;
        if (n > 0) {
            assert(_writebuf_tail+n <= _writebuf_size);
            memcpy(&_writebuf[_writebuf_tail], pBuffer, n);
            BUF_ADVANCETAIL(_writebuf, n);
        }
    }
}

/*
  read a packet. The header bytes have already been read.
*/
void DataFlash_File::ReadBlock(void *pkt, uint16_t size)
{
    if (_read_fd == -1 || !_initialised || _open_error) {
        return;
    }

    memset(pkt, 0, size);
    ::read(_read_fd, pkt, size);
    _read_offset += size;
}


/*
  find the highest log number
 */
uint16_t DataFlash_File::find_last_log(void)
{
    unsigned ret = 0;
    char *fname = _lastlog_file_name();
    if (fname == NULL) {
        return ret;
    }
    FILE *f = ::fopen(fname, "r");
    free(fname);
    if (f != NULL) {
        char buf[10];
        memset(buf, 0, sizeof(buf));
        // PX4 doesn't have fscanf()
        if (fread(buf, 1, sizeof(buf)-1, f) > 0) {
            sscanf(buf, "%u", &ret);            
        }
        fclose(f);    
    }
    return ret;
}


uint32_t DataFlash_File::_get_log_size(uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == NULL) {
        return 0;
    }
    struct stat st;
    if (::stat(fname, &st) != 0) {
        free(fname);
        return 0;
    }
    free(fname);
    return st.st_size;
}

uint32_t DataFlash_File::_get_log_time(uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == NULL) {
        return 0;
    }
    struct stat st;
    if (::stat(fname, &st) != 0) {
        free(fname);
        return 0;
    }
    free(fname);
    return st.st_mtime;
}

/*
  find the number of pages in a log
 */
void DataFlash_File::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page)
{
    start_page = 0;
    end_page = _get_log_size(log_num) / DATAFLASH_PAGE_SIZE;
}

/*
  find the number of pages in a log
 */
int16_t DataFlash_File::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data)
{
    if (!_initialised || _open_error) {
        return -1;
    }
    if (_read_fd != -1 && log_num != _read_fd_log_num) {
        ::close(_read_fd);
        _read_fd = -1;
    }
    if (_read_fd == -1) {
        char *fname = _log_file_name(log_num);
        if (fname == NULL) {
            return -1;
        }
        stop_logging();
        _read_fd = ::open(fname, O_RDONLY);
        if (_read_fd == -1) {
            _open_error = true;
            int saved_errno = errno;
            ::printf("Log read open fail for %s - %s\n",
                     fname, strerror(saved_errno));
            hal.console->printf("Log read open fail for %s - %s\n",
                                fname, strerror(saved_errno));
            free(fname);
            return -1;            
        }
        free(fname);
        _read_offset = 0;
        _read_fd_log_num = log_num;
    }
    uint32_t ofs = page * (uint32_t)DATAFLASH_PAGE_SIZE + offset;

    /*
      this rather strange bit of code is here to work around a bug
      in file offsets in NuttX. Every few hundred blocks of reads
      (starting at around 350k into a file) NuttX will get the
      wrong offset for sequential reads. The offset it gets is
      typically 128k earlier than it should be. It turns out that
      calling lseek() with 0 offset and SEEK_CUR works around the
      bug. We can remove this once we find the real bug.
    */
    if (ofs / 4096 != (ofs+len) / 4096) {
        off_t seek_current = ::lseek(_read_fd, 0, SEEK_CUR);
        if (seek_current != (off_t)_read_offset) {
            ::lseek(_read_fd, _read_offset, SEEK_SET);
        }
    }

    if (ofs != _read_offset) {
        ::lseek(_read_fd, ofs, SEEK_SET);
        _read_offset = ofs;
    }
    int16_t ret = (int16_t)::read(_read_fd, data, len);
    if (ret > 0) {
        _read_offset += ret;
    }
    return ret;
}

/*
  find size and date of a log
 */
void DataFlash_File::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc)
{
    size = _get_log_size(log_num);
    time_utc = _get_log_time(log_num);
}



/*
  get the number of logs - note that the log numbers must be consecutive
 */
uint16_t DataFlash_File::get_num_logs(void)
{
    uint16_t ret;
    uint16_t high = find_last_log();
    for (ret=0; ret<high; ret++) {
        if (_get_log_size(high - ret) <= 0) {
            break;
        }
    }
    return ret;
}

/*
  stop logging
 */
void DataFlash_File::stop_logging(void)
{
    if (_write_fd != -1) {
        int fd = _write_fd;
        _write_fd = -1;
        log_write_started = false;
        ::close(fd);
    }
}


/*
  start writing to a new log file
 */
uint16_t DataFlash_File::start_new_log(void)
{
    stop_logging();

    if (_open_error) {
        // we have previously failed to open a file - don't try again
        // to prevent us trying to open files while in flight
        return 0xFFFF;
    }

    if (_read_fd != -1) {
        ::close(_read_fd);
        _read_fd = -1;
    }

    uint16_t log_num = find_last_log();
    // re-use empty logs if possible
    if (_get_log_size(log_num) > 0 || log_num == 0) {
        log_num++;
    }
    if (log_num > MAX_LOG_FILES) {
        log_num = 1;
    }
    char *fname = _log_file_name(log_num);
    _write_fd = ::open(fname, O_WRONLY|O_CREAT|O_TRUNC, 0666);
    if (_write_fd == -1) {
        _initialised = false;
        _open_error = true;
        int saved_errno = errno;
        ::printf("Log open fail for %s - %s\n",
                 fname, strerror(saved_errno));
        hal.console->printf("Log open fail for %s - %s\n",
                            fname, strerror(saved_errno));
        free(fname);
        return 0xFFFF;
    }
    free(fname);
    _write_offset = 0;
    _writebuf_head = 0;
    _writebuf_tail = 0;
    log_write_started = true;

    // now update lastlog.txt with the new log number
    fname = _lastlog_file_name();
    FILE *f = ::fopen(fname, "w");
    fprintf(f, "%u\r\n", (unsigned)log_num);
    fclose(f);    
    free(fname);

    return log_num;
}

/*
  Read the log and print it on port
*/
void DataFlash_File::LogReadProcess(uint16_t log_num,
                                    uint16_t start_page, uint16_t end_page, 
                                    void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                    AP_HAL::BetterStream *port)
{
    uint8_t log_step = 0;
    if (!_initialised || _open_error) {
        return;
    }
    if (_read_fd != -1) {
        ::close(_read_fd);
        _read_fd = -1;
    }
    char *fname = _log_file_name(log_num);
    if (fname == NULL) {
        return;
    }
    _read_fd = ::open(fname, O_RDONLY);
    free(fname);
    if (_read_fd == -1) {
        return;
    }
    _read_fd_log_num = log_num;
    _read_offset = 0;
    if (start_page != 0) {
        ::lseek(_read_fd, start_page * DATAFLASH_PAGE_SIZE, SEEK_SET);
        _read_offset = start_page * DATAFLASH_PAGE_SIZE;
    }

    uint8_t log_counter = 0;

    while (true) {
        uint8_t data;
        if (::read(_read_fd, &data, 1) != 1) {
            // reached end of file
            break;
        }
        _read_offset++;

        // This is a state machine to read the packets
        switch(log_step) {
            case 0:
                if (data == HEAD_BYTE1) {
                    log_step++;
                }
                break;

            case 1:
                if (data == HEAD_BYTE2) {
                    log_step++;
                } else {
                    log_step = 0;
                }
                break;

            case 2:
                log_step = 0;
                _print_log_entry(data, print_mode, port);
                log_counter++;
                if (log_counter == 10) {
                    log_counter = 0;
                    ::lseek(_read_fd, 0, SEEK_CUR);
                }
                break;
        }
        if (_read_offset >= (end_page+1) * DATAFLASH_PAGE_SIZE) {
            break;
        }
    }

    ::close(_read_fd);
    _read_fd = -1;
}

/*
  this is a lot less verbose than the block interface. Dumping 2Gbyte
  of logs a page at a time isn't so useful. Just pull the SD card out
  and look at it on your PC
 */
void DataFlash_File::DumpPageInfo(AP_HAL::BetterStream *port)
{
    port->printf_P(PSTR("DataFlash: num_logs=%u\n"), 
                   (unsigned)get_num_logs());    
}

void DataFlash_File::ShowDeviceInfo(AP_HAL::BetterStream *port)
{
    port->printf_P(PSTR("DataFlash logs stored in %s\n"), 
                   _log_directory);
}


/*
  list available log numbers
 */
void DataFlash_File::ListAvailableLogs(AP_HAL::BetterStream *port)
{
    uint16_t num_logs = get_num_logs();
    int16_t last_log_num = find_last_log();

    if (num_logs == 0) {
        port->printf_P(PSTR("\nNo logs\n\n"));
        return;
    }
    port->printf_P(PSTR("\n%u logs\n"), (unsigned)num_logs);

    for (uint16_t i=num_logs; i>=1; i--) {
        uint16_t log_num = last_log_num - i + 1;
        off_t size;

        char *filename = _log_file_name(log_num);
        if (filename != NULL) {
            size = _get_log_size(log_num);
            if (size != 0) {
                struct stat st;
                if (stat(filename, &st) == 0) {
                    struct tm *tm = gmtime(&st.st_mtime);
                    port->printf_P(PSTR("Log %u in %s of size %u %u/%u/%u %u:%u\n"), 
                                   (unsigned)log_num, 
                                   filename,
                                   (unsigned)size,
                                   (unsigned)tm->tm_year+1900,
                                   (unsigned)tm->tm_mon+1,
                                   (unsigned)tm->tm_mday,
                                   (unsigned)tm->tm_hour,
                                   (unsigned)tm->tm_min);
                }
            }
            free(filename);
        }
    }
    port->println();    
}


void DataFlash_File::_io_timer(void)
{
    uint16_t _tail;
    if (_write_fd == -1 || !_initialised || _open_error) {
        return;
    }

    uint16_t nbytes = BUF_AVAILABLE(_writebuf);
    if (nbytes == 0) {
        return;
    }
    uint32_t tnow = hal.scheduler->micros();
    if (nbytes < _writebuf_chunk && 
        tnow - _last_write_time < 2000000UL) {
        // write in 512 byte chunks, but always write at least once
        // per 2 seconds if data is available
        return;
    }

    perf_begin(_perf_write);

    _last_write_time = tnow;
    if (nbytes > _writebuf_chunk) {
        // be kind to the FAT PX4 filesystem
        nbytes = _writebuf_chunk;
    }
    if (_writebuf_head > _tail) {
        // only write to the end of the buffer
        nbytes = min(nbytes, _writebuf_size - _writebuf_head);
    }

    // try to align writes on a 512 byte boundary to avoid filesystem
    // reads
    if ((nbytes + _write_offset) % 512 != 0) {
        uint32_t ofs = (nbytes + _write_offset) % 512;
        if (ofs < nbytes) {
            nbytes -= ofs;
        }
    }

    assert(_writebuf_head+nbytes <= _writebuf_size);
    ssize_t nwritten = ::write(_write_fd, &_writebuf[_writebuf_head], nbytes);
    if (nwritten <= 0) {
        perf_count(_perf_errors);
        close(_write_fd);
        _write_fd = -1;
        _initialised = false;
    } else {
        _write_offset += nwritten;
        /*
          the best strategy for minimising corruption on microSD cards
          seems to be to write in 4k chunks and fsync the file on each
          chunk, ensuring the directory entry is updated after each
          write.
         */
#if CONFIG_HAL_BOARD != HAL_BOARD_AVR_SITL
        ::fsync(_write_fd);
#endif
        BUF_ADVANCEHEAD(_writebuf, nwritten);
    }
    perf_end(_perf_write);
}

#endif // HAL_OS_POSIX_IO

