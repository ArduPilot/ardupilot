/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logs/NN.bin in the
   given directory
 */

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
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

extern const AP_HAL::HAL& hal;

#define MAX_LOG_FILES 500U
#define DATAFLASH_PAGE_SIZE 1024UL

int DataFlash_File::_write_fd = -1;
volatile bool DataFlash_File::_initialised = false;

uint8_t *DataFlash_File::_writebuf = NULL;
const uint16_t DataFlash_File::_writebuf_size = 4096;
volatile uint16_t DataFlash_File::_writebuf_head = 0;
volatile uint16_t DataFlash_File::_writebuf_tail = 0;
uint32_t DataFlash_File::_last_write_time = 0;

/*
  constructor
 */
DataFlash_File::DataFlash_File(const char *log_directory) :
    _read_fd(-1),
    _log_directory(log_directory)
{}


// initialisation
void DataFlash_File::Init(void)
{
    // create the log directory if need be
    int ret;
    ret = mkdir(_log_directory, 0777);
    if (ret == -1 && errno != EEXIST) {
        hal.console->printf("Failed to create log directory %s", _log_directory);
        return;
    }
    if (_writebuf != NULL) {
        free(_writebuf);
    }
    _writebuf = (uint8_t *)malloc(_writebuf_size);
    if (_writebuf == NULL) {
        return;
    }
    _writebuf_head = _writebuf_tail = 0;
    _initialised = true;
    hal.scheduler->register_io_process(_io_timer);
}

// return true for CardInserted() if we successfully initialised
bool DataFlash_File::CardInserted(void)
{
    return _initialised;
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
    asprintf(&buf, "%s/%u.bin", _log_directory, (unsigned)log_num);
    return buf;
}

/*
  return path name of the lastlog.txt marker file
  Note: Caller must free.
 */
char *DataFlash_File::_lastlog_file_name(void)
{
    char *buf = NULL;
    asprintf(&buf, "%s/lastlog.txt", _log_directory);
    return buf;
}


// remove all log files
void DataFlash_File::EraseAll()
{
    uint16_t log_num;
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

/*
  buffer handling macros
 */
#define BUF_AVAILABLE(buf) ((buf##_head > (_tail=buf##_tail))? (buf##_size - buf##_head) + _tail: _tail - buf##_head)
#define BUF_SPACE(buf) (((_head=buf##_head) > buf##_tail)?(_head - buf##_tail) - 1:((buf##_size - buf##_tail) + _head) - 1)
#define BUF_EMPTY(buf) (buf##_head == buf##_tail)
#define BUF_ADVANCETAIL(buf, n) buf##_tail = (buf##_tail + n) % buf##_size
#define BUF_ADVANCEHEAD(buf, n) buf##_head = (buf##_head + n) % buf##_size


/* Write a block of data at current offset */
void DataFlash_File::WriteBlock(const void *pBuffer, uint16_t size)
{
    if (_write_fd == -1 || !_initialised) {
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
    if (_read_fd == -1 || !_initialised) {
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

/*
  find the number of pages in a log
 */
void DataFlash_File::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page)
{
    start_page = 0;
    end_page = _get_log_size(log_num) / DATAFLASH_PAGE_SIZE;
}



/*
  get the number of logs - note that the log numbers must be consecutive
 */
uint16_t DataFlash_File::get_num_logs(void)
{
    uint16_t ret;
    uint16_t high = find_last_log();
    for (ret=1; ret<high; ret++) {
        if (_get_log_size(high - ret) <= 0) {
            break;
        }
    }
    return ret;
}


/*
  start writing to a new log file
 */
uint16_t DataFlash_File::start_new_log(void)
{
    if (_write_fd != -1) {
        int fd = _write_fd;
        _write_fd = -1;
        ::close(fd);
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
    free(fname);
    if (_write_fd == -1) {
        _initialised = false;
        return 0xFFFF;
    }

    // now update lastlog.txt with the new log number
    fname = _lastlog_file_name();
    FILE *f = ::fopen(fname, "w");
    fprintf(f, "%u\n", (unsigned)log_num);
    fclose(f);    
    free(fname);

    return log_num;
}

/*
  Read the log and print it on port
*/
void DataFlash_File::LogReadProcess(uint16_t log_num,
                                    uint16_t start_page, uint16_t end_page, 
                                    uint8_t num_types,
                                    const struct LogStructure *structure,
                                    void (*print_mode)(AP_HAL::BetterStream *port, uint8_t mode),
                                    AP_HAL::BetterStream *port)
{
    uint8_t log_step = 0;
    if (!_initialised) {
        return;
    }
    if (_read_fd != -1) {
        ::close(_read_fd);
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
    _read_offset = 0;
    if (start_page != 0) {
        ::lseek(_read_fd, start_page * DATAFLASH_PAGE_SIZE, SEEK_SET);
    }

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
                _print_log_entry(data, num_types, structure, print_mode, port);
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
                port->printf_P(PSTR("Log %u in %s of size %u\n"), 
                               (unsigned)log_num, 
                               filename,
                               (unsigned)size);
            }
            free(filename);
        }
    }
    port->println();    
}


void DataFlash_File::_io_timer(uint32_t tnow)
{
    uint16_t _tail;
    if (_write_fd == -1 || !_initialised) {
        return;
    }
    uint16_t nbytes = BUF_AVAILABLE(_writebuf);
    if (nbytes == 0) {
        return;
    }
    if (nbytes < 512 && 
        tnow - _last_write_time < 2000000UL) {
        // write in 512 byte chunks, but always write at least once
        // per 2 seconds if data is available
        return;
    }
    _last_write_time = tnow;
    if (nbytes > 512) {
        // be kind to the FAT PX4 filesystem
        nbytes = 512;
    }
    if (_writebuf_head > _tail) {
        // only write to the end of the buffer
        nbytes = min(nbytes, _writebuf_size - _writebuf_head);
    }
    assert(_writebuf_head+nbytes <= _writebuf_size);
    ssize_t nwritten = ::write(_write_fd, &_writebuf[_writebuf_head], nbytes);
    if (nwritten <= 0) {
        close(_write_fd);
        _write_fd = -1;
        _initialised = false;
    } else {
        ::fsync(_write_fd);
        BUF_ADVANCEHEAD(_writebuf, nwritten);
    }
}

#endif // CONFIG_HAL_BOARD
