#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>

#include "Storage.h"
using namespace PX4;

/*
  This stores eeprom data in the PX4 MTD interface with a 4k size, and
  a in-memory buffer. This keeps the latency and devices IOs down.
 */

// name the storage file after the sketch so you can use the same sd
// card for ArduCopter and ArduPlane
#define STORAGE_DIR "/fs/microsd/APM"
#define OLD_STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".stg"
#define OLD_STORAGE_FILE_BAK STORAGE_DIR "/" SKETCHNAME ".bak"
//#define SAVE_STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".sav"
#define MTD_PARAMS_FILE "/fs/mtd"
#define MTD_SIGNATURE 0x14012014
#define MTD_SIGNATURE_OFFSET (8192-4)
#define STORAGE_RENAME_OLD_FILE 0

extern const AP_HAL::HAL& hal;

PX4Storage::PX4Storage(void) :
    _fd(-1),
    _dirty_mask(0),
    _perf_storage(perf_alloc(PC_ELAPSED, "APM_storage")),
    _perf_errors(perf_alloc(PC_COUNT, "APM_storage_errors"))
{
}

/*
  get signature from bytes at offset MTD_SIGNATURE_OFFSET
 */
uint32_t PX4Storage::_mtd_signature(void)
{
    int mtd_fd = open(MTD_PARAMS_FILE, O_RDONLY);
    if (mtd_fd == -1) {
        AP_HAL::panic("Failed to open " MTD_PARAMS_FILE);
    }
    uint32_t v;
    if (lseek(mtd_fd, MTD_SIGNATURE_OFFSET, SEEK_SET) != MTD_SIGNATURE_OFFSET) {
        AP_HAL::panic("Failed to seek in " MTD_PARAMS_FILE);
    }
    if (read(mtd_fd, &v, sizeof(v)) != sizeof(v)) {
        AP_HAL::panic("Failed to read signature from " MTD_PARAMS_FILE);
    }
    close(mtd_fd);
    return v;
}

/*
  put signature bytes at offset MTD_SIGNATURE_OFFSET
 */
void PX4Storage::_mtd_write_signature(void)
{
    int mtd_fd = open(MTD_PARAMS_FILE, O_WRONLY);
    if (mtd_fd == -1) {
        AP_HAL::panic("Failed to open " MTD_PARAMS_FILE);
    }
    uint32_t v = MTD_SIGNATURE;
    if (lseek(mtd_fd, MTD_SIGNATURE_OFFSET, SEEK_SET) != MTD_SIGNATURE_OFFSET) {
        AP_HAL::panic("Failed to seek in " MTD_PARAMS_FILE);
    }
    if (write(mtd_fd, &v, sizeof(v)) != sizeof(v)) {
        AP_HAL::panic("Failed to write signature in " MTD_PARAMS_FILE);
    }
    close(mtd_fd);
}

/*
  upgrade from microSD to MTD (FRAM)
 */
void PX4Storage::_upgrade_to_mtd(void)
{
    // the MTD is completely uninitialised - try to get a
    // copy from OLD_STORAGE_FILE
    int old_fd = open(OLD_STORAGE_FILE, O_RDONLY);
    if (old_fd == -1) {
        ::printf("Failed to open %s\n", OLD_STORAGE_FILE);
        return;
    }

    int mtd_fd = open(MTD_PARAMS_FILE, O_WRONLY);
    if (mtd_fd == -1) {
        AP_HAL::panic("Unable to open MTD for upgrade");
    }

    if (::read(old_fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
        close(old_fd);
        close(mtd_fd);
        ::printf("Failed to read %s\n", OLD_STORAGE_FILE);
        return;        
    }
    close(old_fd);
    ssize_t ret;
    if ((ret=::write(mtd_fd, _buffer, sizeof(_buffer))) != sizeof(_buffer)) {
        ::printf("mtd write of %u bytes returned %d errno=%d\n", sizeof(_buffer), ret, errno);
        AP_HAL::panic("Unable to write MTD for upgrade");
    }
    close(mtd_fd);
#if STORAGE_RENAME_OLD_FILE
    rename(OLD_STORAGE_FILE, OLD_STORAGE_FILE_BAK);
#endif
    ::printf("Upgraded MTD from %s\n", OLD_STORAGE_FILE);
}
            

void PX4Storage::_storage_open(void)
{
	if (_initialised) {
		return;
	}

        struct stat st;
        _have_mtd = (stat(MTD_PARAMS_FILE, &st) == 0);

        // PX4 should always have /fs/mtd_params
        if (!_have_mtd) {
            AP_HAL::panic("Failed to find " MTD_PARAMS_FILE);
        }

        /*
          cope with upgrading from OLD_STORAGE_FILE to MTD
         */
        bool good_signature = (_mtd_signature() == MTD_SIGNATURE);
        if (stat(OLD_STORAGE_FILE, &st) == 0) {
            if (good_signature) {
#if STORAGE_RENAME_OLD_FILE
                rename(OLD_STORAGE_FILE, OLD_STORAGE_FILE_BAK);
#endif
            } else {
                _upgrade_to_mtd();
            }
        }

        // we write the signature every time, even if it already is
        // good, as this gives us a way to detect if the MTD device is
        // functional. It is better to panic now than to fail to save
        // parameters in flight
        _mtd_write_signature();

	_dirty_mask = 0;
	int fd = open(MTD_PARAMS_FILE, O_RDONLY);
	if (fd == -1) {
            AP_HAL::panic("Failed to open " MTD_PARAMS_FILE);
	}
        const uint16_t chunk_size = 128;
        for (uint16_t ofs=0; ofs<sizeof(_buffer); ofs += chunk_size) {
            ssize_t ret = read(fd, &_buffer[ofs], chunk_size);
            if (ret != chunk_size) {
                ::printf("storage read of %u bytes at %u to %p failed - got %d errno=%d\n",
                         (unsigned)sizeof(_buffer), (unsigned)ofs, &_buffer[ofs], (int)ret, (int)errno);
                AP_HAL::panic("Failed to read " MTD_PARAMS_FILE);
            }
	}
	close(fd);

#ifdef SAVE_STORAGE_FILE
        fd = open(SAVE_STORAGE_FILE, O_WRONLY|O_CREAT|O_TRUNC, 0644);
        if (fd != -1) {
            write(fd, _buffer, sizeof(_buffer));
            close(fd);
            ::printf("Saved storage file %s\n", SAVE_STORAGE_FILE);
        }
#endif
	_initialised = true;
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
 */
void PX4Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    uint16_t end = loc + length;
    for (uint8_t line=loc>>PX4_STORAGE_LINE_SHIFT;
         line <= end>>PX4_STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask |= 1U << line;
    }
}

void PX4Storage::read_block(void *dst, uint16_t loc, size_t n) 
{
	if (loc >= sizeof(_buffer)-(n-1)) {
		return;
	}
	_storage_open();
	memcpy(dst, &_buffer[loc], n);
}

void PX4Storage::write_block(uint16_t loc, const void *src, size_t n) 
{
	if (loc >= sizeof(_buffer)-(n-1)) {
		return;
	}
	if (memcmp(src, &_buffer[loc], n) != 0) {
		_storage_open();
		memcpy(&_buffer[loc], src, n);
		_mark_dirty(loc, n);
	}
}

void PX4Storage::_timer_tick(void)
{
	if (!_initialised || _dirty_mask == 0) {
		return;
	}
	perf_begin(_perf_storage);

	if (_fd == -1) {
		_fd = open(MTD_PARAMS_FILE, O_WRONLY);
		if (_fd == -1) {
			perf_end(_perf_storage);
			perf_count(_perf_errors);
			return;	
		}
	}

	// write out the first dirty set of lines. We don't write more
	// than one to keep the latency of this call to a minimum
	uint8_t i, n;
	for (i=0; i<PX4_STORAGE_NUM_LINES; i++) {
		if (_dirty_mask & (1<<i)) {
			break;
		}
	}
	if (i == PX4_STORAGE_NUM_LINES) {
		// this shouldn't be possible
		perf_end(_perf_storage);
		perf_count(_perf_errors);
		return;
	}
	uint32_t write_mask = (1U<<i);
	// see how many lines to write
	for (n=1; (i+n) < PX4_STORAGE_NUM_LINES && 
		     n < (PX4_STORAGE_MAX_WRITE>>PX4_STORAGE_LINE_SHIFT); n++) {
		if (!(_dirty_mask & (1<<(n+i)))) {
			break;
		}		
		// mark that line clean
		write_mask |= (1<<(n+i));
	}

	/*
	  write the lines. This also updates _dirty_mask. Note that
	  because this is a SCHED_FIFO thread it will not be preempted
	  by the main task except during blocking calls. This means we
	  don't need a semaphore around the _dirty_mask updates.
	 */
	if (lseek(_fd, i<<PX4_STORAGE_LINE_SHIFT, SEEK_SET) == (i<<PX4_STORAGE_LINE_SHIFT)) {
		_dirty_mask &= ~write_mask;
		if (write(_fd, &_buffer[i<<PX4_STORAGE_LINE_SHIFT], n<<PX4_STORAGE_LINE_SHIFT) != n<<PX4_STORAGE_LINE_SHIFT) {
			// write error - likely EINTR
			_dirty_mask |= write_mask;
			close(_fd);
			_fd = -1;
			perf_count(_perf_errors);
		}
	}
	perf_end(_perf_storage);
}

#endif // CONFIG_HAL_BOARD
