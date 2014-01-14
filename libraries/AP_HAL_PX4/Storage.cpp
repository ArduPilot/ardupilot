#include <AP_HAL.h>
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
  This stores 'eeprom' data on the SD card, with a 4k size, and a
  in-memory buffer. This keeps the latency down.
 */

// name the storage file after the sketch so you can use the same sd
// card for ArduCopter and ArduPlane
#define STORAGE_DIR "/fs/microsd/APM"
#define OLD_STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".stg"
#define OLD_STORAGE_FILE_BAK STORAGE_DIR "/" SKETCHNAME ".bak"
#define MTD_PARAMS_FILE "/fs/mtd_params"
#define MTD_SIGNATURE 0x14012014
#define STORAGE_RENAME_OLD_FILE 0

extern const AP_HAL::HAL& hal;

PX4Storage::PX4Storage(void) :
    _fd(-1),
    _dirty_mask(0),
    _perf_storage(perf_alloc(PC_ELAPSED, "APM_storage")),
    _perf_errors(perf_alloc(PC_COUNT, "APM_storage_errors"))
{
}

void PX4Storage::_storage_create(void)
{
	mkdir(STORAGE_DIR, 0777);
	unlink(_storage_name);
        int fd = open(_storage_name, O_RDWR|O_CREAT, 0666);
	if (fd == -1) {
		hal.scheduler->panic("Failed to create " OLD_STORAGE_FILE);
	}
	for (uint16_t loc=0; loc<sizeof(_buffer); loc += PX4_STORAGE_MAX_WRITE) {
		if (write(fd, &_buffer[loc], PX4_STORAGE_MAX_WRITE) != PX4_STORAGE_MAX_WRITE) {
			hal.scheduler->panic("Error filling " OLD_STORAGE_FILE);			
		}
	}
	// ensure the directory is updated with the new size
	fsync(fd);
	close(fd);
}

/*
  get signature from bytes at offset 8192-4
 */
uint32_t PX4Storage::_mtd_signature(void)
{
    int mtd_fd = open(_storage_name, O_RDONLY);
    if (mtd_fd == -1) {
        hal.scheduler->panic("Failed to open " MTD_PARAMS_FILE);
    }
    uint32_t v;
    if (lseek(mtd_fd, 8192-4, SEEK_SET) != 8192-4) {
        hal.scheduler->panic("Failed to seek in " MTD_PARAMS_FILE);
    }
    if (read(mtd_fd, &v, sizeof(v)) != sizeof(v)) {
        hal.scheduler->panic("Failed to read signature from " MTD_PARAMS_FILE);
    }
    close(mtd_fd);
    return v;
}

/*
  put signature bytes at offset 8192-4
 */
void PX4Storage::_mtd_write_signature(void)
{
    int mtd_fd = open(_storage_name, O_WRONLY);
    if (mtd_fd == -1) {
        hal.scheduler->panic("Failed to open " MTD_PARAMS_FILE);
    }
    uint32_t v = MTD_SIGNATURE;
    if (lseek(mtd_fd, 8192-4, SEEK_SET) != 8192-4) {
        hal.scheduler->panic("Failed to seek in " MTD_PARAMS_FILE);
    }
    if (write(mtd_fd, &v, sizeof(v)) != sizeof(v)) {
        hal.scheduler->panic("Failed to write signature in " MTD_PARAMS_FILE);
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

    int mtd_fd = open(_storage_name, O_WRONLY);
    if (mtd_fd == -1) {
        hal.scheduler->panic("Unable to open MTD for upgrade");
    }

    if (::read(old_fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
        close(old_fd);
        close(mtd_fd);
        ::printf("Failed to read %s\n", OLD_STORAGE_FILE);
        return;        
    }
    close(old_fd);
    if (::write(mtd_fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
        hal.scheduler->panic("Unable to write MTD for upgrade");        
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

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
        // FMUv2 should always have /fs/mtd_params
        if (!_have_mtd) {
            hal.scheduler->panic("Failed to find " MTD_PARAMS_FILE);
        }
#endif

        if (_have_mtd) {
            _storage_name = MTD_PARAMS_FILE;
        } else {
            // NuttX seems to be happier not having the extra directory
            // entries for mapping the case of filenames. Keeping it all
            // uppercase seems to reduce meta-data corruption when rebooting
            // while writing
            if (strcmp(SKETCHNAME, "ArduPlane") == 0) {
                _storage_name = STORAGE_DIR "/" "PLANE.STG";
            } else if (strcmp(SKETCHNAME, "ArduCopter") == 0) {
                _storage_name = STORAGE_DIR "/" "COPTER.STG";
            } else if (strcmp(SKETCHNAME, "APMrover2") == 0) {
                _storage_name = STORAGE_DIR "/" "ROVER.STG";
            } else {
                _storage_name = OLD_STORAGE_FILE;
            }
        }

        /*
          cope with upgrading from OLD_STORAGE_FILE to MTD
         */
        if (_have_mtd) {
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
            if (!good_signature) {
                _mtd_write_signature();
            }
        }

        /**
           NuttX is not properly case insensitive on VFAT. We will create
           the file as uppercase, but if the file already exists we need
           to rename it first
        */
        if (!_have_mtd && stat(_storage_name, &st) == -1 && stat(OLD_STORAGE_FILE, &st) == 0) {
            ::printf("Renaming %s to %s\n", OLD_STORAGE_FILE, _storage_name);
            if (rename(OLD_STORAGE_FILE, _storage_name) != 0) {
                unlink(OLD_STORAGE_FILE);
            }
        }

	_dirty_mask = 0;
	int fd = open(_storage_name, O_RDONLY);
	if (fd == -1) {
		_storage_create();
		fd = open(_storage_name, O_RDONLY);
		if (fd == -1) {
			hal.scheduler->panic("Failed to open " OLD_STORAGE_FILE);
		}
	}
	if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
		close(fd);
		_storage_create();
		fd = open(_storage_name, O_RDONLY);
		if (fd == -1) {
			hal.scheduler->panic("Failed to open " OLD_STORAGE_FILE);
		}
                ssize_t ret;
		if ((ret=read(fd, _buffer, sizeof(_buffer))) != sizeof(_buffer)) {
                    ::printf("read: %d errno=%d\n", (int)ret, (int)errno);
			hal.scheduler->panic("Failed to read " OLD_STORAGE_FILE);
		}
	}
	close(fd);
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
	while (loc < end) {
		uint8_t line = (loc >> PX4_STORAGE_LINE_SHIFT);
		_dirty_mask |= 1 << line;
		loc += PX4_STORAGE_LINE_SIZE;
	}
}

uint8_t PX4Storage::read_byte(uint16_t loc) 
{
	if (loc >= sizeof(_buffer)) {
		return 0;
	}
	_storage_open();
	return _buffer[loc];
}

uint16_t PX4Storage::read_word(uint16_t loc) 
{
	uint16_t value;
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return 0;
	}
	_storage_open();
	memcpy(&value, &_buffer[loc], sizeof(value));
	return value;
}

uint32_t PX4Storage::read_dword(uint16_t loc) 
{
	uint32_t value;
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return 0;
	}
	_storage_open();
	memcpy(&value, &_buffer[loc], sizeof(value));
	return value;
}

void PX4Storage::read_block(void *dst, uint16_t loc, size_t n) 
{
	if (loc >= sizeof(_buffer)-(n-1)) {
		return;
	}
	_storage_open();
	memcpy(dst, &_buffer[loc], n);
}

void PX4Storage::write_byte(uint16_t loc, uint8_t value) 
{
	if (loc >= sizeof(_buffer)) {
		return;
	}
	if (_buffer[loc] != value) {
		_storage_open();
		_buffer[loc] = value;
		_mark_dirty(loc, sizeof(value));
	}
}

void PX4Storage::write_word(uint16_t loc, uint16_t value) 
{
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return;
	}
	if (memcmp(&value, &_buffer[loc], sizeof(value)) != 0) {
		_storage_open();
		memcpy(&_buffer[loc], &value, sizeof(value));
		_mark_dirty(loc, sizeof(value));
	}
}

void PX4Storage::write_dword(uint16_t loc, uint32_t value) 
{
	if (loc >= sizeof(_buffer)-(sizeof(value)-1)) {
		return;
	}
	if (memcmp(&value, &_buffer[loc], sizeof(value)) != 0) {
		_storage_open();
		memcpy(&_buffer[loc], &value, sizeof(value));
		_mark_dirty(loc, sizeof(value));
	}
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
		_fd = open(_storage_name, O_WRONLY);
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
		if (_dirty_mask == 0) {
			if (fsync(_fd) != 0) {
				close(_fd);
				_fd = -1;
				perf_count(_perf_errors);
			}
		}
	}
	perf_end(_perf_storage);
}

#endif // CONFIG_HAL_BOARD
