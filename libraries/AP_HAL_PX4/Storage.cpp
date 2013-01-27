#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

#include "Storage.h"
using namespace PX4;

/*
  This stores 'eeprom' data on the SD card, with a 4k size, and a
  in-memory buffer. This keeps the latency down.
 */

// name the storage file after the sketch so you can use the same sd
// card for ArduCopter and ArduPlane
#define STORAGE_DIR "/fs/microsd/APM"
#define STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".stg"

extern const AP_HAL::HAL& hal;

void PX4Storage::_storage_create(void)
{
	mkdir(STORAGE_DIR, 0777);
	unlink(STORAGE_FILE);
	int fd = open(STORAGE_FILE, O_RDWR|O_CREAT, 0666);
	if (fd == -1) {
		hal.scheduler->panic("Failed to create " STORAGE_FILE);
	}
	for (uint16_t loc=0; loc<sizeof(_buffer); loc += PX4_STORAGE_MAX_WRITE) {
		if (write(fd, &_buffer[loc], PX4_STORAGE_MAX_WRITE) != PX4_STORAGE_MAX_WRITE) {
			hal.scheduler->panic("Error filling " STORAGE_FILE);			
		}
	}
	// ensure the directory is updated with the new size
	fsync(fd);
	close(fd);
}

void PX4Storage::_storage_open(void)
{
	if (_initialised) {
		return;
	}

	_dirty_mask = 0;
	int fd = open(STORAGE_FILE, O_RDONLY);
	if (fd == -1) {
		_storage_create();
		fd = open(STORAGE_FILE, O_RDONLY);
		if (fd == -1) {
			hal.scheduler->panic("Failed to open " STORAGE_FILE);
		}
	}
	if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
		close(fd);
		_storage_create();
		fd = open(STORAGE_FILE, O_RDONLY);
		if (fd == -1) {
			hal.scheduler->panic("Failed to open " STORAGE_FILE);
		}
		if (read(fd, _buffer, sizeof(_buffer)) != sizeof(_buffer)) {
			hal.scheduler->panic("Failed to read " STORAGE_FILE);
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

void PX4Storage::write_block(uint16_t loc, void *src, size_t n) 
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
		_fd = open(STORAGE_FILE, O_WRONLY);
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
