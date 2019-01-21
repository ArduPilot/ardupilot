#include <AP_HAL/AP_HAL.h>

#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "Storage.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }

#if STORAGE_USE_POSIX
    // if we have failed filesystem init don't try again
    if (log_fd == -1) {
        return;
    }
#endif
        
    _dirty_mask.clearall();

#if STORAGE_USE_FLASH
    // load from storage backend
    _flash_load();
#elif STORAGE_USE_POSIX
    log_fd = open(HAL_STORAGE_FILE, O_RDWR|O_CREAT, 0644);
    if (log_fd == -1) {
        hal.console->printf("open failed of " HAL_STORAGE_FILE "\n");
        return;
    }
    int ret = read(log_fd, _buffer, HAL_STORAGE_SIZE);
    if (ret < 0) {
        hal.console->printf("read failed for " HAL_STORAGE_FILE "\n");
        close(log_fd);
        log_fd = -1;
        return;
    }
    // pre-fill to full size
    if (lseek(log_fd, ret, SEEK_SET) != ret ||
        write(log_fd, &_buffer[ret], HAL_STORAGE_SIZE-ret) != HAL_STORAGE_SIZE-ret) {
        hal.console->printf("setup failed for " HAL_STORAGE_FILE "\n");
        close(log_fd);
        log_fd = -1;
        return;        
    }
    using_filesystem = true;
#else
#error "No storage system enabled"
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
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    uint16_t end = loc + length;
    for (uint16_t line=loc>>STORAGE_LINE_SHIFT;
         line <= end>>STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
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

void Storage::_timer_tick(void)
{
    if (!_initialised) {
        return;
    }
    if (_dirty_mask.empty()) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    // write out the first dirty line. We don't write more
    // than one to keep the latency of this call to a minimum
    uint16_t i;
    for (i=0; i<STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }

#if STORAGE_USE_POSIX
    if (using_filesystem && log_fd != -1) {
        uint32_t offset = STORAGE_LINE_SIZE*i;
        if (lseek(log_fd, offset, SEEK_SET) != offset) {
            return;
        }
        if (write(log_fd, &_buffer[offset], STORAGE_LINE_SIZE) != STORAGE_LINE_SIZE) {
            return;
        }
        _dirty_mask.clear(i);
        return;
    } 
#endif
    
#if STORAGE_USE_FLASH
    // save to storage backend
    _flash_write(i);
#endif
}

/*
  load all data from flash
 */
void Storage::_flash_load(void)
{
#if STORAGE_USE_FLASH
    if (!_flash.init()) {
        AP_HAL::panic("unable to init flash storage");
    }
#else
    AP_HAL::panic("unable to init storage");
#endif
}

/*
  write one storage line. This also updates _dirty_mask. 
*/
void Storage::_flash_write(uint16_t line)
{
#if STORAGE_USE_FLASH
    if (_flash.write(line*STORAGE_LINE_SIZE, STORAGE_LINE_SIZE)) {
        // mark the line clean
        _dirty_mask.clear(line);
    }
#endif
}


#if STORAGE_USE_FLASH
/*
  emulate writing to flash
 */
static int flash_fd = -1;

static uint32_t sitl_flash_getpageaddr(uint32_t page)
{
    return page * HAL_STORAGE_SIZE;
}

static void sitl_flash_open(void)
{
    if (flash_fd == -1) {
        flash_fd = open("flash.dat", O_RDWR, 0644);
        if (flash_fd == -1) {
            flash_fd = open("flash.dat", O_RDWR|O_CREAT, 0644);
            if (flash_fd == -1) {
                AP_HAL::panic("Failed to open flash.dat");
            }
            if (ftruncate(flash_fd, 2*HAL_STORAGE_SIZE) != 0) {
                AP_HAL::panic("Failed to create flash.dat");
            }
            uint8_t fill[HAL_STORAGE_SIZE*2];
            memset(fill, 0xff, sizeof(fill));
            pwrite(flash_fd, fill, sizeof(fill), 0);
        }
    }
}

static bool sitl_flash_write(uint32_t addr, const uint8_t *data, uint32_t length)
{
    sitl_flash_open();
    uint8_t old[length];
    if (pread(flash_fd, old, length, addr) != length) {
        AP_HAL::panic("Failed to read flash.dat");
    }
    // check that we are only ever clearing bits (real flash storage can only ever clear bits,
    // except for an erase
    for (uint32_t i=0; i<length; i++) {
        if (data[i] & ~old[i]) {
            AP_HAL::panic("Attempt to set flash byte 0x%02x from 0x%02x at %u\n", data[i], old[i], addr+i);
        }
    }
    return pwrite(flash_fd, data, length, addr) == length;
}

static bool sitl_flash_read(uint32_t addr, uint8_t *data, uint32_t length)
{
    sitl_flash_open();
    return pread(flash_fd, data, length, addr) == length;
}

static bool sitl_flash_erasepage(uint32_t page)
{
    uint8_t fill[HAL_STORAGE_SIZE];
    memset(fill, 0xff, sizeof(fill));
    sitl_flash_open();
    bool ret = pwrite(flash_fd, fill, sizeof(fill), page * HAL_STORAGE_SIZE) == sizeof(fill);
    printf("erase %u -> %u\n", page, ret);
    return ret;
}

/*
  callback to write data to flash
 */
bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    size_t base_address = sitl_flash_getpageaddr(sector);
    bool ret = sitl_flash_write(base_address+offset, data, length);
    if (!ret && _flash_erase_ok()) {
        // we are getting flash write errors while disarmed. Try
        // re-writing all of flash
        uint32_t now = AP_HAL::millis();
        if (now - _last_re_init_ms > 5000) {
            _last_re_init_ms = now;
            bool ok = _flash.re_initialise();
            hal.console->printf("Storage: failed at %u:%u for %u - re-init %u\n",
                                (unsigned)sector, (unsigned)offset, (unsigned)length, (unsigned)ok);
        }
    }
    return ret;
}

/*
  callback to read data from flash
 */
bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    size_t base_address = sitl_flash_getpageaddr(sector);
    return sitl_flash_read(base_address+offset, data, length);
}

/*
  callback to erase flash sector
 */
bool Storage::_flash_erase_sector(uint8_t sector)
{
    return sitl_flash_erasepage(sector);
}

/*
  callback to check if erase is allowed
 */
bool Storage::_flash_erase_ok(void)
{
    // only allow erase while disarmed
    return !hal.util->get_soft_armed();
}
#endif // STORAGE_USE_FLASH

/*
  consider storage healthy if we have nothing to write sometime in the
  last 2 seconds
 */
bool Storage::healthy(void)
{
    return _initialised && AP_HAL::millis() - _last_empty_ms < 2000;
}


