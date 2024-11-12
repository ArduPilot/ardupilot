#include "SIM_RAMTRON.h"

#if AP_SIM_RAMTRON_ENABLED

#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <AP_HAL_SITL/AP_HAL_SITL.h>

using namespace SITL;

extern const HAL_SITL& hal_sitl;

void RAMTRON::open_storage_fd()
{
    if (storage_fd != -1) {
        AP_HAL::panic("Should not have been called");
    }
    const char *filepath = filename();
    uint32_t flags = O_RDWR|O_CREAT;
    if (hal_sitl.get_wipe_storage()) {
        flags |= O_TRUNC;
    }
    storage_fd = open(filepath, flags, 0644);
    if (storage_fd == -1) {
        AP_HAL::panic("open(%s): %s", filepath, strerror(errno));
    }
    if (ftruncate(storage_fd, storage_size()) != 0) {
        AP_HAL::panic("truncate(%s): %s", filepath, strerror(errno));
    }
}

int RAMTRON::rdwr(uint8_t count, SPI::spi_ioc_transfer *&tfrs)
{
    if (storage_fd == -1) {
        open_storage_fd();
    }

    // commands:
    static const uint8_t RAMTRON_RDID  = 0x9f;
    static const uint8_t RAMTRON_READ  = 0x03;
    static const uint8_t RAMTRON_WREN  = 0x06;
    static const uint8_t RAMTRON_WRITE = 0x02;

    for (uint8_t i=0; i<count; i++) {
        SPI::spi_ioc_transfer &tfr = tfrs[i];
        uint8_t *tx_buf = (uint8_t*)(tfr.tx_buf);
        uint8_t *rx_buf = (uint8_t*)(tfr.rx_buf);

        switch (state) {
        case State::WAITING: {
            // find a command
            uint8_t command = tx_buf[0];
            switch (command) {
            case RAMTRON_RDID:
                state = State::READING_RDID;
                break;
            case RAMTRON_READ:
                xfr_addr = tx_buf[1] << 8 | tx_buf[2];
                state = State::READING;
                break;
            case RAMTRON_WRITE:
                xfr_addr = tx_buf[1] << 8 | tx_buf[2];
                state = State::WRITING;
                break;
            case RAMTRON_WREN:
                write_enabled = true;
                break;
            default:
                abort();
            }
            break;
        }
        case State::READING_RDID:
            fill_rdid(rx_buf, tfr.len);
            state = State::WAITING;
            break;
        case State::READING: {
            if (xfr_addr + tfr.len > storage_size()) {
                abort();
            }
            if (lseek(storage_fd, xfr_addr, SEEK_SET) == -1) {
                AP_HAL::panic("lseek(): %s", strerror(errno));
            }
            const size_t read_ret = read(storage_fd, rx_buf, tfr.len);
            if (read_ret != tfr.len) {
                AP_HAL::panic("read(): %s (%d/%u)", strerror(errno), (signed)read_ret, (unsigned)tfr.len);
            }
            state = State::WAITING;
            break;
        }
        case State::WRITING: {
            if (!write_enabled) {
                AP_HAL::panic("Writes not enabled");
            }
            if (xfr_addr + tfr.len > storage_size()) {
                abort();
            }
            if (lseek(storage_fd, xfr_addr, SEEK_SET) == -1) {
                AP_HAL::panic("lseek(): %s", strerror(errno));
            }
            const size_t write_ret = write(storage_fd, tx_buf, tfr.len);
            if (write_ret != tfr.len) {
                AP_HAL::panic("write(): %s (%d/%u)", strerror(errno), (signed)write_ret, (unsigned)tfr.len);
            }
            state = State::WAITING;
            write_enabled = false;
            break;
        }
        }
    }
    return 0;
}

#endif  // AP_SIM_RAMTRON_ENABLED
