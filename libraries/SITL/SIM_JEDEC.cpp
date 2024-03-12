#include "SIM_JEDEC.h"

#if AP_SIM_JEDEC_ENABLED

#include <errno.h>
#include <unistd.h>

#include <AP_HAL_SITL/AP_HAL_SITL.h>

using namespace SITL;

extern const HAL_SITL& hal_sitl;

void JEDEC::open_storage_fd()
{
    if (storage_fd != -1) {
        AP_HAL::panic("Should not have been called");
    }
    const char *filepath = filename();
    if (hal_sitl.get_wipe_storage()) {
        unlink(filepath);
    }
    storage_fd = open(filepath, O_RDWR|O_CREAT, 0644);
    if (storage_fd == -1) {
        AP_HAL::panic("open(%s): %s", filepath, strerror(errno));
    }
    if (ftruncate(storage_fd, get_storage_size()) != 0) {
        AP_HAL::panic("truncate(%s): %s", filepath, strerror(errno));
    }
}

void JEDEC::sector4k_erase (uint32_t addr)
{
    for (uint8_t i=0; i<get_page_per_sector(); i++) {
        page_erase(addr + i*get_page_size());
    }
}

void JEDEC::block64k_erase (uint32_t addr)
{
    // we have 16 sectors in a block
    for (uint16_t i=0; i<16; i++) {
        sector4k_erase(addr + i*get_page_per_sector()*get_page_size());
    }
}

void JEDEC::page_erase (uint32_t addr)
{
    const uint32_t fill_length = get_page_size();
    uint8_t fill[fill_length];
    if (addr + fill_length > get_storage_size()) {
        AP_HAL::panic("trying to write outside memory");
    }
    memset(fill, 0xFF, sizeof(fill));
    const size_t write_ret = pwrite(storage_fd, fill, sizeof(fill), addr);
    if (write_ret != sizeof(fill)) {
        printf("Failed page erase");
    }
}

void JEDEC::bulk_erase()
{
    for (uint16_t i=0; i<get_num_blocks(); i++) {
        block64k_erase(i*get_page_per_block()*get_page_size());
    }
}

uint32_t JEDEC::parse_addr (uint8_t* buffer, uint32_t len)
{
    if (len<4) {
        AP_HAL::panic("address too short");
    }

    // buffer[0] is cmd
    return buffer[1] << 16 | buffer[2] << 8 | buffer[3];
}

void JEDEC::assert_writes_enabled()
{
    if (!write_enabled) {
        AP_HAL::panic("Writes not enabled");
    }
}

int JEDEC::rdwr(uint8_t count, SPI::spi_ioc_transfer *&tfrs)
{
    if (storage_fd == -1) {
        open_storage_fd();
    }

    // commands:
    static const uint8_t JEDEC_RDID             = 0x9f;
    static const uint8_t JEDEC_READ             = 0x03;
    static const uint8_t JEDEC_WREN             = 0x06;
    static const uint8_t JEDEC_WRITE            = 0x02;
    static const uint8_t JEDEC_RDSR             = 0x05;
    static const uint8_t JEDEC_SECTOR4_ERASE    = 0x20;
    static const uint8_t JEDEC_BULK_ERASE       = 0xC7;
    static const uint8_t JEDEC_BLOCK64_ERASE    = 0xD8;

    for (uint8_t i=0; i<count; i++) {
        SPI::spi_ioc_transfer &tfr = tfrs[i];
        uint8_t *tx_buf = (uint8_t*)(tfr.tx_buf);
        uint8_t *rx_buf = (uint8_t*)(tfr.rx_buf);

        switch (state) {
        case State::WAITING: {
            // find a command
            uint8_t command = tx_buf[0];
            switch (command) {
            case JEDEC_RDID:
                state = State::READING_RDID;
                break;
            case JEDEC_READ:
                xfr_addr = parse_addr(tx_buf, tfr.len);
                state = State::READING;
                break;
            case JEDEC_WRITE:
                xfr_addr = parse_addr(tx_buf, tfr.len);
                state = State::WRITING;
                break;
            case JEDEC_WREN:
                write_enabled = true;
                break;
            case JEDEC_RDSR:
                state = State::READING_RDSR;
                break;
            case JEDEC_SECTOR4_ERASE:   {
                xfr_addr = parse_addr(tx_buf, tfr.len);
                assert_writes_enabled();
                sector4k_erase(xfr_addr);
                write_enabled = false;
                break;
            }
            case JEDEC_BULK_ERASE:  {
                assert_writes_enabled();
                bulk_erase();
                write_enabled = false;
                break;
            }
            case JEDEC_BLOCK64_ERASE:   {
                xfr_addr = parse_addr(tx_buf, tfr.len);
                assert_writes_enabled();
                block64k_erase(xfr_addr);
                write_enabled = false;
                break;
            }
            default:
                AP_HAL::panic("Unhandled command received");
            }
            break;
        }
        case State::READING_RDID:
            fill_rdid(rx_buf, tfr.len);
            state = State::WAITING;
            break;
        case State::READING_RDSR:
            fill_rdsr(rx_buf, tfr.len);
            state = State::WAITING;
            break;
        case State::READING: {
            if (xfr_addr + tfr.len > get_storage_size()) {
                AP_HAL::panic("trying to read outside memory");
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
            assert_writes_enabled();
            if (xfr_addr + tfr.len > get_storage_size()) {
                AP_HAL::panic("trying to write outside memory");
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

#endif // AP_SIM_JEDEC_ENABLED
