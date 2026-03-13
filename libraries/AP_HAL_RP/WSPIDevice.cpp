#include "WSPIDevice.h"
#include "hardware/flash.h"
#include "pico/mutex.h"
#include "pico/flash.h"
#include "hardware/address_mapped.h"
#include <cstring>

#if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)

using namespace RP;

namespace {

struct FlashEraseArgs {
    uint32_t offs;
    size_t len;
};

struct FlashProgramArgs {
    uint32_t offs;
    const uint8_t *src;
    size_t len;
};

static void __RAMFUNC__ flash_erase_cb(void *param);
static void __RAMFUNC__ flash_program_cb(void *param);
static inline uint32_t get_addr24(const uint8_t *txbuf, size_t txlen);

} // namespace

WSPIDevice::WSPIDevice(SemaphoreHandle_t lock) : _lock(lock), _write_enabled(false) {
    // Initialize the default header
    memset(&_current_cmd_hdr, 0, sizeof(_current_cmd_hdr));
}

WSPIDeviceManager::WSPIDeviceManager() {
    _flash_mutex = xSemaphoreCreateMutex();
}

AP_HAL::OwnPtr<AP_HAL::WSPIDevice> WSPIDeviceManager::get_device(const char *name) {
    if (strcmp(name, "qspi_flash") != 0) {
        return nullptr;
    }
    return AP_HAL::OwnPtr<AP_HAL::WSPIDevice>(new RP::WSPIDevice(_flash_mutex));
}

void WSPIDevice::set_cmd_header(const CommandHeader& cmd_hdr) {
    _current_cmd_hdr = cmd_hdr;
}

bool WSPIDevice::is_busy() {
    // On RP2350 flash_do_cmd is blocking, so after exiting transfer the device is always free
    // But we can check the flash status register (W25Qxx)
    uint8_t status_reg_cmd = 0x05; // Read Status Register-1
    uint8_t status;
    // Call transfer to check the BUSY bit in the memory itself
    transfer(&status_reg_cmd, 1, &status, 1);
    return (status & 0x01); // Bit 0 - BUSY
}

__RAMFUNC__ bool WSPIDevice::emul_flash_do_cmd(const uint8_t *txbuf, uint8_t *rxbuf, size_t len)
{
    if (txbuf == nullptr || rxbuf == nullptr || len == 0) {
        return false;
    }
    memset(rxbuf, 0, len);
    const uint8_t cmd = txbuf[0];

    switch (cmd) {
    case 0x06: // WREN
        _write_enabled = true;
        return true;

    case 0x04: // WRDI
        _write_enabled = false;
        return true;

    case 0x05: // RDSR
        if (len >= 2) {
            uint8_t sr = 0;
            if (_write_enabled) {
                sr |= 0x02; // WEL
            }
            rxbuf[len - 1] = sr;
        }
        return true;

    case 0x9F: // JEDEC ID
        if (len >= 4) {
            rxbuf[len - 3] = 0xEF;
            rxbuf[len - 2] = 0x40;
            rxbuf[len - 1] = 0x18;
        }
        return true;

    case 0x03: // READ
    {
        const uint32_t flash_offs = get_addr24(txbuf, len);
        const size_t read_len = (len >= 4) ? (len - 4) : 0;
        if (len < 4 || read_len == 0) {
            return false;
        }
        const uint8_t *src = reinterpret_cast<const uint8_t *>(XIP_BASE + flash_offs);
        memcpy(&rxbuf[4], src, read_len);
        return true;
    }

    case 0x20: // 4KB erase
    {
        if (!_write_enabled || len < 4) {
            return false;
        }
        const uint32_t flash_offs = get_addr24(txbuf, len);
        if ((flash_offs % FLASH_SECTOR_SIZE) != 0) {
            return false;
        }
        FlashEraseArgs args {
            .offs = flash_offs,
            .len = FLASH_SECTOR_SIZE
        };
        const bool ok = (flash_safe_execute(flash_erase_cb, &args, UINT32_MAX) == PICO_OK);
        _write_enabled = false;
        return ok;
    }

    case 0xD8: // 64KB erase
    {
        if (!_write_enabled || len < 4) {
            return false;
        }
        constexpr size_t block_size = 64 * 1024;
        const uint32_t flash_offs = get_addr24(txbuf, len);
        if ((flash_offs % block_size) != 0) {
            return false;
        }
        FlashEraseArgs args {
            .offs = flash_offs,
            .len = block_size
        };
        const bool ok = (flash_safe_execute(flash_erase_cb, &args, UINT32_MAX) == PICO_OK);
        _write_enabled = false;
        return ok;
    }

    case 0x02: // PAGE PROGRAM
    {
        if (!_write_enabled || len < 5) {
            return false;
        }
        const uint32_t flash_offs = get_addr24(txbuf, len);
        const uint8_t *prog_src = &txbuf[4];
        const size_t prog_len = len - 4;
        if (prog_len == 0 || prog_len > FLASH_PAGE_SIZE) {
            return false;
        }
        // PAGE PROGRAM command must not cross a 256-byte page boundary
        const uint32_t page_base = flash_offs & ~(uint32_t(FLASH_PAGE_SIZE - 1));
        const uint32_t page_off  = flash_offs &  (uint32_t(FLASH_PAGE_SIZE - 1));
        if ((page_off + prog_len) > FLASH_PAGE_SIZE) {
            return false;
        }
        // Read the current page from XIP
        static uint8_t page_buf[FLASH_PAGE_SIZE];
        const uint8_t *flash_page_ptr = reinterpret_cast<const uint8_t *>(XIP_BASE + page_base);
        memcpy(page_buf, flash_page_ptr, FLASH_PAGE_SIZE);
        // NOR flash only allows 1 -> 0 without erase.
        // If even one bit needs to be raised 0 -> 1, we refuse.
        for (size_t i = 0; i < prog_len; i++) {
            const uint8_t oldv = page_buf[page_off + i];
            const uint8_t newv = prog_src[i];
            if ((oldv & newv) != newv) {
                _write_enabled = false;
                return false;
            }
            page_buf[page_off + i] = newv;
        }
        FlashProgramArgs args {
            .offs = page_base,
            .src  = page_buf,
            .len  = FLASH_PAGE_SIZE
        };
        const bool ok = (flash_safe_execute(flash_program_cb, &args, UINT32_MAX) == PICO_OK);
        _write_enabled = false;
        return ok;
    }

    default:
        return false;
    }
}

__RAMFUNC__ bool WSPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                                                uint8_t *recv, uint32_t recv_len) {
    if (xSemaphoreTake(_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    uint32_t ints = save_and_disable_interrupts();
#if PICO_MULTICORE
    multicore_lockout_start_blocking();
#endif

    bool success = true;
    static uint8_t txbuf[512] {};
    static uint8_t rxbuf[512] {};
    uint32_t txlen = 0;

    if (_current_cmd_hdr.cmd != 0) {
        txbuf[txlen++] = _current_cmd_hdr.cmd;
    }

    if (send != nullptr && send_len > 0) {
        if (txlen + send_len > sizeof(txbuf)) {
            success = false;
        } else {
            memcpy(&txbuf[txlen], send, send_len);
            txlen += send_len;
        }
    }

    if (success && txlen > 0) {
        emul_flash_do_cmd(txbuf, rxbuf, txlen);
        if (recv != nullptr && recv_len > 0) {
            memcpy(recv, &rxbuf[txlen - recv_len], recv_len);
        }
    } else if (txlen == 0) {
        success = false;
    }

    // Resume the second core and interrupts
#if PICO_MULTICORE
    multicore_lockout_end_blocking();
#endif
    restore_interrupts(ints);
    // Clear the header for the next transaction
    _current_cmd_hdr.cmd = 0;
    _current_cmd_hdr.cfg = 0;
    _current_cmd_hdr.addr = 0;
    _current_cmd_hdr.alt = 0;
    _current_cmd_hdr.dummy = 0;

    xSemaphoreGive(_lock);
    return success;
}

namespace {

static void __RAMFUNC__ flash_erase_cb(void *param)
{
    auto *a = static_cast<FlashEraseArgs *>(param);
    flash_range_erase(a->offs, a->len);
}

static void __RAMFUNC__ flash_program_cb(void *param)
{
    auto *a = static_cast<FlashProgramArgs *>(param);
    flash_range_program(a->offs, a->src, a->len);
}

static inline uint32_t get_addr24(const uint8_t *txbuf, size_t txlen)
{
    if (txlen < 4) {
        return 0;
    }
    return (uint32_t(txbuf[1]) << 16) |
           (uint32_t(txbuf[2]) << 8)  |
           (uint32_t(txbuf[3]) << 0);
}

} //namespace

#endif // #if HAL_USE_WSPI == TRUE && defined(HAL_WSPI_DEVICE_LIST)
