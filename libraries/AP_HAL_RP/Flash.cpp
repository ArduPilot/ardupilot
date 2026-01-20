#include "Flash.h"
#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_RP_Namespace.h"

extern const AP_HAL::HAL& hal;

using namespace RP;

// Standard commands for Flash memory
#define JEDEC_WRITE_ENABLE    0x06
#define JEDEC_READ_STATUS     0x05
#define JEDEC_SECTOR_ERASE    0x20
#define JEDEC_PAGE_PROGRAM    0x02

// Status bit Busy
#define STATUS_BUSY_MASK      0x01

uint32_t Flash::getpageaddr(uint32_t page) {
    // HAL_FLASH_STORAGE_OFFSET must be defined in hwdef
    return HAL_FLASH_STORAGE_OFFSET + (page * RP_FLASH_SECTOR_SIZE);
}

uint32_t Flash::getpagesize(uint32_t page) {
    return RP_FLASH_SECTOR_SIZE;
}

uint32_t Flash::getnumpages(void) {
    return HAL_FLASH_STORAGE_NUM_PAGES;
}

bool Flash::is_device_busy(AP_HAL::WSPIDevice* dev) {
    uint8_t status;
    uint8_t cmd = JEDEC_READ_STATUS;
    // Use transfer to read the status register
    dev->transfer(&cmd, 1, &status, 1);
    return (status & STATUS_BUSY_MASK);
}

bool Flash::erasepage(uint32_t page) {
    auto dev = hal.wspi->get_device("qspi_flash");
    if (!dev || !dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    uint32_t addr = getpageaddr(page);

    // Write Enable
    uint8_t cmd_we = JEDEC_WRITE_ENABLE;
    dev->transfer(&cmd_we, 1, nullptr, 0);

    // Erase Sector
    // Form a package: Command (1) + Address (3)
    uint8_t cmd_erase[4];
    cmd_erase[0] = JEDEC_SECTOR_ERASE;
    cmd_erase[1] = (addr >> 16) & 0xFF;
    cmd_erase[2] = (addr >> 8) & 0xFF;
    cmd_erase[3] = addr & 0xFF;

    dev->transfer(cmd_erase, 4, nullptr, 0);

    // Waiting for completion
    while (is_device_busy(dev.get())) {
        hal.scheduler->delay_microseconds(500);
    }

    dev->get_semaphore()->give();
    return true;
}

bool Flash::write(uint32_t addr, const void *buf, uint32_t count) {
    auto dev = hal.wspi->get_device("qspi_flash");
    if (!dev || !dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    const uint8_t* data = (const uint8_t*)buf;
    uint32_t current_addr = addr;
    uint32_t remaining = count;

    while (remaining > 0) {
        // Page Program is usually limited to 256 bytes
        uint32_t chunk = (remaining > 256) ? 256 : remaining;

        // Write Enable
        uint8_t cmd_we = JEDEC_WRITE_ENABLE;
        dev->transfer(&cmd_we, 1, nullptr, 0);

        // Page Program Header
        uint8_t cmd_hdr[4];
        cmd_hdr[0] = JEDEC_PAGE_PROGRAM;
        cmd_hdr[1] = (current_addr >> 16) & 0xFF;
        cmd_hdr[2] = (current_addr >> 8) & 0xFF;
        cmd_hdr[3] = current_addr & 0xFF;

        // Use set_cmd_header to pass the command and address,
        // and then pass the data to transfer
        dev->set_cmd_header(AP_HAL::WSPIDevice::CommandHeader{cmd_hdr[0], 4}); // opcode + addr
        
        // Copy to temporary buffer (command + address + data)
        uint8_t full_packet[256 + 4];
        memcpy(full_packet, cmd_hdr, 4);
        memcpy(&full_packet[4], data, chunk);
        
        dev->transfer(full_packet, chunk + 4, nullptr, 0);

        // Waiting
        while (is_device_busy(dev.get())) {
            hal.scheduler->delay_microseconds(100);
        }

        remaining -= chunk;
        data += chunk;
        current_addr += chunk;
    }

    dev->get_semaphore()->give();
    return true;
}

bool Flash::ispageerased(uint32_t page) {
    uint32_t addr = getpageaddr(page);
    // Reading can be done via XIP direct access (XIP_BASE),
    // this is much faster than via WSPIDevice::transfer
    const uint32_t *ptr = (const uint32_t *)(XIP_BASE + addr);
    for (uint32_t i = 0; i < RP_FLASH_SECTOR_SIZE / 4; i++) {
        if (ptr[i] != 0xFFFFFFFF) {
            return false;
        }
    }
    return true;
}
