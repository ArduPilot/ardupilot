/*
  driver for RAMTRON FRAM persistent memory devices
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

class AP_RAMTRON {
public:
    // initialise the driver
    bool init(void);

    // get size in bytes
    uint32_t get_size(void) const { return (id == UINT8_MAX) ? 0 : ramtron_ids[id].size_kbyte * 1024UL; }

    // read from device
    uint32_t read(uint32_t offset, uint8_t * const buf, uint32_t size);

    // write to device
    uint32_t write(uint32_t offset, uint8_t const * const buf, uint32_t size);

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;

    enum class RDID_type :uint8_t {
        Cypress,
        Fujitsu,
    };

    struct ramtron_id {
        uint8_t id1;
        uint8_t id2;
        uint16_t size_kbyte;
        uint8_t addrlen;
        RDID_type rdid_type;
    };
    static const struct ramtron_id ramtron_ids[];
    uint8_t id = UINT8_MAX;

    bool _populate_addr(uint8_t cmdBuffer[], uint32_t const kCmdBufferSz, uint32_t addr);
};
