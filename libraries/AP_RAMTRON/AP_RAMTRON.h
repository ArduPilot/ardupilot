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
    bool read(uint32_t offset, uint8_t *buf, uint32_t size);

    // write to device
    bool write(uint32_t offset, const uint8_t *buf, uint32_t size);

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;

    enum rdid_type {
        CYPRESS_RDID,
        FUJITSU_RDID,
    };

    struct ramtron_id {
        uint8_t id1;
        uint8_t id2;
        uint16_t size_kbyte;
        uint8_t addrlen;
        enum rdid_type rdid_type;
    };
    static const struct ramtron_id ramtron_ids[];
    uint8_t id = UINT8_MAX;

    // send offset of transfer
    void send_offset(uint8_t cmd, uint32_t offset);
};
