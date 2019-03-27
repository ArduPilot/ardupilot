/*
  driver for RAMTRON FRAM persistent memory devices. These are used
  for parameter and waypoint storage on most FMUv1, FMUv2, FMUv3 and FMUv4
  boards
 */

#include "AP_RAMTRON.h"
#include <AP_Common/Semaphore.h>

extern const AP_HAL::HAL &hal;

// register numbers
#define RAMTRON_RDID      0x9f
#define RAMTRON_READ      0x03
#define RAMTRON_RDSR      0x05
#define RAMTRON_WREN      0x06
#define RAMTRON_WRITE     0x02

/*
  list of supported devices. Thanks to NuttX ramtron driver
 */
const AP_RAMTRON::ramtron_id AP_RAMTRON::ramtron_ids[] = {
    { 0x21, 0x00, 16,  2}, // FM25V01
    { 0x21, 0x08, 16,  2}, // FM25V01A
    { 0x22, 0x00, 32,  2}, // FM25V02
    { 0x22, 0x08, 32,  2}, // FM25V02A
    { 0x22, 0x01, 32,  2}, // FM25VN02
    { 0x23, 0x00, 64,  2}, // FM25V05
    { 0x23, 0x01, 64,  2}, // FM25VN05
    { 0x24, 0x00, 128, 3}, // FM25V10
    { 0x24, 0x01, 128, 3}, // FM25VN10
    { 0x25, 0x08, 256, 3}, // FM25V20A
    { 0x26, 0x08, 512, 3}, // CY15B104Q
    { 0x27, 0x03, 128, 3}, // MB85RS1MT
    { 0x05, 0x09, 32,  3}, // B85RS256B
};

// initialise the driver
bool AP_RAMTRON::init(void)
{
    dev = hal.spi->get_device("ramtron");
    if (!dev) {
        return false;
    }
    WITH_SEMAPHORE(dev->get_semaphore());

    struct rdid {
        uint8_t manufacturer[6];
        uint8_t memory;
        uint8_t id1;
        uint8_t id2;
    } rdid;
    if (!dev->read_registers(RAMTRON_RDID, (uint8_t *)&rdid, sizeof(rdid))) {
        return false;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(ramtron_ids); i++) {
        if (ramtron_ids[i].id1 == rdid.id1 &&
            ramtron_ids[i].id2 == rdid.id2) {
            id = i;
            return true;
        }
    }
    hal.console->printf("Unknown RAMTRON manufacturer=%02x memory=%02x id1=%02x id2=%02x\n",
                        rdid.manufacturer[0], rdid.memory, rdid.id1, rdid.id2);
    return false;
}

/*
  send a command and offset
 */
void AP_RAMTRON::send_offset(uint8_t cmd, uint32_t offset)
{
    if (ramtron_ids[id].addrlen == 3) {
        uint8_t b[4] = { cmd, uint8_t((offset>>16)&0xFF), uint8_t((offset>>8)&0xFF), uint8_t(offset&0xFF) };
        dev->transfer(b, sizeof(b), nullptr, 0);
    } else /* len 2 */ {
        uint8_t b[3] = { cmd, uint8_t((offset>>8)&0xFF), uint8_t(offset&0xFF) };
        dev->transfer(b, sizeof(b), nullptr, 0);
    }
}

// read from device
bool AP_RAMTRON::read(uint32_t offset, uint8_t *buf, uint32_t size)
{
    const uint8_t maxread = 128;
    while (size > maxread) {
        if (!read(offset, buf, maxread)) {
            return false;
        }
        offset += maxread;
        buf += maxread;
        size -= maxread;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_chip_select(true);

    send_offset(RAMTRON_READ, offset);
    
    // get data
    dev->transfer(nullptr, 0, buf, size);

    dev->set_chip_select(false);

    return true;
}

// write to device
bool AP_RAMTRON::write(uint32_t offset, const uint8_t *buf, uint32_t size)
{
    WITH_SEMAPHORE(dev->get_semaphore());

    // write enable
    uint8_t r = RAMTRON_WREN;
    dev->transfer(&r, 1, nullptr, 0);
  
    dev->set_chip_select(true);

    send_offset(RAMTRON_WRITE, offset);

    dev->transfer(buf, size, nullptr, 0);

    dev->set_chip_select(false);

    return true;
}
