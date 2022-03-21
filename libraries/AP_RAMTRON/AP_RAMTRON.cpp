/*
  driver for RAMTRON FRAM persistent memory devices. These are used
  for parameter and waypoint storage on most FMUv1, FMUv2, FMUv3 and FMUv4
  boards
 */

#include "AP_RAMTRON.h"
#include <AP_Math/crc.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

// register numbers
static const uint8_t RAMTRON_RDID  = 0x9f;
static const uint8_t RAMTRON_READ  = 0x03;
static const uint8_t RAMTRON_WREN  = 0x06;
static const uint8_t RAMTRON_WRITE = 0x02;

#define RAMTRON_RETRIES 10
#define RAMTRON_DELAY_MS 10

/*
  list of supported devices. Thanks to NuttX ramtron driver
 */
const AP_RAMTRON::ramtron_id AP_RAMTRON::ramtron_ids[] = {
    { 0x21, 0x00,  16, 2, RDID_type::Cypress }, // FM25V01
    { 0x21, 0x08,  16, 2, RDID_type::Cypress }, // FM25V01A
    { 0x22, 0x00,  32, 2, RDID_type::Cypress }, // FM25V02
    { 0x22, 0x08,  32, 2, RDID_type::Cypress }, // FM25V02A
    { 0x22, 0x48,  32, 2, RDID_type::Cypress }, // FM25V02A - Extended Temperature Version
    { 0x22, 0x01,  32, 2, RDID_type::Cypress }, // FM25VN02
    { 0x23, 0x00,  64, 2, RDID_type::Cypress }, // FM25V05
    { 0x23, 0x01,  64, 2, RDID_type::Cypress }, // FM25VN05
    { 0x24, 0x00, 128, 3, RDID_type::Cypress }, // FM25V10
    { 0x24, 0x01, 128, 3, RDID_type::Cypress }, // FM25VN10
    { 0x25, 0x08, 256, 3, RDID_type::Cypress }, // FM25V20A
    { 0x26, 0x08, 512, 3, RDID_type::Cypress }, // CY15B104Q

    { 0x27, 0x03, 128, 3, RDID_type::Fujitsu }, // MB85RS1MT
    { 0x05, 0x09,  32, 2, RDID_type::Fujitsu }, // MB85RS256B
    { 0x24, 0x03,  16, 2, RDID_type::Fujitsu }, // MB85RS128TY
};

// initialise the driver
bool AP_RAMTRON::init(void)
{
    dev = hal.spi->get_device("ramtron");
    if (!dev) {
        DEV_PRINTF("No RAMTRON device\n");
        return false;
    }
    WITH_SEMAPHORE(dev->get_semaphore());

    struct cypress_rdid {
        uint8_t manufacturer[6];
        uint8_t memory;
        uint8_t id1;
        uint8_t id2;
    };
    struct fujitsu_rdid {
        uint8_t manufacturer[2];
        uint8_t id1;
        uint8_t id2;
    };

    uint8_t rdid[sizeof(cypress_rdid)];

    if (!dev->read_registers(RAMTRON_RDID, rdid, sizeof(rdid))) {
        return false;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(ramtron_ids); i++) {
        if (ramtron_ids[i].rdid_type == RDID_type::Cypress) {
            const cypress_rdid *cypress = (const cypress_rdid *)rdid;
            if (ramtron_ids[i].id1 == cypress->id1 &&
                ramtron_ids[i].id2 == cypress->id2) {
                id = i;
                break;
            }
        } else if (ramtron_ids[i].rdid_type == RDID_type::Fujitsu) {
            const fujitsu_rdid *fujitsu = (const fujitsu_rdid *)rdid;
            if (ramtron_ids[i].id1 == fujitsu->id1 &&
                ramtron_ids[i].id2 == fujitsu->id2) {
                id = i;
                break;
            }
        }
    }

    if (id == UINT8_MAX) {
        DEV_PRINTF("Unknown RAMTRON device\n");
        return false;
    }

    return true;
}

/*
  send a command and offset
 */
void AP_RAMTRON::send_offset(uint8_t cmd, uint32_t offset) const
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
    // Don't allow reads outside of the FRAM memory.
    // NOTE: The FRAM devices will wrap back to address 0x0000 if they read past
    // the end of their internal memory, so while we'll get data back, it won't
    // be what we expect.
    if ((size > get_size()) ||
        (offset > (get_size() - size))) {
        return false;
    }
    const uint8_t maxread = 128;
    while (size > maxread) {
        if (!read(offset, buf, maxread)) {
            return false;
        }
        offset += maxread;
        buf += maxread;
        size -= maxread;
    }

    for (uint8_t r=0; r<RAMTRON_RETRIES; r++) {
        if (r != 0) {
            hal.scheduler->delay(RAMTRON_DELAY_MS);
        }
        /*
          transfer each block twice and compare with a crc. This is to
          prevent transient errors from causing parameter corruption
         */
        {
            WITH_SEMAPHORE(dev->get_semaphore());
            dev->set_chip_select(true);
            send_offset(RAMTRON_READ, offset);
            dev->transfer(nullptr, 0, buf, size);
            dev->set_chip_select(false);
        }

        uint32_t crc1 = crc_crc32(0, buf, size);

        {
            WITH_SEMAPHORE(dev->get_semaphore());
            dev->set_chip_select(true);
            send_offset(RAMTRON_READ, offset);
            dev->transfer(nullptr, 0, buf, size);
            dev->set_chip_select(false);
        }
        uint32_t crc2 = crc_crc32(0, buf, size);

        if (crc1 == crc2) {
            // all good
            return true;
        }
    }

    return false;
}

// write to device
bool AP_RAMTRON::write(uint32_t offset, const uint8_t *buf, uint32_t size)
{
    // Don't allow writes outside of the FRAM memory.
    // NOTE: The FRAM devices will wrap back to address 0x0000 if they write past
    // the end of their internal memory, so we could accidentally overwrite the
    // wrong memory location.
    if ((size > get_size()) ||
        (offset > (get_size() - size))) {
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    for (uint8_t r=0; r<RAMTRON_RETRIES; r++) {
        if (r != 0) {
            hal.scheduler->delay(RAMTRON_DELAY_MS);
        }

        // we need to enable writes every time. The WREN bit is
        // automatically reset on completion of the write call
        dev->set_chip_select(true);
        dev->transfer(&RAMTRON_WREN, 1, nullptr, 0);
        dev->set_chip_select(false);

        dev->set_chip_select(true);
        send_offset(RAMTRON_WRITE, offset);
        dev->transfer(buf, size, nullptr, 0);
        dev->set_chip_select(false);

        /*
          verify first 32 bytes of every write using a crc
         */
        uint8_t rbuf[32] {};
        const uint8_t nverify = MIN(size, sizeof(rbuf));
        uint32_t crc1 = crc_crc32(0, buf, nverify);

        dev->set_chip_select(true);
        send_offset(RAMTRON_READ, offset);
        dev->transfer(nullptr, 0, rbuf, nverify);
        dev->set_chip_select(false);

        uint32_t crc2 = crc_crc32(0, rbuf, nverify);

        if (crc1 == crc2) {
            return true;
        }
    }

    return false;
}
