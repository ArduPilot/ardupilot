/*
  driver for RAMTRON FRAM persistent memory devices. These are used
  for parameter and waypoint storage on most FMUv1, FMUv2, FMUv3 and FMUv4
  boards
 */

#include "AP_RAMTRON.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

// register numbers
static const uint8_t RAMTRON_RDID  = 0x9f;
static const uint8_t RAMTRON_READ  = 0x03;
static const uint8_t RAMTRON_WRSR  = 0x01;
static const uint8_t RAMTRON_RDSR  = 0x05;
static const uint8_t RAMTRON_WREN  = 0x06;
static const uint8_t RAMTRON_WRITE = 0x02;

static const uint8_t RAMTRON_RETRIES = 10;
static const uint8_t RAMTRON_DELAY_MS = 10;

/*
  list of supported devices. Thanks to NuttX ramtron driver
 */
const AP_RAMTRON::ramtron_id AP_RAMTRON::ramtron_ids[] = {
    { 0x21, 0x00,  16, 2, RDID_type::Cypress, }, // FM25V01
    { 0x21, 0x08,  16, 2, RDID_type::Cypress, }, // FM25V01A
    { 0x22, 0x00,  32, 2, RDID_type::Cypress, }, // FM25V02
    { 0x22, 0x08,  32, 2, RDID_type::Cypress, }, // FM25V02A
    { 0x22, 0x01,  32, 2, RDID_type::Cypress, }, // FM25VN02
    { 0x23, 0x00,  64, 2, RDID_type::Cypress, }, // FM25V05
    { 0x23, 0x01,  64, 2, RDID_type::Cypress, }, // FM25VN05
    { 0x24, 0x00, 128, 3, RDID_type::Cypress, }, // FM25V10
    { 0x24, 0x01, 128, 3, RDID_type::Cypress, }, // FM25VN10
    { 0x25, 0x08, 256, 3, RDID_type::Cypress, }, // FM25V20A
    { 0x26, 0x08, 512, 3, RDID_type::Cypress, }, // CY15B104Q

    { 0x27, 0x03, 128, 3, RDID_type::Fujitsu, }, // MB85RS1MT
    { 0x05, 0x09,  32, 2, RDID_type::Fujitsu, }, // MB85RS256B
    { 0x24, 0x03,  16, 2, RDID_type::Fujitsu, }, // MB85RS128TY
};

// initialise the driver
bool AP_RAMTRON::_init(void)
{
    if (!dev) {
        return false;
    }
    WITH_SEMAPHORE(dev->get_semaphore());

    struct cypress_rdid {
        uint8_t manufacturer[6];
        uint8_t memory;
        uint8_t id1;
        uint8_t id2;
    };
    static_assert(sizeof(struct cypress_rdid) == 9, "Bad cypress_rdid size!");
    struct fujitsu_rdid {
        uint8_t manufacturer[2];
        uint8_t id1;
        uint8_t id2;
    };
    static_assert(sizeof(struct fujitsu_rdid) == 4, "Bad fujitsu_rdid size!");

    uint8_t rdid[sizeof(cypress_rdid)];

    if (!dev->read_registers(RAMTRON_RDID, rdid, sizeof(rdid))) {
        hal.console->printf("Unable to read RAMTRON RDID!\n");
        return false;
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(ramtron_ids); i++) {
        if (ramtron_ids[i].rdid_type == RDID_type::Cypress) {
            cypress_rdid const * const cypress = (cypress_rdid const * const)rdid;
            if (ramtron_ids[i].id1 == cypress->id1 &&
                ramtron_ids[i].id2 == cypress->id2) {
                id = i;
                break;
            }
        } else if (ramtron_ids[i].rdid_type == RDID_type::Fujitsu) {
            fujitsu_rdid const * const fujitsu = (fujitsu_rdid const * const)rdid;
            if (ramtron_ids[i].id1 == fujitsu->id1 &&
                ramtron_ids[i].id2 == fujitsu->id2) {
                id = i;
                break;
            }
        }
    }

    if (id == UINT8_MAX) {
        hal.console->printf("Unknown RAMTRON device = [%02x %02x %02x %02x %02x %02x %02x %02x %02x]\n",
                            rdid[0], rdid[1], rdid[2], rdid[3], rdid[4], rdid[5], rdid[6], rdid[7], rdid[8]);
        return false;
    }

    uint8_t status_register;
    if (!dev->read_registers(RAMTRON_RDSR, &status_register, sizeof(status_register))) {
        hal.console->printf("Unable to read RAMTRON RDSR!\n");
        return false;
    }

    char const * const manufacturer = (ramtron_ids[id].rdid_type == RDID_type::Fujitsu) ? "Fujitsu" : "Cypress";
    hal.console->printf("Found %s RAMTRON idx=%u sr=0x%02x\n", manufacturer, id, status_register);

    uint8_t const kWriteEnableLatch   = (1 << 1);
    uint8_t const kBlockProtect0      = (1 << 2);
    uint8_t const kBlockProtect1      = (1 << 3);
    uint8_t const kBlockProtect       = kBlockProtect0 | kBlockProtect1;

    // Ensure Write Enable is set properly
    if ((status_register & kWriteEnableLatch) != kWriteEnableLatch) {
        if (!dev->transfer(&RAMTRON_WREN, 1, nullptr, 0)) {
            hal.console->printf("Failed to set RAMTRON WriteEnableLatch!");
            return false;
        }
    }

    // Ensure Block Protect is disabled
    if ((status_register & kBlockProtect) != 0x00) {
        status_register &= ~kBlockProtect;
        status_register |= kWriteEnableLatch; // Ensure Write Enable is not overwritten
        if (!dev->write_register(RAMTRON_WRSR, status_register)) {
            hal.console->printf("Failed to disable RAMTRON BlockProtect!");
            return false;
        }
    }

    return true;
}

bool AP_RAMTRON::init()
{
    dev = hal.spi->get_device("ramtron");
    if (!dev) {
        hal.console->printf("No RAMTRON SPI device defined!\n");
        return false;
    }

    for (uint8_t r = 0; r < RAMTRON_RETRIES; r++) {
        if (r != 0) {
            hal.scheduler->delay(RAMTRON_DELAY_MS);
        }

        if (_init()) {
            return true;
        }
    }

    return false;
}

bool AP_RAMTRON::_fill_cmd_buffer(uint8_t cmdBuffer[], uint32_t const kCmdBufferSz, uint8_t const cmd, uint32_t addr)
{
    if (kCmdBufferSz == 4) {
        cmdBuffer[1] = uint8_t((addr >> 16) & 0xFF);
        cmdBuffer[2] = uint8_t((addr >>  8) & 0xFF);
        cmdBuffer[3] = uint8_t((addr >>  0) & 0xFF);
    } else if (kCmdBufferSz == 3) {
        cmdBuffer[1] = uint8_t((addr >>  8) & 0xFF);
        cmdBuffer[2] = uint8_t((addr >>  0) & 0xFF);
    } else {
        return false;
    }
    cmdBuffer[0] = cmd;
    return true;
}

// read from device
bool AP_RAMTRON::read(uint32_t offset, uint8_t * const buf, uint32_t size)
{
    // Don't allow reads outside of the FRAM memory.
    // NOTE: The FRAM devices will wrap back to address 0x0000 if they read past
    // the end of their internal memory, so while we'll get data back, it won't
    // be what we expect.
    if ((size > get_size()) ||
        (offset > (get_size() - size))) {
        return false;
    }

    uint32_t const kMaxReadSz = 128;
    uint32_t numRead = 0;

    while (size > 0) {
        uint32_t const kCmdBufferSz = ramtron_ids[id].addrlen + 1;
        uint8_t cmdBuffer[kCmdBufferSz];
        if (!_fill_cmd_buffer(cmdBuffer, kCmdBufferSz, RAMTRON_READ, (offset + numRead))) {
            return false;
        } else {
            WITH_SEMAPHORE(dev->get_semaphore());

            uint32_t const kReadSz = MIN(size, kMaxReadSz);
            // we can let transfer() handle the setting of the chip select, unlike in _write()
            bool ok = dev->transfer(cmdBuffer, kCmdBufferSz, &buf[numRead], kReadSz);

            if (!ok) {
                return false;
            } else {
                numRead += kReadSz;
                size -= kReadSz;
            }
        }
    }

    return true;
}

// write to device
bool AP_RAMTRON::write(uint32_t offset, uint8_t const * const buf, uint32_t size)
{
    // Don't allow writes outside of the FRAM memory.
    // NOTE: The FRAM devices will wrap back to address 0x0000 if they write past
    // the end of their internal memory, so we could accidentally overwrite the
    // wrong memory location.
    if ((size > get_size()) ||
        (offset > (get_size() - size))) {
        return false;
    }

    uint32_t const kCmdBufferSz = ramtron_ids[id].addrlen + 1;
    uint8_t cmdBuffer[kCmdBufferSz];
    if (!_fill_cmd_buffer(cmdBuffer, kCmdBufferSz, RAMTRON_WRITE, offset)) {
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    // We need to explicitly set the chip select here, rather than letting the
    // transfer() function handle it. This is because the write is performed in
    // two parts:
    //     1. configure the address to write to; and
    //     2. transfer the data.
    // If the chip select line is de-asserted between 1. and 2., the FRAM chip
    // resets the write address, and hence will not write the new data properly.
    bool ok =  dev->set_chip_select(true);
    ok = ok && dev->transfer(cmdBuffer, kCmdBufferSz, nullptr, 0);
    ok = ok && dev->transfer(buf, size, nullptr, 0);
    // always want to de-assert chip select.
    ok &= dev->set_chip_select(false);

    return ok;
}
