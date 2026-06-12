/*
 * driver for Page EEPROM persistent memory devices. These are used for parameter and
 * waypoint storage on most boards
 */

#include "AP_PageEEPROM.h"
#include <AP_Math/crc.h>
#include <AP_Math/AP_Math.h>

#if defined(HAL_WITH_PAGE_EEPROM) && HAL_WITH_PAGE_EEPROM
extern const AP_HAL::HAL &hal;

// register numbers
static const uint8_t PageEEPROM_RDID  = 0x9f;
static const uint8_t PageEEPROM_READ  = 0x03;
static const uint8_t PageEEPROM_WREN  = 0x06;
static const uint8_t PageEEPROM_WRITE = 0x02;

#define PG_EEPROM_WRITE_ENABLE  	0x06
#define PG_EEPROM_WRITE_DISABLE		0x04
#define PG_EEPROM_READ_STATUS		0x05
#define PG_EEPROM_WRITE_STATUS		0x01
#define PG_EEPROM_READ_DATA		0x03
#define PG_EEPROM_FAST_READ		0x0b
#define PG_EEPROM_DEVICE_ID		0x9f
#define PG_EEPROM_PAGE_WRITE		0x02

#define PG_EEPROM_BULK_ERASE		0xC7
#define PG_EEPROM_SECTOR4_ERASE		0x20 // 4k erase
#define PG_EEPROM_BLOCK32_ERASE		0x52 // 32k erase
#define PG_EEPROM_BLOCK64_ERASE		0xD8 // 64k erase

#define PG_EEPROM_STATUS_BUSY		0x01
#define PG_EEPROM_STATUS_WRITEPROTECT	0x02
#define PG_EEPROM_STATUS_BP0		0x04
#define PG_EEPROM_STATUS_BP1		0x08
#define PG_EEPROM_STATUS_BP2		0x10
#define PG_EEPROM_STATUS_TP		0x20
#define PG_EEPROM_STATUS_SEC		0x40
#define PG_EEPROM_STATUS_SRP0		0x80

#define PAGE_EEPROM_RETRIES 10
#define PAGE_EEPROM_DELAY_MS 10


/*
 * list of supported devices,
 */
const AP_PageEEPROM::pg_eeprom_id AP_PageEEPROM::pg_eeprom_ids[] = {
    { 0x20, 0x00, 1000, 512, 3, RDID_type::ST },  //M95P08-I 
};

// initialise the driver
bool AP_PageEEPROM::init(void)
{
    dev = hal.spi->get_device_ptr("pg_eeprom");
    if (!dev) {
        DEV_PRINTF("No Page EEPROM device\n");
	return false;
    }

    dev_sem = dev->get_semaphore();

    struct st_rdid {
        uint8_t id1;
	uint8_t id2;
	uint8_t memory_density;
    };

    uint8_t rdid[sizeof(st_rdid)];

    {
        WITH_SEMAPHORE(dev_sem);
	uint8_t cmd = PG_EEPROM_DEVICE_ID;
	if (!dev->read_registers(cmd, rdid, sizeof(rdid))) {
	    return false;
	}
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(pg_eeprom_ids); i++) {
        if (pg_eeprom_ids[i].rdid_type == RDID_type::ST) {
            const st_rdid *st = (const st_rdid *) rdid;
	    if (pg_eeprom_ids[i].id1 == st->id1 &&
		pg_eeprom_ids[i].id2 == st->id2) {
		id = i;
		break;
	    }
        }
    }

    if (id == UINT8_MAX) {
        DEV_PRINTF("Unknown Page EEPROM device\n");
        return false;
    }

    return true;
}


/*
 * send a command and offset
 */
void AP_PageEEPROM::send_offset(uint8_t cmd, uint32_t offset) const
{
    if (pg_eeprom_ids[id].addrlen == 3) {
        uint8_t b[4] = { cmd, 
			 uint8_t((offset>>16)&0xFF), 
			 uint8_t((offset>>8)&0xFF), 
			 uint8_t(offset&0xFF) 
	};
        dev->transfer(b, sizeof(b), nullptr, 0);

    } else /* len 2 */ {
        uint8_t b[3] = { cmd, 
			 uint8_t((offset>>8)&0xFF), 
			 uint8_t(offset&0xFF) 
	};
        dev->transfer(b, sizeof(b), nullptr, 0);
    }
}

// read from device
bool AP_PageEEPROM::read(uint32_t offset, uint8_t *buf, uint32_t size)
{
    // Don't allow reads outside of the EEPROM Page memory
    // Warning: The EEPROM Page devices will go back to address 0x0000 if the limit
    // is reached, so even if we get data back, it won't be what be what we expect.
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

    for (uint8_t r=0; r<PAGE_EEPROM_RETRIES; r++) {
        if (r != 0) {
	    hal.scheduler->delay(PAGE_EEPROM_DELAY_MS);
	}
        /*
          transfer each block twice and compare with a crc. This is to
          prevent transient errors from causing parameter corruption
         */
	{
	    WITH_SEMAPHORE(dev_sem);
	    dev->set_chip_select(true);
	    send_offset(PageEEPROM_READ, offset);
	    dev->transfer(nullptr, 0, buf, size);
	    dev->set_chip_select(false);
        }

	uint32_t crc1 = crc_crc32(0, buf, size);

	{
	    WITH_SEMAPHORE(dev_sem);
	    dev->set_chip_select(true);
	    send_offset(PageEEPROM_READ, offset);
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
bool AP_PageEEPROM::write(uint32_t offset, const uint8_t *buf, uint32_t size)
{
    // Don't allow reads outside of the EEPROM Page memory
    // Warning: The EEPROM Page devices will go back to address 0x0000 if the limit
    // is reached, if this happens we will overwrite the existent data.
    if ((size > get_size()) ||
        (offset > (get_size() - size))) {
	return false;
    }


    WITH_SEMAPHORE(dev_sem);

    const uint16_t page_size = pg_eeprom_ids[id].size_page_byte;

    for (uint8_t r=0; r<PAGE_EEPROM_RETRIES; r++) {
        if (r != 0) {
	    hal.scheduler->delay(PAGE_EEPROM_DELAY_MS);
	}

	// Write one physical page at a time. PP auto-erases the entire page
	// before writing, and the address counter wraps within the page (bit 8
	// resets), so a write that crosses a page boundary would overwrite the
	// start of the same page instead of advancing to the next one.
	uint32_t pos = offset;
	const uint8_t *src = buf;
	uint32_t remaining = size;
	while (remaining > 0) {
	    uint32_t chunk = MIN(remaining, page_size - (pos % page_size));

	    dev->set_chip_select(true);
	    dev->transfer(&PageEEPROM_WREN, 1, nullptr, 0);
	    dev->set_chip_select(false);

	    dev->set_chip_select(true);
	    send_offset(PageEEPROM_WRITE, pos);
	    dev->transfer(src, chunk, nullptr, 0);
	    dev->set_chip_select(false);

	    WaitReady();

	    pos += chunk;
	    src += chunk;
	    remaining -= chunk;
	}

	/*
	 * verify first 32 bytes of every write using a crc
	 */
	uint8_t rbuf[32] {};
	const uint8_t nverify = MIN(size, sizeof(rbuf));
	uint32_t crc1 = crc_crc32(0, buf, nverify);

	dev->set_chip_select(true);
	send_offset(PageEEPROM_READ, offset);
	dev->transfer(nullptr, 0, rbuf, nverify);
	dev->set_chip_select(false);

	uint32_t crc2 = crc_crc32(0, rbuf, nverify);

	if (crc1 == crc2) {
	    return true;
	}
    }
    
    return false;
}

void AP_PageEEPROM::WriteEnable(void) 
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd = PG_EEPROM_WRITE_ENABLE;
    dev->transfer(&cmd, 1, nullptr, 0);
}

uint8_t AP_PageEEPROM::ReadStatusReg(void)
{
    WITH_SEMAPHORE(dev_sem);
    uint8_t cmd = PG_EEPROM_READ_STATUS;
    uint8_t status;
    dev->transfer(&cmd, 1, &status, 1);
    return status;
}

bool AP_PageEEPROM::Busy(void)
{
    return (ReadStatusReg() & (PG_EEPROM_STATUS_BUSY)) != 0;
}

void AP_PageEEPROM::WaitReady(void)
{
    uint32_t t = AP_HAL::millis();
    while (Busy()) {
        hal.scheduler->delay_microseconds(100);
	if (AP_HAL::millis() - t > 5000) {
	    break;
	}

    }
}
#endif //HAL_WITH_PAGE_EEPROM
