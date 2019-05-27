//
// Unit tests for the AP_Math rotations code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_FlashStorage/AP_FlashStorage.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class FlashTest : public AP_HAL::HAL::Callbacks {
public:
    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

private:
    static const uint32_t flash_sector_size = 32U * 1024U;

    uint8_t mem_buffer[AP_FlashStorage::storage_size];
    uint8_t mem_mirror[AP_FlashStorage::storage_size];

    // flash buffer
    uint8_t *flash[2];

    bool flash_write(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length);
    bool flash_read(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length);
    bool flash_erase(uint8_t sector);
    bool flash_erase_ok(void);
    
    AP_FlashStorage storage{mem_buffer,
            flash_sector_size,
            FUNCTOR_BIND_MEMBER(&FlashTest::flash_write, bool, uint8_t, uint32_t, const uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&FlashTest::flash_read, bool, uint8_t, uint32_t, uint8_t *, uint16_t),
            FUNCTOR_BIND_MEMBER(&FlashTest::flash_erase, bool, uint8_t),
            FUNCTOR_BIND_MEMBER(&FlashTest::flash_erase_ok, bool)};

    // write to storage and mem_mirror
    void write(uint16_t offset, const uint8_t *data, uint16_t length);

    bool erase_ok;
};

bool FlashTest::flash_write(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    if (sector > 1) {
        AP_HAL::panic("FATAL: write to sector %u\n", (unsigned)sector);
    }
    if (offset + length > flash_sector_size) {
        AP_HAL::panic("FATAL: write to sector %u at offset %u length %u\n",
                      (unsigned)sector,
                      (unsigned)offset,
                      (unsigned)length);
    }
    uint8_t *b = &flash[sector][offset];
    if ((offset & 1) || (length & 1)) {
        AP_HAL::panic("FATAL: invalid write at %u:%u len=%u\n",
                      sector, offset, length);
    }
    const uint16_t *data16 = (const uint16_t *)data;
    uint16_t *b16 = (uint16_t *)&b[0];
    uint16_t len16 = length/2;
    for (uint16_t i=0; i<len16; i++) {
        if (data16[i] & !b16[i]) {
            AP_HAL::panic("FATAL: invalid write16 at %u:%u 0x%04x 0x%04x\n",
                          sector, offset+i, b[i], data[i]);
        }
#if !AP_FLASHSTORAGE_MULTI_WRITE
        if (data16[i] != b16[i] && data16[i] != 0xFFFF && b16[i] != 0xFFFF) {
            AP_HAL::panic("FATAL: invalid write16 at %u:%u 0x%04x 0x%04x\n",
                          sector, offset+i, b[i], data[i]);
        }
#endif
        b16[i] &= data16[i];
    }
    return true;
}

bool FlashTest::flash_read(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    if (sector > 1) {
        AP_HAL::panic("FATAL: read from sector %u\n", (unsigned)sector);
    }
    if (offset + length > flash_sector_size) {
        AP_HAL::panic("FATAL: read from sector %u at offset %u length %u\n",
                      (unsigned)sector,
                      (unsigned)offset,
                      (unsigned)length);
    }
    memcpy(data, &flash[sector][offset], length);
    return true;
}

bool FlashTest::flash_erase(uint8_t sector)
{
    if (sector > 1) {
        AP_HAL::panic("FATAL: erase sector %u\n", (unsigned)sector);
    }
    memset(&flash[sector][0], 0xFF, flash_sector_size);
    return true;
}

bool FlashTest::flash_erase_ok(void)
{
    return erase_ok;
}

void FlashTest::write(uint16_t offset, const uint8_t *data, uint16_t length)
{
    memcpy(&mem_mirror[offset], data, length);
    memcpy(&mem_buffer[offset], data, length);
    if (!storage.write(offset, length)) {
        if (erase_ok) {
            printf("Failed to write at %u for %u\n", offset, length);
        }
    }
}

/*
 * test flash storage
 */
void FlashTest::setup(void)
{
    hal.console->printf("AP_FlashStorage test\n");
}

void FlashTest::loop(void)
{
    flash[0] = (uint8_t *)malloc(flash_sector_size);
    flash[1] = (uint8_t *)malloc(flash_sector_size);
    flash_erase(0);
    flash_erase(1);

    if (!storage.init()) {
        AP_HAL::panic("Failed first init()");
    }

    // fill with 10k random writes
    for (uint32_t i=0; i<5000000; i++) {
        uint16_t ofs = get_random16() % sizeof(mem_buffer);
        uint16_t length = get_random16() & 0x1F;
        length = MIN(length, sizeof(mem_buffer) - ofs);
        uint8_t data[length];
        for (uint8_t j=0; j<length; j++) {
            data[j] = get_random16() & 0xFF;
        }

        erase_ok = (i % 1000 == 0);
        write(ofs, data, length);

        if (erase_ok) {
            if (memcmp(mem_buffer, mem_mirror, sizeof(mem_buffer)) != 0) {
                AP_HAL::panic("FATAL: data mis-match at i=%u", (unsigned)i);
            }
        }
    }

    // force final write to allow for flush with erase_ok
    erase_ok = true;
    uint8_t b = 42;
    write(37, &b, 1);
    
    if (memcmp(mem_buffer, mem_mirror, sizeof(mem_buffer)) != 0) {
        AP_HAL::panic("FATAL: data mis-match before re-init");
    }
    
    // re-init
    printf("re-init\n");
    memset(mem_buffer, 0, sizeof(mem_buffer));
    if (!storage.init()) {
        AP_HAL::panic("Failed second init()");
    }

    if (memcmp(mem_buffer, mem_mirror, sizeof(mem_buffer)) != 0) {
        AP_HAL::panic("FATAL: data mis-match");
    }
    while (true) {
        hal.console->printf("TEST PASSED");
        hal.scheduler->delay(20000);
    }
}

FlashTest flashtest;

AP_HAL_MAIN_CALLBACKS(&flashtest);
