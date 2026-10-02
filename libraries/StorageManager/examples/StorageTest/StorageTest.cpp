//
// Simple test for the StorageManager class
//

#include <AP_HAL/AP_HAL.h>
#include <StorageManager/StorageManager.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define DO_INITIALISATION 1

/*
  instantiate all 4 storage types
 */
static StorageAccess all_storage[4] = { 
    StorageAccess(StorageManager::StorageParam),
    StorageAccess(StorageManager::StorageMission),
    StorageAccess(StorageManager::StorageFence),
    StorageAccess(StorageManager::StorageRally)
};

/*
  simple random generator, see http://en.wikipedia.org/wiki/Random_number_generation
 */
static uint16_t get_random(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 65535) + (m_z >> 16);
    m_w = 18000 * (m_w & 65535) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}

/*
  return a predictable value for an offset
 */
static uint8_t pvalue(uint16_t offset)
{
    return ((offset * 7) + 13) % 65536;
}

void setup(void)
{
    hal.console->printf("StorageTest startup...\n");
#if DO_INITIALISATION
    for (uint8_t type = 0; type < 4; type++) {
        const StorageAccess &storage = all_storage[type];
        hal.console->printf("Init type %u\n", (unsigned)type);
        for (uint16_t i = 0; i < storage.size(); i++) {
            storage.write_byte(i, pvalue(i));
        }
    }
#endif
}

void loop(void)
{
    static uint32_t count;
    static uint32_t total_bytes_read;
    static uint32_t total_bytes_written;
    static uint32_t last_print_ms = AP_HAL::millis();
    uint8_t type = get_random() % 4;
    const StorageAccess &storage = all_storage[type];
    uint16_t offset = get_random() % storage.size();
    uint8_t  length = (get_random() & 31);
    if (offset + length > storage.size()) {
        length = storage.size() - offset;
    }
    if (length == 0) {
        return;
    }
    uint8_t b[length];
    for (uint8_t i=0; i<length; i++) {
        b[i] = pvalue(offset+i);
    }

    if (get_random() % 2 == 1) {
        if (!storage.write_block(offset, b, length)) {
            hal.console->printf("write failed at offset %u length %u\n",
                                (unsigned)offset, (unsigned)length);
        } else {
            total_bytes_written += length;
        }
    } else {
        uint8_t b2[length];
        if (!storage.read_block(b2, offset, length)) {
            hal.console->printf("read failed at offset %u length %u\n",
                                (unsigned)offset, (unsigned)length);
        } else {
            total_bytes_read += length;
        }
        if (memcmp(b, b2, length) != 0) {
            hal.console->printf("bad data at offset %u length %u\n",
                                (unsigned)offset, (unsigned)length);
        }
    }

    count++;

    uint32_t now = AP_HAL::millis();
    if (now - last_print_ms >= 1000) {
        /*
          Reference test results:
          Board: custom STM32H7 board (HAL_STORAGE_SIZE=16384, onboard flash pages 14-15)
          MCU clock speed: 400Mhz
          Storage backend: onboard flash (no SD card involved)
          Measured rate: ~139-141 kB/s read, ~137-141 kB/s write, ~278-281 kB/s combined (stable over 180k+ ops)
        */
        float elapsed_s = (now - last_print_ms) / 1000.0f;
        float read_rate  = (total_bytes_read  / 1024.0f) / elapsed_s;
        float write_rate = (total_bytes_written / 1024.0f) / elapsed_s;
        float total_rate = read_rate + write_rate;

        hal.console->printf("%u ops: read=%.2f kB/s write=%.2f kB/s total=%.2f kB/s\n", (unsigned)count, read_rate, write_rate, total_rate);

        total_bytes_read = 0;
        total_bytes_written = 0;
        last_print_ms = now;
    }
}



AP_HAL_MAIN();
