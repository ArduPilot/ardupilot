/*
 * driver for Page EEPROM persistent memory devices
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if defined(HAL_WITH_PAGE_EEPROM) && HAL_WITH_PAGE_EEPROM
class AP_PageEEPROM {
public:
    // initialise the driver
    // this will retry PAGE_EEPROM_RETRIES times until successful
    bool init(void);

    // get size in bytes
    uint32_t get_size(void) const { return (id == UINT8_MAX) ? 0 : pg_eeprom_ids[id].size_kbyte * 1024UL; }

    // read from device
    // this will retry PAGE_EEPROM_RETRIES times until two successive reads return the same data 
    bool read(uint32_t offset, uint8_t * const buf, uint32_t size);

    // write to device
    bool write(uint32_t offset, uint8_t const * const buf, uint32_t);

private:
    AP_HAL::SPIDevice *dev;
    AP_HAL::Semaphore *dev_sem;

    enum class RDID_type : uint8_t {
	    ST
    };

    struct pg_eeprom_id {
        uint8_t id1;
	uint8_t id2;
	uint16_t size_kbyte;
	uint16_t size_page_byte;
	uint8_t addrlen;
	RDID_type rdid_type;
    };
    static const struct pg_eeprom_id pg_eeprom_ids[];
    uint8_t id = UINT8_MAX;

    // perform a single device initialization
    bool _init(void);
    // perform a single device read
    bool _read(uint32_t offset, uint8_t * const buf, uint32_t size);

    void send_offset(uint8_t cmd, uint32_t offset) const;

    // write enable
    void WriteEnable(void);

    // Read status
    uint8_t ReadStatusReg(void);

    // Device is busy?
    bool Busy(void);

    // Wait until ready
    void WaitReady(void);
};
#endif //HAL_WITH_PAGE_EEPROM
