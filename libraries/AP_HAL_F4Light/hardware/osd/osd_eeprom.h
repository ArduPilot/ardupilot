#pragma once
/*
    EEPROM emulation for OSD needs.

    uses 16k Flash sector at 0x08004000 (Page1) to emulate (almost) read-only storage
    for OSD configuration

*/

#include "AP_HAL_F4Light/EEPROM.h"

// This is the size of each FLASH ROM page
#define  PAGE_SIZE 0x4000l // real page size

#define EEPROM_PAGE 0x08004000l // Page1

#define EEPROM_SIZE 2048 // 2 k for wear leveling

class OSD_EEPROM /*: public EEPROMClass*/ { // mostly to not redefine some functions

public:
    static uint8_t read(uint16_t addr);
    static void write(uint16_t addr, uint8_t val);
    static void init();

private:
    static uint32_t ee_ptr; // address of current bank

    static uint8_t _read(uint16_t addr);
    static void _write(uint16_t addr, uint8_t val);

    static inline uint32_t read_16(uint32_t addr){
        return *(__IO uint16_t*)addr;
    }

    static inline uint8_t read_8(uint32_t addr){
        return *(__IO uint8_t*)addr;
    }

    static inline uint32_t read_32(uint32_t addr){
        return *(__IO uint32_t*)addr;
    }
    
    static inline FLASH_Status write_16(uint32_t addr, uint16_t data){
        return EEPROMClass::write_16(addr, data);
    }
    

    static inline FLASH_Status write_8(uint32_t addr, uint8_t data){ // need to unlock first
        return EEPROMClass::write_8(addr, data);
    }
    
    static inline FLASH_Status erasePageByAddress(uint32_t Page_Address) {
        return EEPROMClass::_ErasePageByAddress(Page_Address);
    }
    static inline void FLASH_Lock_check() {
        EEPROMClass::FLASH_Lock_check();
    }
    static inline void FLASH_Unlock_dis() {
        EEPROMClass::FLASH_Unlock_dis();
    }
};
