#include <AP_HAL.h>
#if (CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2)

#include <avr/io.h>
#include <avr/eeprom.h>
#include "Storage.h"
using namespace AP_HAL_AVR;

uint8_t AVREEPROMStorage::read_byte(uint16_t loc) {
    return eeprom_read_byte((uint8_t*)loc);
}

uint16_t AVREEPROMStorage::read_word(uint16_t loc) {
    return eeprom_read_word((uint16_t*)loc);
}

uint32_t AVREEPROMStorage::read_dword(uint16_t loc) {
    return eeprom_read_dword((uint32_t*)loc);
}

void AVREEPROMStorage::read_block(void *dst, uint16_t src, size_t n) {
    eeprom_read_block(dst,(const void*)src,n);
}

void AVREEPROMStorage::write_byte(uint16_t loc, uint8_t value) {
    eeprom_write_byte((uint8_t*)loc,value);
}

void AVREEPROMStorage::write_word(uint16_t loc, uint16_t value) {
    eeprom_write_word((uint16_t*)loc,value);
}

void AVREEPROMStorage::write_dword(uint16_t loc, uint32_t value) {
    eeprom_write_dword((uint32_t*)loc,value);
}

void AVREEPROMStorage::write_block(uint16_t dst, void *src, size_t n) {
    eeprom_write_block(src,(void*)dst,n);
}

#endif
