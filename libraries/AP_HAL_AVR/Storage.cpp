
#include <avr/io.h>
#include <avr/eeprom.h>
#include "Storage.h"
using namespace AP_HAL_AVR;

uint8_t AVREEPROMStorage::read_byte(const uint8_t   *p) {
    return eeprom_read_byte(p);
}

uint16_t AVREEPROMStorage::read_word(const uint16_t  *p) {
    return eeprom_read_word(p);
}

uint32_t AVREEPROMStorage::read_dword(const uint32_t *p) {
    return eeprom_read_dword(p);
}

void AVREEPROMStorage::read_block(void *dst, const void *src, size_t n) {
    eeprom_read_block(dst,src,n);
}

void AVREEPROMStorage::write_byte(uint8_t   *p, uint8_t value) {
    eeprom_write_byte(p,value);
}

void AVREEPROMStorage::write_word(uint16_t  *p, uint16_t value) {
    eeprom_write_word(p,value);
}

void AVREEPROMStorage::write_dword(uint32_t *p, uint32_t value) {
    eeprom_write_dword(p,value);
}

void AVREEPROMStorage::write_block(void *src, void *dst, size_t n) {
    eeprom_write_block(src,dst,n);
}

