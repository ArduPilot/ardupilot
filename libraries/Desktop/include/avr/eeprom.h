#ifndef _AVR_EEPROM_H_
#define _AVR_EEPROM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint8_t eeprom_read_byte(const uint8_t *p);
uint16_t eeprom_read_word(const uint16_t *p);
uint32_t eeprom_read_dword(const uint32_t *p);
void eeprom_read_block(void *buf, void *ptr, uint8_t size);

void eeprom_write_byte(uint8_t *p, uint8_t value);
void eeprom_write_word(uint16_t *p, uint16_t value);
void eeprom_write_dword(uint32_t *p, uint32_t value);
void eeprom_write_block(const void *buf, void *ptr, uint8_t size);
	
#ifdef __cplusplus
}
#endif

#endif
