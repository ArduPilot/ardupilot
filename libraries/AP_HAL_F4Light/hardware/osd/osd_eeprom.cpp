/*
(c) 2017 night_ghost@ykoctpa.ru
 
    EEPROM emulation for OSD needs. Uses 1 16K  page of flash to provide 2K of eeprom so it does wear 
     leveling of 8, excluding the case when we should write 0xff value, but OSD never writes it

    in time of page erase all data buffered in RAM so will be lost in case of unexpected power off, but there is no in-flight writes

*/

#include "osd_eeprom.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#include <utility>
#include <util.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

uint32_t OSD_EEPROM::ee_ptr=0;
static uint8_t data[EEPROM_SIZE] IN_CCM; // buffer for all data to be cached


void OSD_EEPROM::init(){

    for(uint16_t i=0;i<EEPROM_SIZE; i++){ // read out all to buffer
        data[i] = _read(i);
    }
}

uint8_t OSD_EEPROM::read(uint16_t addr){
    return data[addr];
} 

void OSD_EEPROM::write(uint16_t addr, uint8_t val){
    data[addr]=val;
    _write(addr, val); // 
}


uint8_t OSD_EEPROM::_read(uint16_t addr){
    
    for(uint8_t i=PAGE_SIZE/EEPROM_SIZE; i!=0;){
        // look most recent value from last stage
        i--;
        uint32_t ea = EEPROM_PAGE + i*EEPROM_SIZE + addr;
        uint8_t val = read_8(ea);
        if(val != 0xFF) {
            ee_ptr=ea;  // remember last read address
            return val;     // first non-FF is a value
        }
    }
    ee_ptr = EEPROM_PAGE + addr;
    return 0xff; // got to begin and still FF - really FF
}


void OSD_EEPROM::_write(uint16_t addr, uint8_t val){
    uint8_t cv = _read(addr);
    if(cv == val) return; // already is

    
    if( /* we can write - there is no '0' where we need '1' */ (~cv & val)==0 ){
        FLASH_Unlock_dis();
        write_8(ee_ptr, val);    // just overwrite last value
        goto done;
    }

    if(val != 0xFF){ // the only way to write FF is to clear all - but OSD don't writes it
        for(uint8_t i=0; i<PAGE_SIZE/EEPROM_SIZE; i++){
            // look 0xFF 
            uint32_t ea = EEPROM_PAGE + i*EEPROM_SIZE + addr;
            cv = read_8(ea);
            if(cv == 0xFF) { // empty 
                FLASH_Unlock_dis();
                write_8(ea, val);
                goto done;
            }
        }
    }

    // no empty slots - so need to erase page
    
    
// 1st - erase page. power loss here cause data loss! In execution time CPU is frozen!
    printf("\nEEprom_OSD erase page %d\n ", (uint16_t)((EEPROM_PAGE & 0x00ffffff) / 0x4000) ); // clear high byte of address and count 16K blocks
    FLASH_Unlock_dis();
    erasePageByAddress(EEPROM_PAGE); 

// 2rd write data back to the beginning of Flash page
    for(uint16_t i=0;i<EEPROM_SIZE; i++){
        write_8(EEPROM_PAGE+i, data[i]);
    }
    
done:
    FLASH_Lock_check();
}

