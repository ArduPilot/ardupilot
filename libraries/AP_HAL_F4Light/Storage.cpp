/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*

(c) 2017 night_ghost@ykoctpa.ru
 

  This uses 2*16k pages of FLASH ROM to emulate an EEPROM
  This storage is retained after power down, and survives reloading of firmware via bootloader
  All multi-byte accesses are reduced to single byte access so that can span EEPROM block boundaries
  
  http://www.st.com/content/ccc/resource/technical/document/application_note/ec/dd/8e/a8/39/49/4f/e5/DM00036065.pdf/files/DM00036065.pdf/jcr:content/translations/en.DM00036065.pdf
 
  problems of such design
  http://ithare.com/journaled-flash-storage-emulating-eeprom-over-flash-acid-transactions-and-more-part-ii-existing-implementations-by-atmel-silabs-ti-stm-and-microchip/

"partial write" problem fixed by requiring that highest bit of address should be 0

 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT

#pragma GCC optimize ("O2")

#include <string.h>
#include "Storage.h"
#include "EEPROM.h"
#include "Scheduler.h"

using namespace F4Light;

extern const AP_HAL::HAL& hal;

// The EEPROM class uses 2x16k FLASH ROM pages to emulate up to 8k of EEPROM.

#if defined(WRITE_IN_THREAD)
volatile uint16_t     Storage::rd_ptr = 0;
volatile uint16_t     Storage::wr_ptr = 0;
Storage::Item Storage::queue[EEPROM_QUEUE_LEN] IN_CCM;
void *Storage::_task;
#endif


#if defined(EEPROM_CACHED)
uint8_t Storage::eeprom_buffer[BOARD_STORAGE_SIZE] IN_CCM;
#endif


// This is the size of each FLASH ROM page
const uint32_t pageSize  = 0x4000; // real page size

// This defines the base addresses of the 2 FLASH ROM pages that will be used to emulate EEPROM
// These are the 2 16k pages in the FLASH ROM address space on the STM32F4 used by HAL
// This will effectively provide a total of 8kb of emulated EEPROM storage
const uint32_t pageBase0 = 0x08008000; // Page2
const uint32_t pageBase1 = 0x0800c000; // Page3

// it is possible to move EEPROM area to sectors 1&2 to free sector 3 for code (firmware from 0x0800c000)
// or use 3 sectors for EEPROM as wear leveling

static EEPROMClass eeprom IN_CCM;
bool Storage::write_deferred IN_CCM;


Storage::Storage()
{}


void Storage::late_init(bool defer) { 
    write_deferred = defer; 
    Scheduler::register_on_disarm( Scheduler::get_handler(do_on_disarm) ); 
}

void Storage::error_parse(uint16_t status){
    switch(status) {
    case EEPROM_NO_VALID_PAGE: // despite repeated attempts, EEPROM does not work, but should
        AP_HAL::panic("EEPROM Error: no valid page\r\n");
        break;

    case EEPROM_OUT_SIZE:
        AP_HAL::panic("EEPROM Error: full\r\n");
        break;
        
    case EEPROM_BAD_FLASH: // 
        AP_HAL::panic("EEPROM Error: page not empty after erase\r\n");
        break;

    case EEPROM_WRITE_FAILED:
        AP_HAL::panic("EEPROM Error: write failed\r\n");
        break;

    case EEPROM_BAD_ADDRESS: // just not found
    case EEPROM_NOT_INIT:    // can't be
    default:
        break; // all OK
    }
}


void Storage::init()
{
    eeprom.init(pageBase1, pageBase0, pageSize);

#if defined(EEPROM_CACHED)
    uint16_t i;
    for(i=0; i<BOARD_STORAGE_SIZE;i+=2){ // read out all data to RAM buffer

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align" // yes I know

        error_parse( eeprom.read(i >> 1, (uint16_t *)&eeprom_buffer[i]));
#pragma GCC diagnostic pop
    }
#endif


    _task = Scheduler::start_task(write_thread, 512); // small stack
    if(_task){
        Scheduler::set_task_priority(_task, MAIN_PRIORITY+2); // slightly less
    }
}

uint8_t Storage::read_byte(uint16_t loc){

#if defined(EEPROM_CACHED)
    return eeprom_buffer[loc];
#else
    return _read_byte(loc);
#endif
}

uint8_t Storage::_read_byte(uint16_t loc){

    // 'bytes' are packed 2 per word
    // Read existing dataword and use upper or lower byte

    uint16_t data;
    error_parse( eeprom.read(loc >> 1, &data) );

    if (loc & 1)
	return data >> 8; // Odd, upper byte
    else
	return data & 0xff; // Even lower byte
}



void Storage::read_block(void* dst, uint16_t loc, size_t n) {
#if defined(EEPROM_CACHED)
    memmove(dst, &eeprom_buffer[loc], n);
#else
    // Treat as a block of bytes
    uint8_t *ptr_b=(uint8_t *)dst;
    
    if(loc & 1){
        *ptr_b++ = read_byte(loc++);
        n--;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
    uint16_t *ptr_w=(uint16_t *)ptr_b;
#pragma GCC diagnostic pop
    
    while(n>=2){
        error_parse( eeprom.read(loc >> 1, ptr_w++) );
        loc+=2;
        n-=2;
    }
    
    if(n){
        ptr_b=(uint8_t *)ptr_w;
        *ptr_b = read_byte(loc);
    }    
#endif
}

void Storage::write_byte(uint16_t loc, uint8_t value){
#if defined(EEPROM_CACHED)
    if(eeprom_buffer[loc]==value) return;
    eeprom_buffer[loc]=value;

    if(write_deferred && hal.util->get_soft_armed()) return; // no changes in EEPROM, just in memory

#endif
    _write_byte(loc,value);    
}

void Storage::_write_byte(uint16_t loc, uint8_t value){
    // 'bytes' are packed 2 per word
    // Read existing data word and change upper or lower byte
    uint16_t data;

#if defined(EEPROM_CACHED)
    memmove(&data,&eeprom_buffer[loc & ~1], 2); // read current value from cache
#else
    error_parse(eeprom.read(loc >> 1, &data)); // read current value
#endif

    if (loc & 1)
	data = (data & 0x00ff) | (value << 8); // Odd, upper byte
    else
	data = (data & 0xff00) | value;        // Even, lower byte
    write_word(loc >> 1, data);
}


void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
#if defined(EEPROM_CACHED)
    memmove(&eeprom_buffer[loc], src, n);

    if(write_deferred && hal.util->get_soft_armed()) return; // no changes in EEPROM, just in memory

#endif

    uint8_t *ptr_b = (uint8_t *)src;     // Treat as a block of bytes
    if(loc & 1){
        _write_byte(loc++, *ptr_b++);      // odd byte
        n--;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
    uint16_t *ptr_w = (uint16_t *)ptr_b;     // Treat as a block of words
#pragma GCC diagnostic pop
    while(n>=2){
        write_word(loc >> 1, *ptr_w++);
        loc+=2;
        n-=2;
    }

    if(n){ // the last one
        ptr_b=(uint8_t *)ptr_w;
        _write_byte(loc, *ptr_b);      // odd byte
    }
}

void Storage::do_on_disarm(){ // save changes to EEPROM
    uint16_t i;
    for(i=0; i<BOARD_STORAGE_SIZE; i+=2){
        uint16_t data;
        error_parse(eeprom.read(i >> 1, &data));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
        uint16_t b_data = *((uint16_t *)&eeprom_buffer[i]);
#pragma GCC diagnostic pop
        
        if(b_data!=data){
            write_word(i >> 1, b_data);
        }
    }
}

void Storage::write_word(uint16_t loc, uint16_t data){
#if defined(WRITE_IN_THREAD)

    Item &d = queue[wr_ptr];
    d.loc=loc;
    d.val=data;

    uint16_t new_wp = wr_ptr+1;

    if(new_wp >= EEPROM_QUEUE_LEN) { // move write pointer
        new_wp=0;                    // ring
    }

    while(new_wp == rd_ptr) { // buffer overflow
        hal_yield(300);      // wait for place
    }

    wr_ptr=new_wp; // move forward
    
    Scheduler::set_task_active(_task); // activate write thread
#else
    error_parse(eeprom.write(loc, data));
#endif
}

#if defined(WRITE_IN_THREAD)
void Storage::write_thread(){
    while(rd_ptr != wr_ptr) { // there are items
        Item d =  queue[rd_ptr++];  // get data and move to next item
        if(rd_ptr >= EEPROM_QUEUE_LEN) { // move write pointer
            rd_ptr=0;                       // ring            
        }
        error_parse(eeprom.write(d.loc, d.val));
    }
}
#endif

#endif

