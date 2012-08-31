
#include "Dataflash.h"

using namespace AP_HAL_AVR;

void CommonDataflash::erase_all() {

}

bool CommonDataflash::need_erase() {

}

void CommonDataflash::start_write(int16_t page) {

}

void CommonDataflash::finish_write() { 

}

void CommonDataflash::write_byte(uint8_t data) {

}

void CommonDataflash::write_word(uint16_t data) {

}

void CommonDataflash::write_dword(uint32_t data) {

}

void CommonDataflash::start_read(int16_t page) {

}

uint8_t CommonDataflash::read_byte() {

}

uint16_t CommonDataflash::read_word() {

}

uint32_t CommonDataflash::read_dword() {

}

void CommonDataflash::set_file(uint16_t filenum) {

}

int16_t CommonDataflash::find_last_log() {

}

void CommonDataflash::get_log_boundaries(uint8_t log,
                                int16_t &startpage, int16_t &endpage) {

}

uint8_t CommonDataflash::get_num_logs() {

}

void CommonDataflash::start_new_log() {

}

