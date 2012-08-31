
#ifndef __AP_HAL_DATAFLASH_H__
#define __AP_HAL_DATAFLASH_H__

#include "AP_HAL_Namespace.h"

/* Dataflash abstract class:
 * This isn't the abstraction ArduPilot needs, but it is the abstraction
 * ArduPilot deserves.
 *
 * I'll build a proper logging driver later which hides the details of using
 * a dataflash / page based system under the hood. For now the Dataflash layer
 * will be a port of the existing DataFlash APM1/APM2 driver.
 * - pch, 31aug12
 * */
class AP_HAL::Dataflash {
public:
    virtual void init(void* implspecific) = 0;
    virtual void read_mfg_id() = 0;
    virtual bool media_present() = 0;
    virtual uint16_t num_pages() = 0;

    virtual uint16_t get_page() = 0;
    virtual uint16_t get_write_page() = 0;

    virtual void erase_all() = 0;
    virtual bool need_erase() = 0;

    virtual void start_write(int16_t page) = 0;
    virtual void finish_write() = 0;
    virtual void write_byte(uint8_t data) = 0;
    virtual void write_word(uint16_t data) = 0;
    virtual void write_dword(uint32_t data) = 0;

    virtual void start_read(int16_t page) = 0;
    virtual uint8_t read_byte() = 0;
    virtual uint16_t read_word() = 0;
    virtual uint32_t read_dword() = 0;

    virtual void set_file(uint16_t filenum) = 0;
    virtual uint16_t get_file() = 0;
    virtual uint16_t get_file_page() = 0;

    virtual int16_t find_last_log() = 0;
    virtual void get_log_boundaries(uint8_t log,
                        int16_t &startpage, int16_t &endpage) = 0;
    virtual uint8_t get_num_logs() = 0;
    virtual void start_new_log() = 0;

};

#endif // __AP_HAL_DATAFLASH_H__

