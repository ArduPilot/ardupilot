
#ifndef __AP_HAL_AVR_DATAFLASH_H__
#define __AP_HAL_AVR_DATAFLASH_H__

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"


/* CommonDataflash: A partial concrete class providing the common
 * implementations for most of the AP_HAL::Dataflash methods. */
class AP_HAL_AVR::CommonDataflash : public AP_HAL::Dataflash {
public:
    /* Implementation-specific public methods (re-exported from AP_Dataflash) */
    virtual void init(void *implspecific) = 0;
    virtual void read_mfg_id() = 0;
    virtual bool media_present() = 0;
    virtual uint16_t num_pages() = 0;

    /* Concrete public methods: */
    uint16_t get_page() { return _read_page_addr - 1; }
    uint16_t get_write_page() { return _page_addr; }

    void erase_all();
    bool need_erase();

    void start_write(int16_t page);
    void finish_write();
    void write_byte(uint8_t data);
    void write_word(uint16_t data);
    void write_dword(uint32_t data);

    void start_read(int16_t page);
    uint8_t read_byte();
    uint16_t read_word();
    uint32_t read_dword();

    void set_file(uint16_t filenum);
    uint16_t get_file() { return _file_num; }
    uint16_t get_file_page() { return _file_page; }

    int16_t find_last_log();
    void get_log_boundaries(uint8_t log,
                int16_t &startpage, int16_t &endpage);
    uint8_t get_num_logs();
    void start_new_log();

protected:
    /* Implementation-specific private methods: */
    virtual void _wait_ready() = 0;
    virtual void _page_to_buffer(uint8_t buffer_num, uint16_t page_addr) = 0;

    uint8_t _buffer_num;
    uint8_t _read_buffer_num;
    uint16_t _buffer_idx;
    uint16_t _read_buffer_idx;
    uint16_t _page_addr;
    uint16_t _read_page_addr;
    uint8_t _stop_write;
    uint16_t _file_num;
    uint16_t _file_page;

    uint8_t _mfg;
    uint16_t _device;
    uint16_t _page_size;
};

/* APM1Dataflash and APM2Dataflash: fully concrete classes implementing
 * the remaining methods left virtual by CommonDataflash.
 */
class AP_HAL_AVR::APM1Dataflash : public AP_HAL_AVR::CommonDataflash {
public:
    void init(void* machtnichts);
    void read_mfg_id();
    bool media_present();
    uint16_t num_pages();
private:
    void _wait_ready();
    void _page_to_buffer(uint8_t buffer_num, uint16_t page_addr);
};

class AP_HAL_AVR::APM2Dataflash : public AP_HAL_AVR::CommonDataflash {
public:
    void init(void* machtnichts);
    void read_mfg_id();
    bool media_present();
    uint16_t num_pages();
private:
    void _wait_ready();
    void _page_to_buffer(uint8_t buffer_num, uint16_t page_addr);
};

#endif // __AP_HAL_AVR_DATAFLASH_H__

