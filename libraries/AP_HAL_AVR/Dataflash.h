
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

    /* Concrete public methods: */
    uint16_t num_pages() { return _num_pages; }
    uint8_t  mfg_id() { return _mfg; }
    uint16_t device_id() { return _device; }
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
    virtual void _buffer_to_page(uint8_t buffer_num, uint16_t page_addr,
                                    bool wait) = 0;
    virtual void _page_erase(uint16_t page_addr) = 0;
    virtual void _block_erase(uint16_t block_addr) = 0;
    virtual void _chip_erase() = 0;
    
    virtual void _buffer_write(uint8_t buffer_num, uint16_t page_addr,
                                uint8_t data) = 0;
    virtual uint8_t  _buffer_read(uint8_t buffer_num, uint16_t page_addr) = 0;

    /* Concrete private methods: */

    int16_t _find_last_page();
    int16_t _find_last_page_of_log(uint16_t log_num);
    bool _check_wrapped();

    /* Instance variables: */
    uint8_t _buffer_num;
    uint8_t _read_buffer_num;
    uint16_t _buffer_idx;
    uint16_t _read_buffer_idx;
    uint16_t _page_addr;
    uint16_t _read_page_addr;
    bool _stop_write;
    uint16_t _file_num;
    uint16_t _file_page;

    /* Instance variables which should be initialized by the child class: */
    uint8_t _mfg;
    uint16_t _device;
    uint16_t _page_size;
    uint16_t _num_pages;
};

/* APM1Dataflash and APM2Dataflash: fully concrete classes implementing
 * the remaining methods left virtual by CommonDataflash.
 */
class AP_HAL_AVR::APM1Dataflash : public AP_HAL_AVR::CommonDataflash {
public:
    void init(void* machtnichts);
    void read_mfg_id();
    bool media_present();
private:
    void _wait_ready();

    void _page_to_buffer(uint8_t buffer_num, uint16_t page_addr);
    void _buffer_to_page(uint8_t buffer_num, uint16_t page_addr, bool wait);

    void _page_erase(uint16_t page_addr);
    void _block_erase(uint16_t block_addr);
    void _chip_erase();

    void _buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data);
    uint8_t  _buffer_read(uint8_t buffer_num, uint16_t page_addr);

    uint8_t _read_status_reg();
    uint8_t _read_status();

    void _cs_active();
    void _cs_inactive();
};

class AP_HAL_AVR::APM2Dataflash : public AP_HAL_AVR::CommonDataflash {
public:
    void init(void* machtnichts);
    void read_mfg_id();
    bool media_present();
private:
    void _wait_ready();

    void _page_to_buffer(uint8_t buffer_num, uint16_t page_addr);
    void _buffer_to_page(uint8_t buffer_num, uint16_t page_addr, bool wait);

    void _page_erase(uint16_t page_addr);
    void _block_erase(uint16_t block_addr);
    void _chip_erase();

    void _buffer_write(uint8_t buffer_num, uint16_t page_addr, uint8_t data);
    uint8_t  _buffer_read(uint8_t buffer_num, uint16_t page_addr);
    
    uint8_t _read_status_reg();
    uint8_t _read_status();
    
    void _cs_active();
    void _cs_inactive();
    uint8_t _transfer(uint8_t data);
};

#endif // __AP_HAL_AVR_DATAFLASH_H__

