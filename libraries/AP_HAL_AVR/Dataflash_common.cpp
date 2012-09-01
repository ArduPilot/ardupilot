
#include <AP_HAL.h>
#include "Dataflash.h"

using namespace AP_HAL_AVR;

extern const AP_HAL::HAL& hal;

// 0: When reach the end page stop, 1: Start overwriting from page 1
#define DF_OVERWRITE_DATA true

// the last page holds the log format in first 4 bytes. Please change
// this if (and only if!) the low level format changes
#define DF_LOGGING_FORMAT    0x28122011

// we use an invalie logging format to test the chip erase
#define DF_LOGGING_FORMAT_INVALID   0x28122012

void CommonDataflash::erase_all() {
    for (uint16_t i = 1; i <= (_num_pages+1)/8; i++) {
        _block_erase(i);
        hal.scheduler->delay(1);
    }
    start_write(_num_pages+1);
    write_dword(DF_LOGGING_FORMAT);
    finish_write();
}

bool CommonDataflash::need_erase() {
    start_read(_num_pages+1);
    return (read_dword() != DF_LOGGING_FORMAT);
}

void CommonDataflash::start_write(int16_t page) {
    _buffer_num = 1;
    _buffer_idx = 4;
    _page_addr = page;
    _stop_write = false;

    _wait_ready();

    _buffer_write(_buffer_num, 0, _file_num >> 8 );
    _buffer_write(_buffer_num, 1, _file_num&0xFF );
    _buffer_write(_buffer_num, 2, _file_page >> 8 );
    _buffer_write(_buffer_num, 3, _file_page&0xFF );
}

void CommonDataflash::finish_write() { 
    _buffer_idx = 0;
    /* Write buffer to memory, no wait. */
    _buffer_to_page(_buffer_num, _page_addr, false);

    _page_addr++;
    if (DF_OVERWRITE_DATA) {
        if (_page_addr > _num_pages) {
            _page_addr = 1;
        }
    } else {
        if (_page_addr > _num_pages) {
            _stop_write = true;
        }
    }

    /* switch buffer to continue writing */
    _buffer_num = (_buffer_num == 1) ? 2 : 1; 
}

void CommonDataflash::write_byte(uint8_t data) {
    if (_stop_write) return;
    _buffer_write( _buffer_num, _buffer_idx, data );
    _buffer_idx++;
    /* end of buffer? */
    if ( _buffer_idx > _page_size) {
        /* 4 bytes for filenumber, filepage */
        _buffer_idx = 4;
        /* write buffer to memory, no waiting */
        _buffer_to_page(_buffer_num, _page_addr, false);
        _page_addr++;
        if (DF_OVERWRITE_DATA) {
            if (_page_addr > _num_pages) {
                _page_addr = 1;
            }
        } else {
            if (_page_addr > _num_pages) {
                _stop_write = true;
            }
        }
        /* switch buffer to continue writing */
        _buffer_num = (_buffer_num == 1) ? 2 : 1;

        /* We are starting a new page. write filenumber and filepage. */
        _buffer_write(_buffer_num, 0, _file_num >> 8 );
        _buffer_write(_buffer_num, 1, _file_num&0xFF );
        _buffer_write(_buffer_num, 2, _file_page >> 8 );
        _buffer_write(_buffer_num, 3, _file_page&0xFF );
    }
}

void CommonDataflash::write_word(uint16_t data) {
    write_byte( data >> 8 ); /* high byte */
    write_byte( data & 0xFF ); /* low byte */
}

void CommonDataflash::write_dword(uint32_t data) {
    write_byte( data >> 24 ); /* high byte */
    write_byte( data >> 16 );
    write_byte( data >> 8 );
    write_byte( data & 0xFF ); /* low byte */
}

void CommonDataflash::start_read(int16_t page) {
    _read_buffer_num = 1;
    _read_buffer_idx = 4;
    _read_page_addr = page;
    _wait_ready();
    /* Write memory page to buffer. */
    _page_to_buffer(_read_buffer_num, _read_page_addr);
    _read_page_addr++;

    /* We are starting a new page. Read file number and file page */
    _file_num  = _buffer_read(_read_buffer_num, 0);
    _file_num  = (_file_num  << 8) | _buffer_read(_read_buffer_num, 1);
    _file_page = _buffer_read(_read_buffer_num, 2);
    _file_page = (_file_page << 8 ) | _buffer_read(_read_buffer_num, 3);
}

uint8_t CommonDataflash::read_byte() {
    _wait_ready();
    uint8_t result = _buffer_read( _read_buffer_num, _read_buffer_idx);
    _read_buffer_idx++;
    /* Check if we reached the end of buffer */
    if ( _read_buffer_idx >= _page_size ) {
        /* 4 bytes for file number, file page */
        _read_buffer_idx = 4;
        /* Write memory page to buffer */
        _page_to_buffer(_read_buffer_num, _read_page_addr);
        _read_page_addr++;
        /* If we reach the end o fmemory, start from the beginning */
        if (_read_page_addr > _num_pages) {
            _read_page_addr = 0; 
        }
        /* We are starting a new page. read file number and file page. */
        _file_num  = _buffer_read(_read_buffer_num, 0);
        _file_num  = (_file_num  << 8) | _buffer_read(_read_buffer_num, 1);
        _file_page = _buffer_read(_read_buffer_num, 2);
        _file_page = (_file_page << 8) | _buffer_read(_read_buffer_num, 3);

    }
    return result;
}

uint16_t CommonDataflash::read_word() {
    uint16_t result = read_byte(); /* High byte */
    result = (result << 8) | read_byte(); /* Low byte */
    return result;
}

uint32_t CommonDataflash::read_dword() {
    uint32_t result = read_byte(); /* High byte */
    result = (result << 8) | read_byte();
    result = (result << 8) | read_byte();
    result = (result << 8) | read_byte(); /* Low byte */
    return result;
}

void CommonDataflash::set_file(uint16_t filenum) {
    _file_num = filenum;
    _file_page = 1;
}

int16_t CommonDataflash::find_last_log() {
    int16_t last_page = _find_last_page();
    /* start_read will populate _file_num. */
    start_read(last_page);
    return _file_num; 
}

void CommonDataflash::get_log_boundaries(uint8_t log,
                                int16_t &startpage, int16_t &endpage) {
    /* XXX Here be dragons. I transliterated this code from DataFlash_Class::
     * get_log_boundaries - pch 04sept12 */ 
    int16_t num_logs = get_num_logs();
    if ( num_logs == 1 ) {
        /* Read the file number from the last page. */
        start_read(_num_pages);
        /* invariant: find_last_page_of_log does not change _file_num */
        endpage = _find_last_page_of_log((uint16_t)log);
        if (_file_num == 0xFFFF) {
            startpage = 1;
        } else { 
            startpage = endpage + 1;
        }
    } else {
        if (log == 1) {
            start_read(_num_pages);
            if (_file_num == 0xFFFF) {
                startpage = 1;
            } else {
                startpage = _find_last_page() + 1;
            }
        } else {
            if ( log == (find_last_log() - num_logs + 1) ) {
                startpage = _find_last_page() + 1;
            } else {
                int16_t look = num_logs - 1;
                do { startpage = _find_last_page_of_log(look) + 1;
                     look--;
                } while (startpage <= 0 && look >= 1);
            }
        }
    }
    if (startpage == ( (int16_t) _num_pages + 1 ) ||  startpage == 0 ) {
        startpage = 1;
    }
    endpage = _find_last_page_of_log((uint16_t) log);
    if (endpage <= 0) { 
        endpage = startpage;
    }
}

uint8_t CommonDataflash::get_num_logs() {
    /* First try _find_last_page */
    int16_t last_page = _find_last_page();
    if (last_page == 1) {
        return 0;
    }
    /* Read _file_num from page 1 */
    start_read(1);
    if (_file_num == 0xFFFF) {
        return 0;
    }

    /* Read _file_num from last page */
    start_read(last_page);
    uint16_t last = get_file();
    /* XXX bounds check on last_page+2? */
    /* Read _file_num from last_page+2 */
    start_read(last_page+2);
    uint16_t first = _file_num;
    if (first > last) {
        /* We wrapped aroung, so get the file_num from page 1. */
        start_read(1);
        first = _file_num;
    }

    if (last == first) {
        return 1;
    } else {
        return (last - first + 1);
    }
}

void CommonDataflash::start_new_log() {
    uint16_t last_page = _find_last_page();
    start_read(last_page);
    /* XXX I'm pretty sure there's a bug here - find_last_log will overwrite
     * the _file_num invariant from start_read(last_page).
     * However, I'm reproducing the existing DataFlash_Class faithfully. */
    if (find_last_log() == 0 || _file_num == 0xFFFF) {
        set_file(1);
        start_write(1);
        return;
    }
    /* Check for log of length 1 page and suppress */
    if (get_file() <= 1) {
        /* Last log is too short, reuse its number */
        set_file(_file_num);
        /* and overwrite it */
        start_write(last_page);
    } else {
        /* XXX shouldn't we have checked (== 0xFFFF) before using last_page
         * in the case above? */
        if (last_page == 0xFFFF) {
            last_page = 0;
        }
        set_file(get_file() + 1);
        start_write(last_page + 1);
    }
}

int16_t CommonDataflash::_find_last_page() {
    uint16_t top = _num_pages;
    uint16_t bottom = 1;
    start_read(bottom);
    uint32_t bottom_hash = ((uint32_t) _file_num) << 16 | _file_page;

    while ( top - bottom > 1 ) {
        uint16_t look = (top + bottom) / 2;
        start_read(look);
        uint32_t look_hash = ((uint32_t) _file_num) << 16 | _file_page;
        if (look_hash >= 0xFFFF0000) {
            look_hash = 0;
        }
        if (look_hash < bottom_hash) {
            /* move down */
            top = look;
        } else {
            /* move up */
            bottom = look;
            bottom_hash = look_hash;
        }
    }

    start_read(top);
    uint32_t top_hash = ((uint32_t) _file_num) << 16 | _file_page;
    if (top_hash >= 0xFFFF0000) {
        top_hash = 0;
    }
    if (top_hash > bottom_hash) {
        return top;
    } else {
        return bottom;
    }
}

int16_t CommonDataflash::_find_last_page_of_log(uint16_t log_num) {

    uint16_t bottom, top;

    if (_check_wrapped()) {
        start_read(1);
        bottom = _file_num;
        if (bottom > log_num) {
            bottom = _find_last_page();
            top = _num_pages; 
        } else {
            bottom = 1;
            top = _find_last_page();
        }
    } else {
        bottom = 1;
        top = _find_last_page();
    }

    uint32_t check_hash = ((int32_t) log_num) << 16 | 0xFFFF;
    while (top - bottom > 1) {
        uint16_t look = (top + bottom) / 2;
        start_read(look);
        uint32_t look_hash = ((uint32_t) _file_num) << 16 | _file_page;
        if (look_hash >= 0xFFFF0000) {
            look_hash = 0;
        }
        if (look_hash > check_hash) {
            top = look; /* move down */
        } else {
            bottom = look; /* move up */
        }
    }

    start_read(top);
    if (_file_num == log_num) {
        return top;
    }
    start_read(bottom);
    if (_file_num == log_num) {
        return bottom;
    }
    return -1;
}

bool CommonDataflash::_check_wrapped() {
    start_read(_num_pages);
    return (_file_num != 0xFFFF);
}
