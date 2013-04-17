/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "DataFlash.h"

// This function determines the number of whole or partial log files in the DataFlash
// Wholly overwritten files are (of course) lost.
uint16_t DataFlash_Block::get_num_logs(void)
{
    uint16_t lastpage;
    uint16_t last;
    uint16_t first;

    if (find_last_page() == 1) {
        return 0;
    }

    StartRead(1);

    if (GetFileNumber() == 0xFFFF) {
        return 0;
    }

    lastpage = find_last_page();
    StartRead(lastpage);
    last = GetFileNumber();
    StartRead(lastpage + 2);
    first = GetFileNumber();
    if(first > last) {
        StartRead(1);
        first = GetFileNumber();
    }

    if (last == first) {
        return 1;
    }

    return (last - first + 1);
}


// This function starts a new log file in the DataFlash
uint16_t DataFlash_Block::start_new_log(void)
{
    uint16_t last_page = find_last_page();

    StartRead(last_page);
    //Serial.print("last page: ");	Serial.println(last_page);
    //Serial.print("file #: ");	Serial.println(GetFileNumber());
    //Serial.print("file page: ");	Serial.println(GetFilePage());

    if(find_last_log() == 0 || GetFileNumber() == 0xFFFF) {
        SetFileNumber(1);
        StartWrite(1);
        //Serial.println("start log from 0");
        return 1;
    }

    uint16_t new_log_num;

    // Check for log of length 1 page and suppress
    if(GetFilePage() <= 1) {
        new_log_num = GetFileNumber();
        // Last log too short, reuse its number
        // and overwrite it
        SetFileNumber(new_log_num);
        StartWrite(last_page);
    } else {
        new_log_num = GetFileNumber()+1;
        if (last_page == 0xFFFF) {
            last_page=0;
        }
        SetFileNumber(new_log_num);
        StartWrite(last_page + 1);
    }
    return new_log_num;
}

// This function finds the first and last pages of a log file
// The first page may be greater than the last page if the DataFlash has been filled and partially overwritten.
void DataFlash_Block::get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page)
{
    uint16_t num = get_num_logs();
    uint16_t look;

    if (df_BufferIdx != 0) {
        FinishWrite();
    }

    if(num == 1)
    {
        StartRead(df_NumPages);
        if (GetFileNumber() == 0xFFFF)
        {
            start_page = 1;
            end_page = find_last_page_of_log((uint16_t)log_num);
        } else {
            end_page = find_last_page_of_log((uint16_t)log_num);
            start_page = end_page + 1;
        }

    } else {
        if(log_num==1) {
            StartRead(df_NumPages);
            if(GetFileNumber() == 0xFFFF) {
                start_page = 1;
            } else {
                start_page = find_last_page() + 1;
            }
        } else {
            if(log_num == find_last_log() - num + 1) {
                start_page = find_last_page() + 1;
            } else {
                look = log_num-1;
                do {
                    start_page = find_last_page_of_log(look) + 1;
                    look--;
                } while (start_page <= 0 && look >=1);
            }
        }
    }
    if (start_page == df_NumPages+1 || start_page == 0) {
        start_page = 1;
    }
    end_page = find_last_page_of_log(log_num);
    if (end_page == 0) {
        end_page = start_page;
    }
}

bool DataFlash_Block::check_wrapped(void)
{
    StartRead(df_NumPages);
    if(GetFileNumber() == 0xFFFF)
        return 0;
    else
        return 1;
}


// This funciton finds the last log number
uint16_t DataFlash_Block::find_last_log(void)
{
    uint16_t last_page = find_last_page();
    StartRead(last_page);
    return GetFileNumber();
}

// This function finds the last page of the last file
uint16_t DataFlash_Block::find_last_page(void)
{
    uint16_t look;
    uint16_t bottom = 1;
    uint16_t top = df_NumPages;
    uint32_t look_hash;
    uint32_t bottom_hash;
    uint32_t top_hash;

    StartRead(bottom);
    bottom_hash = ((int32_t)GetFileNumber()<<16) | GetFilePage();

    while(top-bottom > 1) {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int32_t)GetFileNumber()<<16 | GetFilePage();
        if (look_hash >= 0xFFFF0000) look_hash = 0;

        if(look_hash < bottom_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
            bottom_hash = look_hash;
        }
    }

    StartRead(top);
    top_hash = ((int32_t)GetFileNumber()<<16) | GetFilePage();
    if (top_hash >= 0xFFFF0000) {
        top_hash = 0;
    }
    if (top_hash > bottom_hash) {
        return top;
    }

    return bottom;
}

// This function finds the last page of a particular log file
uint16_t DataFlash_Block::find_last_page_of_log(uint16_t log_number)
{
    uint16_t look;
    uint16_t bottom;
    uint16_t top;
    uint32_t look_hash;
    uint32_t check_hash;

    if(check_wrapped())
    {
        StartRead(1);
        bottom = GetFileNumber();
        if (bottom > log_number)
        {
            bottom = find_last_page();
            top = df_NumPages;
        } else {
            bottom = 1;
            top = find_last_page();
        }
    } else {
        bottom = 1;
        top = find_last_page();
    }

    check_hash = (int32_t)log_number<<16 | 0xFFFF;

    while(top-bottom > 1)
    {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (int32_t)GetFileNumber()<<16 | GetFilePage();
        if (look_hash >= 0xFFFF0000) look_hash = 0;

        if(look_hash > check_hash) {
            // move down
            top = look;
        } else {
            // move up
            bottom = look;
        }
    }

    StartRead(top);
    if (GetFileNumber() == log_number) return top;

    StartRead(bottom);
    if (GetFileNumber() == log_number) return bottom;

    return -1;
}


/*
  Read the DataFlash log memory
  Call the callback() function on each log message found in the page
  range. Return the number of log messages found

  Note that for the block oriented backend the log_num is ignored
*/
void DataFlash_Block::log_read_process(uint16_t log_num,
                                       uint16_t start_page, uint16_t end_page, 
                                       void (*callback)(uint8_t msgid))
{
    uint8_t log_step = 0;
    uint16_t page = start_page;

    if (df_BufferIdx != 0) {
        FinishWrite();
    }

    StartRead(start_page);

	while (true) {
		uint8_t data;
        ReadBlock(&data, 1);

		// This is a state machine to read the packets
		switch(log_step) {
			case 0:
				if (data == HEAD_BYTE1) {
					log_step++;
                }
				break;

			case 1:
				if (data == HEAD_BYTE2) {
					log_step++;
                } else {
					log_step = 0;
				}
				break;

			case 2:
				log_step = 0;
                callback(data);
                break;
		}
        uint16_t new_page = GetPage();
        if (new_page != page) {
            if (new_page == end_page || new_page == start_page) {
                return;
            }
            page = new_page;
        }
	}
}

/*
  dump header information from all log pages
 */
void DataFlash_Block::DumpPageInfo(AP_HAL::BetterStream *port)
{
    for (uint16_t count=1; count<=df_NumPages; count++) {
        StartRead(count);
        port->printf_P(PSTR("DF page, log file #, log page: %u,\t"), (unsigned)count);
        port->printf_P(PSTR("%u,\t"), (unsigned)GetFileNumber());
        port->printf_P(PSTR("%u\n"), (unsigned)GetFilePage());
    }
}

/*
  show information about the device
 */
void DataFlash_Block::ShowDeviceInfo(AP_HAL::BetterStream *port)
{
    if (!CardInserted()) {
        port->println_P(PSTR("No dataflash inserted"));
        return;
    }
    ReadManufacturerID();
    port->printf_P(PSTR("Manufacturer: 0x%02x   Device: 0x%04x\n"),
                    (unsigned)df_manufacturer,
                    (unsigned)df_device);
    port->printf_P(PSTR("NumPages: %u  PageSize: %u\n"),
                   (unsigned)df_NumPages+1,
                   (unsigned)df_PageSize);
}

/*
  list available log numbers
 */
void DataFlash_Block::ListAvailableLogs(AP_HAL::BetterStream *port)
{
    uint16_t num_logs = get_num_logs();
    int16_t last_log_num = find_last_log();
    uint16_t log_start = 0;
    uint16_t log_end = 0;

    if (num_logs == 0) {
        port->printf_P(PSTR("\nNo logs\n\n"));
        return;
    }
    port->printf_P(PSTR("\n%u logs\n"), (unsigned)num_logs);

    for (uint16_t i=num_logs; i>=1; i--) {
        uint16_t last_log_start = log_start, last_log_end = log_end;
        uint16_t temp = last_log_num - i + 1;
        get_log_boundaries(temp, log_start, log_end);
        port->printf_P(PSTR("Log %u,    start %u,   end %u\n"), 
                       (unsigned)temp, 
                       (unsigned)log_start, 
                       (unsigned)log_end);
        if (last_log_start == log_start && last_log_end == log_end) {
            // we are printing bogus logs
            break;
        }
    }
    port->println();
}
