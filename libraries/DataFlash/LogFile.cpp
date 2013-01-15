/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "DataFlash.h"

// This function determines the number of whole or partial log files in the DataFlash
// Wholly overwritten files are (of course) lost.
uint8_t DataFlash_Class::get_num_logs(void)
{
    uint16_t lastpage;
    uint16_t last;
    uint16_t first;

    if (find_last_page() == 1) {
        return 0;
    }

    StartRead(1);

    if (GetFileNumber() == 0XFFFF) {
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
void DataFlash_Class::start_new_log(void)
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
        return;
    }

    // Check for log of length 1 page and suppress
    if(GetFilePage() <= 1) {
        SetFileNumber(GetFileNumber());                 // Last log too short, reuse its number
        StartWrite(last_page);                                          // and overwrite it
        //Serial.println("start log from short");
    } else {
        if(last_page == 0xFFFF) last_page=0;
        SetFileNumber(GetFileNumber()+1);
        StartWrite(last_page + 1);
        //Serial.println("start log normal");
    }
}

// This function finds the first and last pages of a log file
// The first page may be greater than the last page if the DataFlash has been filled and partially overwritten.
void DataFlash_Class::get_log_boundaries(uint8_t log_num, int16_t & start_page, int16_t & end_page)
{
    int16_t num = get_num_logs();
    int16_t look;

    if(num == 1)
    {
        StartRead(df_NumPages);
        if(GetFileNumber() == 0xFFFF)
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
    if(start_page == (int16_t)df_NumPages+1 || start_page == 0) start_page=1;
    end_page = find_last_page_of_log((uint16_t)log_num);
    if(end_page <= 0) end_page = start_page;
}

bool DataFlash_Class::check_wrapped(void)
{
    StartRead(df_NumPages);
    if(GetFileNumber() == 0xFFFF)
        return 0;
    else
        return 1;
}


// This funciton finds the last log number
int16_t DataFlash_Class::find_last_log(void)
{
    int16_t last_page = find_last_page();
    StartRead(last_page);
    return GetFileNumber();
}

// This function finds the last page of the last file
int16_t DataFlash_Class::find_last_page(void)
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
int16_t DataFlash_Class::find_last_page_of_log(uint16_t log_number)
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


// Read the DataFlash log memory : Packet Parser. Call the callback() 
// function on each log message found in the page range. Return the
// number of log messages found
uint16_t DataFlash_Class::log_read_process(uint16_t start_page, uint16_t end_page, 
                                           void (*callback)(uint8_t msgid))
{
    uint8_t log_step = 0;
    uint16_t page = start_page;
    uint16_t packet_count = 0;

    StartRead(start_page);

	while (true) {
		uint8_t data = ReadByte();

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
                packet_count++;
                break;
		}
        uint16_t new_page = GetPage();
        if (new_page != page) {
            if (new_page == end_page) break;
            page = new_page;
        }
	}

	return packet_count;
}
