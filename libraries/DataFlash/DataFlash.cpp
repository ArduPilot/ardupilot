/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	DataFlash.cpp - DataFlash log library generic code
*/

#include <FastSerial.h>
#include <stdint.h>
#include "DataFlash.h"


// *** DATAFLASH PUBLIC FUNCTIONS ***
void DataFlash_Class::StartWrite(int16_t PageAdr)
{
    df_BufferNum=1;
    df_BufferIdx=4;
    df_PageAdr=PageAdr;
    df_Stop_Write=0;
    WaitReady();

    // We are starting a new page - write FileNumber and FilePage
    BufferWrite(df_BufferNum,0,df_FileNumber>>8);   // High byte
    BufferWrite(df_BufferNum,1,df_FileNumber&0xFF); // Low byte
    BufferWrite(df_BufferNum,2,df_FilePage>>8);   // High byte
    BufferWrite(df_BufferNum,3,df_FilePage&0xFF); // Low byte
}

void DataFlash_Class::FinishWrite(void)
{
	df_BufferIdx=0;
	BufferToPage(df_BufferNum,df_PageAdr,0);  // Write Buffer to memory, NO WAIT
	df_PageAdr++;
	if (DF_OVERWRITE_DATA==1) {
        if (df_PageAdr>df_NumPages)  // If we reach the end of the memory, start from the begining
            df_PageAdr = 1;
    } else {
        if (df_PageAdr>df_NumPages)  // If we reach the end of the memory, stop here
            df_Stop_Write=1;
    }

	if (df_BufferNum==1)  // Change buffer to continue writing...
        df_BufferNum=2;
	else
        df_BufferNum=1;
}


void DataFlash_Class::WriteByte(uint8_t data)
{
    if (!df_Stop_Write) {
        BufferWrite(df_BufferNum,df_BufferIdx,data);
        df_BufferIdx++;
        if (df_BufferIdx >= df_PageSize)  // End of buffer?
        {
            df_BufferIdx=4;		//(4 bytes for FileNumber, FilePage)
            BufferToPage(df_BufferNum,df_PageAdr,0);  // Write Buffer to memory, NO WAIT
            df_PageAdr++;
            if (DF_OVERWRITE_DATA==1)
            {
                if (df_PageAdr>df_NumPages)  // If we reach the end of the memory, start from the begining
                    df_PageAdr = 1;
            }
            else
            {
                if (df_PageAdr>df_NumPages)  // If we reach the end of the memory, stop here
                    df_Stop_Write=1;
            }

            if (df_BufferNum==1)  // Change buffer to continue writing...
                df_BufferNum=2;
            else
                df_BufferNum=1;

            // We are starting a new page - write FileNumber and FilePage
            BufferWrite(df_BufferNum,0,df_FileNumber>>8);   // High byte
            BufferWrite(df_BufferNum,1,df_FileNumber&0xFF); // Low byte
            df_FilePage++;
            BufferWrite(df_BufferNum,2,df_FilePage>>8);   // High byte
            BufferWrite(df_BufferNum,3,df_FilePage&0xFF); // Low byte
        }
    }
}

void DataFlash_Class::WriteInt(int16_t data)
{
    WriteByte(data>>8);   // High byte
    WriteByte(data&0xFF); // Low byte
}

void DataFlash_Class::WriteLong(int32_t data)
{
    WriteByte(data>>24);   // First byte
    WriteByte(data>>16);
    WriteByte(data>>8);
    WriteByte(data&0xFF);  // Last byte
}

// Get the last page written to
int16_t DataFlash_Class::GetWritePage()
{
    return df_PageAdr;
}

// Get the last page read
int16_t DataFlash_Class::GetPage()
{
  return df_Read_PageAdr-1;
}

void DataFlash_Class::StartRead(int16_t PageAdr)
{
    df_Read_BufferNum=1;
    df_Read_BufferIdx=4;
    df_Read_PageAdr=PageAdr;
    WaitReady();
    PageToBuffer(df_Read_BufferNum,df_Read_PageAdr);  // Write Memory page to buffer
//Serial.print(df_Read_PageAdr, DEC);	Serial.print("\t");
    df_Read_PageAdr++;

    // We are starting a new page - read FileNumber and FilePage
    df_FileNumber = BufferRead(df_Read_BufferNum,0);   // High byte
//Serial.print(df_FileNumber, DEC);	Serial.print("\t");
    df_FileNumber = (df_FileNumber<<8) | BufferRead(df_Read_BufferNum,1); // Low byte
//Serial.println(df_FileNumber, DEC);	Serial.print("\t");
    df_FilePage = BufferRead(df_Read_BufferNum,2);   // High byte
    df_FilePage = (df_FilePage<<8) | BufferRead(df_Read_BufferNum,3); // Low byte
}

uint8_t DataFlash_Class::ReadByte()
{
    uint8_t result;

    WaitReady();
    result = BufferRead(df_Read_BufferNum,df_Read_BufferIdx);
    df_Read_BufferIdx++;
    if (df_Read_BufferIdx >= df_PageSize)  // End of buffer?
    {
        df_Read_BufferIdx=4;		//(4 bytes for FileNumber, FilePage)
        PageToBuffer(df_Read_BufferNum,df_Read_PageAdr);  // Write memory page to Buffer
        df_Read_PageAdr++;
        if (df_Read_PageAdr>df_NumPages)  // If we reach the end of the memory, start from the begining
        {
            df_Read_PageAdr = 0;
        }

        // We are starting a new page - read FileNumber and FilePage
		df_FileNumber = BufferRead(df_Read_BufferNum,0);   // High byte
		df_FileNumber = (df_FileNumber<<8) | BufferRead(df_Read_BufferNum,1); // Low byte
		df_FilePage = BufferRead(df_Read_BufferNum,2);   // High byte
		df_FilePage = (df_FilePage<<8) | BufferRead(df_Read_BufferNum,3); // Low byte
    }
    return result;
}

int16_t DataFlash_Class::ReadInt()
{
    uint16_t result;

    result = ReadByte();               // High byte
    result = (result<<8) | ReadByte(); // Low byte
    return (int16_t)result;
}

int32_t DataFlash_Class::ReadLong()
{
    uint32_t result;

    result = ReadByte();               // First byte
    result = (result<<8) | ReadByte();
    result = (result<<8) | ReadByte();
    result = (result<<8) | ReadByte(); // Last byte
    return (int32_t)result;
}

void DataFlash_Class::SetFileNumber(uint16_t FileNumber)
{
	df_FileNumber = FileNumber;
	df_FilePage = 1;
}

uint16_t DataFlash_Class::GetFileNumber()
{
	return df_FileNumber;
}

uint16_t DataFlash_Class::GetFilePage()
{
	return df_FilePage;
}

void DataFlash_Class::EraseAll(void (*delay_cb)(unsigned long))
{
	for(uint16_t j = 1; j <= (df_NumPages+1)/8; j++) {
		BlockErase(j);
		delay_cb(1);
	}
    // write the logging format in the last page
    StartWrite(df_NumPages+1);
    WriteLong(DF_LOGGING_FORMAT);
	FinishWrite();
}

/*
  we need to erase if the logging format has changed
 */
bool DataFlash_Class::NeedErase(void)
{
    StartRead(df_NumPages+1);
    return ReadLong() != DF_LOGGING_FORMAT;
}



// This function determines the number of whole or partial log files in the DataFlash
// Wholly overwritten files are (of course) lost.
uint8_t DataFlash_Class::get_num_logs(void)
{
	uint16_t lastpage;
	uint16_t last;
	uint16_t first;

	if(find_last_page() == 1) return 0;

	StartRead(1);

	if(GetFileNumber() == 0XFFFF) return 0;

	lastpage = find_last_page();
	StartRead(lastpage);
	last = GetFileNumber();
	StartRead(lastpage + 2);
	first = GetFileNumber();
	if(first > last) {
		StartRead(1);
		first = GetFileNumber();
	}
	if(last == first)
	{
		return 1;
	} else {
		return (last - first + 1);
	}
}

// This function starts a new log file in the DataFlash
void DataFlash_Class::start_new_log(void)
{
	uint16_t	last_page = find_last_page();

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
		SetFileNumber(GetFileNumber());		// Last log too short, reuse its number
		StartWrite(last_page);					// and overwrite it
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
    bottom_hash = ((long)GetFileNumber()<<16) | GetFilePage();

    while(top-bottom > 1) {
        look = (top+bottom)/2;
        StartRead(look);
        look_hash = (long)GetFileNumber()<<16 | GetFilePage();
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
    top_hash = ((long)GetFileNumber()<<16) | GetFilePage();
    if (top_hash >= 0xFFFF0000) {
        top_hash = 0;
    }
    if (top_hash > bottom_hash)	{
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

	check_hash = (long)log_number<<16 | 0xFFFF;

	while(top-bottom > 1)
	{
		look = (top+bottom)/2;
		StartRead(look);
		look_hash = (long)GetFileNumber()<<16 | GetFilePage();
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
