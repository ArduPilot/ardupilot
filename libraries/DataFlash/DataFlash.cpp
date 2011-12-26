/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	DataFlash.cpp - DataFlash log library generic code
*/

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
