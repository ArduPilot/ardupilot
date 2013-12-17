/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  hacked up DataFlash library for Desktop support
*/

#include "DataFlash_Empty.h"
#define DF_PAGE_SIZE 512
#define DF_NUM_PAGES 4096

// Public Methods //////////////////////////////////////////////////////////////
void DataFlash_Empty::Init(const struct LogStructure *structure, uint8_t num_types)
{
    DataFlash_Class::Init(structure, num_types);
	df_PageSize = DF_PAGE_SIZE;
    // reserve last page for config information
    df_NumPages   = DF_NUM_PAGES - 1;
}

// This function is mainly to test the device
void DataFlash_Empty::ReadManufacturerID()
{
	df_manufacturer = 1;
	df_device = 0x0203;
}

bool DataFlash_Empty::CardInserted(void)
{
    return true;
}

// Read the status register
uint8_t DataFlash_Empty::ReadStatusReg()
{
	return 0;
}

// Read the status of the DataFlash
inline
uint8_t DataFlash_Empty::ReadStatus()
{
	return 1;
}


inline uint16_t DataFlash_Empty::PageSize()
{ return 0; }


// Wait until DataFlash is in ready state...
void DataFlash_Empty::WaitReady()
{ }

void DataFlash_Empty::PageToBuffer(unsigned char BufferNum, uint16_t PageAdr)
{ }

void DataFlash_Empty::BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait)
{ }

void DataFlash_Empty::BufferWrite (unsigned char BufferNum,
        uint16_t IntPageAdr, unsigned char Data)
{ }

unsigned char DataFlash_Empty::BufferRead (unsigned char BufferNum,
        uint16_t IntPageAdr)
{ return 0; }

void DataFlash_Empty::BlockWrite(uint8_t BufferNum, uint16_t IntPageAdr, 
                    const void *pHeader, uint8_t hdr_size,
                    const void *pBuffer, uint16_t size)
{ }

bool DataFlash_Empty::BlockRead(uint8_t BufferNum, uint16_t IntPageAdr,
        void *pBuffer, uint16_t size)
{ 
    memset(pBuffer, 0, size);
    return false; 
}

// *** END OF INTERNAL FUNCTIONS ***

void DataFlash_Empty::PageErase (uint16_t PageAdr) { }

void DataFlash_Empty::BlockErase (uint16_t BlockAdr) { }

void DataFlash_Empty::ChipErase() { }


