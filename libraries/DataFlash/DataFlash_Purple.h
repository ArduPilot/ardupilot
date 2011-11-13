/* ************************************************************ */
/* DataFlash_Purple Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_PURPLE_H__
#define __DATAFLASH_PURPLE_H__

#include "DataFlash.h"

class DataFlash_Purple : public DataFlash_Class
{
  private:
	// DataFlash Log variables...
	unsigned char df_BufferNum;
	unsigned char df_Read_BufferNum;
	unsigned int df_BufferIdx;
	unsigned int df_Read_BufferIdx;
	unsigned int df_PageAdr;
	unsigned int df_Read_PageAdr;
	unsigned char df_Read_END;
	unsigned char df_Stop_Write;
	//Methods
	unsigned char BufferRead (unsigned char BufferNum, unsigned int IntPageAdr);
	void BufferWrite (unsigned char BufferNum, unsigned int IntPageAdr, unsigned char Data);
	void BufferToPage (unsigned char BufferNum, unsigned int PageAdr, unsigned char wait);
	void PageToBuffer(unsigned char BufferNum, unsigned int PageAdr);
	void WaitReady();
	unsigned char ReadStatusReg();
	unsigned char ReadStatus();
	unsigned int PageSize();

	unsigned char SPI_transfer(unsigned char data);
	void CS_inactive();
	void CS_active();


  public:
	unsigned char df_manufacturer;
	unsigned char df_device_0;
	unsigned char df_device_1;
	unsigned int df_PageSize;

	DataFlash_Purple(); // Constructor
	void Init();
	void ReadManufacturerID();
    int CardInserted();
	int GetPage();
	int GetWritePage();
	void PageErase (unsigned int PageAdr);
	void ChipErase ();
	// Write methods
	void StartWrite(int PageAdr);
	void FinishWrite();
	void WriteByte(unsigned char data);
	void WriteInt(int data);
	void WriteLong(long data);

	// Read methods
	void StartRead(int PageAdr);
	unsigned char ReadByte();
	int ReadInt();
	long ReadLong();
};

#endif
