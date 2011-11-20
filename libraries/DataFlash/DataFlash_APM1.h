/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef __DATAFLASH_APM1_H__
#define __DATAFLASH_APM1_H__

#include "DataFlash.h"

// flash size
#define DF_LAST_PAGE 4096

class DataFlash_APM1 : public DataFlash_Class
{
  private:
	// DataFlash Log variables...
	unsigned char df_BufferNum;
	unsigned char df_Read_BufferNum;
	uint16_t df_BufferIdx;
	uint16_t df_Read_BufferIdx;
	uint16_t df_PageAdr;
	uint16_t df_Read_PageAdr;
	unsigned char df_Read_END;
	unsigned char df_Stop_Write;
	uint16_t df_FileNumber;
	uint16_t df_FilePage;
	//Methods
	unsigned char BufferRead (unsigned char BufferNum, uint16_t IntPageAdr);
	void BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data);
	void BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait);
	void PageToBuffer(unsigned char BufferNum, uint16_t PageAdr);
	void WaitReady();
	unsigned char ReadStatusReg();
	unsigned char ReadStatus();
	uint16_t PageSize();

  public:
	unsigned char df_manufacturer;
	unsigned char df_device_0;
	unsigned char df_device_1;
	uint16_t df_PageSize;

	DataFlash_APM1(); // Constructor
	void Init();
	void ReadManufacturerID();
	int16_t GetPage();
	int16_t GetWritePage();
	void PageErase (uint16_t PageAdr);
	void ChipErase ();
	// Write methods
	void StartWrite(int16_t PageAdr);
	void FinishWrite();
	void WriteByte(unsigned char data);
	void WriteInt(int16_t data);
	void WriteLong(int32_t data);

	// Read methods
	void StartRead(int16_t PageAdr);
	unsigned char ReadByte();
	int16_t ReadInt();
	int32_t ReadLong();

	void SetFileNumber(uint16_t FileNumber);
	uint16_t GetFileNumber();
	uint16_t GetFilePage();
};

#endif // __DATAFLASH_APM1_H__
