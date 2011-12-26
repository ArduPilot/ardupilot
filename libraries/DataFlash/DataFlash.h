/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

#include <stdint.h>

#define DF_OVERWRITE_DATA 1 // 0: When reach the end page stop, 1: Start overwriting from page 1

class DataFlash_Class
{
  private:
	// DataFlash Log variables...
	unsigned char df_BufferNum;
	unsigned char df_Read_BufferNum;
	uint16_t df_BufferIdx;
	uint16_t df_Read_BufferIdx;
	uint16_t df_PageAdr;
	uint16_t df_Read_PageAdr;
	unsigned char df_Stop_Write;
	uint16_t df_FileNumber;
	uint16_t df_FilePage;

	virtual void WaitReady() = 0;
	virtual void BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data) = 0;
	virtual void BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait) = 0;
	virtual void PageToBuffer(unsigned char BufferNum, uint16_t PageAdr) = 0;
	virtual unsigned char BufferRead (unsigned char BufferNum, uint16_t IntPageAdr) = 0;

  public:
	unsigned char df_manufacturer;
	unsigned char df_device_0;
	unsigned char df_device_1;

	DataFlash_Class() {} // Constructor

	virtual void Init(void) = 0;
	virtual void ReadManufacturerID() = 0;
	int16_t GetPage(void);
	int16_t GetWritePage(void);
	virtual void PageErase(uint16_t PageAdr) = 0;
	virtual void ChipErase(void) = 0;

	// Write methods
	void StartWrite(int16_t PageAdr);
	void FinishWrite(void);
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

	uint16_t df_PageSize;
	uint16_t df_NumPages;
};

#include "DataFlash_APM1.h"
#include "DataFlash_APM2.h"

#endif
